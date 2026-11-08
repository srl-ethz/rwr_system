#!/bin/env python3

import os
import glob
import argparse
from scipy.spatial.transform import Rotation as R, Slerp
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import interp1d
import numpy as np
import h5py
import cv2
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import re
import yaml

TOPIC_TO_STRING = {
    Float32MultiArray: "Float32MultiArray",
    PoseStamped: "PoseStamped",
    Image: "Image",
    String: "String",
}

def get_topic_names(h5_path):
    with h5py.File(h5_path, 'r') as h5_file:
        topic_names = list(h5_file.keys())
        print(f"Topics in the HDF5 file: {topic_names}")
    return topic_names

class Topic2KeyMatcher:
    """
    A class to match topic names and dtype to the proper key name in hdf.
    """
    def __init__(self, keys_and_topics):
        for k, v in keys_and_topics.items():
            assert 'dtype' in v
            assert 'topic' in v
            v['rgx'] = re.compile('^'+v['topic']+'$')
        self.keys_and_topics = keys_and_topics
        self.match_result_cache = {}

    def match(self, topic, dtype):
        """
        Match the given topic to a key keys_and_topics.
        """
        keys = []
        if (topic, dtype) in self.match_result_cache:
            return self.match_result_cache[(topic, dtype)]
        
        for key, value in self.keys_and_topics.items():
            if dtype != value['dtype']:
                continue
            m = value['rgx'].match(topic)
            if m:
                for i, group in enumerate(m.groups()):
                    key = key.replace(f"{{{i}}}", group)
                keys.append(key)
        if not len(keys):
            print(f"Warning: No keys found for topic {topic} with dtype {dtype}.")
        self.match_result_cache[(topic, dtype)] = keys
        return keys


def sample_and_sync_h5(keys_and_topics, topics_and_types, input_h5_path, output_h5_path, sampling_frequency, compress, resize_to):
    """
    Sample images and interpolate data for synchronization.
    
    Parameters:
        input_h5_path (str): Path to the input HDF5 file.
        output_h5_path (str): Path to the output HDF5 file.
        sampling_frequency (float): Sampling frequency in Hz.
        topic_types (dict): Dictionary mapping topics to their types.
    """
    key_matcher = Topic2KeyMatcher(keys_and_topics)
    if sampling_frequency <= 0:
        raise ValueError("sampling_frequency must be positive")
    with h5py.File(input_h5_path, 'r') as input_h5, h5py.File(output_h5_path, 'w') as output_h5:
        # Determine sampling timestamps
        start_time = None
        end_time = None
        for topic, dtype in topics_and_types.items():
            if topic in input_h5:
                if dtype == "std_msgs.msg.String" or any(k in topic for k in ("intrinsics", "extrinsics", "projection")):
                    continue
                if not key_matcher.match(topic, dtype):
                    continue
                timestamps = np.sort(np.array(list(map(int, input_h5[topic].keys()))))
                if len(timestamps) < 2:
                    raise ValueError(f"Need at least two samples for {topic}")
                if start_time is None or timestamps[0] > start_time:
                    start_time = timestamps[0]
                if end_time is None or timestamps[-1] < end_time:
                    end_time = timestamps[-1]
        if start_time is None or end_time <= start_time:
            raise ValueError("Recorded streams have no shared time interval")
        desired_timestamps = np.arange(
            start_time, end_time, 1e9 / sampling_frequency
        ).astype(int)
        output_h5.create_dataset("timestamps", data=desired_timestamps)
        output_h5.attrs["sampling_frequency"] = sampling_frequency

        # Process each topic
        for topic, topic_type_name in topics_and_types.items():
            if topic not in input_h5:
                print(f"Topic {topic} not found in the HDF5 file. Skipping...")
                continue
            if not key_matcher.match(topic, topic_type_name):
                continue
            print(f"Processing topic: {topic}")
            topic_group = input_h5[topic]

            if topic_type_name == "std_msgs.msg.String":
                string_data = topic_group["description"]
                keys = key_matcher.match(topic, topic_type_name)
                for k in keys:
                    output_h5.create_dataset(k, data=string_data)
                continue

            topic_timestamps = np.array(list(map(int, topic_group.keys())))
            topic_timestamps.sort()

            if any(k in topic for k in ("intrinsics", "extrinsics", "projection")):
                # Calibration is static; it must not shorten the motion/image overlap.
                for key in key_matcher.match(topic, topic_type_name):
                    dataset = output_h5.create_dataset(key, data=topic_group[str(topic_timestamps[-1])][:])
                    for name, value in topic_group.attrs.items():
                        dataset.attrs[name] = value
                continue

            if topic_type_name == "sensor_msgs.msg.Image":
                # Sample images
                sampled_images = []
                for t in desired_timestamps:
                    closest_idx = np.abs(topic_timestamps - t).argmin()
                    closest_timestamp = topic_timestamps[closest_idx]
                    sampled_images.append(topic_group[str(closest_timestamp)][:])

                if resize_to is not None:
                    sampled_images = [cv2.resize(img, resize_to, interpolation=cv2.INTER_LINEAR) for img in sampled_images]

                sampled_images = np.array(sampled_images)  # TxHxWxC
                chunk_size = (1,) + tuple(sampled_images.shape[1:])
                
                target_keys = key_matcher.match(topic, topic_type_name)
                for k in target_keys:
                    if compress:
                        output_h5.create_dataset(k, data=sampled_images, chunks = chunk_size, compression="lzf")
                    else:
                        output_h5.create_dataset(k, data=sampled_images, chunks = chunk_size)
                    print(f"Saved Image {k} from topic {topic}")

            elif topic_type_name == "geometry_msgs.msg.PoseStamped":
                # Interpolate PoseStamped data
                pose_data = np.array([topic_group[str(ts)][:] for ts in topic_timestamps])
                positions = pose_data[:, :3]
                quaternions = pose_data[:, 3:]
                
                interp_position = interp1d(
                    topic_timestamps, positions, axis=0, kind="linear", bounds_error=True
                )
                relative_times = (topic_timestamps - topic_timestamps[0]) / 1e9
                interp_quaternions = Slerp(relative_times, R.from_quat(quaternions))

                sampled_positions = interp_position(desired_timestamps)
                sampled_quaternions = interp_quaternions(
                    (desired_timestamps - topic_timestamps[0]) / 1e9
                ).as_quat()
                
                keys = key_matcher.match(topic, topic_type_name)
                for k in keys:
                    output_h5.create_dataset(k, data=np.concatenate((sampled_positions, sampled_quaternions), axis=1))
                    print(f"Saved Pose {k} from topic {topic}")

            elif topic_type_name == "std_msgs.msg.Float32MultiArray":
                # Interpolate Float32MultiArray data
                array_data = np.array([topic_group[str(ts)][:] for ts in topic_timestamps])
                interp_array = interp1d(
                    topic_timestamps, array_data, axis=0, kind="linear", bounds_error=True
                )
                sampled_array = interp_array(desired_timestamps)
                
                keys = key_matcher.match(topic, topic_type_name)
                for k in keys:
                    output_h5.create_dataset(k, data=sampled_array)
                    print(f"Saved Array {k} from topic {topic}")
            else:
                print(f"Unsupported topic type {topic_type_name} for topic {topic}. Skipping...")
    print(f"Processed data saved to: {output_h5_path}")

import concurrent.futures

def process_folder(keys_and_topics, topics_and_types, input_folder, sampling_frequency, compress, resize_to, max_parallel=8):
    """
    Process all HDF5 files in the given folder in parallel (up to max_parallel at a time)
    and save the processed files with a running index in a new folder named <input_folder>_processed.
    
    Parameters:
        input_folder (str): Path to the folder containing input HDF5 files.
        sampling_frequency (float): Sampling frequency in Hz.
        compress (bool): Whether to compress the output HDF5 files.
        resize_to (tuple or None): Target size of the image as a tuple of integers.
        max_parallel (int): Maximum number of files to process in parallel.
    """
    # Get all HDF5 files in the folder
    h5_files = sorted(glob.glob(os.path.join(input_folder, "*.h5")))
    if not h5_files:
        print(f"No HDF5 files found in {input_folder}.")
        return

    # Create the output folder
    output_folder = os.path.join(os.path.dirname(input_folder), 
                                 os.path.basename(input_folder) + "_processed" + f"_{int(sampling_frequency)}hz")
    if compress:
        output_folder += "_lzf"
    os.makedirs(output_folder, exist_ok=True)
    print(f"Output folder created: {output_folder}")

    # Use a ProcessPoolExecutor to process files in parallel
    with concurrent.futures.ProcessPoolExecutor(max_workers=max_parallel) as executor:
        futures = []
        for idx, input_file in enumerate(h5_files):
            output_file = os.path.join(output_folder, f"{idx:04d}.h5")
            print(f"Submitting file for processing: {input_file}")
            futures.append(
                executor.submit(sample_and_sync_h5, keys_and_topics, topics_and_types, input_file, output_file, sampling_frequency, compress, resize_to)
            )
        
        failures = []
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
                print("File processed successfully.")
            except Exception as e:
                print(f"Error processing file: {e}")
                failures.append(e)
        if failures:
            raise RuntimeError(f"Failed to process {len(failures)} recording(s)") from failures[0]

    print(f"All files processed. Processed files are saved in {output_folder}.")

def main():
    parser = argparse.ArgumentParser(description="Process and synchronize HDF5 files.")
    parser.add_argument("input_folder", type=str, help="Path to the folder containing input HDF5 files.")
    parser.add_argument("--cfg_file", type=str, default="")
    parser.add_argument("--sampling_freq", type=float, default=10, help="Sampling frequency in Hz.")
    parser.add_argument("--compress",  action="store_true", help="Compress the output HDF5 files. [it might boost the performance on aws but might decrease the performance on local machine]")
    parser.add_argument(
        '--resize_to',
        type=lambda s: tuple(map(int, s.strip("()").split(","))),
        help="Target size of the image as a tuple of integers, e.g., '(width, height)'.",
        default=None
    )
    args = parser.parse_args()

    if args.cfg_file == "":
        cfg_file = os.path.join(get_package_share_directory("experiments"), "cfgs", "logger", "datacollection_default.yaml")
    else:
        cfg_file = args.cfg_file
    cfgyaml = yaml.safe_load(open(cfg_file, 'r'))
    keys_and_topics = cfgyaml['keys_and_topics']
    topics_and_types = cfgyaml['topics_and_types']
    process_folder(keys_and_topics, topics_and_types, args.input_folder, args.sampling_freq, args.compress, args.resize_to)

if __name__ == "__main__":
    main()
