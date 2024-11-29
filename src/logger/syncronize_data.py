#!/bin/env python3

import os
import glob
import argparse
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import numpy as np
import h5py
from logger_node import TOPICS_TYPES  # Import the predefined topic types
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

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

def sample_and_sync_h5(input_h5_path, output_h5_path, sampling_frequency, topic_types):
    qpos_franka = None
    qpos_hand = None
    actions_franka = None
    actions_hand = None
    """
    Sample images and interpolate data for synchronization.
    
    Parameters:
        input_h5_path (str): Path to the input HDF5 file.
        output_h5_path (str): Path to the output HDF5 file.
        sampling_frequency (float): Sampling frequency in Hz.
        topic_types (dict): Dictionary mapping topics to their types.
    """
    with h5py.File(input_h5_path, 'r') as input_h5, h5py.File(output_h5_path, 'w') as output_h5:
        # Determine sampling timestamps
        start_time = None
        end_time = None
        for topic in topic_types:
            if topic in input_h5:
                if topic == "/task_description":
                    continue
                timestamps = np.array(list(map(int, input_h5[topic].keys())))
                if start_time is None or timestamps[0] < start_time:
                    start_time = timestamps[0]
                if end_time is None or timestamps[-1] > end_time:
                    end_time = timestamps[-1]

        desired_timestamps = np.arange(
            start_time, end_time, 1e9 / sampling_frequency
        ).astype(int)

        # Process each topic
        for topic, topic_type in topic_types.items():
            if topic not in input_h5:
                print(f"Topic {topic} not found in the HDF5 file. Skipping...")
                continue
            
            
            print(f"Processing topic: {topic}")
            topic_group = input_h5[topic]

            if topic == "/task_description":
                if TOPIC_TO_STRING[topic_type] == "String":
                    string_data = topic_group["description"]
                    output_h5.create_dataset("task_description", data=string_data)
                continue


            topic_timestamps = np.array(list(map(int, topic_group.keys())))
            topic_timestamps.sort()

            if "intrinsics" in topic or "extrinsics" in topic or "projection" in topic:
                data = np.array(topic_group[str(topic_timestamps[0])][:])
                output_h5.create_dataset(f"observations/images/{topic}", data=data)
                continue

            if TOPIC_TO_STRING[topic_type] == "Image":
                # Sample images
                sampled_images = []
                for t in desired_timestamps:
                    closest_idx = np.abs(topic_timestamps - t).argmin()
                    closest_timestamp = topic_timestamps[closest_idx]
                    sampled_images.append(topic_group[str(closest_timestamp)][:])
                sampled_images = np.array(sampled_images)  # Tx3xHxW
                output_h5.create_dataset(f"observations/images/{topic}", data=sampled_images)

            elif TOPIC_TO_STRING[topic_type] == "PoseStamped":
                # Interpolate PoseStamped data
                pose_data = np.array([topic_group[str(ts)][:] for ts in topic_timestamps])
                positions = pose_data[:, :3]
                quaternions = pose_data[:, 3:]
                
                interp_position = interp1d(
                    topic_timestamps, positions, axis=0, kind="linear", fill_value="extrapolate"
                )
                interp_quaternions = interp1d(
                    topic_timestamps, quaternions, axis=0, kind="linear", fill_value="extrapolate"
                )

                sampled_positions = interp_position(desired_timestamps)
                sampled_quaternions = interp_quaternions(desired_timestamps)
                sampled_quaternions /= np.linalg.norm(
                    sampled_quaternions, axis=1, keepdims=True
                )  # Normalize quaternions
                
                if topic == "/franka/end_effector_pose":
                    qpos_franka = np.concatenate((sampled_positions, sampled_quaternions), axis=1)
                elif topic == "/franka/end_effector_pose_cmd":
                    actions_franka = np.concatenate((sampled_positions, sampled_quaternions), axis=1)


            elif TOPIC_TO_STRING[topic_type] == "Float32MultiArray":
                # Interpolate Float32MultiArray data
                array_data = np.array([topic_group[str(ts)][:] for ts in topic_timestamps])
                interp_array = interp1d(
                    topic_timestamps, array_data, axis=0, kind="linear", fill_value="extrapolate"
                )
                sampled_array = interp_array(desired_timestamps)
                
                qpos_hand = sampled_array
                actions_hand = sampled_array
            
        
            # create observations group
        if qpos_franka is not None:
            output_h5.create_dataset("observations/qpos_franka", data=qpos_franka)
        if qpos_hand is not None:
            output_h5.create_dataset("observations/qpos_hand", data=qpos_hand)
        if actions_franka is not None:
            output_h5.create_dataset("actions_franka", data=actions_franka)
        if actions_hand is not None:
            output_h5.create_dataset("actions_hand", data=actions_hand)



    print(f"Processed data saved to: {output_h5_path}")

def process_folder(input_folder, sampling_frequency, topic_types):
    """
    Process all HDF5 files in the given folder and save the processed files
    with a running index in a new folder named <input_folder>_processed.
    
    Parameters:
        input_folder (str): Path to the folder containing input HDF5 files.
        sampling_frequency (float): Sampling frequency in Hz.
        topic_types (dict): Dictionary mapping topics to their types.
    """
    # Get all HDF5 files in the folder
    h5_files = sorted(glob.glob(os.path.join(input_folder, "*.h5")))
    if not h5_files:
        print(f"No HDF5 files found in {input_folder}.")
        return

    # Create the output folder
    output_folder = os.path.dirname(input_folder) + "_processed"
    os.makedirs(output_folder, exist_ok=True)
    print(f"Output folder created: {output_folder}")

    # Process each file
    for idx, input_file in enumerate(h5_files):
        try:
            output_file = os.path.join(output_folder, f"{idx:04d}.h5")
            print(f"Processing file: {input_file}")
            sample_and_sync_h5(input_file, output_file, sampling_frequency, topic_types)
            print(f"Processed file saved as: {output_file}")
        except Exception as e:
            print(e)

    print(f"All files processed. Processed files are saved in {output_folder}.")

def main():
    parser = argparse.ArgumentParser(description="Process and synchronize HDF5 files.")
    parser.add_argument("input_folder", type=str, help="Path to the folder containing input HDF5 files.")
    parser.add_argument("--sampling_freq", type=float, default=100, help="Sampling frequency in Hz.")
    args = parser.parse_args()

    # Process all files in the folder
    process_folder(args.input_folder, args.sampling_freq, TOPICS_TYPES)

if __name__ == "__main__":
    main()
