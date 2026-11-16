#!/bin/env python3

import h5py
import numpy as np
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from ingress.msg import CameraIntrinsics
from cv_bridge import CvBridge  
bridge = CvBridge()

def convert_to_h5(input_bag_path: str, output_h5_path: str):
    """Convert the rosbag to HDF5 format."""
    bag_folder = input_bag_path
    h5_file_path = output_h5_path
    print(f"Converting rosbag from '{bag_folder}' to HDF5 file '{h5_file_path}'")

    # Open HDF5 file for writing
    with h5py.File(h5_file_path, 'w') as h5_file:
        # Set up rosbag2 reader
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_folder), storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader.open(storage_options, converter_options)

        # Get topic information and verify types
        topic_types = reader.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in topic_types}
        print(f"Available topics: {topic_type_map}")

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()

            # Get the message type based on the topic
            msg_type_str = topic_type_map[topic_name]
            # Accept standard ROS type names and legacy bags containing only class names.
            msg_class = {cls.__name__: cls for cls in (
                Image, PoseStamped, Float32MultiArray, String, CameraIntrinsics
            )}.get(msg_type_str.rsplit("/", 1)[-1])

            if msg_class is None:
                print(f"Unsupported message type for topic '{topic_name}': {msg_type_str}")
                continue

            # Deserialize message
            msg = deserialize_message(data, msg_class)

            # Create HDF5 group if it does not exist
            if topic_name not in h5_file:
                h5_file.create_group(topic_name)
                print(f"Creating group for topic '{topic_name}'")

            # Save message data to HDF5 based on the message type
            if isinstance(msg, CameraIntrinsics):
                matrix = np.array([[msg.fx, 0, msg.cx], [0, msg.fy, msg.cy], [0, 0, 1]])
                h5_file[topic_name].create_dataset(f"{timestamp}", data=matrix)
                h5_file[topic_name].attrs["distortion"] = msg.disto

            elif isinstance(msg, Float32MultiArray):
                # For Float32MultiArray, save the data array
                h5_file[topic_name].create_dataset(f"{timestamp}", data=msg.data)

            elif isinstance(msg, PoseStamped):
                # For PoseStamped, save position and orientation data
                pose_data = [
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
                ]
                h5_file[topic_name].create_dataset(f"{timestamp}", data=pose_data)

            elif isinstance(msg, Image):
                # Check if the image is color or depth
                if msg.encoding in ['rgb8', 'bgr8']:
                    # For color images, save the raw image data array
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                    h5_file[topic_name].create_dataset(f"{timestamp}", data=cv_image)
                elif msg.encoding == 'mono16':
                    # For depth images, save the depth data
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    h5_file[topic_name].create_dataset(f"{timestamp}", data=cv_image)
                else:
                    print(f"Unsupported image encoding for topic '{topic_name}': {msg.encoding}")
            
            elif isinstance(msg, String):
                # For String, save the string data
                h5_file[topic_name].create_dataset("description", data=msg.data)
                

            else:
                print(f"Message type for topic '{topic_name}' not supported for conversion.")

    print(f"Conversion complete. HDF5 file saved to '{h5_file_path}'")
