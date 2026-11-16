#!/bin/env python3

import rclpy
from rclpy.node import Node
import rosbag2_py
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from faive_system.src.logger.bag2h5_converter import convert_to_h5
from rclpy.serialization import serialize_message
from pathlib import Path
from datetime import datetime
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
import importlib
import yaml

def _import_msg_class(type_name: str):
    """
    Turn 'geometry_msgs.msg.PoseStamped' into the actual Python class.
    """
    try:
        module_name, class_name = type_name.rsplit(".", 1)
    except ValueError as exc:
        raise ValueError(f"Malformed message type '{type_name}'") from exc

    module = importlib.import_module(module_name)
    if not hasattr(module, class_name):
        raise ValueError(f"Module '{module_name}' does not have class '{class_name}'")
    return getattr(module, class_name)


class DemoLogger(Node):
    def __init__(self):
        super().__init__('demo_logger')
        
        self.declare_parameter("topics_and_types_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("storage_base_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("data_name", rclpy.Parameter.Type.STRING)

        self.topics_and_types_path = Path(self.get_parameter("topics_and_types_path").value)
        self.base_path = Path(self.get_parameter("storage_base_path").value)
        self.data_name = self.get_parameter("data_name").value

        if self.data_name is None: # use current time as default name
            self.data_name = datetime.now().strftime("unnamed_%Y%m%d_%H%M%S")

        if not self.topics_and_types_path.exists():
            self.get_logger().error("No topics and types path provided. Please set 'topics_and_types_path' parameter.")
            raise ValueError("No topics and types path provided.")
        self.topics_and_types = yaml.safe_load(self.topics_and_types_path.read_text())['topics_and_types']
        assert isinstance(self.topics_and_types, dict), "topics_and_types should be a dictionary mapping topic names to types."
        # Convert topic names to actual message classes
        self.topics_and_types = {
            topic: (type_name,_import_msg_class(type_name)) for topic, type_name in self.topics_and_types.items()
        }

        self.writer = None
        self.file_operation_lock = threading.Lock() # prevents "start_recording" and "stop_recording" from being called at the same time
        self.logger_state = "idle" # "recording" or "processing"
        
        # Ensure the base directory exists
        if not self.base_path.exists():
            self.base_path.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Base directory '{self.base_path}' created.")
        else:
            self.get_logger().info(f"Using existing base directory '{self.base_path}'.")

        if self.data_name is None:
            self.data_name = input("Enter the task name (this will be the output folder): ")
        
        self.task_folder = self.base_path / self.data_name
        self.task_folder_bag = self.task_folder / "rosbag2"
        # Create the task directory if it doesn't exist
        if not self.task_folder.exists():
            self.task_folder.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Directory '{self.task_folder}' created.")
        else:
            self.get_logger().info(f"Directory '{self.task_folder}' already exists.")


        self.move2init_service = self.create_service(
            Trigger, 
            f"/demo_logger/start_recording",   # service name
            self.start_recording_cb
        )
        self.pause_service = self.create_service(
            Trigger, 
            f"/demo_logger/stop_recording",   # service name
            self.stop_recording_cb
        )
        self.resume_service = self.create_service(
            Trigger,
            f"/demo_logger/save_recording",  # service name
            self.save_recording_cb
        )

    def start_recording_cb(self, request, response): 
        if self.logger_state == "recording":
            self.get_logger().warn("Already recording. Please stop the current recording first.")
            response.success = False
            return response
        if self.logger_state == "processing":
            self.get_logger().warn("Currently processing. Please stop the current recording first.")
            response.success = False
            return response
        
        self.delete_recording(self.task_folder_bag)
        response.success = True
        # folder inside the task folder
        self.start_recording(self.task_folder_bag)
        return response

    def stop_recording_cb(self, request, response): 
        if self.logger_state == "recording":
            self.stop_recording()
            response.success = True
        else:
            self.get_logger().warn("No active recording to stop.")
            response.success = False
            return response
        return response

    def save_recording_cb(self, request, response): 
        if self.logger_state == "recording":
            self.get_logger().warn("Stop the recording before saving.")
            response.success = False
            return response
        
        if not self.task_folder_bag.exists():
            self.get_logger().warn(f"No recording found in '{self.task_folder_bag}'.")
            response.success = False
            return response
        
        self.logger_state = "processing"

        # Generate a unique name based on the current date and time
        h5_filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".h5"
        h5_filepath = self.task_folder / h5_filename
        self.get_logger().info(f"Recording saved as '{h5_filename}' in folder '{self.task_folder}'")
        
        # Convert to HDF5 and delete the bag file afterward
        with self.file_operation_lock:
            convert_to_h5(input_bag_path=str(self.task_folder_bag), output_h5_path=str(h5_filepath))
        self.get_logger().info(f"Recording converted to HDF5: {h5_filepath}")
        self.delete_recording(self.task_folder_bag)
        response.success = True
        
        self.logger_state = "idle"
        return response


    def start_recording(self, task_folder_bag):
        if not self.topics_and_types:
            self.get_logger().error("No topics specified to record. Check the configuration file.")
            return
        
        with self.file_operation_lock:
            self.logger_state = "recording"
            # Set up rosbag2 writer
            storage_options = rosbag2_py.StorageOptions(uri=str(task_folder_bag), storage_id="sqlite3")
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )

            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(storage_options, converter_options)

        # Add topics to be recorded
        self.subscribers = []
        for topic_id, (topic_name, (topic_type_name, topic_type)) in enumerate(self.topics_and_types.items(), 1):
            
            # Create topic metadata for rosbag
            metadata = dict(
                name=topic_name,
                type=topic_type_name.replace(".msg.", "/msg/"),
                serialization_format="cdr"
            )
            try:
                topic_metadata = rosbag2_py.TopicMetadata(id=topic_id, **metadata)
            except TypeError:  # Humble's constructor has no id argument.
                topic_metadata = rosbag2_py.TopicMetadata(**metadata)
            self.writer.create_topic(topic_metadata)

            # Subscribe and add callback to write messages
            qos_profile = QoSProfile(depth=10)
            if topic_type is Image:
                qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
            elif any(key in topic_name for key in ("intrinsics", "extrinsics", "projection")):
                qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
            
            self.subscribers.append(
                self.create_subscription(
                    topic_type,
                    topic_name,
                    lambda msg, topic_name=topic_name: self.writer.write(
                        topic_name, serialize_message(msg), self.get_clock().now().nanoseconds
                    ),
                    qos_profile,
                )
            )

        self.get_logger().info(f"Recording started for task '{self.data_name}' with topics: {self.topics_and_types.keys()} at {task_folder_bag}")

    def stop_recording(self):
        if self.writer:
            self.writer = None
            for sub in self.subscribers:
                self.destroy_subscription(sub)
            self.get_logger().info("Stopped recording.")
        else:
            self.get_logger().warn("No active recording to stop.")
        self.logger_state = "idle"

    def delete_recording(self, task_folder_bag):
        # Delete all files in the task folder after converting
        with self.file_operation_lock:
            if self.logger_state == "recording":
                return False

            if task_folder_bag.exists():
                for file in task_folder_bag.iterdir():
                    file.unlink()
            # delete the folder
                task_folder_bag.rmdir()
            self.get_logger().info(f"Recording deleted from folder: {task_folder_bag}")
        return True



def main(args=None):
    # Initialize ROS and create DemoLogger instance
    rclpy.init(args=args)
    demo_logger = DemoLogger()
    
    rclpy.spin(demo_logger)

    demo_logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
