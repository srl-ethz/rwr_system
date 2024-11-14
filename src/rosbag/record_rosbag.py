import subprocess
import argparse
from datetime import datetime
import os

def record_ros2_bag(file_location, topics=None):
    # Format the filename with the current date and time
    formatted_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_name = f"recording_{formatted_time}"
    full_path = os.path.join(file_location, bag_name)

    # Create the ros2 bag record command
    command = ["ros2", "bag", "record", "-o", full_path]

    if topics:
        command.extend(topics)  # Add specified topics
    else:
        command.append("-a")  # Record all topics

    print(f"Recording ROS 2 bag to {full_path} with command: {' '.join(command)}")
    
    # Execute the ros2 bag record command
    subprocess.run(command)

def main():
    parser = argparse.ArgumentParser(description="Record a ROS 2 bag with optional topic selection.")
    parser.add_argument("--file_location", type=str, required=True, help="Directory to save the rosbag.")
    parser.add_argument("--topics", type=str, nargs="*", help="List of topics to record. If not provided, records all topics.")

    args = parser.parse_args()
    record_ros2_bag(args.file_location, args.topics)

if __name__ == "__main__":
    main()
