from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments with default values
    policy_ckpt_arg = DeclareLaunchArgument(
        'policy_ckpt_path',
        default_value='',
        description='The ckpt path of the model to load. There should be `config.yaml` in its directory or parent directory'
    )

    # Use LaunchConfiguration to capture the values passed via command line
    policy_ckpt_path = LaunchConfiguration('policy_ckpt_path')

    # Define the node with parameters from the launch arguments
    policy_node = Node(
        package='experiments',
        executable='model_inference_node.py',
        name="model_inference",
        output='screen',
        parameters=[{
            'camera_topics': [
                "/oakd_front_view/color", "/oakd_side_view/color"
            ],
            'camera_names': [
                "oakd_front_view_images", "oakd_side_view_images"
            ],
            "policy_ckpt_path": policy_ckpt_path
         }]
    )

    # Return the LaunchDescription with all the launch arguments and nodes
    return LaunchDescription([
        policy_ckpt_arg,
        policy_node
])