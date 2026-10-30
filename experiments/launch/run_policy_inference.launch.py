from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# Run an imitation learning policy. Start the cameras and the hand first with
# controllers_and_cameras.launch.py (and the Franka arm if the policy uses it).
#
# ros2 launch experiments run_policy_inference.launch.py policy_ckpt_path:=/path/to/policy.ckpt

def launch_setup(context, *args, **kwargs):
    # Get the launch arguments
    policy_ckpt_path = LaunchConfiguration('policy_ckpt_path')

    # The inference config carries camera_topics/camera_names and the
    # observation/action <-> ros topic mapping (single- or dual-arm).
    inference_cfg_file = LaunchConfiguration('inference_cfg_file').perform(context)
    if inference_cfg_file == '':
        inference_cfg_file = os.path.join(
                        get_package_share_directory("experiments"),
                        "cfgs",
                        "inference_config_scissors_task.yaml"
                        # "inference_config_cubestacking_task.yaml"
        )

    policy_node = Node(
        package='experiments',
        executable='model_inference_node.py',
        name="model_inference",
        output='screen',
        parameters=[{
            "policy_ckpt_path": policy_ckpt_path,
            "inference_cfg_file": inference_cfg_file,
            "logfile_root": "inference_logs",
         }]
    )

    return [policy_node]

def generate_launch_description():
    # Declare launch arguments with default values
    policy_ckpt_arg = DeclareLaunchArgument(
        'policy_ckpt_path',
        default_value='',
        description='The ckpt path of the model to load. There should be `config.yaml` in its directory or parent directory'
    )

    inference_cfg_arg = DeclareLaunchArgument(
        'inference_cfg_file',
        default_value='',
        description='Inference config path. Empty falls back to the task config selected in this file.'
    )

    # Return the LaunchDescription with all the launch arguments and nodes
    return LaunchDescription([
        policy_ckpt_arg,
        inference_cfg_arg,
        OpaqueFunction(function=launch_setup)
    ])
