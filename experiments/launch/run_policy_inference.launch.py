from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# HAND_NAME = "mimic" # "franka"
HAND_NAME = "orca" # "mimic"
TASK_NAME = ""
TASK_DESCRIPTION="red"

def launch_setup(context, *args, **kwargs):
    # Get the launch arguments
    policy_ckpt_path = LaunchConfiguration('policy_ckpt_path')
    online = LaunchConfiguration('online').perform(context)
    
    # The inference config now also carries camera_topics/camera_names and the
    # observation/action <-> ros topic mapping (single- or dual-arm).
    inference_cfg_file = LaunchConfiguration('inference_cfg_file').perform(context)
    if inference_cfg_file == '':
        inference_cfg_file = os.path.join(
                        get_package_share_directory("experiments"),
                        "cfgs",
                        # f"inference_config_scissors_task.yaml"
                        # "inference_config_cubestacking_task.yaml"
                        "inference_config_pour_task.yaml"
        )

    # Choose executable and prefix based on online argument
    if online.lower() == 'true':
        executable = 'online_rl_model_inference_node.py'
        prefix = 'gnome-terminal  --'
    else:
        executable = 'model_inference_node.py'
        prefix = ''
    
    policy_node = Node(
        package='experiments',
        executable=executable,
        name="model_inference",
        output='screen',
        prefix=prefix,
        parameters=[{
            "policy_ckpt_path": policy_ckpt_path,
            "inference_cfg_file": inference_cfg_file,
            "logfile_root": "inference_logs",
            "hand_name": HAND_NAME,
            "task_name": TASK_NAME,
            "task_description":TASK_DESCRIPTION
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
    
    online_arg = DeclareLaunchArgument(
        'online',
        default_value='false',
        description='Whether to run online RL inference (false) or policy inference (false)'
    )

    inference_cfg_arg = DeclareLaunchArgument(
        'inference_cfg_file',
        default_value='',
        description='Inference config path. Empty falls back to the task config selected in this file.'
    )

    # Return the LaunchDescription with all the launch arguments and nodes
    return LaunchDescription([
        policy_ckpt_arg,
        online_arg,
        inference_cfg_arg,
        OpaqueFunction(function=launch_setup)
    ])
