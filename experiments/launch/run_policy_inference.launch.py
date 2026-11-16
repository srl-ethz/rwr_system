from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# Run an imitation learning policy with the camera and robot feedback streams
# that match the training data.
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
            "inference_config_cube_sorting.yaml",
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
            "preview": LaunchConfiguration("preview").perform(context) == "true",
            "allow_arm_init": LaunchConfiguration("allow_arm_init").perform(context) == "true",
            "arm_init_pose": ParameterValue(LaunchConfiguration("arm_init_pose"), value_type=str),
         }]
    )

    nodes = []
    oakd_config = LaunchConfiguration("oakd_config").perform(context)
    if oakd_config not in ("", "none"):
        nodes.append(Node(
            package="ingress",
            executable="oakd_node.py",
            name="oakd_node",
            output="screen",
            parameters=[{"config_file": oakd_config}],
        ))

    if LaunchConfiguration("enable_hand_controller").perform(context) == "true":
        nodes.append(Node(
            package="hand_control",
            executable="hand_control_node.py",
            name="hand_controller_node",
            namespace="right",
            output="screen",
            parameters=[{
                "hand_controller/hand_name": "orca_v2",
                "hand_controller/hand_chirality": "right",
                "hand_controller/hand_cfg_name": "orcahand_v2",
                "hand_controller/calibrate": False,
            }],
        ))

    return [*nodes, policy_node]

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
        description='Inference config path. Empty uses the RWR26 cube sorting config.'
    )

    oakd_config_arg = DeclareLaunchArgument(
        "oakd_config",
        default_value=os.path.join(
            get_package_share_directory("experiments"), "cfgs", "cams", "cube_sorting_task.yaml"
        ),
        description="OAK-D camera config path. Set to 'none' if the cameras are already running.",
    )

    enable_hand_controller_arg = DeclareLaunchArgument(
        "enable_hand_controller",
        default_value="true",
        choices=["true", "false"],
        description="Start the right ORCA v2 controller. The hand moves to neutral on startup.",
    )

    preview_arg = DeclareLaunchArgument(
        "preview",
        default_value="false",
        choices=["true", "false"],
        description="Keep policy outputs on /policy_inference only. The ORCA driver still initializes.",
    )
    allow_arm_init_arg = DeclareLaunchArgument(
        "allow_arm_init",
        default_value="false",
        choices=["true", "false"],
        description="Accept an explicit init_arm command in live mode.",
    )
    arm_init_pose_arg = DeclareLaunchArgument(
        "arm_init_pose",
        default_value="",
        description="Operator-confirmed [x, y, z, qx, qy, qz, qw] in the configured arm command frame.",
    )

    # Return the LaunchDescription with all the launch arguments and nodes
    return LaunchDescription([
        policy_ckpt_arg,
        inference_cfg_arg,
        oakd_config_arg,
        enable_hand_controller_arg,
        preview_arg,
        allow_arm_init_arg,
        arm_init_pose_arg,
        OpaqueFunction(function=launch_setup)
    ])
