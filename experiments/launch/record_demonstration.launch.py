from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# Record imitation learning demonstrations: cameras, ORCA v2 hand + retargeter, Rokoko glove,
# Franka arm teleop, logger, and the keyboard_runner terminal to start/stop the recordings.
# Select the cameras and the dataset name in generate_launch_description().


def _orcahand_v2_robot_description(hand_chirality):
    """Render the orcahand_v2 URDF in-memory from the xacro."""
    xacro_path = os.path.join(
        get_package_share_directory("viz"),
        "models", "orcahand_v2", "orcahand_v2.urdf.xacro",
    )
    return ParameterValue(
        Command(['xacro ', xacro_path, ' hand_chirality:=', hand_chirality]),
        value_type=str,
    )


def spawn_hand_nodes(hand_chirality):
    """Spawn the ORCA v2 hand controller and the retargeter for one side."""
    hand_name = "orcahand_v2"
    hand_scheme_filepath = os.path.join(get_package_share_directory("viz"), "models", hand_name, f"scheme_{hand_name}.yaml")
    mano_adjustments_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "retargeter", f"mano_adjustment_{hand_name}.yaml")
    retargeter_cfg_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "retargeter", f"retargeter_cfgs_{hand_name}.yaml")

    # HAND CONTROLLER NODE
    hand_controller_node = Node(
        package="hand_control",
        executable="hand_control_node.py",
        name="hand_controller_node",
        output="screen",
        namespace=hand_chirality,
        parameters=[
            {
                "hand_controller/hand_name": "orca_v2",
                "hand_controller/hand_chirality": hand_chirality,
                "hand_controller/hand_cfg_name": hand_name,
                "hand_controller/calibrate": False,
            }
        ],
    )

    retargeter_node = Node(
        package="retargeter",
        executable="retargeter_node.py",
        name="retargeter_node",
        output="screen",
        namespace=hand_chirality,
        parameters=[{
            "retarget/hand_scheme": hand_scheme_filepath,
            "retarget/hand_chirality": hand_chirality,
            "retarget/mano_adjustments": mano_adjustments_filepath,
            "retarget/retargeter_cfg": retargeter_cfg_filepath,
            "enable_at_start": False,
            "robot_description": _orcahand_v2_robot_description(hand_chirality),
        }],
    )
    return [hand_controller_node, retargeter_node]


def launch_setup(context, *args, **kwargs):
    oakd_config = LaunchConfiguration("oakd_config").perform(context)
    zed_config = LaunchConfiguration("zed_config").perform(context)
    enable_hand_controller = LaunchConfiguration("enable_hand_controller").perform(context).lower() == "true"
    enable_keyboard_runner = LaunchConfiguration("enable_keyboard_runner").perform(context).lower() == "true"
    # the ORCA v2 hand is right-handed only (src/hand_control/orca/configs/orcahand_v2/right)
    LEFT_SIDE = False
    RIGHT_SIDE = True

    camera_nodes = []

    if oakd_config:
        camera_nodes.append(Node(
            package="ingress",
            executable="oakd_node.py",
            name="oakd_node",
            output="log",
            parameters=[{
                "config_file": oakd_config,
            }],
        ))

    if zed_config:
        camera_nodes.append(Node(
            package="ingress",
            executable="zed_node.py",
            name="zed_node",
            output="screen",
            parameters=[{
                "config_file": zed_config,
            }],

        ))

    rokoko_node = Node(
        package="ingress",
        executable="rokoko_node.py",
        name="rokoko_node",
        output="screen",
        parameters=[
            {"rokoko_tracker/ip": "0.0.0.0"},
            {"rokoko_tracker/port": 14043},
            {"rokoko_tracker/use_coil": True},
            {"rokoko_tracker/enable_left": LEFT_SIDE},
            {"rokoko_tracker/enable_right": RIGHT_SIDE},
        ],
    )

    logger_node = Node(
        package="logger",
        executable="logger_node.py",
        name="logger_node",
        output="screen",
        parameters=[
            {
            "topics_and_types_path": LaunchConfiguration("logger_config"),
            "storage_base_path": LaunchConfiguration("storage_base_path"),
            "data_name": LaunchConfiguration("dataset_name")
            }
        ],
    )
    # move the arm
    arm_teleop_node = Node(
        package="experiments",
        executable="run_teleop_rokoko.py",
        name="arm_teleop_node",
        output="log",
        parameters=[
            {"enable_left": LEFT_SIDE},
            {"enable_right": RIGHT_SIDE},
            {"interactive": False},
        ],
    )

    keyboard_ctrl = Node(
            package='experiments',
            executable='keyboard_runner.py',
            name='keyboard_runner_node',
            # Arguments are passed as a list of strings
            arguments=[
                '--retargeter',
                '--teleop_pause_and_resume',
                '--teleop_back_to_init',
                '--demo_logger'
            ] + (['--right_hand'] if RIGHT_SIDE else []) + (['--left_hand'] if LEFT_SIDE else []),
            prefix='gnome-terminal  --' ,
            output='screen'
        )

    hand_nodes = spawn_hand_nodes("right") if RIGHT_SIDE else []
    if not enable_hand_controller:
        hand_nodes = hand_nodes[1:]
    return camera_nodes + hand_nodes + [rokoko_node, logger_node, arm_teleop_node] + (
        [keyboard_ctrl] if enable_keyboard_runner else []
    )


def generate_launch_description():
    share = get_package_share_directory("experiments")
    defaults = {
        "oakd_config": os.path.join(share, "cfgs", "cams", "oakd_scissors_task.yaml"),
        "zed_config": "",
        "logger_config": os.path.join(share, "cfgs", "logger", "datacollection_default.yaml"),
        "storage_base_path": "tmp",
        "dataset_name": "demo",
        "enable_hand_controller": "true",
        "enable_keyboard_runner": "true",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
