from launch import LaunchDescription
from launch.substitutions import Command
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
            "robot_description": _orcahand_v2_robot_description(hand_chirality),
        }],
    )
    return [hand_controller_node, retargeter_node]


def generate_launch_description():
    OAKD_CAMERA_CFG_NAME = "oakd_scissors_task" # file in cfgs/cams, or None
    ZED_CAMERA_CFG_NAME = None # "zed_color_arena" # or None
    # the ORCA v2 hand is right-handed only (src/hand_control/orca/configs/orcahand_v2/right)
    LEFT_SIDE = False
    RIGHT_SIDE = True
    DATASET_NAME = "demo"
    DATA_RECORDING_CONFIG_PATH = os.path.join(
        get_package_share_directory("experiments"), "cfgs", "logger", "datacollection_default.yaml"
    )

    camera_nodes = []

    if OAKD_CAMERA_CFG_NAME is not None:
        camera_nodes.append(Node(
            package="ingress",
            executable="oakd_node.py",
            name="oakd_node",
            output="log",
            parameters=[{
                "config_file": os.path.join(get_package_share_directory("experiments"),
                                            "cfgs", "cams", f"{OAKD_CAMERA_CFG_NAME}.yaml")
            }],
        ))

    if ZED_CAMERA_CFG_NAME is not None:
        camera_nodes.append(Node(
            package="ingress",
            executable="zed_node.py",
            name="zed_node",
            output="screen",
            parameters=[{
                "config_file": os.path.join(get_package_share_directory("experiments"),
                                            "cfgs", "cams", f"{ZED_CAMERA_CFG_NAME}.yaml")
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
            {"rokoko_tracker/use_coil": True}
        ],
    )

    logger_node = Node(
        package="logger",
        executable="logger_node.py",
        name="logger_node",
        output="screen",
        parameters=[
            {
            "topics_and_types_path": DATA_RECORDING_CONFIG_PATH,
            "storage_base_path": "tmp",
            "data_name": DATASET_NAME
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

    return LaunchDescription(
        camera_nodes
        + (spawn_hand_nodes("right") if RIGHT_SIDE else [])
        + (spawn_hand_nodes("left") if LEFT_SIDE else [])
        + [
            rokoko_node,
            logger_node,
            arm_teleop_node,
            keyboard_ctrl
        ]
    )
