from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# Teleoperate the ORCA v2 hand (and the Franka arm) with the Rokoko glove, without recording.
# Use it to test the teleop before recording demonstrations with record_demonstration.launch.py.
#
# ros2 launch experiments teleop_rokoko.launch.py
# ros2 launch experiments teleop_rokoko.launch.py enable_hand_controller:=false  # hand controller runs elsewhere


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


def spawn_hand_nodes(hand_chirality, enable_hand_controller=True):
    """
    Spawn the ORCA v2 hand controller and the retargeter for one side.
    arg: enable_hand_controller: skip the hand controller when it is already running
        (e.g. started by controllers_and_cameras.launch.py).
    """
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
    return ([hand_controller_node] if enable_hand_controller else []) + [retargeter_node]



def launch_setup(context, *args, **kwargs):
    # the ORCA v2 hand is right-handed only (src/hand_control/orca/configs/orcahand_v2/right)
    LEFT_SIDE = False
    RIGHT_SIDE = True
    TELEOP_ARM = True

    enable_hand_controller = LaunchConfiguration("enable_hand_controller").perform(context).lower() == "true"
    enable_keyboard_runner = LaunchConfiguration("enable_keyboard_runner").perform(context).lower() == "true"

    rokoko_ingress = Node(
                package="ingress",
                executable="rokoko_node.py",
                name="rokoko_node",
                output="log",
                parameters=[
                    {"rokoko_tracker/ip": "0.0.0.0"},
                    {"rokoko_tracker/port": 14043},
                    {"rokoko_tracker/use_coil": True},
                    {"rokoko_tracker/enable_left": LEFT_SIDE},
                    {"rokoko_tracker/enable_right": RIGHT_SIDE},
                ],
            )

    arm_teleop_node = Node(
        package="experiments",
        executable="run_teleop_rokoko.py",
        name="arm_teleop_node",
        parameters=[
            {"enable_left": LEFT_SIDE},
            {"enable_right": RIGHT_SIDE},
            {"interactive": False},
        ],
        output="screen",
    )

    keyboard_ctrl = Node(
            package='experiments',
            executable='keyboard_runner.py',
            name='keyboard_runner_node',
            # Arguments are passed as a list of strings
            arguments=[
                '--teleop_pause_and_resume',
                '--teleop_back_to_init',
                '--retargeter',
            ] \
            + [arg for enabled, arg in zip([LEFT_SIDE, RIGHT_SIDE], ['--left_hand', '--right_hand']) if enabled],
            prefix='gnome-terminal  --' ,
            output='screen'
        )


    return (
        [rokoko_ingress]
        + ([keyboard_ctrl] if enable_keyboard_runner else [])
        + (spawn_hand_nodes("right", enable_hand_controller) if RIGHT_SIDE else [])
        + (spawn_hand_nodes("left", enable_hand_controller) if LEFT_SIDE else [])
        + ([arm_teleop_node] if TELEOP_ARM else [])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_hand_controller", default_value="true",
            description="false when the hand controller is already running elsewhere.",
        ),
        DeclareLaunchArgument(
            "enable_keyboard_runner", default_value="true",
            description="false to run without the keyboard_runner terminal.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
