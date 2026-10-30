from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def _orcahand_v1b_robot_description(hand_chirality):
    """Render the orcahand_v1b URDF in-memory from the mirror-aware xacro."""
    xacro_path = os.path.join(
        get_package_share_directory("viz"),
        "models", "orcahand_v1b", "orcahand_v1b.urdf.xacro",
    )
    return ParameterValue(
        Command(['xacro ', xacro_path, ' hand_chirality:=', hand_chirality]),
        value_type=str,
    )


def spawn_hand_nodes(
    hand_chirality,
    hand_name,
    cmd_topic_suffix="",
    enable_hand_controller=True,
    retargeter_type="continuous",
    calibration_file="",
    debounce_samples=3,
    hysteresis_ratio=0.05,
):
    """
    arg: hand_name: "orcahand_v1b" # "orca_v1" "mimic", None
    arg: cmd_topic_suffix: when non-empty, the retargeter publishes to
        `/<chirality>/hand/policy_output<suffix>` instead of the real command topic, so the
        teleop stack no longer drives the hand directly (used by the HIL launch, where the
        policy node forwards the operator's command only while it has taken over).
    arg: enable_hand_controller: skip the hand controller when it is already running
        (e.g. started by controllers_and_cameras.launch.py).
    arg: retargeter_type: ``continuous`` for the optimizer-based retargeter or
        ``discrete`` for calibrated nearest-neighbour posture transitions.
    """
    if hand_name is None:
        return []

    robot_description = None  # only set for hands that render xacro at launch time

    if "mimic" in hand_name:
        urdf_filepath = os.path.join(get_package_share_directory('viz'), "models/mimic/converted.urdf")
        hand_scheme_filepath = os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            hand_name,
                            "scheme.yaml"
            )
        hand_family = "mimic"

    elif "orca" in hand_name:
        if hand_name == "orcahand_v1b":
            # orcahand_v1b: chirality-aware URDF (rendered at launch from the
            # mirror-aware xacro) and a left-specific scheme + mano_adjustment.
            urdf_filepath = None
            robot_description = _orcahand_v1b_robot_description(hand_chirality)
            hand_scheme_filepath = os.path.join(
                get_package_share_directory("viz"),
                f"models/{hand_name}/scheme_{hand_name}.yaml",
            )
        else:
            urdf_filepath = os.path.join(get_package_share_directory('viz'), f"models/{hand_name}/urdf/{hand_name}.urdf")
            hand_scheme_filepath = os.path.join(get_package_share_directory("viz"), f"models/{hand_name}/scheme_{hand_name}.yaml")
        hand_family = "orca"


    mano_adjust_suffix = "_left" if hand_name == "orcahand_v1b" and hand_chirality == "left" else ""
    mano_adjustments_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "retargeter", f"mano_adjustment_{hand_name}{mano_adjust_suffix}.yaml")
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
                "hand_controller/hand_name": hand_family,
                "hand_controller/hand_chirality": hand_chirality,
                "hand_controller/hand_cfg_name": hand_name,
                "hand_controller/calibrate": False,
            }
        ],
    )

    if retargeter_type == "continuous":
        retargeter_executable = "retargeter_node.py"
        retargeter_params = {
            "retarget/hand_scheme": hand_scheme_filepath,
            "retarget/hand_chirality": hand_chirality,
            "retarget/mano_adjustments": mano_adjustments_filepath,
            "retarget/retargeter_cfg": retargeter_cfg_filepath,
        }
        if robot_description is not None:
            retargeter_params["robot_description"] = robot_description
        else:
            retargeter_params["retarget/urdf_filepath"] = urdf_filepath
    elif retargeter_type == "discrete":
        retargeter_executable = "discretized_retargeter_node.py"
        retargeter_params = {
            "retarget/hand_chirality": hand_chirality,
            "retarget/calibration_file": calibration_file,
            "retarget/debounce_samples": debounce_samples,
            "retarget/hysteresis_ratio": hysteresis_ratio,
        }
    else:
        raise ValueError(
            f"Unknown retargeter_type {retargeter_type!r}; expected 'continuous' or 'discrete'"
        )

    hand_cmd_topic = f"/{hand_chirality}/hand/policy_output"
    retargeter_node = Node(
        package="retargeter",
        executable=retargeter_executable,
        name="retargeter_node",
        output="screen",
        namespace=hand_chirality,
        parameters=[retargeter_params],
        remappings=([(hand_cmd_topic, hand_cmd_topic + cmd_topic_suffix)]
                    if cmd_topic_suffix else []),
    )
    return ([hand_controller_node] if enable_hand_controller else []) + [retargeter_node]



def launch_setup(context, *args, **kwargs):
    hand_name= "orcahand_v1b" # "orca_v1" "mimic", None
    LEFT_SIDE = True
    RIGHT_SIDE = True
    TELEOP_ARM = True

    # Launch arguments (defaults reproduce the standalone teleop session exactly)
    cmd_topic_suffix = LaunchConfiguration("cmd_topic_suffix").perform(context)
    interactive = LaunchConfiguration("interactive").perform(context).lower() == "true"
    enable_hand_controller = LaunchConfiguration("enable_hand_controller").perform(context).lower() == "true"
    enable_keyboard_runner = LaunchConfiguration("enable_keyboard_runner").perform(context).lower() == "true"
    retargeter_type = LaunchConfiguration("retargeter_type").perform(context).lower()
    left_calibration_file = LaunchConfiguration("left_retargeter_calibration_file").perform(context)
    right_calibration_file = LaunchConfiguration("right_retargeter_calibration_file").perform(context)
    debounce_samples = int(LaunchConfiguration("discrete_debounce_samples").perform(context))
    hysteresis_ratio = float(LaunchConfiguration("discrete_hysteresis_ratio").perform(context))

    rokoko_ingress = Node(
                package="ingress",
                executable="rokoko_node.py",
                name="rokoko_node",
                output="log",
                parameters=[
                    {"rokoko_tracker/ip": "0.0.0.0"},
                    {"rokoko_tracker/port": 14043},
                    {"rokoko_tracker/use_coil": True}
                ],
            )

    arm_cmd_topics = [f"/{chirality}/arm/end_effector_pose_cmd"
                      for chirality, enabled in zip(["left", "right"], [LEFT_SIDE, RIGHT_SIDE])
                      if enabled]
    arm_teleop_node = Node(
        package="experiments",
        executable="run_teleop_rokoko.py",
        name="arm_teleop_node",
        parameters=[
            {"enable_left": LEFT_SIDE},
            {"enable_right": RIGHT_SIDE},
            {"interactive": interactive},
        ],
        output="screen",
        remappings=([(topic, topic + cmd_topic_suffix) for topic in arm_cmd_topics]
                    if cmd_topic_suffix else []),
    )

    keyboard_ctrl = Node(
            package='experiments',
            executable='keyboard_runner.py',
            name='keyboard_runner_node',
            # Arguments are passed as a list of strings
            arguments=[
                '--teleop_pause_and_resume',
                '--teleop_back_to_init',
            ] \
            + (['--retargeter'] if hand_name is not None else []) \
            + [arg for enabled, arg in zip([LEFT_SIDE, RIGHT_SIDE], ['--left_hand', '--right_hand']) if enabled],
            prefix='gnome-terminal  --' ,
            output='screen'
        )


    return (
        [rokoko_ingress]
        + ([keyboard_ctrl] if enable_keyboard_runner else [])
        + (spawn_hand_nodes(
            "right", hand_name, cmd_topic_suffix, enable_hand_controller,
            retargeter_type, right_calibration_file, debounce_samples, hysteresis_ratio,
        ) if RIGHT_SIDE else [])
        + (spawn_hand_nodes(
            "left", hand_name, cmd_topic_suffix, enable_hand_controller,
            retargeter_type, left_calibration_file, debounce_samples, hysteresis_ratio,
        ) if LEFT_SIDE else [])
        + ([arm_teleop_node] if TELEOP_ARM else [])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "cmd_topic_suffix", default_value="",
            description="Suffix appended to the teleop command topics. Non-empty detaches the "
                        "teleop stack from the robot so another node can gate it (HIL).",
        ),
        DeclareLaunchArgument(
            "interactive", default_value="true",
            description="false: run_teleop_rokoko.py skips its stdin prompts and is driven "
                        "only through its pause/resume services.",
        ),
        DeclareLaunchArgument(
            "enable_hand_controller", default_value="true",
            description="false when the hand controllers are already running elsewhere.",
        ),
        DeclareLaunchArgument(
            "enable_keyboard_runner", default_value="true",
            description="false to leave stdin to another node (the HIL policy node owns it).",
        ),
        DeclareLaunchArgument(
            "retargeter_type", default_value="continuous",
            description="continuous or discrete hand retargeting.",
        ),
        DeclareLaunchArgument(
            "left_retargeter_calibration_file", default_value="",
            description="Left-hand calibration JSON. Empty selects the newest matching file.",
        ),
        DeclareLaunchArgument(
            "right_retargeter_calibration_file", default_value="",
            description="Right-hand calibration JSON. Empty selects the newest matching file.",
        ),
        DeclareLaunchArgument(
            "discrete_debounce_samples", default_value="3",
            description="Consecutive MANO frames required before accepting a posture change.",
        ),
        DeclareLaunchArgument(
            "discrete_hysteresis_ratio", default_value="0.05",
            description="Required relative distance improvement over the current posture.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
