from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hand_name = "orcahand_v1b" # "orca_v1" "mimic"
    hand_chirality = "right"
    ## use_saved_keypoint tells the helper to use some predefined mano keypoints so that no ingress is necessary
    ## if "" is used, the helper will subscribe to the ingress topic and ingress node will be spawned
    use_saved_keypoint = "" # "" "hand_open" 
    use_gesture_runner = False
    ingress_system = "rokoko"  # "rokoko" "avp"
    if use_saved_keypoint != "":
        ingress_system = "saved_keypoint"
    publish_retarget_result = True
    launch_hand_controller = True
    avp_ip = os.environ.get('AVP_IP', '192.168.0.137')  # Vision Pro IP address


    robot_description = None

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
            xacro_path = os.path.join(get_package_share_directory("viz"),
                                      "models", hand_name, f"{hand_name}.urdf.xacro")
            robot_description = ParameterValue(
                Command(['xacro ', xacro_path, ' hand_chirality:=', hand_chirality]),
                value_type=str,
            )
            urdf_filepath = None
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
    ingress_adjustments_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "avp_ingress_adjustment.yaml")

    launch_description = []

    ## Define nodes
    # INGRESS NODE
    if ingress_system == "rokoko":
        ingress_node = Node(
                    package="ingress",
                    executable="rokoko_node.py",
                    name="rokoko_node",
                    output="log",
                    parameters=[
                        {"rokoko_tracker/ip": "0.0.0.0"},
                        {"rokoko_tracker/port": 14043},
                        {"rokoko_tracker/use_coil": False}
                    ],
                )
        launch_description.append(ingress_node)
    elif ingress_system == "avp":
        ingress_node =  Node(
            package="ingress",
            executable="avp_node.py",
            name="avp_node",
            output="screen",
            parameters=[
                {"avp_tracker/ip": avp_ip},
                {"avp_tracker/record": False},
                {"avp_tracker/ingress_adjustments": ingress_adjustments_filepath},
            ],
        )
        launch_description.append(ingress_node)

    if hand_name == "orcahand_v1b":
        # use the base launch file which launches all necessary hand related nodes at once

        v1b_launch =IncludeLaunchDescription(
            PathJoinSubstitution([
            FindPackageShare('hand_control'),
            'launch',
            'orcahand_v1b_controller_launch.yaml'
            ]),
            launch_arguments={
            'launch_rviz': 'false',
            'hand_chirality': hand_chirality,
            'calibrate': 'false',
            'launch_hand_controller': "true" if launch_hand_controller else "false",
            }.items()
        )
        launch_description.append(v1b_launch)
    
    else:
        # there is no base launch file for these hands (yet), so launch each node individually
        if not hand_name in ["mimic", "orca_v1"]:
            raise NotImplementedError(f"Launch file for {hand_name} is not set up yet")
        with open(urdf_filepath, 'r') as infp:
            robot_desc = infp.read()
        if launch_hand_controller:
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
                        "hand_controller/calibrate": True
                    }
                ],
            )
            launch_description.append(hand_controller_node)

        # Robot State Publisher Node
        robot_state_publisher_node = Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher_right',
                    namespace=hand_chirality,
                    output='screen',
                    parameters=[{'robot_description': robot_desc,}],
                    arguments=[urdf_filepath])
        launch_description.append(robot_state_publisher_node)
        # Joint Visualization Node
        joint_viz_node = Node(
                    package="viz",
                    executable="visualize_joints.py",
                    name="visualize_joints",
                    namespace=hand_chirality,
                    output='screen',
                    parameters=[
                        {
                            "scheme_path": hand_scheme_filepath,
                            "viz/hand_chirality": hand_chirality
                        },
                    ],
                )
        launch_description.append(joint_viz_node)

    # RETARGET CONFIG HELPER NODE
    helper_params = {
        "retarget/hand_scheme": hand_scheme_filepath,
        "retarget/hand_chirality": hand_chirality,
        "retarget/use_saved_keypoint": use_saved_keypoint,
        "retarget/publish_retarget_result": publish_retarget_result,
        "retarget/mano_adjustments": mano_adjustments_filepath,
        "retarget/retargeter_cfg": retargeter_cfg_filepath,
    }
    if robot_description is not None:
        helper_params["robot_description"] = robot_description
    else:
        helper_params["retarget/urdf_filepath"] = urdf_filepath

    retargeter_config_helper_node = Node(
                package="retargeter",
                executable="retargeter_config_helper.py",
                name="retargeter_config_helper",
                output="screen",
                namespace=hand_chirality,
                parameters=[helper_params],
            )
    launch_description.append(retargeter_config_helper_node)

    # RVIZ node
    rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen', 
                arguments=['-d', os.path.join(get_package_share_directory('viz'), 'config/rviz', 'retarget_helper.rviz')],
            )
    launch_description.append(rviz_node)

    if not publish_retarget_result:
        joint_gui_node = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                namespace=hand_chirality,
                name="joint_state_publisher_gui",
                output="screen",
            )
        launch_description.append(joint_gui_node)
        
    if use_gesture_runner:
        gesture_runner_node = Node(
                package="experiments",
                executable="gesture_runner.py",
                name="gesture_runner",
                output="screen",
                parameters=[
                    {
                        "height_threshold": 0.10,
                        "frequency": 10.0,
                        "retargeter_set_enable_topic":
                            f"/{hand_chirality}/retargeter/set_enabled",
                    }
                ],
            )
        launch_description.append(gesture_runner_node)

    return LaunchDescription(launch_description)
