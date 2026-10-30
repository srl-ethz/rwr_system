from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hand_name = "orcahand_v2"
    hand_chirality = "right"  # the ORCA v2 hand is right-handed only
    ## use_saved_keypoint tells the helper to use some predefined mano keypoints so that no ingress is necessary
    ## if "" is used, the helper will subscribe to the ingress topic and the rokoko node will be spawned
    use_saved_keypoint = "" # "" "hand_open" 
    use_gesture_runner = False
    publish_retarget_result = True
    launch_hand_controller = True

    xacro_path = os.path.join(get_package_share_directory("viz"),
                              "models", hand_name, f"{hand_name}.urdf.xacro")
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' hand_chirality:=', hand_chirality]),
        value_type=str,
    )
    hand_scheme_filepath = os.path.join(get_package_share_directory("viz"), "models", hand_name, f"scheme_{hand_name}.yaml")
    mano_adjustments_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "retargeter", f"mano_adjustment_{hand_name}.yaml")
    retargeter_cfg_filepath = os.path.join(get_package_share_directory("experiments"), "cfgs", "retargeter", f"retargeter_cfgs_{hand_name}.yaml")

    launch_description = []

    ## Define nodes
    # INGRESS NODE
    if use_saved_keypoint == "":
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

    # use the base launch file which launches all necessary hand related nodes at once
    hand_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
        FindPackageShare('hand_control'),
        'launch',
        'orcahand_v2_controller_launch.yaml'
        ]),
        launch_arguments={
        'launch_rviz': 'false',
        'hand_chirality': hand_chirality,
        'calibrate': 'false',
        'launch_hand_controller': "true" if launch_hand_controller else "false",
        }.items()
    )
    launch_description.append(hand_launch)

    # RETARGET CONFIG HELPER NODE
    helper_params = {
        "retarget/hand_scheme": hand_scheme_filepath,
        "retarget/hand_chirality": hand_chirality,
        "retarget/use_saved_keypoint": use_saved_keypoint,
        "retarget/publish_retarget_result": publish_retarget_result,
        "retarget/mano_adjustments": mano_adjustments_filepath,
        "retarget/retargeter_cfg": retargeter_cfg_filepath,
    }
    helper_params["robot_description"] = robot_description

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
