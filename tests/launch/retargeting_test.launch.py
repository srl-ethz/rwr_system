from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

# Replay recorded Rokoko hand poses (tests/artifacts) through the ORCA v2 retargeter and show the
# result in RViz, without the glove or the hand. Record new poses with experiments/rokoko_recorder.py
# and pass the file name (copied to tests/artifacts) with handposes_file:=<name>.json
#
# ros2 launch faive_test retargeting_test.launch.py


def generate_launch_description():
    hand_name = "orcahand_v2"
    xacro_path = os.path.join(
        get_package_share_directory("viz"), "models", hand_name, f"{hand_name}.urdf.xacro")
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' hand_chirality:=right']), value_type=str)
    hand_scheme_filepath = os.path.join(
        get_package_share_directory("viz"), "models", hand_name, f"scheme_{hand_name}.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "handposes_file", default_value="handposes_20241127222652.json",
                description="JSON file in tests/artifacts with the recorded hand poses",
            ),

            # RETARGET NODE
            Node(
                package="faive_test",
                executable="retargeter_test.py",
                name="retargeter_test_node",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "retarget/hand_scheme": hand_scheme_filepath,
                        "retarget/mano_adjustments": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter",
                            f"mano_adjustment_{hand_name}.yaml"
                        ),
                        "retarget/retargeter_cfg": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter",
                            f"retargeter_cfgs_{hand_name}.yaml"
                        ),
                    },
                    {"debug": True, "handposes_file": LaunchConfiguration("handposes_file")},
                ],
            ),

            # VISUALIZATION NODE
            Node(
                package="viz",
                executable="visualize_joints.py",
                name="visualize_joints",
                parameters=[
                    {
                        "scheme_path": hand_scheme_filepath,
                        "viz/hand_chirality": "right",
                        "viz/joint_name_contains_chirality": False,
                    }
                ],
                output="screen",
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}]),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(get_package_share_directory('viz'), 'config', 'rviz', 'retarget_config.rviz')],
                ),
        ]
    )
