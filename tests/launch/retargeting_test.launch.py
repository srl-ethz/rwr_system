from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf = os.path.join(
    get_package_share_directory('viz'),
    "models",
    "faive_hand_p4_4",
    "urdf",
    "p4_4.urdf")

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription(
        [
            # RETARGET NODE
            Node(
                package="faive_test",
                executable="retargeter_test.py",
                name="retargeter_test_node",
                output="screen",
                # COMMENT OR UNCOMMENT THE FOLLOWING LINES TO SWITCH BETWEEN MJCF AND URDF, JUST ONE OF THEM SHOULD BE ACTIVE TODO: Make this a parameter
                parameters=[
                    {
                        "retarget/urdf_filepath": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "faive_hand_p4_4",
                            "urdf",
                            "p4_4.urdf",
                        ),
                        "retarget/hand_scheme": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "faive_hand_p4_4",
                            "scheme_p4.yaml",
                        ),
                        "retarget/mano_adjustments": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter_adjustment.yaml"
                        ),
                        "retarget/retargeter_cfg": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter_cfgs_p4.yaml"
                        ),
                    },
                    {"debug": True},
                ],
            ),

            # # HAND CONTROLLER NODE
            # Node(
            #     package="hand_control",
            #     executable="hand_control_node.py",
            #     name="hand_controller_node",
            #     output="screen"),

            # VISUALIZATION NODE
            Node(
                package="viz",
                executable="visualize_joints.py",
                name="visualize_joints",
                parameters=[
                    {
                        "scheme_path": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "faive_hand_p4_4",
                            "scheme_p4.yaml",
                        )
                    }
                ],
                output="screen",
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc,}],
                arguments=[urdf]),
            
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen', 
                arguments=['-d', os.path.join(get_package_share_directory('viz'), 'config', 'rviz', 'retarget_config.rviz')],
                ),
        ]
    )