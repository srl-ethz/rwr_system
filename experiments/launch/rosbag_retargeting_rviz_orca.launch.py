from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    urdf = os.path.join(
    get_package_share_directory('viz'),
    "models",
    "orca1_hand",
    "urdf",
    "orca1.urdf")

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
    
            Node(
                package="ingress",  # Replace with your package name
                executable="rosbag_node.py",  # Replace with your script's entry point (name set in setup.py)
                name="rosbag_node",
                output="screen",
                parameters=[
                    {
                         "rosbag_path": os.path.join(
                            get_package_share_directory('viz'),
                            "rosbag",
                            "recordings",
                            "recording_2024-11-14_17-46-39/",
                        )
                    },
                ],
            ),
            Node(
                package="hand_control",
                executable="hand_control_node.py",
                name="hand_control_node",
                output="screen"
            ),
            # RETARGET NODE
            Node(
                package="retargeter",
                executable="retargeter_node.py",
                name="retargeter_node",
                output="screen",
                # COMMENT OR UNCOMMENT THE FOLLOWING LINES TO SWITCH BETWEEN MJCF AND URDF, JUST ONE OF THEM SHOULD BE ACTIVE TODO: Make this a parameter
                parameters=[
                    # {
                    #     "retarget/mjcf_filepath": os.path.join(
                    #         get_package_share_directory("viz"),
                    #         "models",
                    #         "faive_hand_p4",
                    #         "hand_p4.xml",
                    #     )
                    # },
                    {
                        "retarget/urdf_filepath": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "orca1_hand",
                            "urdf",
                            "orca1.urdf",
                        )
                    },
                    {"retarget/hand_scheme": "orca1"},
                    {"debug": True},
                ],
            ),
            
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
                            "orca1_hand",
                            "scheme_orca1.yaml",
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
                arguments=['-d', os.path.join(get_package_share_directory('viz'), 'rviz', 'retarget_config_orca1.rviz')],
                ),

        ]
    )
