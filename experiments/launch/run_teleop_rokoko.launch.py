from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            
            Node(
                package="ingress",
                executable="rokoko_node.py",
                name="rokoko_node",
                output="log",
                parameters=[
                    {"rokoko_tracker/ip": "0.0.0.0"},
                    {"rokoko_tracker/port": 14043},
                    {"rokoko_tracker/use_coil": True}
                ],
            ),

            # HAND CONTROLLER NODE
            Node(
                package="hand_control",
                executable="hand_controller_node.py",
                name="hand_controller_node",
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
                            "faive_hand_p4",
                            "urdf",
                            "p4.urdf",
                        )
                    },
                    {"retarget/hand_scheme": "p4"},
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
                            "faive_hand_p4",
                            "scheme_p4.yaml",
                        )
                    }
                ],
            ),
        ]
    )
