from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# select the cameras to be used

cameras = {"front_view": True, "side_view": True, "wrist_view": True}


def generate_launch_description():
    return LaunchDescription(
        [
            # CAMERA INGRESS NODE
            Node(
                package="ingress",
                executable="oakd_node.py",
                name="oakd_node",
                output="log",
                parameters=[
                    {"enable_front_camera": cameras["front_view"]},
                    {"enable_side_camera": cameras["side_view"]},
                    {"enable_wrist_camera": cameras["wrist_view"]},
                ],
            ),
            
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
                executable="hand_control_node.py",
                name="hand_control_node",
                output="screen"
            ),
            
            # RETARGET NODE
            Node(
                package="retargeter",
                executable="retargeter_node.py",
                name="retargeter_node",
                output="log",
                parameters=[
                    {
                        "retarget/urdf_filepath": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "faive_hand_p4",
                            "urdf",
                            "p4.urdf",
                        ),
                        "retarget/hand_scheme": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "faive_hand_p4",
                            "scheme_p4.yaml",
                        )
                    },
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
