from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# select the cameras to be used


def spawn_hand_controller(hand_chirality, hand_name):
    """Spawn the hand controller node for one side (left/right).

    arg: hand_name: "orcahand_v1b" # "orca_v1" "mimic", None
    """
    if hand_name is None:
        return []

    if "mimic" in hand_name:
        hand_family = "mimic"
    elif "orca" in hand_name:
        hand_family = "orca"
    else:
        raise ValueError(f"Unknown hand_name: {hand_name}")

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
    return [hand_controller_node]


def generate_launch_description():
    OAKD_CAMERA_CFG_NAME = "oakd_pour_task" # or None
    ZED_CONFIG_FILE = None
    LEFT_SIDE = True
    RIGHT_SIDE = True
    hand_name = "orcahand_v1b" # "orca_v1" "mimic", None

    camera_nodes = []

    if OAKD_CAMERA_CFG_NAME is not None:
        camera_nodes.append(Node(
            package="ingress",
            executable="oakd_node.py",
            name="oakd_node",
            output="screen",
            parameters=[{
                "config_file": os.path.join(get_package_share_directory("experiments"),
                                            "cfgs", "cams", f"{OAKD_CAMERA_CFG_NAME}.yaml")
            }],
        ))

    if ZED_CONFIG_FILE is not None:
        camera_nodes.append(Node(
            package="ingress",
            executable="zed_node.py",
            name="zed_node",
            output="screen",
            parameters=[{
                "config_file":ZED_CONFIG_FILE,
            }]
        ))

    return LaunchDescription(
        camera_nodes
        + (spawn_hand_controller("right", hand_name) if RIGHT_SIDE else [])
        + (spawn_hand_controller("left", hand_name) if LEFT_SIDE else [])
    )
