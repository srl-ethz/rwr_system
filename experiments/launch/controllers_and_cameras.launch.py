from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# Start the cameras and the ORCA v2 hand controller, e.g. for running an imitation learning
# policy with run_policy_inference.launch.py. Select the cameras in generate_launch_description().


def spawn_hand_controller(hand_chirality):
    """Spawn the ORCA v2 hand controller node for one side (left/right)."""
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
                "hand_controller/hand_cfg_name": "orcahand_v2",
                "hand_controller/calibrate": False,
            }
        ],
    )
    return [hand_controller_node]


def generate_launch_description():
    OAKD_CAMERA_CFG_NAME = "oakd_scissors_task" # file in cfgs/cams, or None
    ZED_CAMERA_CFG_NAME = None # "zed_color_arena" # or None
    # the ORCA v2 hand is right-handed only (src/hand_control/orca/configs/orcahand_v2/right)
    LEFT_SIDE = False
    RIGHT_SIDE = True

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

    if ZED_CAMERA_CFG_NAME is not None:
        camera_nodes.append(Node(
            package="ingress",
            executable="zed_node.py",
            name="zed_node",
            output="screen",
            parameters=[{
                "config_file": os.path.join(get_package_share_directory("experiments"),
                                            "cfgs", "cams", f"{ZED_CAMERA_CFG_NAME}.yaml")
            }],
        ))

    return LaunchDescription(
        camera_nodes
        + (spawn_hand_controller("right") if RIGHT_SIDE else [])
        + (spawn_hand_controller("left") if LEFT_SIDE else [])
    )
