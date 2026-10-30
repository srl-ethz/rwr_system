#!/usr/bin/env python3
"""
Complete RWR Task 1 Launch File with Cube Detection and Policy, for the ORCA v2 hand

This launch file combines everything needed for the complete system:
1. Robot hand visualization (URDF + state publisher), showing the pose measured by the motors
2. Hand controller node
3. OakD camera
4. Static TF publishers (tag→camera, tag→hand_root)
5. Cube detection node
6. Policy inference node (policy_inference_cube_spin_v2.py)
7. RViz2 visualization

Command line usage:
    ros2 launch experiments cube_spin_v2.launch.py \
        calibration:=/path/to/camera_extrinsics.npy \
        model_path:=/path/to/pose_estimation/finetuned_models/maskrcnn-finetuned_lightning_new_adamw.ckpt \
        policy:=/path/to/exported/policy.pt \
        enable_hand_controller:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np
import xacro


def generate_launch_description():
    """Generate launch description with all arguments"""
    
    # ===== Calibration and Policy Arguments =====
    calibration_arg = DeclareLaunchArgument(
        'calibration',
        default_value='',
        description='Path to camera calibration file (cam_T_tag as .npy)'
    )
    
    policy_arg = DeclareLaunchArgument(
        'policy',
        default_value='',
        description='Path to the policy exported by faive_lab (exported/policy.pt)'
    )

    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='true: the policy computes and logs actions but does not command the hand'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to the cube detection checkpoint (.ckpt from the pose_estimation repo)'
    )


    # ===== Tag to Hand Transform Arguments =====
    # Pose of the tag seen from the mount (mount_T_tag): x/y/z along the mount's axes, roll/pitch/yaw of the tag relative to them
    # NOTE: these defaults were measured for the ORCA v1b mount. Measure them again for your ORCA v2 setup.
    hand_to_tag_x_arg = DeclareLaunchArgument('hand_to_tag_x', default_value='0.0')
    hand_to_tag_y_arg = DeclareLaunchArgument('hand_to_tag_y', default_value='0.05')
    hand_to_tag_z_arg = DeclareLaunchArgument('hand_to_tag_z', default_value='0.055')
    hand_to_tag_roll_arg = DeclareLaunchArgument('hand_to_tag_roll', default_value='90')
    hand_to_tag_pitch_arg = DeclareLaunchArgument('hand_to_tag_pitch', default_value='0')
    hand_to_tag_yaw_arg = DeclareLaunchArgument('hand_to_tag_yaw', default_value='180')
    
    # ===== Camera Arguments =====
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='oakd_bottom_view',
        description='Camera name for TF frames'
    )
    
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value='oakd_cube_task',
        description='Camera configuration file name (without .yaml)'
    )
    
    # ===== Enable/Disable Components =====
    enable_hand_controller_arg = DeclareLaunchArgument(
        'enable_hand_controller',
        default_value='false',
        description='Enable hand controller node (connects to real robot)'
    )
    
    # Launch the nodes using OpaqueFunction
    nodes = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        calibration_arg,
        policy_arg,
        dry_run_arg,
        model_path_arg,
        hand_to_tag_x_arg,
        hand_to_tag_y_arg,
        hand_to_tag_z_arg,
        hand_to_tag_roll_arg,
        hand_to_tag_pitch_arg,
        hand_to_tag_yaw_arg,
        camera_name_arg,
        camera_config_arg,
        enable_hand_controller_arg,
        nodes,
    ])


def launch_setup(context, *args, **kwargs):
    """Setup function to create all nodes with parameters"""
    from scipy.spatial.transform import Rotation as R
    
    # Get launch arguments
    calibration_file = LaunchConfiguration('calibration').perform(context)
    policy_file = LaunchConfiguration('policy').perform(context)
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    model_path = LaunchConfiguration('model_path').perform(context)
    hand_to_tag_x = float(LaunchConfiguration('hand_to_tag_x').perform(context))
    hand_to_tag_y = float(LaunchConfiguration('hand_to_tag_y').perform(context))
    hand_to_tag_z = float(LaunchConfiguration('hand_to_tag_z').perform(context))
    hand_to_tag_roll = float(LaunchConfiguration('hand_to_tag_roll').perform(context))
    hand_to_tag_pitch = float(LaunchConfiguration('hand_to_tag_pitch').perform(context))
    hand_to_tag_yaw = float(LaunchConfiguration('hand_to_tag_yaw').perform(context))
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_config = LaunchConfiguration('camera_config').perform(context)
    enable_hand_controller = LaunchConfiguration('enable_hand_controller').perform(context).lower() == 'true'
    
    # ===== 1. Robot Hand Visualization =====
    orca_xacro = os.path.join(
        get_package_share_directory('viz'),
        "models",
        "orcahand_v2",
        "orcahand_v2.urdf.xacro"
    )
    orca_robot_desc = xacro.process_file(orca_xacro, mappings={'hand_chirality': 'right'}).toxml()

    orca_scheme_filepath = os.path.join(
        get_package_share_directory("viz"),
        f"models/orcahand_v2/scheme_orcahand_v2.yaml"
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='right/hand',
        name='hand_state_publisher',
        output='screen',
        parameters=[{'robot_description': orca_robot_desc}],
        remappings=[('/joint_states', '/right/hand/joint_states')]
    )
    
    # ===== 2. Hand Joint Visualizer =====
    # shows the joint positions measured by the motors
    visualize_joints = Node(
        package="viz",
        executable="visualize_joints.py",
        name="visualize_joints",
        namespace='right',
        output='log',
        parameters=[{
            "scheme_path": orca_scheme_filepath,
            "viz/hand_chirality": "right",
            "viz/joint_name_contains_chirality": False
        }],
        remappings=[('/right/joint_states', '/right/hand/joint_states'),
                    ('hand/policy_output', '/right/hand/joint_position_from_motor')]
    )
    
    # ===== 3. OakD Camera Node =====
    oakd_node = Node(
        package="ingress",
        executable="oakd_node.py", 
        name="oakd_node",
        output="log",
        parameters=[{
            "config_file": os.path.join(
                get_package_share_directory("experiments"), 
                "cfgs", "cams", f"{camera_config}.yaml"
            ),
            "use_depth": False
        }],
    )
    
    # ===== 4. Static TF Publishers =====
    # TF tree: camera_frame → tag_frame → hand_root
    # This allows: cube (detected in camera) → tag → hand
    
    # 4a. Tag → Hand (tag_T_hand transform)
    # hand_to_tag_x/y/z are measured from the mount (orcahand_v2.urdf.xacro), which sits at (-0.01, 0, -0.058) in hand_root
    hand_to_tag_t = np.array([hand_to_tag_x, hand_to_tag_y, hand_to_tag_z]) + np.array([-0.01, 0.0, -0.058])
    hand_to_tag_quat = R.from_euler('XYZ', [hand_to_tag_roll, hand_to_tag_pitch, hand_to_tag_yaw], degrees=True).as_quat()
    rot_inv = R.from_quat(hand_to_tag_quat).inv()
    tag_to_hand_t = -rot_inv.apply(hand_to_tag_t)
    tag_to_hand_quat = rot_inv.as_quat()


    static_tf_tag_to_hand = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tag_to_hand',
        arguments=[
            str(tag_to_hand_t[0]), str(tag_to_hand_t[1]), str(tag_to_hand_t[2]),
            str(tag_to_hand_quat[0]), str(tag_to_hand_quat[1]), str(tag_to_hand_quat[2]), str(tag_to_hand_quat[3]),
            'tag_frame', 'hand_root'
        ]
    )
    
    # 4b. Tag → Camera (from calibration file - cam_T_tag inverted to get tag_T_cam)
    static_tf_tag_to_camera = None
    if os.path.exists(calibration_file):
        cam_T_tag = np.load(calibration_file)
        tag_T_cam = np.linalg.inv(cam_T_tag)
        
        cam_t = tag_T_cam[:3, 3]
        cam_rot_matrix = tag_T_cam[:3, :3]
        cam_quat = R.from_matrix(cam_rot_matrix).as_quat()
        
        static_tf_tag_to_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_tag_to_camera',
            arguments=[
                str(cam_t[0]), str(cam_t[1]), str(cam_t[2]),
                str(cam_quat[0]), str(cam_quat[1]), str(cam_quat[2]), str(cam_quat[3]),
                'tag_frame', f'{camera_name}_frame'
            ]
        )
    else:
        print(f"WARNING: Calibration file not found: {calibration_file}")
        print("Skipping tag→camera TF publisher. Cube detection will not work without calibration.")
    
    # ===== 5. Cube Detection Node =====
    cube_detection_node = None
    if model_path:
        cube_detection_node = Node(
            package='experiments',
            executable='cube_detection_node.py',
            name='cube_detection_node',
            output='screen',
            parameters=[{
                'camera_name': camera_name,
                'model_path': os.path.expanduser(model_path),
                'show_windows': False
            }]
        )
    else:
        print("WARNING: model_path not set, skipping cube detection.")

    # ===== 6. Policy Inference Node (optional) =====
    policy_inference_node = None
    if policy_file:
        policy_inference_node = Node(
            package='experiments',
            executable='policy_inference_cube_spin_v2.py',
            name='policy_inference_node',
            output='screen',
            parameters=[{
                'policy_ckpt_path': os.path.expanduser(policy_file),
                'dry_run': dry_run,
                'hand_config_path': os.path.join(
                    get_package_share_directory('hand_control'),
                    'orca', 'configs', 'orcahand_v2', 'right', 'config.yaml'
                ),
            }],
        )

    # ===== 7. Hand Controller Node (optional) =====
    hand_controller_node = None
    if enable_hand_controller:
        hand_controller_node = Node(
            package="hand_control",
            executable="hand_control_node.py",
            name="hand_controller_node",
            output="screen",
            namespace="right",
            parameters=[{
                "hand_controller/hand_name": "orca_v2",
                "hand_controller/hand_chirality": "right",
                "hand_controller/hand_cfg_name": "orcahand_v2",
                "hand_controller/calibrate": False
            }],
        )
    
    # ===== 8. RViz2 =====
    rviz_config = os.path.join(
        get_package_share_directory('experiments'), 
        'cfgs', 'rviz', 'cubespin.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )
    
    # ===== Build Node List =====
    nodes_to_launch = [
        # Core visualization and robot
        robot_state_publisher,
        visualize_joints,
        oakd_node,
        
        # TF publishers (camera → tag → hand)
        static_tf_tag_to_hand,
        
        # Detection and policy
        cube_detection_node,
        
        # Visualization
        rviz_node,
    ]
    
    # Add optional tag→camera TF if calibration exists
    if static_tf_tag_to_camera is not None:
        nodes_to_launch.insert(4, static_tf_tag_to_camera)  # Insert after static_tf_hand_to_tag
    
    # Add optional policy node
    if policy_inference_node is not None:
        nodes_to_launch.append(policy_inference_node)
    
    # Add optional hand controller
    if hand_controller_node is not None:
        nodes_to_launch.insert(3, hand_controller_node)  # After oakd_node

    return [node for node in nodes_to_launch if node is not None]
