from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf = os.path.join(
    get_package_share_directory('viz'),
    "models",
    "biomimic_hand_v3",
    "urdf",
    "biomimic_hand.urdf")

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [

            Node(
                package="ingress",
                executable="mediapipe_node.py",
                name="mediapipe_node",
                output="log",
            ),

            # RETARGET NODE
            Node(
                package="retargeter",
                executable="retargeter_node.py",
                name="retargeter_node",
                output="screen",
                # COMMENT OR UNCOMMENT THE FOLLOWING LINES TO SWITCH BETWEEN MJCF AND URDF, JUST ONE OF THEM SHOULD BE ACTIVE TODO: Make this a parameter
                parameters=[
                    {
                        "retarget/mjcf_filepath": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "biomimic_hand_v3",
                            "Biomimc_hand_job.xml",
                        )
                    },
                    # {
                    #     "retarget/urdf_filepath": os.path.join(
                    #         get_package_share_directory("viz"),
                    #         "models",
                    #         "biomimic_hand",
                    #         "urdf",
                    #         "biomimic_hand.urdf",
                    #     )
                    # },
                    {"retarget/hand_scheme": "biomimic"},
                    {"debug": True},
                ],
            ),

            # REMAPPER NODE
            # Node(
            #     package="retargeter",
            #     executable="remapper_node",
            #     name="remapper",
            #     output="screen"
            # ),

            # PLOTTING NODE
            Node(
                package="retargeter",
                executable="plot_target_joint_angles.py",
                name="plot_joints",
                output="screen",
                emulate_tty=True,
            ),

            # VISUALIZATION NODE
            # Node(
            #     package="viz",
            #     executable="visualize_joints.py",
            #     name="visualize_joints",
            #     parameters=[
            #         {
            #             "scheme_path": os.path.join(
            #                 get_package_share_directory("viz"),
            #                 "models",
            #                 "biomimic_hand",
            #                 "scheme_biomimic.yaml",
            #             )
            #         },
            #     ],
            #     output="screen",
            # ),

            # VISUALIZATION NODE (MuJoCo)
            # Node(
            #     package="viz",
            #     executable="mujoco_visualizer_node.py",
            #     name="mujoco_visualizer",
            #     parameters=[
            #         {
            #             "viz/model_path": os.path.join(
            #                 get_package_share_directory("viz"),
            #                 "models",
            #                 "biomimic_hand_v3",
            #                 "Biomimic_hand_job.xml",
            #             )
            #         }
            #     ],
            #     output="screen"
            # ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc,}],
                arguments=[urdf]),

            # Node(
            #     package='rviz2',
            #     executable='rviz2',
            #     name='rviz2',
            #     output='screen',
            #     arguments=['-d', os.path.join(get_package_share_directory('viz'), 'rviz', 'retarget_config.rviz')],
            #     ),

        ]
    )