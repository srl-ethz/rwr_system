import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
Visualize the robot hand in rviz and show the GUI to adjust the joint angles.
Does not connect to real robot nor start simulation.
"""

def generate_launch_description():

    urdf = os.path.join(
        get_package_share_directory('viz'),
        "models",
        "franka_panda",
        "panda.urdf")

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='arm',
            output='screen',
            parameters=[{'robot_description': robot_desc,}],
            arguments=[urdf],
            remappings=[('/joint_states', '/right/joint_states')]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui",
        #     output="screen",
        # ),

    ])
