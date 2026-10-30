from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

"""
Control the joints of the Orca hand using the joint_state_publisher_gui and visualize it in RViz.
Also works as a minimal example of a full controller of the Orca hand.
"""
def generate_launch_description():
  hand_chirality = LaunchConfiguration('hand_chirality')
  scheme_path = PathJoinSubstitution([
      FindPackageShare('viz'), 'models', 'orcahand_v2',
      'scheme_orcahand_v2.yaml',
  ])
  is_left = PythonExpression(["'", hand_chirality, "' == 'left'"])

  return LaunchDescription([
      DeclareLaunchArgument(
        'hand_chirality',
        default_value='right',
        description='Chirality of the hand to be controlled',
      ),
      Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=hand_chirality,
        name='joint_state_publisher_gui',
        output='screen',
      ),
      Node(
        package='viz',
        executable='reverse_visualize_joints.py',
        namespace=hand_chirality,
        name='reverse_visualize_joints_node',
        output='screen',
        condition=UnlessCondition(is_left),
        parameters=[{
            'viz/hand_chirality': hand_chirality,
            'viz/joint_name_contains_chirality': False,
            'scheme_path': scheme_path,
        }]
      ),
      Node(
        package='viz',
        executable='reverse_visualize_joints.py',
        namespace=hand_chirality,
        name='reverse_visualize_joints_node',
        output='screen',
        condition=IfCondition(is_left),
        parameters=[{
            'viz/hand_chirality': hand_chirality,
            'viz/joint_name_contains_chirality': False,
            'scheme_path': scheme_path,
        }]
      ),
      IncludeLaunchDescription(
        PathJoinSubstitution([
          FindPackageShare('hand_control'),
          'launch',
          'orcahand_v2_controller_launch.yaml'
        ]),
        launch_arguments={
          'launch_rviz': 'true',
          'hand_chirality': hand_chirality,
          'calibrate': 'false',
          'compliant_test_mode': 'false',
          # special case: we publish /joint_states from the joint_state_publisher_gui and convert it to /policy_output in this launch file
          # so don't re-do the conversion in the main controller (or there will be a loop)
          'suppress_visualize_joints': 'true',
        }.items()
      )
    ])