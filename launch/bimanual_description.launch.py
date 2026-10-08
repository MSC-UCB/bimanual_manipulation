"""Publish the composite model and its joint-state-driven global TF tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from bimanual_manipulation.composite_description import build_description, selected_sides


def _nodes(context):
    selection = LaunchConfiguration('sides', default='both').perform(context)
    sides = selected_sides(selection)
    description = build_description(
        LaunchConfiguration('hand_mounts').perform(context) or None, sides=selection)
    sources = ([f'/crx5ia/{side}/joint_states' for side in sides]
               + [f'/sharpa/{side}_hand/joint_states' for side in sides])
    return [
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            namespace='bimanual', name='joint_state_merger', output='screen',
            parameters=[{
                'robot_description': description, 'source_list': sources,
                'rate': 100, 'publish_default_positions': False,
            }],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='bimanual', name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': description}],
            remappings=[('joint_states', '/bimanual/joint_states')],
        ),
        Node(
            package='rviz2', executable='rviz2', name='bimanual_rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('bimanual_manipulation'), 'rviz', 'bimanual.rviz'])],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sides', default_value='both', choices=['both', 'left', 'right']),
        DeclareLaunchArgument('hand_mounts', default_value='',
                              description='Path to calibrated or mock hand_mounts.yaml'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
