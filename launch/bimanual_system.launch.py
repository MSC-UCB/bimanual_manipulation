"""Bring up both existing control stacks and one composite preview model."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def _source(package, file):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', file]))


def generate_launch_description():
    crx = IncludeLaunchDescription(
        _source('dual_crx_control', 'dual_arm.launch.py'),
        launch_arguments={
            'mock': LaunchConfiguration('crx_mock'),
            'rviz': 'false',
            'method': 'linear',
            'input_rate_hz': '100.0',
            'left_robot_ip': LaunchConfiguration('left_robot_ip'),
            'right_robot_ip': LaunchConfiguration('right_robot_ip'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('crx_mock', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sharpa_backend', default_value='mock',
                              choices=['mock', 'sharpa_sdk']),
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.10.100'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.10.200'),
        DeclareLaunchArgument('hand_mounts', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        # The CRX launch also starts its own arm-only state publisher. Scope its
        # TF away from /tf so only the composite model publishes these frames.
        GroupAction([
            SetRemap(src='/tf', dst='/crx5ia/arm_only_tf'),
            SetRemap(src='/tf_static', dst='/crx5ia/arm_only_tf_static'),
            crx,
        ]),
        IncludeLaunchDescription(
            _source('dual_sharpa_wave', 'dual_sharpa.launch.py'),
            launch_arguments={
                'backend': LaunchConfiguration('sharpa_backend'),
                'use_rviz': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            _source('bimanual_manipulation', 'bimanual_description.launch.py'),
            launch_arguments={
                'hand_mounts': LaunchConfiguration('hand_mounts'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
