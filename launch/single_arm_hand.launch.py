"""Start one CRX arm and its same-side Sharpa hand; software mock by default."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, UnsetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

ARM_OVERRIDES = ('left_robot_ip', 'right_robot_ip', 'method', 'input_rate_hz',
                 'ruckig_target_mode', 'ruckig_target_timeout',
                 'collision_force_limit', 'collision_force_threshold_n')


def _include(package, file, arguments):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', file]))
    return IncludeLaunchDescription(source, launch_arguments=arguments.items())


def _system(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    prefix = value('prefix')
    if prefix not in ('left_', 'right_'):
        raise ValueError('prefix must be left_ or right_')
    crx_args = {'prefix': prefix, 'control': 'none', 'mock': value('crx_mock'),
                'rviz': 'false', 'read_only': value('read_only')}
    crx_args.update({name: value(name) for name in ARM_OVERRIDES if value(name)})
    # Includes inherit launch configurations. Unset blank overrides within the
    # arm group so upstream DeclareLaunchArgument can supply its own defaults.
    unset = [UnsetLaunchConfiguration(name) for name in ARM_OVERRIDES if not value(name)]
    hand_args = {
        'prefix': prefix,
        'backend': value('sharpa_backend'), 'use_rviz': 'false',
        'config_file': value('sharpa_config'),
        'publish_rate_hz': value('sharpa_publish_rate_hz'),
        'mit_kp_ratio': LaunchConfiguration('mit_kp_ratio', default='').perform(context),
        'mit_kd_ratio': LaunchConfiguration('mit_kd_ratio', default='').perform(context),
        'read_only': 'true' if value('read_only') == 'true' else value('sharpa_read_only'),
    }
    return [
        GroupAction([
            *unset,
            SetRemap(src='/tf', dst='/crx5ia/arm_only_tf'),
            SetRemap(src='/tf_static', dst='/crx5ia/arm_only_tf_static'),
            _include('dual_crx_control', 'single_arm.launch.py', crx_args),
        ]),
        GroupAction([_include('dual_sharpa_wave', 'single_sharpa.launch.py', hand_args)]),
        GroupAction([_include('bimanual_manipulation', 'bimanual_description.launch.py', {
            'hand_mounts': value('hand_mounts'), 'rviz': value('rviz'), 'sides': prefix[:-1],
        })]),
        Node(package='bimanual_manipulation', executable='tactile_wrench_viewer.py',
             name='tactile_wrench_viewer', output='screen',
             arguments=['--force-max', value('force_max'),
                        '--resultant-force-max', value('resultant_force_max'),
                        '--wrench-force-max', value('wrench_force_max'),
                        '--wrench-torque-max', value('wrench_torque_max')],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('prefix', default_value='left_', choices=['left_', 'right_']),
        DeclareLaunchArgument('crx_mock', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false'],
                              description='Read the selected arm and hand without motion control'),
        DeclareLaunchArgument('sharpa_backend', default_value='mock', choices=['mock', 'sharpa_sdk']),
        DeclareLaunchArgument('sharpa_config', default_value='',
                              description='Hand YAML path; blank selects upstream backend config'),
        DeclareLaunchArgument('sharpa_publish_rate_hz', default_value='',
                              description='Blank preserves the hand YAML rate'),
        DeclareLaunchArgument('mit_kp_ratio', default_value='0.6',
                              description='Startup Kp multiplier for the selected hand; blank uses YAML'),
        DeclareLaunchArgument('mit_kd_ratio', default_value='0.8',
                              description='Startup Kd multiplier for the selected hand; blank uses YAML'),
        DeclareLaunchArgument('sharpa_read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hand_mounts', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('viewer', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('force_max', default_value='15', description='Tactile magnitude, SDK units'),
        DeclareLaunchArgument('resultant_force_max', default_value='15',
                              description='F6 first-three-component magnitude bar, SDK units'),
        DeclareLaunchArgument('wrench_force_max', default_value='20', description='Wrench force +/- N'),
        DeclareLaunchArgument('wrench_torque_max', default_value='3', description='Wrench torque +/- N m'),
    ]
    for name in ARM_OVERRIDES:
        arguments.append(DeclareLaunchArgument(
            name, default_value='50.0' if name == 'collision_force_threshold_n' else '',
            description='Blank uses dual_crx_control launch default'))
    return LaunchDescription([*arguments, OpaqueFunction(function=_system)])
