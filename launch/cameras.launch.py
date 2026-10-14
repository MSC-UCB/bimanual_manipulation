"""Optional RealSense cameras, usable independently of the robot launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from bimanual_manipulation.cameras import enabled_roles, load_camera_config


def _cameras(context):
    roles = enabled_roles(
        LaunchConfiguration('enable_wrist_camera').perform(context) == 'true',
        LaunchConfiguration('enable_fix_camera').perform(context) == 'true')
    if not roles:
        return []
    config_path = LaunchConfiguration('camera_config').perform(context)
    if not config_path:
        config_path = Path(get_package_share_directory('bimanual_manipulation')) / 'config/cameras.yaml'
    cameras = load_camera_config(config_path, roles)
    try:
        driver_launch = Path(get_package_share_directory('realsense2_camera')) / 'launch/rs_launch.py'
    except PackageNotFoundError as error:
        raise RuntimeError('Enabled cameras require realsense2_camera. Install '
                           'ros-jazzy-realsense2-camera and source ROS/workspace setup.bash.') from error
    actions = []
    for role, config in cameras.items():
        arguments = {
            'camera_namespace': '/camera', 'camera_name': role,
            # Upstream removes this prefix; it prevents numeric YAML coercion.
            'serial_no': '_' + config['serial_no'],
            'enable_color': 'true', 'enable_depth': str(config['enable_depth']).lower(),
            'rgb_camera.color_profile': config['color_profile'],
            'depth_module.color_profile': config['color_profile'],
            'depth_module.depth_profile': config['depth_profile'],
            'enable_infra': 'false', 'enable_infra1': 'false', 'enable_infra2': 'false',
            'enable_gyro': 'false', 'enable_accel': 'false',
            'pointcloud.enable': 'false', 'align_depth.enable': 'false',
            'enable_sync': 'false', 'publish_tf': 'false',
        }
        # A fresh scope prevents robot-launch or sibling-driver settings leaking in.
        actions.append(GroupAction(scoped=True, forwarding=False, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver_launch)),
                                     launch_arguments=arguments.items()),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_wrist_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_fix_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('camera_config', default_value='',
                              description='Camera role YAML; blank uses installed config/cameras.yaml'),
        OpaqueFunction(function=_cameras),
    ])
