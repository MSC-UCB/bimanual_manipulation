"""Camera configuration, launch routing and image handling without devices."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml
from cv_bridge import CvBridge
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from sensor_msgs.msg import Image

from bimanual_manipulation.cameras import CAMERA_ROLES, enabled_roles, load_camera_config
from bimanual_manipulation.tactile_wrench_viewer import SensorData, parse_args, render, subscribe

ROOT = Path(__file__).parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    return module, context


def write_config(tmp_path, **overrides):
    config = {role: {'serial_no': str(100 + i)} for i, role in enumerate(CAMERA_ROLES)}
    config.update(overrides)
    path = tmp_path / 'cameras.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


def test_disabled_cameras_need_neither_config_nor_driver(monkeypatch):
    module, ctx = load_launch('cameras.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: pytest.fail(name))
    assert module._cameras(ctx) == []
    assert load_camera_config('/does/not/exist', ()) == {}
    assert all(v['serial_no'] == '' for v in yaml.safe_load((ROOT / 'config/cameras.yaml').read_text()).values())


@pytest.mark.parametrize('entry,match', [
    ({'serial_no': ''}, 'nonempty'),
    ({'serial_no': 123}, 'quoted'),
    ({'serial_no': '101'}, 'duplicate'),
    ({'serial_no': '123', 'color_profile': '640,480,30'}, 'WIDTHxHEIGHTxFPS'),
    ({'serial_no': '123', 'enable_depth': 'false'}, 'boolean'),
    ({'serial_no': '123', 'colour_profile': '640x480x30'}, 'unknown'),
])
def test_invalid_enabled_config(tmp_path, entry, match):
    path = write_config(tmp_path, wrist_left=entry)
    with pytest.raises(ValueError, match=match):
        load_camera_config(path, enabled_roles(True))
    # A broken disabled role cannot block the independent fixed camera.
    assert list(load_camera_config(path, enabled_roles(False, True))) == ['fixed']


@pytest.mark.parametrize('wrist,fixed', [(False, False), (True, False), (False, True), (True, True)])
@pytest.mark.parametrize('name', ['bimanual_system.launch.py', 'single_arm_hand.launch.py'])
def test_system_routes_camera_flags(monkeypatch, name, wrist, fixed):
    module, ctx = load_launch(name)
    assert ctx.launch_configurations['enable_wrist_camera'] == 'false'
    assert ctx.launch_configurations['enable_fix_camera'] == 'false'
    ctx.launch_configurations.update(enable_wrist_camera=str(wrist).lower(),
        enable_fix_camera=str(fixed).lower(), camera_config='/custom/cameras.yaml')
    calls = {}
    def include(package, filename, arguments):
        calls[filename] = arguments
        return []
    monkeypatch.setattr(module, '_include', include)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    viewer = module._system(ctx)[-1]
    assert ('--enable-wrist-camera' in viewer['arguments']) == wrist
    assert ('--enable-fix-camera' in viewer['arguments']) == fixed
    assert ('cameras.launch.py' in calls) == (wrist or fixed)
    if wrist or fixed:
        assert calls['cameras.launch.py']['camera_config'] == '/custom/cameras.yaml'


def test_camera_launch_isolates_roles_and_passes_profiles(tmp_path, monkeypatch):
    module, ctx = load_launch('cameras.launch.py')
    path = write_config(tmp_path, fixed={'serial_no': 'fixed123', 'color_profile': '1280x720x15'})
    ctx.launch_configurations.update(enable_wrist_camera='true', enable_fix_camera='true', camera_config=str(path))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: '/fake/share')
    monkeypatch.setattr(module, 'GroupAction', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda source, launch_arguments: dict(launch_arguments))
    groups = module._cameras(ctx)
    assert len(groups) == 3
    for role, group in zip(CAMERA_ROLES, groups):
        assert group['scoped'] and not group['forwarding']
        params = group['actions'][0]
        assert params['camera_name'] == role
        assert params['camera_namespace'] == '/camera'
        assert params['serial_no'].startswith('_')
        assert params['enable_depth'] == 'false'
    assert groups[2]['actions'][0]['depth_module.color_profile'] == '1280x720x15'
    assert groups[2]['actions'][0]['rgb_camera.color_profile'] == '1280x720x15'


@pytest.mark.parametrize('fixed', [False, True])
def test_color_conversion_status_recovery_and_layout(fixed):
    data = SensorData(enabled_roles(True, fixed))
    bridge = CvBridge()
    rgb = np.zeros((30, 40, 3), np.uint8)
    rgb[:] = [255, 0, 0]
    data.receive_camera('wrist_left', bridge.cv2_to_imgmsg(rgb, 'rgb8'), 10.)
    data.receive_camera('wrist_right', bridge.cv2_to_imgmsg(rgb, 'bgr8'), 10.)
    assert list(data.cameras['wrist_left'][0][0, 0]) == [0, 0, 255]
    assert list(data.cameras['wrist_right'][0][0, 0]) == [255, 0, 0]
    status = data.status(10.1, 1.)
    assert status['camera/wrist_left'] == 'LIVE'
    assert status['camera/fixed'] == ('NO DATA' if fixed else 'DISABLED')
    if fixed:
        data.receive_camera('fixed', bridge.cv2_to_imgmsg(rgb, 'rgb8'), 10.)
        assert data.status(10.1, 1.)['camera/fixed'] == 'LIVE'
    live = render(data, now=10.1)
    assert live.shape == (870, 1880, 3)
    assert np.array_equal(live[:, :1400], render(SensorData(), now=10.1))
    assert data.status(12., 1.)['camera/wrist_left'] == 'STALE'
    assert not np.array_equal(live[:, 1400:], render(data, now=12.)[:, 1400:])
    data.receive_camera('wrist_left', Image(height=2, width=2, encoding='rgb8', step=6, data=[0]), 12.)
    assert data.status(12., 1.)['camera/wrist_left'] == 'INVALID DATA'
    render(data, now=12.)
    data.receive_camera('wrist_left', bridge.cv2_to_imgmsg(rgb, 'rgb8'), 13.)
    assert data.status(13., 1.)['camera/wrist_left'] == 'LIVE'
    assert data.status(13., 1.)['camera/wrist_right'] == 'STALE'


def test_camera_subscriptions_and_cli_flags():
    assert not parse_args([]).enable_wrist_camera
    assert parse_args(['--enable-fix-camera']).enable_fix_camera
    class Node:
        def __init__(self):
            self.topics = []
        def create_subscription(self, message_type, topic, callback, qos):
            self.topics.append(topic)
            assert qos.depth == 1
    for wrist, fixed in ((False, False), (True, False), (False, True), (True, True)):
        roles = enabled_roles(wrist, fixed)
        data, node = SensorData(roles), Node()
        subscribe(node, data)
        assert [t for t in node.topics if t.startswith('/camera/')] == [
            f'/camera/{role}/color/image_raw' for role in roles]
        for role in roles:
            assert data.status(1., 1.)[f'camera/{role}'] == 'NO DATA'
