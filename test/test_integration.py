"""Offline checks for integration-specific behavior; no ROS node or SDK connection."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time

from bimanual_manipulation import move_to_default_pose as move
from bimanual_manipulation.composite_description import build_description
from bimanual_manipulation.tactile_wrench_viewer import SensorData, bar_extent, render
from dual_sharpa_wave.joint_names import joint_names
from dual_sharpa_wave.tactile import FINGERS, to_image
from dual_sharpa_wave.tactile_viewer import force_heatmap


def test_composite_names_and_mounts():
    robot = ET.fromstring(build_description())
    movable = {j.get('name') for j in robot.findall('joint') if j.get('type') == 'revolute'}
    expected = {f'{side}_J{i}' for side in ('left', 'right') for i in range(1, 7)}
    expected.update(name for side in ('left', 'right') for name in joint_names(side))
    assert movable == expected
    for side in ('left', 'right'):
        mount = robot.find(f"joint[@name='{side}_arm_to_hand']")
        assert mount.find('parent').get('link') == f'{side}_ee_mount'
        assert mount.find('child').get('link') == f'{side}_hand_flange'


def test_preview_and_failure(tmp_path):
    args = argparse.Namespace(execute=False, arm_rate=None, hand_rate=None)
    commands = move.commands(args)
    assert all('--execute' not in command for command in commands)
    preview = subprocess.run([sys.executable, '-m', move.__name__], capture_output=True, text=True)
    assert preview.returncode == 0, preview.stderr
    assert preview.stdout.count('Preview only') == 2
    marker = tmp_path / 'should_not_run'
    assert move.run([[sys.executable, '-c', 'raise SystemExit(7)'],
                     [sys.executable, '-c', f'open({str(marker)!r}, "w").close()']]) == 7
    assert not marker.exists()


def test_interrupt_stops_child(tmp_path):
    # Exercise real process-group cleanup using a harmless sleeping Python child.
    marker = tmp_path / 'child_pid'
    code = (f'import os,time; from pathlib import Path; '
            f'Path({str(marker)!r}).write_text(str(os.getpid())); time.sleep(60)')
    parent_code = (
        'import sys; from bimanual_manipulation.move_to_default_pose import run\n'
        'try:\n'
        f'    run([[sys.executable, "-c", {code!r}]])\n'
        'except KeyboardInterrupt:\n'
        '    raise SystemExit(130)\n')
    parent = subprocess.Popen([sys.executable, '-c', parent_code])
    try:
        deadline = time.monotonic() + 5
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert marker.exists()
        pid = int(marker.read_text())
        parent.send_signal(signal.SIGINT)
        assert parent.wait(timeout=10) == 130
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()


def populated():
    data = SensorData()
    for side in ('left', 'right'):
        message = WrenchStamped()
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = -25., 0., 80.
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = -2., 1., 7.
        data.receive_wrench(side, message, 10.)
        array = np.zeros((60, 60, 3), np.float32)
        array[:, :, 0] = np.linspace(0, 15, 60)[None, :]
        for finger in FINGERS:
            data.tactile.receive((side, finger, 'DIST_FORCE'), to_image(array, Time(sec=10)), 10.)
            data.receive_f6(side, finger, Float64MultiArray(data=[-3., 4., 0., 99., 99., 99.]), 10.)
    return data


def test_signed_bars_and_shared_tactile_scale():
    assert [bar_extent(v, 50, 100) for v in (-100, -25, 0, 25, 100)] == [-100, -50, 0, 50, 100]
    array = np.array([[[3., 4., 0.], [0., 0., 15.]]], dtype=np.float32)
    expected = cv2.applyColorMap(np.array([[85, 255]], np.uint8), cv2.COLORMAP_JET)
    assert np.array_equal(force_heatmap(array, 15), expected)
    data = populated()
    assert set(data.status(10.1, 1.).values()) == {'LIVE'}
    live = render(data, now=10.1)
    stale = render(data, now=12.)
    assert live.shape == (870, 1400, 3)
    assert set(data.status(12., 1.).values()) == {'STALE'}
    assert not np.array_equal(live, stale)
    # Same tactile sample in every tile must have the same colors on both hands.
    reference = live[342:522, 20:284]
    for row in range(2):
        for col in range(5):
            assert np.array_equal(reference, live[342+row*265:522+row*265, 20+col*276:284+col*276])


def test_missing_and_invalid_data():
    data = SensorData()
    assert set(data.status(1., 1.).values()) == {'NO DATA'}
    assert render(data, now=1.).shape == (870, 1400, 3)
    message = WrenchStamped()
    message.wrench.force.x = float('nan')
    data.receive_wrench('left', message, 1.)
    bad = to_image(np.zeros((2, 2), np.uint8), Time(sec=1))
    data.tactile.receive(('left', 'thumb', 'DIST_FORCE'), bad, 1.)
    assert data.status(1., 1.)['left/wrench'] == 'INVALID DATA'
    assert data.status(1., 1.)['left/thumb'] == 'INVALID DATA'
    render(data, now=1.)


def test_resultant_magnitude_and_independent_freshness():
    data = populated()
    assert data.resultants['left', 'thumb'][0] == 5.
    data.receive_f6('left', 'thumb', Float64MultiArray(data=[0.] * 6), 12.)
    assert data.resultants['left', 'thumb'][0] == 0.
    status = data.status(12., 1.)
    assert status['left/thumb/resultant'] == 'LIVE'
    assert status['left/thumb'] == 'STALE'
    for bad in ([1.] * 5, [float('nan')] * 6, [float('inf')] * 6):
        data.receive_f6('left', 'thumb', Float64MultiArray(data=bad), 12.)
        assert data.status(12., 1.)['left/thumb/resultant'] == 'INVALID DATA'
        render(data, now=12.)
    data.receive_f6('left', 'thumb', Float64MultiArray(data=[30., 40., 0., 0., 0., 0.]), 13.)
    assert data.resultants['left', 'thumb'][0] == 50.
    assert data.status(13., 1.)['left/thumb/resultant'] == 'LIVE'
    render(data, now=13., resultant_force_max=5.)


def test_readonly_driver_description():
    from ament_index_python.packages import get_package_share_directory
    from dual_crx_control.robot.description import arm_description
    xacro = Path(get_package_share_directory('fanuc_hardware_interface')) / 'robot/crx5ia.urdf.xacro'
    for side in ('left', 'right'):
        root = ET.fromstring(arm_description(str(xacro), side, '192.0.2.1', False, read_only=True))
        assert root.find("ros2_control/hardware/param[@name='motion_control']").text == '0'
        assert root.find("ros2_control/hardware/plugin").text == 'fanuc_robot_driver/FanucHardwareInterface'
