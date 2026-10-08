"""Installed arm + hand mock bringup; enable with SINGLE_SIDE_ROS_MOCK=1."""

from contextlib import contextmanager
import os
import re
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from dual_sharpa_wave.joint_names import joint_names

pytestmark = pytest.mark.skipif(os.environ.get('SINGLE_SIDE_ROS_MOCK') != '1',
                                reason='set SINGLE_SIDE_ROS_MOCK=1 for software mock launch checks')


@contextmanager
def stack(tmp_path, side='left', read_only=False, dual=False, viewer=False, rviz=False):
    filename = 'bimanual_system.launch.py' if dual else 'single_arm_hand.launch.py'
    log = tmp_path / 'launch.log'
    env = dict(os.environ, ROS_DOMAIN_ID='202', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    with log.open('w') as stream:
        process = subprocess.Popen([
            'ros2', 'launch', 'bimanual_manipulation', filename, 'crx_mock:=true', 'sharpa_backend:=mock',
            f'prefix:={side}_', f'rviz:={str(rviz).lower()}', f'viewer:={str(viewer).lower()}',
            f'read_only:={str(read_only).lower()}'], env=env, stdout=stream,
            stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(domain_id=202)
    node = Node('single_side_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def wait(predicate, timeout=25., allow_exit=False):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)
            if predicate():
                return
            if not allow_exit:
                assert process.poll() is None, log.read_text()
        pytest.fail(log.read_text())
    try:
        yield node, wait, process, log
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


def isolated(node, side):
    other = 'right' if side == 'left' else 'left'
    for name, _ in node.get_topic_names_and_types() + node.get_service_names_and_types():
        assert f'/crx5ia/{other}/' not in name and f'/sharpa/{other}_hand/' not in name
    assert not any(namespace in (f'/crx5ia/{other}', f'/sharpa/{other}_hand')
                   for _, namespace in node.get_node_names_and_namespaces())


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('read_only', [False, True])
def test_single_system_feedback_motion_tf_and_stop(tmp_path, side, read_only):
    with stack(tmp_path, side, read_only) as (node, wait, process, log):
        combined, arm, hand, states = [], [], [], []
        transforms = {}
        node.create_subscription(JointState, '/bimanual/joint_states', combined.append, 10)
        node.create_subscription(JointState, '/crx5ia/joint_states', arm.append, 10)
        node.create_subscription(JointState, f'/sharpa/{side}_hand/joint_states', hand.append, 10)
        def tf(message):
            transforms.update({t.child_frame_id: t for t in message.transforms})
        node.create_subscription(TFMessage, '/tf', tf, 100)
        node.create_subscription(TFMessage, '/tf_static', tf,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        root = f'/crx5ia/{side}/controller_manager'
        controllers = node.create_client(ListControllers, root + '/list_controllers')
        hardware = node.create_client(ListHardwareComponents, root + '/list_hardware_components')
        force = [0.]
        if not read_only:
            node.create_subscription(String, '/crx5ia/collision_force_limiter/state',
                                     lambda msg: states.append(msg.data),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            # Inject only into this isolated mock graph; the mock sensor also publishes zero.
            pub_force = node.create_publisher(WrenchStamped, f'/crx5ia/{side}/force_torque_sensor_broadcaster/wrench', 1)
            def wrench():
                msg = WrenchStamped()
                msg.wrench.force.x = force[0]
                pub_force.publish(msg)
            node.create_timer(.01, wrench)
        wait(lambda: combined and len(combined[-1].position) == 28
             and arm and len(arm[-1].position) == 6 and hand and controllers.service_is_ready()
             and f'{side}_hand_flange' in transforms and f'{side}_tcp' in transforms)
        arm_names = [f'{side}_J{i}' for i in range(1, 7)]
        expected = set(arm_names) | set(joint_names(side))
        assert len(combined[-1].position) == 28 and set(combined[-1].name) == expected
        assert arm[-1].name == arm_names and hand[-1].name == list(joint_names(side))
        isolated(node, side)
        other = 'right' if side == 'left' else 'left'
        assert not any(name.startswith(other + '_') for name in transforms)
        for topic in ('/tf', '/tf_static'):
            assert {(e.node_name, e.node_namespace) for e in node.get_publishers_info_by_topic(topic)} == {
                ('robot_state_publisher', '/bimanual')}
        def query(client, service):
            future = client.call_async(service.Request())
            wait(future.done)
            return future.result()
        def active():
            return {c.name for c in query(controllers, ListControllers).controller if c.state == 'active'}
        required = {'joint_state_broadcaster', 'force_torque_sensor_broadcaster'}
        wait(lambda: required.issubset(active()))
        if read_only:
            assert 'forward_position_controller' not in active()
            assert 'joint_interpolation' not in node.get_node_names()
            assert 'collision_force_limiter' not in node.get_node_names()
            assert node.count_subscribers(f'/sharpa/{side}_hand/joint_command') == 0
            assert node.count_publishers('/crx5ia/joint_targets') == 0
        else:
            wait(lambda: states and states[-1] == 'ARMED')
            assert 'forward_position_controller' in active()
            old_tf = {key: t.transform for key, t in transforms.items()}
            arm_target = list(arm[-1].position)
            arm_target[0] += .001
            hand_target = [.01] + [0.] * 21
            pub_arm = node.create_publisher(JointState, '/crx5ia/joint_targets', 1)
            pub_hand = node.create_publisher(JointState, f'/sharpa/{side}_hand/joint_command', 1)
            wait(lambda: pub_arm.get_subscription_count() > 0 and pub_hand.get_subscription_count() == 1)
            pub_arm.publish(JointState(name=arm_names, position=arm_target))
            hand_timer = node.create_timer(.05, lambda: pub_hand.publish(
                JointState(name=list(joint_names(side)), position=hand_target)))
            wait(lambda: np.allclose(arm[-1].position, arm_target, atol=1e-8, rtol=0)
                 and list(hand[-1].position) == hand_target)
            def merged():
                by_name = dict(zip(combined[-1].name, combined[-1].position))
                return (np.allclose([by_name[n] for n in arm_names], arm_target, atol=1e-8, rtol=0)
                        and [by_name[n] for n in joint_names(side)] == hand_target)
            wait(merged)
            wait(lambda: any(t.transform != old_tf.get(n) for n, t in transforms.items()))
            hand_timer.cancel()
            force[0] = 51.
            wait(lambda: 'TRIPPED' in states)
            wait(lambda: 'Selected hardware component confirmed inactive' in log.read_text())
            assert 'forward_position_controller' not in active()
            assert any(c.name == 'crx5ia' and c.state.id == 2 for c in query(hardware, ListHardwareComponents).component)
            before = len(hand)
            wait(lambda: len(hand) > before + 3)
            assert list(hand[-1].position) == hand_target
        assert 'Traceback' not in log.read_text() and 'process has died' not in log.read_text()
        assert process.poll() is None
        isolated(node, side)


def test_dual_system_still_merges_both_sides(tmp_path):
    with stack(tmp_path, dual=True, read_only=True) as (node, wait, _, log):
        received = []
        node.create_subscription(JointState, '/bimanual/joint_states', received.append, 10)
        wait(lambda: received and len(received[-1].position) == 56)
        wait(lambda: all(('hand_node', f'/sharpa/{s}_hand') in node.get_node_names_and_namespaces()
                        and ('controller_manager', f'/crx5ia/{s}') in node.get_node_names_and_namespaces()
                        for s in ('left', 'right')))
        expected = {f'{s}_J{i}' for s in ('left', 'right') for i in range(1, 7)}
        expected.update(n for s in ('left', 'right') for n in joint_names(s))
        assert set(received[-1].name) == expected
        assert 'Traceback' not in log.read_text()


def test_optional_viewer_does_not_start_inactive_drivers(tmp_path):
    with stack(tmp_path, viewer=True, read_only=True) as (node, wait, _, log):
        wait(lambda: 'tactile_wrench_viewer' in node.get_node_names()
             and node.count_subscribers('/crx5ia/right/force_torque_sensor_broadcaster/wrench') > 0)
        assert node.count_publishers('/crx5ia/right/force_torque_sensor_broadcaster/wrench') == 0
        assert ('hand_node', '/sharpa/right_hand') not in node.get_node_names_and_namespaces()
        assert 'process has died' not in log.read_text()


def test_limiter_exit_shuts_down_included_system(tmp_path):
    with stack(tmp_path) as (node, wait, process, log):
        feedback = []
        node.create_subscription(JointState, '/bimanual/joint_states', feedback.append, 10)
        wait(lambda: feedback and 'collision_force_limiter' in node.get_node_names())
        match = re.search(r'\[collision_force_limiter.py-\d+\]: process started with pid \[(\d+)\]', log.read_text())
        assert match, log.read_text()
        os.kill(int(match[1]), signal.SIGTERM)
        wait(lambda: process.poll() is not None, allow_exit=True)
        wait(lambda: ('hand_node', '/sharpa/left_hand') not in node.get_node_names_and_namespaces()
             and ('controller_manager', '/crx5ia/left') not in node.get_node_names_and_namespaces(),
             allow_exit=True)


def test_selected_combined_model_in_rviz(tmp_path):
    with stack(tmp_path, side='right', read_only=True, rviz=True) as (node, wait, _, log):
        received = []
        node.create_subscription(JointState, '/bimanual/joint_states', received.append, 10)
        wait(lambda: received and len(received[-1].position) == 28
             and 'bimanual_rviz' in node.get_node_names())
        wait(lambda: node.count_subscribers('/bimanual/robot_description') >= 2)
        isolated(node, 'right')
        assert 'process has died' not in log.read_text()


@pytest.mark.parametrize('side,selector', [('left', '1'), ('right', '2')])
def test_keyboard_jog_uses_default_quintic(tmp_path, side, selector):
    from dual_crx_control.teleop.keyboard import KeyboardControl, MOVES
    with stack(tmp_path, side=side) as (node, wait, _, log):
        key = ['']
        def read_key():
            value, key[0] = key[0], ''
            return value
        keyboard = KeyboardControl(read_key)
        feedback, targets = [], []
        node.create_subscription(JointState, f'/crx5ia/{side}/joint_states', feedback.append, 10)
        node.create_subscription(JointState, '/crx5ia/joint_targets', targets.append, 10)
        def keyboard_wait(predicate):
            def poll():
                rclpy.spin_once(keyboard, timeout_sec=.001)
                return predicate()
            wait(poll)
        def press(value):
            key[0] = value
            keyboard_wait(lambda: key[0] == '')
        try:
            press(selector)
            keyboard_wait(lambda: keyboard.readiness_error() is None and feedback)
            assert keyboard.quintic_ready
            assert set(keyboard.positions) == {side}
            press('e')
            assert keyboard.jog is not None
            start_pose = keyboard.jog.pose.copy()
            for direction in MOVES:
                try:
                    keyboard.jog.candidate(direction, keyboard.positions[side])
                    break
                except ValueError:
                    continue
            else:
                pytest.fail(f'No valid mock jog direction for {side}')
            press(direction)
            keyboard_wait(lambda: targets)
            assert targets[-1].name == [f'{side}_J{i}' for i in range(1, 7)]
            goal = np.array(targets[-1].position)
            press(' ')
            assert keyboard.jog is None
            keyboard_wait(lambda: np.allclose(feedback[-1].position, goal, atol=1e-7, rtol=0))
            final_pose = keyboard.models[side].fk(goal)
            assert np.linalg.norm(final_pose[:3, 3] - start_pose[:3, 3]) == pytest.approx(.001, abs=2e-5)
            other = 'right' if side == 'left' else 'left'
            press('2' if other == 'right' else '1')
            press('e')
            assert keyboard.jog is None
            assert keyboard.readiness_error() == 'stale/missing feedback'
            assert node.count_publishers(f'/crx5ia/{other}/joint_states') == 0
            assert 'process has died' not in log.read_text()
        finally:
            keyboard.destroy_node()
