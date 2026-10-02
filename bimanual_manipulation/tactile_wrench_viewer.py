"""ROS subscriptions only: wrench bars, tactile heatmaps and F6 magnitude bars."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from dual_sharpa_wave.tactile import FINGERS, f6_array
from dual_sharpa_wave.tactile_viewer import DisplayData, force_heatmap, text

SIDES = ('left', 'right')
LABELS = ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')
WINDOW = 'Bimanual | wrench and tactile'
WIDTH, HEIGHT = 1400, 870
TILE_W, TILE_H = 264, 180
HEAT_W = 180
BG, PANEL = (24, 26, 30), (37, 40, 45)


class SensorData:
    def __init__(self):
        self.tactile = DisplayData()
        self.wrenches = {}
        self.errors = {}
        self.resultants = {}
        self.resultant_errors = set()

    def receive_f6(self, side, finger, message, now):
        key = (side, finger)
        try:
            # Provisional SDK convention: first three F6 components are force.
            # Keep SDK units until the vendor's physical definition is confirmed.
            value = math.hypot(*f6_array(message.data)[:3])
            if not math.isfinite(value):
                raise ValueError('nonfinite resultant')
        except (ValueError, TypeError):
            self.resultant_errors.add(key)
            return
        self.resultants[key] = (value, now)
        self.resultant_errors.discard(key)

    def receive_wrench(self, side, message, now):
        wrench = message.wrench
        values = tuple(getattr(vector, axis) for vector in (wrench.force, wrench.torque)
                       for axis in ('x', 'y', 'z'))
        if not all(math.isfinite(v) for v in values):
            self.errors[side] = 'nonfinite wrench'
            return
        self.wrenches[side] = (values, now)
        self.errors.pop(side, None)

    def status(self, now, stale_after):
        result = {}
        for side in SIDES:
            sample = self.wrenches.get(side)
            result[f'{side}/wrench'] = (
                'INVALID DATA' if side in self.errors else
                'NO DATA' if sample is None else
                'STALE' if now - sample[1] > stale_after else 'LIVE')
            for finger in FINGERS:
                resultant = self.resultants.get((side, finger))
                result[f'{side}/{finger}/resultant'] = (
                    'INVALID DATA' if (side, finger) in self.resultant_errors else
                    'NO DATA' if resultant is None else
                    'STALE' if now - resultant[1] > stale_after else 'LIVE')
                key = (side, finger, 'DIST_FORCE')
                sample = self.tactile.samples.get(key)
                result[f'{side}/{finger}'] = (
                    'INVALID DATA' if key in self.tactile.errors else
                    'NO DATA' if sample is None else
                    'STALE' if now - sample.received > stale_after else 'LIVE')
        return result


def bar_extent(value, maximum, half_width):
    return round(np.clip(value / maximum, -1., 1.) * half_width)


def render(data, *, now, force_max=15., resultant_force_max=15.,
           wrench_force_max=10., wrench_torque_max=5., stale_after=1.):
    canvas = np.full((HEIGHT, WIDTH, 3), BG, np.uint8)
    status = data.status(now, stale_after)
    text(canvas, 'BIMANUAL | force / torque and fingertip force magnitude', 20, 27, scale=.65)
    for side_index, side in enumerate(SIDES):
        x = 20 + side_index * 700
        state = status[f'{side}/wrench']
        text(canvas, f'{side.upper()} ARM  {state}', x, 58, scale=.55)
        sample = data.wrenches.get(side)
        for index, label in enumerate(LABELS):
            y = 83 + index * 30
            maximum = wrench_force_max if index < 3 else wrench_torque_max
            unit = 'N' if index < 3 else 'N m'
            center, half = x + 310, 170
            cv2.rectangle(canvas, (center - half, y - 12), (center + half, y + 6), PANEL, -1)
            text(canvas, label, x, y + 2, scale=.5)
            if sample is not None and state != 'INVALID DATA':
                value = sample[0][index]
                extent = bar_extent(value, maximum, half)
                level = np.uint8([[round(min(abs(value) / maximum, 1.) * 255)]])
                color = tuple(int(c) for c in cv2.applyColorMap(level, cv2.COLORMAP_JET)[0, 0])
                if state != 'LIVE':
                    color = tuple(c // 4 for c in color)
                cv2.rectangle(canvas, (min(center, center + extent), y - 12),
                              (max(center, center + extent), y + 6), color, -1)
                suffix = ' !' if abs(value) > maximum else ''
                text(canvas, f'{value:+.3f} {unit}{suffix}', x + 505, y + 2, scale=.45)
            cv2.line(canvas, (center, y - 15), (center, y + 8), (180, 180, 180), 1)
        text(canvas, f'Bar ranges: +/-{wrench_force_max:g} N; +/-{wrench_torque_max:g} N m',
             x, 272, scale=.43)

    for row, side in enumerate(SIDES):
        top = 295 + row * 265
        text(canvas, f'{side.upper()} HAND', 20, top + 17, scale=.55)
        for column, finger in enumerate(FINGERS):
            x = 20 + column * 276
            y = top + 47
            key = (side, finger, 'DIST_FORCE')
            sample = data.tactile.samples.get(key)
            state = status[f'{side}/{finger}']
            text(canvas, finger.upper(), x, top + 38, scale=.45)
            tile = np.full((TILE_H, HEAT_W, 3), PANEL, np.uint8)
            if sample is not None and state != 'INVALID DATA':
                picture = force_heatmap(sample.array, force_max)
                h, w = picture.shape[:2]
                ratio = min(HEAT_W / w, TILE_H / h)
                picture = cv2.resize(picture, (max(1, round(w * ratio)), max(1, round(h * ratio))),
                                     interpolation=cv2.INTER_NEAREST)
                h, w = picture.shape[:2]
                tile[(TILE_H-h)//2:(TILE_H-h)//2+h, (HEAT_W-w)//2:(HEAT_W-w)//2+w] = picture
                if state == 'STALE':
                    tile //= 4
            if state != 'LIVE':
                text(tile, state, 8, TILE_H // 2, (70, 150, 240), .5)
            canvas[y:y+TILE_H, x:x+HEAT_W] = tile
            bx = x + HEAT_W + 8
            force_state = status[f'{side}/{finger}/resultant']
            resultant = data.resultants.get((side, finger))
            text(canvas, 'F6 |F|', bx, y + 12, scale=.36)
            bar_top, bar_bottom = y + 35, y + 139
            cv2.rectangle(canvas, (bx + 19, bar_top), (bx + 39, bar_bottom), PANEL, -1)
            text(canvas, f'{resultant_force_max:g}', bx + 18, y + 28, scale=.3)
            text(canvas, '0', bx + 25, y + 152, scale=.3)
            if resultant is not None and force_state != 'INVALID DATA':
                value = resultant[0]
                extent = round(min(value / resultant_force_max, 1.) * (bar_bottom - bar_top))
                color = (80, 200, 240) if force_state == 'LIVE' else (40, 65, 70)
                if extent:
                    cv2.rectangle(canvas, (bx + 19, bar_bottom - extent + 1),
                                  (bx + 39, bar_bottom), color, -1)
                suffix = '!' if value > resultant_force_max else ''
                text(canvas, f'{value:.3g}{suffix}', bx, y + 166, color, .37)
            short_state = {'INVALID DATA': 'INVALID', 'NO DATA': 'NO DATA',
                           'STALE': 'STALE', 'LIVE': 'SDK units'}[force_state]
            text(canvas, short_state, bx, y + 179, scale=.3)
            age = f'  age {max(0., now - sample.received):.1f}s' if sample is not None else ''
            text(canvas, state + age, x, y + TILE_H + 19, scale=.4)
    ramp = np.tile(np.arange(256, dtype=np.uint8), (12, 1))
    canvas[840:852, 1110:1366] = cv2.applyColorMap(ramp, cv2.COLORMAP_JET)
    text(canvas, f'Tactile: 0 ... {force_max:g} (SDK units) | fixed shared color scale', 20, 850, scale=.48)
    text(canvas, 'q / Esc: close | subscriptions only', 665, 850, scale=.43)
    return canvas


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name, default in (('force-max', 15.), ('resultant-force-max', 15.), ('wrench-force-max', 10.),
                          ('wrench-torque-max', 5.), ('stale-after', 1.), ('duration', 5.)):
        parser.add_argument('--' + name, type=float, default=default)
    parser.add_argument('--snapshot', type=Path, help='Headless: save PNG and status JSON after --duration seconds')
    args = parser.parse_args(argv)
    for name in ('force_max', 'resultant_force_max', 'wrench_force_max', 'wrench_torque_max', 'stale_after', 'duration'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'{name} must be finite and positive')
    return args


def subscribe(node, data):
    from geometry_msgs.msg import WrenchStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    for side in SIDES:
        node.create_subscription(
            WrenchStamped, f'/crx5ia/{side}/force_torque_sensor_broadcaster/wrench',
            lambda msg, side=side: data.receive_wrench(side, msg, time.monotonic()), qos)
        for finger in FINGERS:
            node.create_subscription(
                Float64MultiArray, f'/sharpa/{side}_hand/tactile/{finger}/f6',
                lambda msg, side=side, finger=finger: data.receive_f6(
                    side, finger, msg, time.monotonic()), qos)
            key = (side, finger, 'DIST_FORCE')
            node.create_subscription(
                Image, f'/sharpa/{side}_hand/tactile/{finger}/dist_force',
                lambda msg, key=key: data.tactile.receive(key, msg, time.monotonic()), qos)


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    # launch_ros appends --ros-args, including the node-name remapping.
    cli = sys.argv if argv is None else [sys.argv[0], *argv]
    args = parse_args(remove_ros_args(args=cli)[1:])
    if args.snapshot is None and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('A desktop display is required; use --snapshot /path/view.png for headless capture.', file=sys.stderr)
        return 1
    rclpy.init(args=cli[1:])
    node = None
    data = SensorData()
    try:
        node = Node('tactile_wrench_viewer')
        subscribe(node, data)
        if args.snapshot is None:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, WIDTH, HEIGHT)
        began = time.monotonic()
        while rclpy.ok():
            deadline = time.monotonic() + .025
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.001)
            now = time.monotonic()
            if args.snapshot is not None and now - began < args.duration:
                continue
            frame = render(data, now=now, force_max=args.force_max,
                           resultant_force_max=args.resultant_force_max,
                           wrench_force_max=args.wrench_force_max,
                           wrench_torque_max=args.wrench_torque_max, stale_after=args.stale_after)
            if args.snapshot is not None:
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(args.snapshot), frame):
                    raise OSError(f'Could not write {args.snapshot}')
                status = data.status(now, args.stale_after)
                args.snapshot.with_suffix('.json').write_text(json.dumps(status, indent=2) + '\n')
                print(json.dumps(status, indent=2))
                return 0
            cv2.imshow(WINDOW, frame)
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print(f'Viewer failed: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if args.snapshot is None:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    raise SystemExit(main())
