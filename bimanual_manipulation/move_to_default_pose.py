"""One entry point: move both arms, then both hands; preview unless --execute."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys

from ament_index_python.packages import get_package_prefix


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return result


def commands(args):
    result = []
    for package, rate in (('dual_crx_control', args.arm_rate),
                          ('dual_sharpa_wave', args.hand_rate)):
        script = Path(get_package_prefix(package)) / 'lib' / package / 'move_to_default_pose.py'
        if not script.is_file():
            raise FileNotFoundError(f'{script}: rebuild and source {package}')
        command = [str(script)]
        if args.execute:
            command.append('--execute')
        if rate is not None:
            command.extend(['--rate', str(rate)])
        result.append(command)
    return result


def stop(child):
    if child.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            return
        try:
            child.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            pass


def run(sequence):
    for command in sequence:
        print('Running: ' + ' '.join(command), flush=True)
        child = subprocess.Popen(command, start_new_session=True)
        try:
            status = child.wait()
        finally:
            stop(child)
        if status:
            print('Default-pose step failed; no further step will start.', file=sys.stderr)
            return status if status > 0 else 128 - status
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Actually command arms, then hands')
    parser.add_argument('--arm-rate', type=positive, help='Match arm input_rate_hz; upstream default 100')
    parser.add_argument('--hand-rate', type=positive, help='Hand command rate; upstream default 100')
    args = parser.parse_args(argv)
    if (args.arm_rate is not None and args.arm_rate > 500 or
            args.hand_rate is not None and args.hand_rate > 100):
        parser.error('arm rate must be <= 500 Hz; hand rate must be <= 100 Hz')

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        return run(commands(args))
    except KeyboardInterrupt:
        print('Interrupted; stopped child command sender. This is not an emergency stop.', file=sys.stderr)
        return 130
    except (OSError, LookupError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    raise SystemExit(main())
