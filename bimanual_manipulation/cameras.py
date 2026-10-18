"""Camera roles and validated, model-independent RealSense configuration."""

from pathlib import Path
import re

import yaml

CAMERA_ROLES = ('wrist_left', 'wrist_right', 'fixed')
CAMERA_LABELS = ('LEFT WRIST', 'RIGHT WRIST', 'FIXED')


def enabled_roles(wrist=False, fixed=False):
    return tuple(role for role in CAMERA_ROLES
                 if (fixed if role == 'fixed' else wrist))


def image_topic(role):
    return f'/camera/{role}/color/image_raw'


def load_camera_config(path, roles):
    """Validate only enabled roles. Never fall back to an arbitrary USB device."""
    if not roles:
        return {}
    try:
        config = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot read camera_config {path}: {error}') from error
    if not isinstance(config, dict):
        raise ValueError('camera_config must contain a mapping of camera roles')
    result, serials = {}, set()
    allowed = {'serial_no', 'color_profile', 'depth_profile', 'enable_depth'}
    for role in roles:
        entry = config.get(role)
        if not isinstance(entry, dict):
            raise ValueError(f'{role}: missing camera configuration')
        unknown = set(entry) - allowed
        if unknown:
            raise ValueError(f'{role}: unknown settings: {sorted(unknown)}')
        serial = entry.get('serial_no', '')
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f'{role}: serial_no must be a nonempty quoted string in {path}')
        serial = serial.strip()
        if not re.fullmatch(r'[A-Za-z0-9]+', serial):
            raise ValueError(f'{role}: serial_no must contain only letters/digits (no underscore prefix)')
        if serial in serials:
            raise ValueError(f'{role}: duplicate serial_no {serial}')
        serials.add(serial)
        settings = {'serial_no': serial}
        for key in ('color_profile', 'depth_profile'):
            profile = entry.get(key, '640x480x30')
            if not isinstance(profile, str) or not re.fullmatch(r'[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*', profile):
                raise ValueError(f'{role}: {key} must be WIDTHxHEIGHTxFPS, e.g. "640x480x30"')
            settings[key] = profile
        depth = entry.get('enable_depth', False)
        if not isinstance(depth, bool):
            raise ValueError(f'{role}: enable_depth must be a YAML boolean')
        settings['enable_depth'] = depth
        result[role] = settings
    return result
