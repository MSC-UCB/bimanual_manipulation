"""Build one URDF from the installed CRX xacro and both installed hand URDFs."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


def _mounts(path):
    config = yaml.safe_load(Path(path).read_text())
    result = {}
    for side in ('left', 'right'):
        values = {}
        for field in ('xyz', 'rpy'):
            raw = config[side][field]
            if not isinstance(raw, list) or len(raw) != 3:
                raise ValueError(f'{side}.{field} must be a list of three numbers')
            values[field] = tuple(float(value) for value in raw)
            if not all(math.isfinite(value) for value in values[field]):
                raise ValueError(f'{side}.{field} must contain finite numbers')
        result[side] = values
    return result


def _validate_tree(robot):
    links = [link.get('name') for link in robot.findall('link')]
    joints = robot.findall('joint')
    joint_names = [joint.get('name') for joint in joints]
    if None in links or len(links) != len(set(links)):
        raise ValueError('composite URDF has missing or duplicate link names')
    if None in joint_names or len(joint_names) != len(set(joint_names)):
        raise ValueError('composite URDF has missing or duplicate joint names')
    children = []
    for joint in joints:
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            raise ValueError(f'{joint.get("name")}: missing parent or child')
        if parent.get('link') not in links or child.get('link') not in links:
            raise ValueError(f'{joint.get("name")}: parent or child link is absent')
        children.append(child.get('link'))
    if len(children) != len(set(children)):
        raise ValueError('composite URDF has a link with multiple parents')
    if set(links) - set(children) != {'world'} or len(joints) != len(links) - 1:
        raise ValueError('composite URDF must have one world root and form a tree')
    movable = sum(joint.get('type') in ('revolute', 'continuous', 'prismatic') for joint in joints)
    if movable != 56:
        raise ValueError(f'expected 56 movable joints, found {movable}')


def build_description(mount_config=None):
    crx_share = Path(get_package_share_directory('dual_crx_control'))
    hand_share = Path(get_package_share_directory('dual_sharpa_wave'))
    own_share = Path(get_package_share_directory('bimanual_manipulation'))
    mounts = _mounts(mount_config or own_share / 'config' / 'hand_mounts.yaml')
    robot = ET.fromstring(xacro.process_file(str(crx_share / 'urdf' / 'dual_crx.urdf.xacro')).toxml())
    robot.set('name', 'bimanual_manipulation')
    for side in ('left', 'right'):
        hand = ET.parse(hand_share / 'urdf' / f'{side}_sharpa_wave_with_flange.urdf').getroot()
        for element in hand:
            if element.tag in ('link', 'joint', 'material'):
                robot.append(element)
        mount = ET.SubElement(robot, 'joint', name=f'{side}_arm_to_hand', type='fixed')
        ET.SubElement(mount, 'parent', link=f'{side}_ee_mount')
        ET.SubElement(mount, 'child', link=f'{side}_hand_flange')
        ET.SubElement(mount, 'origin',
                      xyz=' '.join(map(str, mounts[side]['xyz'])),
                      rpy=' '.join(map(str, mounts[side]['rpy'])))
    _validate_tree(robot)
    return ET.tostring(robot, encoding='unicode')
