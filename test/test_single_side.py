"""Offline selected-side models and launch routing; no devices are started."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import pytest
import yaml

from bimanual_manipulation.composite_description import build_description
from dual_sharpa_wave.joint_names import joint_names

ROOT = Path(__file__).parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    return module, context


@pytest.mark.parametrize('side', ['left', 'right'])
def test_single_model_preserves_calibration_and_selected_mount(tmp_path, side):
    mounts = tmp_path / 'mount.yaml'
    mounts.write_text(yaml.safe_dump({side: {'xyz': [.01, .02, .03], 'rpy': [.1, .2, .3]}}))
    full = ET.fromstring(build_description())
    single = ET.fromstring(build_description(mounts, sides=side))
    joints = {j.get('name') for j in single.findall('joint') if j.get('type') == 'revolute'}
    assert joints == {f'{side}_J{i}' for i in range(1, 7)} | set(joint_names(side))
    assert len(joints) == 28
    other = 'right' if side == 'left' else 'left'
    assert not any(e.get('name', '').startswith(other + '_') for e in single)
    for name in (f'{side}_base_joint', f'{side}_ee_to_tcp', 'world_to_table', 'table_to_calibration_surface'):
        assert ET.tostring(single.find(f"joint[@name='{name}']")) == ET.tostring(full.find(f"joint[@name='{name}']"))
    origin = single.find(f"joint[@name='{side}_arm_to_hand']/origin")
    assert origin.get('xyz') == '0.01 0.02 0.03'
    assert origin.get('rpy') == '0.1 0.2 0.3'
    parent_of = {j.find('child').get('link'): j.find('parent').get('link') for j in single.findall('joint')}
    for link in parent_of:
        seen = set()
        while link != 'world':
            assert link not in seen
            seen.add(link)
            link = parent_of[link]


@pytest.mark.parametrize('selection', ['', 'left_', 'invalid'])
def test_bad_selection_rejected(selection):
    with pytest.raises(ValueError, match='sides'):
        build_description(sides=selection)


@pytest.mark.parametrize('selection', ['left', 'right', 'both'])
def test_description_sources_match_model(monkeypatch, selection):
    module, ctx = load_launch('bimanual_description.launch.py')
    ctx.launch_configurations['sides'] = selection
    monkeypatch.setattr(module, 'Node', lambda **kw: kw)
    merger, publisher, _ = module._nodes(ctx)
    sides = ('left', 'right') if selection == 'both' else (selection,)
    params = merger['parameters'][0]
    assert params['source_list'] == ([f'/crx5ia/{s}/joint_states' for s in sides]
                                      + [f'/sharpa/{s}_hand/joint_states' for s in sides])
    assert params['robot_description'] == publisher['parameters'][0]['robot_description']
    root = ET.fromstring(params['robot_description'])
    assert sum(j.get('type') == 'revolute' for j in root.findall('joint')) == 28 * len(sides)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('global_readonly,hand_readonly', [('false', 'false'), ('false', 'true'), ('true', 'false')])
def test_system_routes_side_readonly_and_overrides(monkeypatch, side, global_readonly, hand_readonly):
    module, ctx = load_launch('single_arm_hand.launch.py')
    assert ctx.launch_configurations['crx_mock'] == 'true'
    assert ctx.launch_configurations['sharpa_backend'] == 'mock'
    assert ctx.launch_configurations['viewer'] == 'false'
    ctx.launch_configurations.update(prefix=side + '_', read_only=global_readonly,
        sharpa_read_only=hand_readonly, method='ruckig', input_rate_hz='50.0',
        mit_kp_ratio='', mit_kd_ratio='0.8')
    calls = {}
    def include(package, filename, arguments):
        calls[package] = (filename, arguments)
        return []
    monkeypatch.setattr(module, '_include', include)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    monkeypatch.setattr(module, 'Node', lambda **kw: kw)
    groups = module._system(ctx)
    filename, arm = calls['dual_crx_control']
    assert filename == 'single_arm.launch.py'
    assert arm['prefix'] == side + '_' and arm['control'] == 'none'
    assert arm['read_only'] == global_readonly
    assert arm['method'] == 'ruckig' and arm['input_rate_hz'] == '50.0'
    assert arm['collision_force_threshold_n'] == '50.0'
    assert 'mit_kd_ratio' not in arm
    filename, hand = calls['dual_sharpa_wave']
    assert filename == 'single_sharpa.launch.py' and hand['prefix'] == arm['prefix']
    assert hand['read_only'] == ('true' if global_readonly == 'true' else hand_readonly)
    assert hand['mit_kp_ratio'] == '' and hand['mit_kd_ratio'] == '0.8'
    assert hand['use_rviz'] == arm['rviz'] == 'false'
    assert calls['bimanual_manipulation'][1]['sides'] == side
    assert any(type(a).__name__ == 'UnsetLaunchConfiguration' for a in groups[0])
    assert sum(type(a).__name__ == 'SetRemap' for a in groups[0]) == 2


def test_system_rejects_invalid_prefix():
    module, ctx = load_launch('single_arm_hand.launch.py')
    ctx.launch_configurations['prefix'] = 'both'
    with pytest.raises(ValueError, match='prefix'):
        module._system(ctx)
