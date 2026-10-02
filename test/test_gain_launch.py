"""Construct launch descriptions only; no processes or hardware are started."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import pytest


@pytest.mark.parametrize('kp_ratio,kd_ratio', [('', ''), ('0.6', '0.8'), ('', '1.0')])
def test_startup_gain_ratio_forwarded_only_to_hands(monkeypatch, kp_ratio, kd_ratio):
    path = Path(__file__).parents[1] / 'launch/bimanual_system.launch.py'
    spec = importlib.util.spec_from_file_location('bimanual_gain_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    context.launch_configurations.update(mit_kp_ratio=kp_ratio, mit_kd_ratio=kd_ratio)
    calls = {}

    def include(package, filename, arguments):
        calls[package] = arguments
        return []

    monkeypatch.setattr(module, '_include', include)
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    module._system(context)
    assert calls['dual_sharpa_wave']['mit_kp_ratio'] == kp_ratio
    assert calls['dual_sharpa_wave']['mit_kd_ratio'] == kd_ratio
    assert 'mit_kp_ratio' not in calls['dual_crx_control']
    assert 'mit_kd_ratio' not in calls['dual_crx_control']
