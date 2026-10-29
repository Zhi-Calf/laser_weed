"""Verify launch overlay ordering without requiring ROS on the development laptop."""
import importlib.util
from pathlib import Path
import sys
import types

import pytest


def load(monkeypatch):
    class Configuration:
        def __init__(self,name): self.name=name
        def perform(self,context): return context[self.name]
    stubs={
        'ament_index_python':{},
        'ament_index_python.packages':{'get_package_share_directory':lambda _: '/share'},
        'launch':{'LaunchDescription':list},
        'launch.actions':{'DeclareLaunchArgument':lambda *a,**k:None,'OpaqueFunction':lambda **k:None},
        'launch.substitutions':{'LaunchConfiguration':Configuration},
        'launch_ros':{},'launch_ros.actions':{'Node':lambda **kwargs:kwargs}}
    for name,attrs in stubs.items():
        module=types.ModuleType(name);module.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,module)
    path=Path(__file__).parents[3]/'crop_row_perception/launch/observer.launch.py'
    spec=importlib.util.spec_from_file_location('observer_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_overlay_preserves_base_config_and_is_applied_last(tmp_path,monkeypatch):
    module=load(monkeypatch)
    overlay=tmp_path/'published.yaml';overlay.write_text('')
    assert module.observer_nodes({'config':'base.yaml','calibration':''})[0]['parameters']==['base.yaml']
    assert module.observer_nodes({'config':'base.yaml','calibration':str(overlay)})[0]['parameters']==['base.yaml',str(overlay)]
    with pytest.raises(RuntimeError): module.observer_nodes({'config':'base.yaml','calibration':'missing.yaml'})
