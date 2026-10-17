"""The automatic announcement describes the destination, not actuator ramp steps."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def load_methods():
  source = Path(__file__).parents[1] / "cruise.py"
  tree = ast.parse(source.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "VCruiseCarrot")
  names = {"_apply_road_limit_set_speed", "_apply_road_limit_ramp", "_cancel_road_limit_ramp"}
  methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
  assert len(methods) == len(names)
  namespace = {"np": np}
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
  return namespace


@pytest.mark.parametrize("prefix", ["Auto road limit 80->50 =>", "Cruise speed set to road limit", "Speed camera zone"])
@pytest.mark.parametrize("start,target", [(80, 50), (30, 80), (50, 52)])
def test_announcement_does_not_change_ramp_or_set_speed(prefix, start, target):
  methods = load_methods()
  commands, resume, logs = [], [], []
  control = SimpleNamespace(
    frame=1000, _road_limit_ramp_target_kph=0, _road_limit_ramp_last_frame=-10000,
    _road_limit_ramp_interval_frames=30, _road_limit_ramp_step_kph=2,
    _cruise_speed_min=10, _cruise_speed_max=160,
    _road_limit_cluster_kph=lambda *args, **kwargs: target,
    _queue_pcm_set_speed=commands.append, _store_resume_cruise_speed=resume.append, _add_log=logs.append,
  )
  for name in ("_apply_road_limit_ramp", "_cancel_road_limit_ramp"):
    setattr(control, name, methods[name].__get__(control))
  result = methods["_apply_road_limit_set_speed"](control, SimpleNamespace(), start, prefix, road_limit_kph=50, smooth=True)
  expected = start + np.clip(target - start, -2, 2)
  assert result == expected
  assert commands == [expected] and resume == [expected]
  announced = target if prefix.startswith("Auto road limit ") else expected
  assert logs == [f"{prefix} 50->{announced:.0f}"]
