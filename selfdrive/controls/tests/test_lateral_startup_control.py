"""Run the production control loop with IPC, PID and longitudinal test doubles."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
from typing import SupportsFloat

import numpy as np
import pytest

from cereal import car, log
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate, lateral_vehicle_parameters
from openpilot.selfdrive.controls.tests.test_lateral_readiness import Messages


def load_state_control():
  path = Path(__file__).parents[1] / 'controlsd.py'
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'state_control')
  ns = dict(math=math, np=np, car=car, SupportsFloat=SupportsFloat, DT_CTRL=0.01,
            CV=NS(KPH_TO_MS=1 / 3.6), MIN_LATERAL_CONTROL_SPEED=0.3,
            LaneChangeState=log.LaneChangeState, LaneChangeDirection=log.LaneChangeDirection,
            ACTUATOR_FIELDS=tuple(car.CarControl.Actuators.schema.fields),
            lateral_vehicle_parameters=lateral_vehicle_parameters,
            clip_curvature=lambda v, prev, new, roll: (new, False),
            get_lag_adjusted_curvature=lambda cp, v, psis, curves, delay, distances: curves[0],
            cloudlog=NS(error=lambda msg: pytest.fail(msg)))
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
  return ns['state_control']


class Controller:
  def __init__(self):
    self.resets = 0
    self.curvatures = []

  def reset(self):
    self.resets += 1

  def update(self, active, cs, vm, lp, limited, curvature, *args, **kwargs):
    self.curvatures.append(curvature)
    return (0.2 if active else 0.0), cs.steeringAngleDeg, None


def fixture():
  sm = Messages()
  sm.update(longitudinalPlan=NS(), radarState=NS(), carrotMan=NS(vTurnSpeed=200),
            liveDelay=NS(lateralDelay=0.1))
  sm.frame = 1
  sm.recv_frame = {'longitudinalPlan': 1}
  sm['carState'].gearShifter = 'drive'
  sm['carState'].latEnabled = True
  sm['modelV2'].action.desiredCurvature = 0.01
  cp = car.CarParams.new_message(steerRatio=14.3, minSteerSpeed=0, openpilotLongitudinalControl=True)
  cp.lateralTuning.init('torque')
  vm = NS(sR=14.3, calc_curvature=lambda angle, v, roll: angle / 42.9)
  vm.update_params = lambda stiffness, ratio: setattr(vm, 'sR', ratio)
  settings = {'AlwaysLateral': 1, 'SteerRatioRate': 100, 'CustomSR': 0, 'LatSmoothSec': 6}
  params = NS(get_float=lambda k: float(settings.get(k, 0)), get_int=lambda k: int(settings.get(k, 0)),
              get_bool=lambda k: bool(settings.get(k, False)))
  control = NS(CP=cp, CI=NS(get_pid_accel_limits=lambda *args: (-3, 2)), sm=sm, params=params,
               VM=vm, LaC=Controller(), LoC=NS(reset=lambda: None, update=lambda *args: (0.7, 0.2, 0.1),
                                           long_control_state='pid'),
               carrot_controls=NS(lat_suspend_control=lambda cs, active: active),
               desired_curvature=-0.15, lateral_startup=LateralStartupGate(), lateral_started=False,
               steer_limited_by_controls=False)
  return control, settings


@pytest.mark.parametrize('enabled', [False, True])
def test_startup_blocks_always_lateral_and_regular_engagement_without_changing_longitudinal(enabled):
  control, _ = fixture()
  step = load_state_control()
  sm = control.sm
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = enabled
  live = sm['liveParameters']
  sm['liveParameters'] = log.LiveParametersData.new_message()
  sm.seen['modelV2'] = False
  for _ in range(100):
    cc, _ = step(control)
    assert not cc.latActive and cc.actuators.torque == 0
    assert control.VM.sR == pytest.approx(14.3)
    assert cc.longActive == enabled and cc.actuators.accel == pytest.approx(0.7)
  sm['liveParameters'] = live
  sm.seen['modelV2'] = True
  control.desired_curvature = -0.15
  cc, _ = step(control)
  assert cc.latActive and control.lateral_started
  assert abs(cc.actuators.curvature - control.curvature) < 0.001
  resets = control.LaC.resets
  sm.alive['modelV2'] = False
  cc, _ = step(control)
  assert cc.latActive and control.LaC.resets == resets
  sm['carState'].latEnabled = False
  step(control)
  assert control.LaC.resets == resets + 1
  sm['carState'].latEnabled = True
  control.desired_curvature = 0.01
  cc, _ = step(control)
  assert cc.latActive and cc.actuators.curvature == pytest.approx(0.01)


def test_manual_steering_ratio_is_preserved_during_startup_and_live_geometry_after_startup():
  control, settings = fixture()
  step = load_state_control()
  settings['CustomSR'] = 160
  control.sm.seen['liveParameters'] = False
  step(control)
  assert control.VM.sR == 16
  control.sm.seen['liveParameters'] = True
  step(control)
  settings['CustomSR'] = 0
  settings['SteerRatioRate'] = 90
  control.sm.valid['liveParameters'] = False
  control.sm['liveParameters'].steerRatio = 15.5
  step(control)
  assert control.VM.sR == pytest.approx(15.5 * 0.9)
