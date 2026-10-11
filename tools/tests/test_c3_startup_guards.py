"""Exercise isolated production guards without Panda or native device modules."""
import ast
from enum import IntFlag
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import car

ROOT = Path(__file__).resolve().parents[2]


def method(path, cls, name):
  tree = ast.parse((ROOT / path).read_text())
  node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
  return next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == name)


def execute(nodes, ns):
  exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), '<production guard>', 'exec'), ns)


class Flags(IntFlag):
  CANFD = 1
  CANFD_CAMERA_SCC = 2
  ANGLE_CONTROL = 4


@pytest.mark.parametrize('angle', [False, True])
@pytest.mark.parametrize('camera', [False, True])
def test_camera_template_guard_is_initial_only_and_does_not_mutate_command(angle, camera):
  update = method('opendbc_repo/opendbc/car/hyundai/carcontroller.py', 'CarController', 'update')
  guard = update.body[0]
  assert isinstance(guard, ast.If) and 'steering_template_ready' in ast.unparse(guard.test)
  flags = Flags.CANFD | (Flags.CANFD_CAMERA_SCC if camera else 0) | (Flags.ANGLE_CONTROL if angle else 0)
  controller = NS(CP=NS(flags=flags), steering_template_ready=False)
  cs = NS(lfa=None, lfa_alt=None)
  cc = car.CarControl.new_message(latActive=True, longActive=True)
  cc.actuators.accel = 0.7
  ns = dict(self=controller, CC=cc.as_reader(), CS=cs, HyundaiFlags=Flags)
  for _ in range(200):
    ns['CC'] = cc.as_reader()
    execute([guard], ns)
    assert ns['CC'].latActive == (not camera)
    assert ns['CC'].longActive and ns['CC'].actuators.accel == pytest.approx(0.7)
    assert cc.latActive
  setattr(cs, 'lfa_alt' if angle else 'lfa', {})
  ns['CC'] = cc.as_reader()
  execute([guard], ns)
  assert ns['CC'].latActive
  if camera:
    assert controller.steering_template_ready
    setattr(cs, 'lfa_alt' if angle else 'lfa', None)
    execute([guard], ns)
    assert ns['CC'].latActive


def test_inactive_torque_history_keeps_first_allowed_command_bounded():
  update = method('opendbc_repo/opendbc/car/hyundai/carcontroller.py', 'CarController', 'update')
  guard = update.body[0]
  inactive = next(n for n in update.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'not CC.latActive')
  limiter = next(n for n in ast.parse((ROOT / 'opendbc_repo/opendbc/car/__init__.py').read_text()).body
                 if isinstance(n, ast.FunctionDef) and n.name == 'apply_driver_steer_torque_limits')
  limits = NS(STEER_MAX=409, STEER_DELTA_UP=3, STEER_DELTA_DOWN=7,
              STEER_DRIVER_ALLOWANCE=250, STEER_DRIVER_FACTOR=1, STEER_DRIVER_MULTIPLIER=2)
  controller = NS(CP=NS(flags=Flags.CANFD | Flags.CANFD_CAMERA_SCC), steering_template_ready=False,
                  apply_torque_last=0, lkas_max_torque=0)
  cs = NS(lfa=None)
  cc = car.CarControl.new_message(latActive=True)
  ns = dict(np=np, self=controller, CS=cs, HyundaiFlags=Flags)
  execute([limiter], ns)
  for _ in range(200):
    ns['CC'] = cc.as_reader()
    execute([guard], ns)
    ns['apply_torque'] = ns['apply_driver_steer_torque_limits'](409, controller.apply_torque_last, 0, limits)
    execute([inactive], ns)
    controller.apply_torque_last = ns['apply_torque']
    assert controller.apply_torque_last == 0
  cs.lfa = {}
  ns['CC'] = cc.as_reader()
  execute([guard], ns)
  ns['apply_torque'] = ns['apply_driver_steer_torque_limits'](409, controller.apply_torque_last, 0, limits)
  execute([inactive], ns)
  assert ns['CC'].latActive and ns['apply_torque'] == 3


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('initializing', [False, True])
def test_startup_alert_priority_does_not_remove_events_or_other_alerts(enabled, initializing):
  update = method('selfdrive/selfdrived/selfdrived.py', 'SelfdriveD', 'update_alerts')
  guard = next(n for n in update.body if isinstance(n, ast.If) and 'selfdriveInitializing' in ast.unparse(n.test))
  names = ['wrongGear'] + (['selfdriveInitializing'] if initializing else [])
  alerts = [NS(alert_type=k) for k in ('wrongGear/noEntry', 'selfdriveInitializing/noEntry', 'canError/noEntry')]
  state = NS(enabled=enabled, events=NS(names=names), AM=NS(alerts={'wrongGear/noEntry': object(), 'canError/noEntry': object()}))
  ns = dict(self=state, alerts=alerts, EventName=NS(selfdriveInitializing='selfdriveInitializing'))
  execute([guard], ns)
  suppressed = initializing and not enabled
  assert ('wrongGear/noEntry' not in [a.alert_type for a in ns['alerts']]) == suppressed
  assert ('wrongGear/noEntry' not in state.AM.alerts) == suppressed
  assert 'canError/noEntry' in state.AM.alerts
  assert state.events.names == names
