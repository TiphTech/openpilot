import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.carrot.speed_tuning import path_offset_for_mode
from openpilot.selfdrive.controls.lib.lane_centering import (
  AutoLaneCentering, CENTER_DEADBAND_M, MAX_OFFSET_M, MAX_OFFSET_RATE_M_S,
)

DT = 0.05
ROOT = Path(__file__).parents[3]


def model_and_path(error=0.3, width=3.6, manual_offset=0.0):
  model = log.ModelDataV2.new_message()
  x = np.linspace(0.0, 80.0, 33).tolist()
  t = np.linspace(0.0, 4.0, 33).tolist()
  model.laneLines = [dict(x=x, y=[error + side * width] * 33, t=t) for side in (-1.5, -0.5, 0.5, 1.5)]
  model.laneLineProbs = [0.99] * 4
  model.laneLineStds = [0.05] * 4
  model.roadEdges = [dict(x=x, y=[error - width] * 33, t=t), dict(x=x, y=[error + 2 * width] * 33, t=t)]
  model.roadEdgeStds = [0.05, 0.05]
  model.position = dict(x=x, y=[error + manual_offset] * 33, z=[0.0] * 33, t=t)
  model.orientation = dict(x=[0.0] * 33, z=[0.0] * 33)
  model.orientationRate = dict(z=[0.0] * 33)
  model.velocity = dict(x=[20.0] * 33, y=[0.0] * 33, z=[0.0] * 33)
  model.acceleration = dict(x=[0.0] * 33)
  path = np.column_stack((x, model.position.y, model.position.z))
  return model, path


def update(trim, model, path, **kwargs):
  args = dict(enabled=True, lateral_active=True, lane_mode_active=True, maneuver_active=False,
              speed=20.0, curvature=0.0, manual_offset=0.0)
  args.update(kwargs)
  return trim.update(model, path, **args)


@pytest.mark.parametrize('error', [-0.6, -0.3, 0.3, 0.6])
def test_sign_confirmation_rate_and_saturation(error):
  model, path = model_and_path(error)
  trim = AutoLaneCentering(DT)
  for _ in range(39):
    assert update(trim, model, path) == 0.0
  previous = 0.0
  for _ in range(1600):
    offset = update(trim, model, path)
    assert 0.0 < offset * np.sign(error) <= MAX_OFFSET_M
    assert abs(offset - previous) <= MAX_OFFSET_RATE_M_S * DT + 1e-12
    previous = offset
  assert offset == pytest.approx(math.copysign(MAX_OFFSET_M, error))


@pytest.mark.parametrize('error', [-0.07, 0.0, 0.07])
def test_center_deadband_does_not_accumulate_noise(error):
  model, path = model_and_path(error)
  trim = AutoLaneCentering(DT)
  for _ in range(200):
    assert update(trim, model, path) == 0.0


def test_disabled_by_default_needs_no_model_and_does_not_touch_path():
  trim = AutoLaneCentering(DT)
  assert update(trim, None, None, enabled=False) == 0.0
  model, path = model_and_path()
  original = path.copy()
  for _ in range(100):
    update(trim, model, path, enabled=False)
  np.testing.assert_array_equal(path, original)


@pytest.mark.parametrize('override', [dict(enabled=False), dict(lane_mode_active=False), dict(maneuver_active=True),
                                    dict(speed=0.0), dict(speed=4.9), dict(curvature=0.004),
                                    dict(speed=math.nan), dict(curvature=math.inf), dict(manual_offset=math.nan)])
def test_gates_stop_learning_and_fade_without_a_step(override):
  model, path = model_and_path()
  trim = AutoLaneCentering(DT)
  trim.offset_m = 0.1
  assert update(trim, model, path, **override) == pytest.approx(0.1 - MAX_OFFSET_RATE_M_S * DT)
  assert trim.confirm_time_s == 0.0
  for _ in range(210):
    update(trim, model, path, **override)
  assert trim.offset_m == pytest.approx(0.0)


def test_disengaging_clears_trim_and_reengaging_requires_confirmation():
  model, path = model_and_path()
  trim = AutoLaneCentering(DT)
  trim.offset_m = 0.1
  assert update(trim, None, None, lateral_active=False) == 0.0
  for _ in range(39):
    assert update(trim, model, path) == 0.0
  assert update(trim, model, path) > 0.0


@pytest.mark.parametrize('failure', ['left_prob', 'right_prob', 'std', 'negative_std', 'nan', 'missing_line',
                                   'short_line', 'duplicate_x', 'nan_x', 'narrow', 'wide', 'taper',
                                   'outside_lane', 'large_bias', 'left_edge', 'right_edge', 'short_edge'])
def test_invalid_or_ambiguous_geometry_cannot_build_trim(failure):
  model, path = model_and_path()
  if failure == 'left_prob':
    model.laneLineProbs = [0.99, 0.79, 0.99, 0.99]
  elif failure == 'right_prob':
    model.laneLineProbs = [0.99, 0.99, 0.79, 0.99]
  elif failure in ('std', 'negative_std', 'nan'):
    model.laneLineStds = [0.05, {'std': 0.16, 'negative_std': -0.1, 'nan': math.nan}[failure], 0.05, 0.05]
  elif failure == 'missing_line':
    model.laneLines = []
  elif failure == 'short_line':
    model.laneLines[1].x = [0.0, 5.0]
    model.laneLines[1].y = [-1.5, -1.5]
  elif failure in ('duplicate_x', 'nan_x'):
    model.laneLines[1].x = [0.0] * 33 if failure == 'duplicate_x' else [math.nan] * 33
  elif failure in ('narrow', 'wide', 'outside_lane', 'large_bias'):
    model, path = model_and_path(error={'outside_lane': 1.0, 'large_bias': 0.7}.get(failure, 0.0),
                                 width={'narrow': 2.5, 'wide': 5.0}.get(failure, 3.6))
  elif failure == 'taper':
    model.laneLines[2].y = np.linspace(2.1, 5.5, 33).tolist()
  elif failure in ('left_edge', 'right_edge'):
    model.roadEdges[0 if failure == 'left_edge' else 1].y = [0.0] * 33
  elif failure == 'short_edge':
    model.roadEdges[0].x = []
    model.roadEdges[0].y = []
  trim = AutoLaneCentering(DT)
  for _ in range(100):
    assert update(trim, model, path) == 0.0
  assert trim.status != 'tracking'


@pytest.mark.parametrize('failure', ['deliberate_offset', 'nan', 'duplicate_x', 'short', 'shape'])
def test_invalid_or_intentionally_offset_trajectory_is_not_overridden(failure):
  model, path = model_and_path()
  if failure == 'deliberate_offset':
    path[:, 1] += 0.4
  elif failure == 'nan':
    path[2, 1] = math.nan
  elif failure == 'duplicate_x':
    path[:, 0] = 0.0
  elif failure == 'short':
    path = path[:3]
  else:
    path = np.zeros((33, 1))
  trim = AutoLaneCentering(DT)
  for _ in range(100):
    assert update(trim, model, path) == 0.0
  assert trim.status == 'trajectory'


def test_manual_offset_is_preserved_and_road_edges_never_define_center():
  # Lane centered, manual preference 20 cm right. Asymmetric multi-lane road edges do not pull left/right.
  model, path = model_and_path(error=-0.2, manual_offset=0.2)
  trim = AutoLaneCentering(DT)
  for _ in range(100):
    assert update(trim, model, path, manual_offset=0.2) == pytest.approx(0.0)
  assert trim.error_m == pytest.approx(0.0, abs=1e-6)


def test_adaptation_stops_when_residual_bias_reaches_deadband():
  trim = AutoLaneCentering(DT)
  for _ in range(4000):
    residual = 0.2 - trim.offset_m
    model, path = model_and_path(error=residual)
    update(trim, model, path)
  assert trim.offset_m == pytest.approx(0.2 - CENTER_DEADBAND_M, abs=0.003)
  previous = trim.offset_m
  model, path = model_and_path(error=0.0)
  for _ in range(100):
    assert update(trim, model, path) == previous


def test_parameter_registered_persistent_and_in_both_menus():
  params = json.loads((ROOT / 'selfdrive/carrot_settings.json').read_text())['params']
  entry, = [p for p in params if p['name'] == 'AutoLaneCentering']
  assert (entry['default'], entry['min'], entry['max']) == (0, 0, 1)
  assert '{"AutoLaneCentering", {PERSISTENT, BOOL, "0"}}' in (ROOT / 'common/params_keys.h').read_text()
  assert 'ParamControl("AutoLaneCentering"' in (ROOT / 'selfdrive/ui/qt/offroad/settings.cc').read_text()


def production_planner():
  # Exercise the production update method without loading the Linux-only native MPC/Params on macOS.
  source = ROOT / 'selfdrive/controls/lib/lateral_planner.py'
  tree = ast.parse(source.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LateralPlanner')

  class MPC:
    def reset(self, **kwargs):
      self.x_sol = np.zeros((17, 4))
      self.cost = 0.0
      self.solution_status = 0

    def set_weights(self, *args):
      pass

    def run(self, x0, p, y, heading, yaw_rate):
      self.input_y = y.copy()

  env = dict(np=np, time=NS(monotonic=lambda: 0.0), DT_MDL=DT, MIN_SPEED=0.1, TRAJECTORY_SIZE=33,
             LAT_MPC_N=16, AutoLaneCentering=AutoLaneCentering, FirstOrderFilter=FirstOrderFilter,
             LateralMpc=MPC, Params=lambda: NS(get_int=lambda key: 100 if key == 'UseLaneLineSpeed' else 0),
             LanePlanner=lambda: NS(), log=log, get_speed_error=lambda md, speed: 0.0,
             path_offset_for_mode=path_offset_for_mode, UnknownKeyName=KeyError,
             yaw_from_path_no_scipy=lambda *args, **kwargs: (np.zeros(33), np.zeros(33)))
  exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), env)
  planner = env['LateralPlanner'](NS(wheelbase=2.9, centerToFront=1.4, mass=2000.0, tireStiffnessRear=100000.0))
  planner.params = NS(get_int=lambda key: 0, get_float=lambda key: 0.0, get_bool=lambda key: True)
  planner.LP.parse_model = lambda md: None
  planner.LP.get_d_path = lambda cs, speed, times, path, curve: (path, True)
  planner.LP.offset_total = 0.0
  planner.LP.debugText = ''
  return planner


@pytest.mark.parametrize('blocker', [None, 'leftBlinker', 'rightBlinker', 'steeringPressed', 'desire', 'lane_change',
                                  'atc', 'invalid_model', 'invalid_car', 'invalid_control', 'inactive', 'laneless', 'curve_gate',
                                  'disabled', 'old_params_registry'])
def test_production_planner_routes_trim_into_mpc_only_when_eligible(blocker):
  planner = production_planner()
  model, path = model_and_path()
  cs = NS(vEgo=30.0, useLaneLineSpeed=100, leftBlinker=False, rightBlinker=False, steeringPressed=False)
  carrot = NS(atc_active=False)
  if blocker in ('leftBlinker', 'rightBlinker', 'steeringPressed'):
    setattr(cs, blocker, True)
  elif blocker == 'desire':
    model.meta.desire = log.Desire.laneChangeLeft
  elif blocker == 'lane_change':
    model.meta.laneChangeState = log.LaneChangeState.laneChangeFinishing
  elif blocker == 'atc':
    carrot.atc_active = True
  elif blocker == 'laneless':
    planner.LP.get_d_path = lambda *args: (args[3], False)
  elif blocker == 'curve_gate':
    planner.params.get_int = lambda key: 100 if key == 'UseLaneLineCurveSpeed' else 0
  elif blocker == 'disabled':
    planner.params.get_bool = lambda key: False
  elif blocker == 'old_params_registry':
    def unknown_key(key):
      raise KeyError(key)
    planner.params.get_bool = unknown_key

  class Messages(dict):
    def all_checks(self, services):
      return not (blocker == 'invalid_model' and 'modelV2' in services or blocker == 'invalid_car' and 'carState' in services or
                  blocker == 'invalid_control' and 'carControl' in services)

  sm = Messages(carState=cs, controlsState=NS(curvature=0.0), modelV2=model,
                carrotMan=NS(vTurnSpeed=90.0), carControl=NS(latActive=blocker != 'inactive'))
  for _ in range(100):
    planner.update(sm, carrot)
  if blocker is None:
    assert planner.auto_lane_centering.offset_m > 0.0
    np.testing.assert_allclose(planner.lat_mpc.input_y, path[:17, 1] + planner.auto_lane_centering.offset_m)
  else:
    assert planner.auto_lane_centering.offset_m == 0.0
    np.testing.assert_array_equal(planner.lat_mpc.input_y, path[:17, 1])
