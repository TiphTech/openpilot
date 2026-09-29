import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot.curve_speed import (
  APPROACH_DECEL, CurveSpeed, NO_LIMIT_KPH, VisionCurveSpeed, curve_speed,
)
from openpilot.selfdrive.modeld.constants import ModelConstants


TIMES = np.asarray(ModelConstants.T_IDXS)


def model(speed=15.0, curvature=0.01, start=0.0):
  distance = TIMES * speed
  curves = np.where(distance >= start, curvature, 0.0)
  return SimpleNamespace(
    position=SimpleNamespace(x=distance, y=np.zeros_like(TIMES), z=np.zeros_like(TIMES)),
    velocity=SimpleNamespace(x=np.full_like(TIMES, speed)),
    orientationRate=SimpleNamespace(z=curves * speed),
  )


@pytest.mark.parametrize("direction", [-1, 1])
def test_same_curve_has_same_target_at_different_model_and_vehicle_speeds(direction):
  expected = math.sqrt(190.0) * 3.6
  for prediction in (25 / 3.6, 50 / 3.6, 75 / 3.6):
    for ego in (0.0, 25 / 3.6, 50 / 3.6, 75 / 3.6):
      result = curve_speed(model(prediction, direction * .01), ego)
      assert result.curve_kph == pytest.approx(expected)
      assert result.approach_kph == pytest.approx(expected)
      assert result.direction == direction


def test_remaining_distance_relaxes_approach_but_not_curve_target():
  results = [curve_speed(model(20., .025, start), 60 / 3.6) for start in (0., 40., 80.)]
  assert results[0].approach_kph < results[1].approach_kph < results[2].approach_kph
  assert len({result.curve_kph for result in results}) == 1
  for result in results:
    required = ((result.approach_kph / 3.6)**2 - (result.curve_kph / 3.6)**2) / (2 * max(result.distance, .001))
    assert required <= APPROACH_DECEL


def test_acceleration_tightens_the_approach_envelope():
  path = model(20., .025, 80.)
  coast = curve_speed(path, 60 / 3.6)
  accelerating = curve_speed(path, 60 / 3.6, a_ego=1.5)
  assert accelerating.approach_kph < coast.approach_kph
  assert accelerating.curve_kph == coast.curve_kph


@pytest.mark.parametrize("index", [0, 1, 8, 17, 23])
def test_isolated_yaw_spike_does_not_create_a_curve(index):
  path = model(curvature=0.)
  path.orientationRate.z[index] = 3.
  assert curve_speed(path, 15.).approach_kph == NO_LIMIT_KPH


def test_invalid_or_slow_models_do_not_limit_cruise():
  assert curve_speed(model(speed=.5, curvature=.5), 15.) is None
  broken = model()
  broken.position.x = []
  assert curve_speed(broken, 15.) is None
  assert curve_speed(model(), math.nan) is None


def test_tighter_curve_is_immediate_but_brief_release_does_not_jump():
  state = VisionCurveSpeed()
  assert state.update(CurveSpeed(40., 35., 20., -1.), 0.) == -40.
  for now in (.05, .10, .15, .20):
    assert state.update(CurveSpeed(), now) == -40.
  assert state.update(CurveSpeed(30., 30., 0., 1.), .25) == 30.


def test_sustained_model_loss_recovers_at_a_bounded_rate():
  state = VisionCurveSpeed()
  state.update(CurveSpeed(40.), 0.)
  outputs = [state.update(None, float(now)) for now in np.arange(.05, 1.05, .05)]
  assert outputs[0] == 40.
  assert 40. < outputs[-1] < 46.
  assert max(np.diff(outputs)) <= .360001


def test_repeated_model_frame_does_not_confirm_a_curve_exit():
  state = VisionCurveSpeed()
  state.update(CurveSpeed(30.), 0., model_time=1)
  for now in np.arange(.05, 1., .05):
    assert state.update(CurveSpeed(100.), float(now), model_time=2) == 30.
  assert state.update(CurveSpeed(25.), 1., model_time=3) == 25.
