from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import get_future_curvature
from openpilot.selfdrive.modeld.constants import ModelConstants


def model_message(yaw_rate, velocity):
  count = ModelConstants.IDX_N
  return SimpleNamespace(
    orientationRate=SimpleNamespace(z=[yaw_rate] * count),
    velocity=SimpleNamespace(x=[velocity] * count),
  )


def test_future_curvature_uses_model_yaw_rate_and_velocity():
  curvature = 0.02
  velocity = 10.0

  result = get_future_curvature(model_message(curvature * velocity, velocity), 0.0)

  assert result == curvature


def test_future_curvature_falls_back_for_invalid_model_data():
  fallback = -0.01
  message = model_message(float("nan"), 10.0)

  assert get_future_curvature(message, fallback) == fallback
  assert get_future_curvature(SimpleNamespace(
    orientationRate=SimpleNamespace(z=[]), velocity=SimpleNamespace(x=[]),
  ), fallback) == fallback
