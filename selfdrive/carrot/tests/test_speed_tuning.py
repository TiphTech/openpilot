import math

from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.carrot.speed_tuning import camera_yaw_trim_deg, high_speed_tuning_active, torque_accel_factor


def test_high_speed_tuning_uses_configured_threshold():
  assert not high_speed_tuning_active(130 * CV.KPH_TO_MS, 0, True)
  assert not high_speed_tuning_active(99 * CV.KPH_TO_MS, 100, True)
  assert high_speed_tuning_active(100 * CV.KPH_TO_MS, 100, True)
  assert not high_speed_tuning_active(99 * CV.MPH_TO_MS, 100, False)
  assert high_speed_tuning_active(100 * CV.MPH_TO_MS, 100, False)


def test_camera_yaw_switches_to_fixed_high_speed_value():
  assert math.isclose(camera_yaw_trim_deg(1.0, 100, True, 99 * CV.KPH_TO_MS), 1.0)
  assert math.isclose(camera_yaw_trim_deg(1.0, 100, True, 100 * CV.KPH_TO_MS), 0.4)


def test_torque_accel_factor_uses_valid_configured_value():
  assert torque_accel_factor(4400.0, False) == 4400.0
  assert torque_accel_factor(5000.0, True) == 5000.0


def test_torque_accel_factor_uses_mode_default_when_param_is_missing():
  assert torque_accel_factor(0.0, False) == 2500.0
  assert torque_accel_factor(0.0, True) == 5000.0
