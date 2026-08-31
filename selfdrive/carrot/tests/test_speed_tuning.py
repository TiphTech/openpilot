import math

from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.carrot.speed_tuning import camera_yaw_trim_deg, high_speed_tuning_active


def test_high_speed_tuning_uses_configured_threshold():
  assert not high_speed_tuning_active(130 * CV.KPH_TO_MS, 0, True)
  assert not high_speed_tuning_active(99 * CV.KPH_TO_MS, 100, True)
  assert high_speed_tuning_active(100 * CV.KPH_TO_MS, 100, True)
  assert not high_speed_tuning_active(99 * CV.MPH_TO_MS, 100, False)
  assert high_speed_tuning_active(100 * CV.MPH_TO_MS, 100, False)


def test_camera_yaw_switches_to_fixed_high_speed_value():
  assert math.isclose(camera_yaw_trim_deg(1.0, 100, True, 99 * CV.KPH_TO_MS), 1.0)
  assert math.isclose(camera_yaw_trim_deg(1.0, 100, True, 100 * CV.KPH_TO_MS), 0.4)
