from openpilot.common.conversions import Conversions as CV


LOW_SPEED_TORQUE_ACCEL_FACTOR_DEFAULT = 2500.0
HIGH_SPEED_TORQUE_ACCEL_FACTOR_DEFAULT = 5000.0


def high_speed_tuning_active(v_ego, use_lane_line_speed, is_metric):
  speed_threshold_kph = use_lane_line_speed if is_metric else use_lane_line_speed * CV.MPH_TO_KPH
  return speed_threshold_kph > 0.0 and v_ego * CV.MS_TO_KPH >= speed_threshold_kph


def torque_accel_factor(value, high_speed_tuning):
  default = HIGH_SPEED_TORQUE_ACCEL_FACTOR_DEFAULT if high_speed_tuning else LOW_SPEED_TORQUE_ACCEL_FACTOR_DEFAULT
  return value if 1000.0 <= value <= 6000.0 else default


def path_offset_for_mode(configured_offset, lane_lines_active):
  return configured_offset if lane_lines_active else 0.0
