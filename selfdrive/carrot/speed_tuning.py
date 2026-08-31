from openpilot.common.conversions import Conversions as CV


HIGH_SPEED_CAMERA_YAW_TRIM_DEG = 0.4


def high_speed_tuning_active(v_ego, use_lane_line_speed, is_metric):
  speed_threshold_kph = use_lane_line_speed if is_metric else use_lane_line_speed * CV.MPH_TO_KPH
  return speed_threshold_kph > 0.0 and v_ego * CV.MS_TO_KPH >= speed_threshold_kph


def camera_yaw_trim_deg(configured_trim_deg, use_lane_line_speed, is_metric, v_ego):
  if high_speed_tuning_active(v_ego, use_lane_line_speed, is_metric):
    return HIGH_SPEED_CAMERA_YAW_TRIM_DEG
  return configured_trim_deg
