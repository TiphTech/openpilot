APN_SPEED_CAMERA_TYPES = frozenset((0, 1, 2, 3, 4, 7, 8, 75, 76, 100, 101))
SPEED_CAMERA_ENTRY_DISTANCE_M = 200


def apn_speed_camera_active(active_carrot, speed_type, speed_limit, speed_distance,
                            entry_distance=SPEED_CAMERA_ENTRY_DISTANCE_M):
  return (
    active_carrot >= 2 and
    speed_type in APN_SPEED_CAMERA_TYPES and
    speed_limit >= 30 and
    speed_distance <= entry_distance
  )


def vehicle_speed_camera_active(speed_limit, speed_distance, entry_distance=SPEED_CAMERA_ENTRY_DISTANCE_M):
  return speed_limit >= 30 and 0 < speed_distance <= entry_distance


def speed_camera_zone_info(active_carrot, speed_type, apn_speed_limit, apn_speed_distance,
                           vehicle_speed_limit, vehicle_speed_distance):
  """Return the active camera limit and whether the APN already adjusted it."""
  if apn_speed_camera_active(active_carrot, speed_type, apn_speed_limit, apn_speed_distance):
    return float(apn_speed_limit), True
  if vehicle_speed_camera_active(vehicle_speed_limit, vehicle_speed_distance):
    return float(vehicle_speed_limit), False
  return 0.0, False
