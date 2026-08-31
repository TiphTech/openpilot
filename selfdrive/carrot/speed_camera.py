APN_SPEED_CAMERA_TYPES = frozenset((0, 1, 2, 3, 4, 7, 8, 75, 76, 100, 101))


def apn_speed_camera_active(active_carrot, speed_type, speed_limit, speed_distance, entry_distance=200):
  return (
    active_carrot >= 2 and
    speed_type in APN_SPEED_CAMERA_TYPES and
    speed_limit >= 30 and
    speed_distance <= entry_distance
  )
