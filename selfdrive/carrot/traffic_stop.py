import numpy as np


def get_traffic_stop_obstacle_distance(stop_distance: float, cruise_obstacle_distance: float,
                                       distance_adjust: float, release_distance: float = 50.0) -> float:
  """Progressively expose a signal-stop obstacle before the historical 50 m boundary."""
  signal_obstacle = max(0.0, float(stop_distance) + float(distance_adjust))
  cruise_obstacle = max(0.0, float(cruise_obstacle_distance))
  release_distance = max(0.0, float(release_distance))

  if release_distance < signal_obstacle < cruise_obstacle:
    release = float(np.interp(signal_obstacle, [release_distance, cruise_obstacle], [1.0, 0.0]))
    return cruise_obstacle + release * (signal_obstacle - cruise_obstacle)
  return signal_obstacle
