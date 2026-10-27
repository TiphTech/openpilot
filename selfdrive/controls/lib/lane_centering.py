import math

import numpy as np


MAX_OFFSET_M = 0.15
MAX_OFFSET_RATE_M_S = 0.01
CENTER_DEADBAND_M = 0.08
CONFIRM_TIME_S = 2.0


def sample_boundary(boundary, distances):
  x = np.asarray(boundary.x, dtype=float)
  y = np.asarray(boundary.y, dtype=float)
  if (x.ndim != 1 or y.shape != x.shape or len(x) < 3 or
      not np.isfinite(x).all() or not np.isfinite(y).all() or
      not (np.diff(x) > 0).all() or x[0] > 3.0 or x[-1] < distances[-1]):
    return None
  return np.interp(distances, x, y)


class AutoLaneCentering:
  """Slow, bounded trim for persistent tracking bias inside a well-observed lane."""

  def __init__(self, dt):
    self.dt = dt
    self.reset()

  def reset(self):
    self.offset_m = 0.0
    self.confirm_time_s = 0.0
    self.error_m = None
    self.status = "off"

  def _fade(self, reason):
    self.confirm_time_s = 0.0
    self.error_m = None
    self.status = reason
    step = MAX_OFFSET_RATE_M_S * self.dt
    self.offset_m -= float(np.clip(self.offset_m, -step, step))
    return self.offset_m

  def update(self, model, path_xyz, *, enabled, lateral_active, lane_mode_active, maneuver_active,
             speed, curvature, manual_offset=0.0):
    if not lateral_active:
      self.reset()
      return 0.0
    if not enabled:
      return self._fade("off")
    if not lane_mode_active:
      return self._fade("laneless")
    if maneuver_active:
      return self._fade("maneuver")
    if (not all(math.isfinite(v) for v in (speed, curvature, manual_offset)) or
        speed < 5.0 or abs(curvature) * speed ** 2 > 1.5):
      return self._fade("speed/curve")

    lines, probs, stds = model.laneLines, model.laneLineProbs, model.laneLineStds
    if (len(lines) < 4 or len(probs) < 4 or len(stds) < 4 or
        not all(math.isfinite(v) for v in (probs[1], probs[2], stds[1], stds[2])) or
        min(probs[1], probs[2]) < 0.8 or min(stds[1], stds[2]) < 0.0 or max(stds[1], stds[2]) > 0.15):
      return self._fade("lane quality")

    distances = np.array([0.0, 5.0, min(25.0, max(10.0, speed * 0.8))])
    left = sample_boundary(lines[1], distances)
    right = sample_boundary(lines[2], distances)
    if left is None or right is None:
      return self._fade("lane geometry")
    widths = right - left
    if (min(widths) < 2.6 or max(widths) > 4.5 or np.ptp(widths) > 0.4 or
        left[0] > -1.05 or right[0] < 1.05):
      return self._fade("lane geometry")

    # Road edges may veto inconsistent lane detections, never define a multi-lane road's center.
    for side in range(min(2, len(model.roadEdges), len(model.roadEdgeStds))):
      if math.isfinite(model.roadEdgeStds[side]) and 0.0 <= model.roadEdgeStds[side] < 0.3:
        edge = sample_boundary(model.roadEdges[side], distances)
        if edge is None or (np.any(edge > left + 0.3) if side == 0 else np.any(edge < right - 0.3)):
          return self._fade("road edge")

    center = (left + right) / 2.0 + manual_offset
    error = float(center[0])
    if abs(error) > 0.65:
      return self._fade("lane ambiguous")
    path_xyz = np.asarray(path_xyz, dtype=float)
    if (path_xyz.ndim != 2 or path_xyz.shape[1] < 2 or len(path_xyz) < 3 or
        not np.isfinite(path_xyz[:, :2]).all() or not (np.diff(path_xyz[:, 0]) > 0).all() or
        path_xyz[0, 0] > 3.0 or path_xyz[-1, 0] < distances[-1]):
      return self._fade("trajectory")
    # Do not learn against a model path that deliberately departs from the lane center.
    planned_y = np.interp(distances[1:], path_xyz[:, 0], path_xyz[:, 1])
    if np.any(np.abs(planned_y - center[1:]) > 0.3):
      return self._fade("trajectory")

    self.error_m = error
    self.confirm_time_s = min(CONFIRM_TIME_S, self.confirm_time_s + self.dt)
    self.status = "waiting"
    if self.confirm_time_s >= CONFIRM_TIME_S:
      rate = math.copysign(max(0.0, abs(error) - CENTER_DEADBAND_M), error) * 0.02
      step = float(np.clip(rate, -MAX_OFFSET_RATE_M_S, MAX_OFFSET_RATE_M_S)) * self.dt
      self.offset_m = float(np.clip(self.offset_m + step, -MAX_OFFSET_M, MAX_OFFSET_M))
      self.status = "tracking"
    return self.offset_m
