#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

struct LanePosition {
  float width_m;
  float offset_left_m;
  float danger_ratio;
};

inline std::optional<LanePosition> lane_position(float left_y, float right_y,
                                               float left_prob, float right_prob) {
  if (!std::isfinite(left_y) || !std::isfinite(right_y) ||
      !std::isfinite(left_prob) || !std::isfinite(right_prob) ||
      left_prob < 0.3f || right_prob < 0.3f || left_y >= 0.0f || right_y <= 0.0f) {
    return std::nullopt;
  }
  // Model coordinates: y points right. Positive offset means the car is left of lane center.
  const float width = right_y - left_y;
  if (width < 2.0f || width > 5.0f) return std::nullopt;
  const float offset_left = (left_y + right_y) * 0.5f;
  const float clearance = std::max(0.1f, width * 0.5f - 0.9f);
  return LanePosition{width, offset_left, std::abs(offset_left) / clearance};
}
