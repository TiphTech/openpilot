#include <cassert>
#include <cmath>
#include <limits>

#include "selfdrive/ui/qt/onroad/lane_position.h"

int main() {
  auto center = lane_position(-1.8f, 1.8f, 0.99f, 0.99f);
  assert(center && std::abs(center->width_m - 3.6f) < 1e-6f);
  assert(center->offset_left_m == 0.0f && center->danger_ratio == 0.0f);

  auto left = lane_position(-1.5f, 2.1f, 0.99f, 0.99f);
  auto right = lane_position(-2.1f, 1.5f, 0.99f, 0.99f);
  assert(left && right);
  assert(std::abs(left->offset_left_m - 0.3f) < 1e-6f);
  assert(std::abs(right->offset_left_m + 0.3f) < 1e-6f);
  assert(std::abs(left->danger_ratio - 1.0f / 3.0f) < 1e-6f);
  assert(left->danger_ratio == right->danger_ratio);

  // A wider real lane has more clearance; do not fall back to a fictitious 3 m lane.
  auto wide = lane_position(-1.7f, 2.3f, 0.99f, 0.99f);
  assert(wide && wide->danger_ratio < left->danger_ratio);
  assert(!lane_position(-1.8f, 1.8f, 0.1f, 0.99f));
  assert(!lane_position(-1.8f, 1.8f, 0.99f, 0.1f));
  assert(!lane_position(1.8f, -1.8f, 0.99f, 0.99f));
  assert(!lane_position(-0.5f, 0.5f, 0.99f, 0.99f));
  assert(!lane_position(-3.5f, 3.5f, 0.99f, 0.99f));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  assert(!lane_position(nan, 1.8f, 0.99f, 0.99f));
  assert(!lane_position(-1.8f, 1.8f, nan, 0.99f));
}
