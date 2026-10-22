#pragma once

#include <algorithm>
#include <cmath>

class BrakeHaloIndicator {
public:
  bool update(bool brake_pressed, bool cruise_enabled, float cruise_accel,
              float ego_accel, float decel_threshold) {
    const float activation_threshold = std::max(0.1f, decel_threshold);
    const float release_threshold = std::max(0.05f, activation_threshold - 0.2f);
    decel_active = std::isfinite(ego_accel) &&
                   -ego_accel >= (decel_active ? release_threshold : activation_threshold);
    cruise_braking = cruise_enabled && std::isfinite(cruise_accel) &&
                     -cruise_accel >= (cruise_braking ? 0.03f : 0.1f);
    return brake_pressed || cruise_braking || decel_active;
  }

  void reset() {
    decel_active = false;
    cruise_braking = false;
  }

private:
  bool decel_active = false;
  bool cruise_braking = false;
};
