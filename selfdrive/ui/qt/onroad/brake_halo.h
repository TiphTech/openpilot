#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

class BrakeHaloIndicator {
public:
  bool update(bool brake_pressed, bool cruise_enabled, float cruise_accel,
              float ego_accel, float decel_threshold, uint64_t now_ms) {
    const float activation_threshold = std::max(0.1f, decel_threshold);
    const float release_threshold = std::max(0.05f, activation_threshold - 0.2f);
    if (last_threshold != activation_threshold) {
      decel_transition_since.reset();
      last_threshold = activation_threshold;
    }
    if (!std::isfinite(ego_accel)) {
      decel_active = false;
      decel_transition_since.reset();
    } else {
      const bool requested = -ego_accel >= (decel_active ? release_threshold : activation_threshold);
      if (requested == decel_active) {
        decel_transition_since.reset();
      } else if (!decel_transition_since || now_ms < *decel_transition_since) {
        decel_transition_since = now_ms;
      } else if (now_ms - *decel_transition_since >= (decel_active ? 400U : 150U)) {
        decel_active = requested;
        decel_transition_since.reset();
      }
    }
    cruise_braking = cruise_enabled && std::isfinite(cruise_accel) &&
                     -cruise_accel >= (cruise_braking ? 0.03f : 0.1f);
    return brake_pressed || cruise_braking || decel_active;
  }

  void reset() {
    decel_active = false;
    cruise_braking = false;
    decel_transition_since.reset();
    last_threshold = 0.0f;
  }

private:
  bool decel_active = false;
  bool cruise_braking = false;
  std::optional<uint64_t> decel_transition_since;
  float last_threshold = 0.0f;
};
