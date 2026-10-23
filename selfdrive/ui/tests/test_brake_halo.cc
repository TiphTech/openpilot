#include <cassert>
#include <limits>

#include "selfdrive/ui/qt/onroad/brake_halo.h"

int main() {
  BrakeHaloIndicator indicator;
  auto step = [&](uint64_t time, float accel, float threshold = 0.8f) {
    return indicator.update(false, false, 0.0f, accel, threshold, time);
  };

  // Brief peaks and duplicate paint calls must not complete the activation delay.
  assert(!step(0, -0.79f));
  assert(!step(50, -0.8f));
  assert(!step(150, -0.79f));
  assert(!step(200, -0.81f));
  assert(!step(200, -0.81f));
  assert(!step(300, -0.81f));
  assert(!step(349, -0.81f));
  assert(step(350, -0.81f));
  assert(step(350, -0.81f));

  // Hysteresis holds the indication below activation; a recovery cancels release.
  assert(step(400, -0.79f));
  assert(step(450, -0.81f));
  assert(step(500, -0.61f));
  assert(step(600, -0.59f));
  assert(step(900, -0.61f));
  assert(step(1000, -0.59f));
  assert(step(1399, -0.59f));
  assert(!step(1400, -0.59f));
  assert(!step(1450, -0.79f));

  // Pedal and cruise inputs retain their immediate behavior and independent state.
  indicator.reset();
  assert(indicator.update(true, false, 0.0f, -0.7f, 0.8f, 0));
  assert(!step(50, -0.7f));
  assert(indicator.update(false, true, -0.1f, 0.0f, 0.8f, 100));
  assert(indicator.update(false, true, -0.08f, 0.0f, 0.8f, 150));
  assert(indicator.update(false, true, -0.04f, 0.0f, 0.8f, 200));
  assert(!indicator.update(false, true, -0.02f, 0.0f, 0.8f, 250));
  assert(!indicator.update(false, true, -0.08f, 0.0f, 0.8f, 300));
  assert(indicator.update(false, true, -0.2f, 0.0f, 0.8f, 350));
  assert(!indicator.update(false, false, -0.2f, 0.0f, 0.8f, 400));

  // A new configured threshold starts a fresh validation interval.
  indicator.reset();
  assert(!step(0, -0.9f));
  assert(!step(100, -1.1f, 1.0f));
  assert(!step(249, -1.1f, 1.0f));
  assert(step(250, -1.1f, 1.0f));
  assert(step(300, -0.79f, 1.0f));
  assert(!step(700, -0.79f, 1.0f));
  assert(!step(800, -0.7f, 0.6f));
  assert(step(950, -0.7f, 0.6f));
  assert(step(1000, -0.39f, 0.6f));
  assert(!step(1400, -0.39f, 0.6f));

  // Low thresholds still release at standstill; invalid data clears pending state.
  indicator.reset();
  assert(!step(0, -0.1f, 0.1f));
  assert(step(150, -0.1f, 0.1f));
  assert(step(200, 0.0f, 0.1f));
  assert(!step(600, 0.0f, 0.1f));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  assert(!step(650, -0.9f));
  assert(!step(750, nan));
  assert(!step(800, -0.9f));
  assert(step(950, -0.9f));
  assert(!step(1000, nan));
  assert(!step(1100, -0.9f));
  indicator.reset();
  assert(!step(2000, -0.9f));
  assert(step(2150, -0.9f));
}
