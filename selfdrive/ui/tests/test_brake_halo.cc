#include <cassert>
#include <limits>

#include "selfdrive/ui/qt/onroad/brake_halo.h"

int main() {
  BrakeHaloIndicator indicator;
  assert(!indicator.update(false, false, 0.0f, -0.79f, 0.8f));
  assert(indicator.update(false, false, 0.0f, -0.8f, 0.8f));
  for (float decel : {0.79f, 0.81f, 0.75f, 0.61f}) {
    assert(indicator.update(false, false, 0.0f, -decel, 0.8f));
  }
  assert(!indicator.update(false, false, 0.0f, -0.59f, 0.8f));
  assert(!indicator.update(false, false, 0.0f, -0.79f, 0.8f));

  // Pedal and cruise inputs must not latch the measured-deceleration branch.
  assert(indicator.update(true, false, 0.0f, -0.7f, 0.8f));
  assert(!indicator.update(false, false, 0.0f, -0.7f, 0.8f));
  assert(indicator.update(false, true, -0.1f, 0.0f, 0.8f));
  assert(indicator.update(false, true, -0.08f, 0.0f, 0.8f));
  assert(indicator.update(false, true, -0.04f, 0.0f, 0.8f));
  assert(!indicator.update(false, true, -0.02f, 0.0f, 0.8f));
  assert(!indicator.update(false, true, -0.08f, 0.0f, 0.8f));
  assert(indicator.update(false, true, -0.2f, 0.0f, 0.8f));
  assert(!indicator.update(false, false, -0.2f, 0.0f, 0.8f));

  assert(indicator.update(false, false, 0.0f, -0.9f, 0.8f));
  indicator.reset();
  assert(!indicator.update(false, false, 0.0f, -0.7f, 0.8f));
  assert(!indicator.update(false, false, 0.0f, -0.9f, 1.0f));
  assert(indicator.update(false, false, 0.0f, -1.0f, 1.0f));
  assert(!indicator.update(false, false, 0.0f, -0.79f, 1.0f));

  // Small configured thresholds still release at standstill; invalid data clears the latch.
  assert(indicator.update(false, false, 0.0f, -0.1f, 0.1f));
  assert(!indicator.update(false, false, 0.0f, 0.0f, 0.1f));
  assert(indicator.update(false, true, -0.2f, -0.9f, 0.8f));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  assert(!indicator.update(false, true, nan, nan, 0.8f));
}
