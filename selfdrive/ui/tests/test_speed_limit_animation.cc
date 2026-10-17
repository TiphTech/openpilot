#include <cassert>
#include <iostream>

#include "selfdrive/ui/qt/onroad/speed_limit_animation.h"

int main() {
  RoadLimitSpeedAnimation animation;
  auto update = [&](int limit, const QString &log, qint64 now = 1000,
                    bool automatic = true, bool engaged = true, bool camera = false, bool healthy = true) {
    return animation.update(limit, log, automatic, engaged, camera, healthy, now);
  };
  // No replay at startup, engagement or for a manual speed change.
  assert(!update(50, "Auto road limit 0->50 => 50->52"));
  assert(!update(50, "Cruise adjust 25"));
  assert(!update(50, "Auto road limit 0->50 => 50->52"));
  assert(!update(50, "Cruise speed set to road limit 50->52"));
  assert(!update(50, "Speed camera zone 50->50"));

  assert(update(80, "Auto road limit 50->80 => 80->82") == 82);
  assert(!update(80, "Auto road limit 50->80 => 80->82"));
  assert(!update(80, ""));
  assert(!update(80, "Auto road limit 50->80 => 80->27"));
  assert(!update(80, "Cruise adjust 26"));
  assert(!update(80, "Auto road limit 50->80 => 80->82"));

  // Accept delayed confirmation, once; never show intermediate ramp steps.
  assert(!update(50, "Auto road limit 50->80 => 80->82"));
  assert(update(50, "Auto road limit 80->50 => 50->52", 1100) == 52);
  assert(!update(50, "Auto road limit 80->50 => 50->78", 1200));
  assert(update(80, "Auto road limit 50->80 => 80->82", 1300) == 82);
  assert(update(50, "Auto road limit 80->50 => 50->52", 1400) == 52);

  // Disabling AUTO, disengaging, radar zones and stale services consume the transition.
  for (int block = 0; block < 4; ++block) {
    assert(!update(80, "Auto road limit 50->80 => 80->82", 1500, block != 0, block != 1, block == 2, block != 3));
    assert(!update(80, "Auto road limit 50->80 => 80->82", 1600));
    if (block == 3) {
      assert(!update(50, "", 1700));
    } else {
      assert(update(50, "Auto road limit 80->50 => 50->52", 1700) == 52);
    }
  }
  assert(!update(0, ""));
  assert(!update(50, "Auto road limit 0->50 => 50->52"));
  assert(!update(80, "", 2000));
  assert(!update(80, "Auto road limit 50->80 => 80->82", 5101));
  assert(!update(50, "Auto road limit 80->50 => 80->52", 5200));
  assert(!update(50, "Auto road limit 80->50 => 50->999", 5200));
  assert(!update(50, "Auto road limit 80->50 => 50->52 extra", 5200));
  assert(update(50, "Auto road limit 80->50 => 50->52", 5300) == 52);
  std::cout << "Automatic sign-change animation checks passed\n";
}
