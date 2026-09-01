from openpilot.selfdrive.carrot.carrot_man import CarrotMan


class FakePubMaster:
  def __init__(self):
    self.messages = []

  def send(self, service, message):
    self.messages.append((service, message))


def test_empty_navd_route_clears_carrot_navigation_state():
  manager = CarrotMan.__new__(CarrotMan)
  manager.navi_points = [(1.0, 2.0)]
  manager.navi_points_start_index = 4
  manager.navi_points_active = True
  manager.navd_active = True
  manager.pm = FakePubMaster()

  manager.send_routes([], from_navd=True)

  assert manager.navi_points == []
  assert manager.navi_points_start_index == 0
  assert not manager.navi_points_active
  assert not manager.navd_active
  assert len(manager.pm.messages) == 1
  service, message = manager.pm.messages[0]
  assert service == "navRoute"
  assert len(message.navRoute.coordinates) == 0
