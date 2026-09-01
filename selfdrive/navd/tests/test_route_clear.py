from openpilot.selfdrive.navd.navd import RouteEngine


class FakePubMaster:
  def __init__(self):
    self.messages = []

  def send(self, service, message):
    self.messages.append((service, message))


def test_clear_route_publishes_empty_route_once():
  engine = RouteEngine.__new__(RouteEngine)
  engine.route = [object()]
  engine.route_geometry = [[object()]]
  engine.step_idx = 0
  engine.nav_destination = object()
  engine.pm = FakePubMaster()

  engine.clear_route()

  assert engine.route is None
  assert engine.route_geometry is None
  assert engine.step_idx is None
  assert engine.nav_destination is None
  assert len(engine.pm.messages) == 1
  service, message = engine.pm.messages[0]
  assert service == "navRouteNavd"
  assert len(message.navRouteNavd.coordinates) == 0


def test_clear_route_does_not_republish_when_already_empty():
  engine = RouteEngine.__new__(RouteEngine)
  engine.route = None
  engine.route_geometry = None
  engine.step_idx = None
  engine.nav_destination = None
  engine.pm = FakePubMaster()

  engine.clear_route()

  assert engine.pm.messages == []
