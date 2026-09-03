import pytest

from openpilot.selfdrive.carrot.traffic_stop import get_traffic_stop_obstacle_distance


def test_traffic_stop_obstacle_is_exposed_progressively():
  cruise_obstacle = 100.0

  assert get_traffic_stop_obstacle_distance(100.0, cruise_obstacle, 0.0) == pytest.approx(100.0)
  assert get_traffic_stop_obstacle_distance(75.0, cruise_obstacle, 0.0) == pytest.approx(87.5)
  assert get_traffic_stop_obstacle_distance(50.0, cruise_obstacle, 0.0) == pytest.approx(50.0)


def test_traffic_stop_obstacle_applies_adjustment_and_bounds_distance():
  assert get_traffic_stop_obstacle_distance(60.0, 100.0, 10.0) == pytest.approx(82.0)
  assert get_traffic_stop_obstacle_distance(1.0, 100.0, -2.0) == 0.0
