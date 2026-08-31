from opendbc.car.hyundai.carstate import get_canfd_brake_lights


def test_canfd_brake_lights_uses_dedicated_lamp_state():
  assert get_canfd_brake_lights({"BRAKE_LIGHT": 1}, False)
  assert not get_canfd_brake_lights({"BRAKE_LIGHT": 0}, True)


def test_canfd_brake_lights_falls_back_to_brake_pedal():
  assert get_canfd_brake_lights(None, True)
  assert not get_canfd_brake_lights(None, False)
