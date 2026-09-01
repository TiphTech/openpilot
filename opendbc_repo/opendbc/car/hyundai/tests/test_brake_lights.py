from opendbc.car.hyundai.carstate import get_canfd_brake_lights


def test_canfd_brake_lights_uses_dedicated_lamp_state():
  assert get_canfd_brake_lights({"BRAKE_LIGHT": 1}, 0, False)
  assert not get_canfd_brake_lights({"BRAKE_LIGHT": 0}, 0, True)


def test_canfd_brake_lights_uses_only_exact_tcs_lamp_state():
  assert get_canfd_brake_lights({"BRAKE_LIGHT": 0}, 1, False)
  assert not get_canfd_brake_lights({"BRAKE_LIGHT": 0}, 2, False)
  assert not get_canfd_brake_lights({"BRAKE_LIGHT": 0}, 3, False)


def test_canfd_brake_lights_falls_back_to_brake_pedal_without_dedicated_message():
  assert get_canfd_brake_lights(None, 0, True)
  assert not get_canfd_brake_lights(None, 0, False)
