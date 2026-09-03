from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.hyundaicanfd import apply_accel_jerk_limit, create_acc_control_scc2


def test_canfd_accel_value_uses_asymmetric_jerk_limits():
  assert apply_accel_jerk_limit(-4.0, 0.0, jerk_u=1.0, jerk_l=5.0) == pytest.approx(-0.1)
  assert apply_accel_jerk_limit(0.0, -2.0, jerk_u=1.0, jerk_l=5.0) == pytest.approx(-1.98)


def test_camera_scc_accel_value_keeps_ramping_from_previous_value():
  class FakePacker:
    @staticmethod
    def make_can_msg(name, bus, values):
      return name, bus, values.copy()

  can = SimpleNamespace(ECAN=0)
  cs = SimpleNamespace(
    scc_control={"ACC_ObjRelSpd": 0.0, "InfoDisplay": 0},
    softHoldActive=0,
    paddle_button_prev=0,
    out=SimpleNamespace(
      aEgo=0.0,
      vEgo=20.0,
      brakeHoldActive=False,
      parkingBrake=False,
    ),
  )
  hud_control = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=1.0, jerk_l=5.0)

  first_msg, first_value = create_acc_control_scc2(
    FakePacker(), can, True, 0.0, -4.0, False, False, 100.0, hud_control, jerk, cs,
  )
  second_msg, second_value = create_acc_control_scc2(
    FakePacker(), can, True, first_value, -4.0, False, False, 100.0, hud_control, jerk, cs,
  )

  assert first_msg[2]["aReqRaw"] == pytest.approx(-4.0)
  assert first_msg[2]["aReqValue"] == pytest.approx(-0.1)
  assert second_msg[2]["aReqValue"] == pytest.approx(-0.2)
  assert second_value == pytest.approx(-0.2)


def test_camera_scc_interlock_disables_accel_and_stop_request():
  class FakePacker:
    @staticmethod
    def make_can_msg(name, bus, values):
      return name, bus, values.copy()

  cs = SimpleNamespace(
    scc_control={"ACC_ObjRelSpd": 0.0, "InfoDisplay": 0},
    softHoldActive=0,
    paddle_button_prev=0,
    out=SimpleNamespace(
      aEgo=0.0,
      vEgo=0.0,
      brakeHoldActive=True,
      parkingBrake=False,
    ),
  )
  hud_control = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=1.0, jerk_l=5.0)

  msg, value = create_acc_control_scc2(
    FakePacker(), SimpleNamespace(ECAN=0), True, -1.0, -2.0, True, False, 30.0, hud_control, jerk, cs,
  )

  assert msg[2]["ACCMode"] == 0
  assert msg[2]["StopReq"] == 0
  assert msg[2]["aReqValue"] == 0
  assert value == 0
