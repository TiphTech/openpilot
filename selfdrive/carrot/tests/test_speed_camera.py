from openpilot.selfdrive.carrot.speed_camera import apn_speed_camera_active


def test_apn_speed_camera_requires_valid_apn_zone():
  assert apn_speed_camera_active(2, 1, 80, 200)
  assert apn_speed_camera_active(3, 101, 110, -50)

  assert not apn_speed_camera_active(1, 1, 80, 100)
  assert not apn_speed_camera_active(2, 22, 35, 100)
  assert not apn_speed_camera_active(2, 1, 80, 201)
  assert not apn_speed_camera_active(2, 1, 0, 100)
