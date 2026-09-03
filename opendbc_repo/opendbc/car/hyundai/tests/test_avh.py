from opendbc.can import CANDefine
from opendbc.can.dbc import DBC
from opendbc.car.hyundai.carstate import CANFD_AVH_RELEASE_GRACE_FRAMES, update_canfd_auto_hold_interlock_state


def test_canfd_avh_status_definition():
  dbc = DBC("hyundai_canfd_generated")
  signals = dbc.name_to_msg["ESP_STATUS"].sigs

  assert signals["AVH_Sta"].start_bit == 192
  assert signals["AVH_Sta"].size == 2
  assert signals["AVH_I_LAMP"].start_bit == 218
  assert signals["AVH_I_LAMP"].size == 2
  assert signals["AVH_LAMP"].start_bit == 220
  assert signals["AVH_LAMP"].size == 3

  definitions = CANDefine("hyundai_canfd_generated").dv["ESP_STATUS"]
  assert definitions["AVH_Sta"][1] == "VEHICLE_IS_HELD_BY_THE_SERVICE_BRAKE"
  assert definitions["AVH_LAMP"][2] == "AVH_ACTIVE"
  assert definitions["AVH_LAMP"][3] == "AVH_READY"


def classify_avh_sequence(sequence):
  oem_hold_latched = False
  release_grace_frames = 0
  results = []
  for avh_state, avh_lamp in sequence:
    oem_hold_latched, release_grace_frames = update_canfd_auto_hold_interlock_state(
      avh_state, avh_lamp, oem_hold_latched, release_grace_frames,
    )
    results.append(oem_hold_latched)
  return results


def test_canfd_oem_autohold_latches_and_releases():
  sequence = [(0, 3), (1, 2), (2, 3)]
  assert classify_avh_sequence(sequence) == [False, True, True]

  release = [(1, 2)] + [(0, 3)] * CANFD_AVH_RELEASE_GRACE_FRAMES
  results = classify_avh_sequence(release)
  assert all(results[:-1])
  assert not results[-1]


def test_canfd_scc_or_soft_hold_does_not_latch_autohold():
  sequence = [(0, 0), (1, 0), (2, 0), (0, 0)]
  assert classify_avh_sequence(sequence) == [False] * len(sequence)
