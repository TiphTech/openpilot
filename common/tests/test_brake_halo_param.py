from openpilot.common.params import Params, ParamKeyFlag


def test_brake_halo_threshold_survives_startup_and_driving_transitions(tmp_path):
  params_path = str(tmp_path / "params")
  params = Params(params_path)
  key = "TopBarBrakeDecelThreshold"
  assert params.get_default_value(key) == 6
  params.put_int(key, 8)

  for flag in (ParamKeyFlag.CLEAR_ON_MANAGER_START,
               ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
               ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION,
               ParamKeyFlag.CLEAR_ON_IGNITION_ON,
               ParamKeyFlag.DEVELOPMENT_ONLY):
    params.clear_all(flag)
    assert Params(params_path).get_int(key) == 8
