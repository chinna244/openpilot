"""Which torque tune version will run, per model size (develop's controls/lib/torque_tune.py)."""

# one tune per model size, keyed by modelV2.big
TUNE_PARAM_BY_SIZE = {False: "TorqueControlTune", True: "TorqueControlTuneBig"}
V2 = 2.0


def stored_tune_versions(params) -> dict[bool, float]:
  """What the params store per size, unset resolved through the declared defaults (v0 small, v1 big)."""
  return {big: float(params.get(key, return_default=True)) for big, key in TUNE_PARAM_BY_SIZE.items()}


def resolved_tune_versions(params, torque_lateral_tuning: bool = True) -> dict[bool, float | None]:
  """The tune version the controller will actually run per model size, or None for the
  upstream controller. With EnforceTorqueControl off, torque-tuned cars run v0 regardless
  of the stored tune, and non-torque cars run the upstream controller."""
  if not params.get_bool("EnforceTorqueControl"):
    return dict.fromkeys(TUNE_PARAM_BY_SIZE, 0.0 if torque_lateral_tuning else None)
  return stored_tune_versions(params)

