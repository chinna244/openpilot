from opendbc.car import structs
from opendbc.car.mazda.values import CAR, MazdaFlags

CARS = frozenset((CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))


def qualified(cp, *, marked_only=False):
  if not (cp.brand == "mazda" and cp.carFingerprint in CARS and not cp.passive and not cp.notCar and not cp.dashcamOnly and
          cp.transmissionType != structs.CarParams.TransmissionType.manual and
          cp.fingerprintSource != structs.CarParams.FingerprintSource.fixed and cp.pcmCruise and len(cp.safetyConfigs) == 1 and
          cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
          cp.alternativeExperience in ((32,) if marked_only else (0, 32))):
    return False
  flags, word = int(cp.flags), int(cp.safetyConfigs[0].safetyParam)
  legacy = flags == int(MazdaFlags.GEN1) and word == 0 and not cp.openpilotLongitudinalControl
  known = int(MazdaFlags.GEN1 | MazdaFlags.EPS_HW | MazdaFlags.G46L_RADAR | MazdaFlags.HAS_BSM)
  measured = (flags & ~known == 0 and flags & int(MazdaFlags.GEN1) and
              flags & int(MazdaFlags.EPS_HW) in (int(MazdaFlags.STEER_TO_ZERO_EPS), int(MazdaFlags.LEGACY_FW_EPS)) and
              word in (2, 3, 10, 11) and bool(word & 1) == bool(cp.openpilotLongitudinalControl) and
              (not cp.openpilotLongitudinalControl or
               cp.carFingerprint == CAR.MAZDA_CX5_2022 and bool(flags & int(MazdaFlags.STEER_TO_ZERO_EPS)) and cp.alphaLongitudinalAvailable))
  return bool(legacy or measured)


def temporary_restriction(cp, state):
  return bool(state is not None and qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and
              not state.steerFaultPermanent and (state.gearShifter != structs.CarState.GearShifter.drive or
                                                 state.steerFaultTemporary or state.standstill or (cp.minSteerSpeed > 0 and state.vEgo < cp.minSteerSpeed)))


def native_lateral_pause(cp, carstate) -> bool:
  """mazda_aol_permission withholds lateral while 0x241 LKAS_BLOCK is raised: true only while that
  native owner is active (AOL marked), from the interface's raw parse of the same frame. Steer-to-zero
  firmware only: older firmware raises the block below its 45 kph floor by design, where AOL's own
  restriction (vEgo < minSteerSpeed) covers it and carstate keeps base StarPilot's speed-gated fault."""
  return bool(int(cp.flags) & int(MazdaFlags.STEER_TO_ZERO_EPS) and qualified(cp, marked_only=True) and
              getattr(carstate, 'lkas_blocked', False))


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns):
  lost = not panda_ready or native is None
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000
