"""Send Mazda CAM_LKAS whenever panda mazda safety (relay) is already up.

Route ff7df7d6f9c3403b|00000014--19759b82aa (TOP 34b5bd37, OpenDBC 3251e92):
panda leftover mazda / safetyParam 2 from t≈0, card waited for
selfdriveInitializing (~9 s) before the first CAM_LKAS. Stock 0x243 was
relay-blocked. FSC set CAM_LANEINFO ERR_BIT then CAM_LKAS ERR_BIT_1/2 at
~3.83 s. OEM LKAS latched for the rest of the ignition cycle; recovery
requires a full vehicle power cycle.

Do not send during initializing while panda is still elm327: stock CAM_LKAS
still reaches the EPS on bus 0 (route 00000013).
"""


def panda_safety_model_is_mazda(safety_model) -> bool:
  name = str(safety_model).rsplit(".", 1)[-1]
  return name == "mazda"


def should_send_mazda_cam_lkas(*, passive: bool, brand: str, initialized: bool,
                               panda_safety_mazda: bool) -> bool:
  if passive:
    return False
  if initialized:
    return True
  return brand == "mazda" and panda_safety_mazda
