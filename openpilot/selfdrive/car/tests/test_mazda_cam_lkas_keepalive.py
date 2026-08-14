"""Startup CAM_LKAS keepalive: leftover mazda relay must not starve 0x243.

Failing ignition: ff7df7d6f9c3403b|00000014--19759b82aa
  panda mazda/safetyParam 2 from t≈0 (relay already blocking stock CAM_LKAS)
  card running ~0.56 s
  FSC CAM_LANEINFO ERR_BIT at ~3.831 s, CAM_LKAS ERR_BIT_1/2 at ~3.861 s
  first openpilot CAM_LKAS sendcan ~9.165 s
  OEM LKAS latched until vehicle power cycle

The unfixed card.py gate (`if initialized`) does not send on this timeline.
"""

from openpilot.selfdrive.car.mazda_cam_lkas_keepalive import (
  panda_safety_model_is_mazda, should_send_mazda_cam_lkas,
)

# Route 00000014 relative seconds (rlog).
ROUTE_00000014_CARD_RUNNING_S = 0.56
ROUTE_00000014_FSC_ERR_S = 3.831
ROUTE_00000014_FIRST_OP_CAM_LKAS_S = 9.165
FSC_LATCH_MS = 3831


def test_unfixed_initialized_gate_gaps_past_fsc_latch():
  # Pre-fix: card.py only called controls_update when selfdriveInitializing cleared.
  initialized = False
  unfixed_would_send = (not False) and initialized  # `if not passive and initialized`
  assert unfixed_would_send is False
  unfixed_gap_ms = (ROUTE_00000014_FIRST_OP_CAM_LKAS_S - ROUTE_00000014_CARD_RUNNING_S) * 1000
  assert unfixed_gap_ms > FSC_LATCH_MS
  assert (ROUTE_00000014_FSC_ERR_S - ROUTE_00000014_CARD_RUNNING_S) * 1000 > 3000


def test_route_00000014_keepalive_sends_while_initializing():
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=False, panda_safety_mazda=True,
  )


def test_first_onroad_elm327_still_waits_for_initialized():
  # Route 00000013: stock CAM_LKAS flows until mazda+ControlsReady together.
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=False, panda_safety_mazda=False,
  ) is False
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=True, panda_safety_mazda=False,
  )


def test_keepalive_is_mazda_only():
  assert should_send_mazda_cam_lkas(
    passive=False, brand="toyota", initialized=False, panda_safety_mazda=True,
  ) is False
  assert should_send_mazda_cam_lkas(
    passive=True, brand="mazda", initialized=False, panda_safety_mazda=True,
  ) is False


def test_route_0000001a_stock_cam_lkas_until_mazda_safety():
  # Cold start: panda elm327 until ControlsReady. Stock CAM_LKAS remains on
  # bus 0. Do not treat first openpilot CAM_LKAS at initialized (~11 s) as a
  # keepalive failure: relay was not blocking stock yet.
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=False, panda_safety_mazda=False,
  ) is False
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=True, panda_safety_mazda=False,
  )
  # Leftover mazda relay still sends before initialized (route 00000014).
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=False, panda_safety_mazda=True,
  )
  assert should_send_mazda_cam_lkas(
    passive=False, brand="mazda", initialized=True, panda_safety_mazda=True,
  )


def test_panda_safety_model_name():
  assert panda_safety_model_is_mazda("mazda")
  assert panda_safety_model_is_mazda("CarParams.SafetyModel.mazda")
  assert not panda_safety_model_is_mazda("elm327")
  assert not panda_safety_model_is_mazda("noOutput")
