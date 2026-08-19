from pathlib import Path
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.mazda.mazdacan import (OEM_LL1_HUD_GREEN, OEM_LL1_HUD_OFF,
                                       OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3,
                                       OEM_LL1_HUD_WHITE)

from openpilot.sunnypilot.tools.mazda_hud_route_audit import (CAM_LANEINFO_ADDR, FSC_BUS,
                                                             DrivingContext, EvidenceCollector,
                                                             classify_payload, decode_cam_laneinfo)


def _packed_440(**updates) -> bytes:
  values = decode_cam_laneinfo(OEM_LL1_HUD_OFF)
  values.update(updates)
  return bytes(CANPacker("mazda_2017").make_can_msg("CAM_LANEINFO", 0, values)[1])


def _message(which, timestamp_s=1.0, **values):
  return SimpleNamespace(which=lambda: which, logMonoTime=int(timestamp_s * 1e9), **values)


def test_conservative_payload_classification():
  assert classify_payload(OEM_LL1_HUD_OFF)[0] == "baseline_family"
  assert classify_payload(OEM_LL1_HUD_WHITE)[0] == "baseline_family"
  assert classify_payload(OEM_LL1_HUD_OFF_TJA2)[0] == "experimental_variant"
  assert classify_payload(OEM_LL1_HUD_OFF_TJA3)[0] == "experimental_variant"
  assert classify_payload(OEM_LL1_HUD_GREEN)[0] == "known_green_template"

  ldw = _packed_440(LDW_WARN_LL=1)
  classification, reasons = classify_payload(ldw)
  assert classification == "warning_or_fault"
  assert reasons == ["ldw_warn_ll"]

  unknown = bytearray(OEM_LL1_HUD_OFF)
  unknown[0] ^= 0x1  # LINE_VISIBLE changes without setting any known warning/fault bit.
  assert classify_payload(bytes(unknown))[0] == "unreviewed_no_known_warning"


def test_collector_correlates_context_transitions_and_output():
  collector = EvidenceCollector()
  collector.start_file(Path("route/rlog.zst"), "route-1")
  collector.context = DrivingContext(lat_active=True, mads_enabled=True, mads_active=True,
                                     lateral_auth=True, v_ego=12.5, left_lane_prob=0.8,
                                     right_lane_prob=0.7)
  collector.observe_input(OEM_LL1_HUD_OFF_TJA2, 10.0)
  collector.observe_input(OEM_LL1_HUD_OFF_TJA2, 10.5)
  collector.observe_output(OEM_LL1_HUD_GREEN)
  collector.observe_input(OEM_LL1_HUD_OFF_TJA3, 11.0)
  collector.observe_output(OEM_LL1_HUD_GREEN)

  report = collector.to_dict()
  payloads = {item["payload"]: item for item in report["fsc_payloads"]}
  tja2 = payloads[OEM_LL1_HUD_OFF_TJA2.hex()]
  assert tja2["count"] == 2
  assert tja2["longest_contiguous_run_s"] == 0.5
  assert tja2["contexts"]["lat_active"] == {"true": 2}
  assert tja2["speed"]["mean_mps"] == 12.5
  assert report["input_transitions"] == {
    f"{OEM_LL1_HUD_OFF_TJA2.hex()}->{OEM_LL1_HUD_OFF_TJA3.hex()}": 1,
  }
  assert report["input_to_output"][OEM_LL1_HUD_OFF_TJA2.hex()] == {OEM_LL1_HUD_GREEN.hex(): 1}
  assert report["input_to_output"][OEM_LL1_HUD_OFF_TJA3.hex()] == {OEM_LL1_HUD_GREEN.hex(): 1}


def test_process_message_reads_fsc_and_sendcan_only():
  collector = EvidenceCollector()
  collector.start_file(Path("route/rlog.zst"), "route-1")
  packer = CANPacker("mazda_2017")
  crz_btns = bytes(packer.make_can_msg("CRZ_BTNS", 0, {
    "TJA_BUTTON": 1, "MODE_X": 0, "MODE_Y": 0, "CAN_OFF": 0, "RES": 0, "SET_P": 0, "SET_M": 0,
  })[1])
  cam_lkas = bytes(packer.make_can_msg("CAM_LKAS", 0, {
    "ERR_BIT_1": 0, "ERR_BIT_2": 0, "LINE_NOT_VISIBLE": 0, "LDW": 0, "LKAS_REQUEST": 100,
  })[1])
  collector.process_message(_message(
    "carControl",
    carControl=SimpleNamespace(
      enabled=True,
      latActive=True,
      actuators=SimpleNamespace(torque=0.25),
      hudControl=SimpleNamespace(
        leftLaneDepart=False, rightLaneDepart=False, lanesVisible=True,
        leftLaneVisible=True, rightLaneVisible=True, visualAlert="none",
      ),
    ),
  ))
  collector.process_message(_message(
    "carControlSP",
    carControlSP=SimpleNamespace(mads=SimpleNamespace(enabled=True, active=True, available=True, state="enabled")),
  ))
  collector.process_message(_message(
    "can", 2.0,
    can=[
      SimpleNamespace(address=CAM_LANEINFO_ADDR, src=FSC_BUS, dat=OEM_LL1_HUD_OFF_TJA2),
      SimpleNamespace(address=CAM_LANEINFO_ADDR, src=0, dat=OEM_LL1_HUD_OFF),
      SimpleNamespace(address=0x09D, src=0, dat=crz_btns),
      SimpleNamespace(address=0x243, src=FSC_BUS, dat=cam_lkas),
    ],
  ))
  collector.process_message(_message(
    "sendcan", 2.1,
    sendcan=[SimpleNamespace(address=CAM_LANEINFO_ADDR, src=0, dat=OEM_LL1_HUD_GREEN)],
  ))

  report = collector.to_dict()
  assert report["summary"]["total_fsc_frames"] == 1
  assert report["fsc_payloads"][0]["payload"] == OEM_LL1_HUD_OFF_TJA2.hex()
  assert report["outgoing_payloads"] == {OEM_LL1_HUD_GREEN.hex(): 1}
  assert report["related_can"]["can_src_2:CAM_LANEINFO"]["count"] == 1
  assert report["related_can"]["sendcan_bus_0:CAM_LANEINFO"]["count"] == 1
  assert report["related_can"]["can_src_0:CRZ_BTNS"]["state_signatures"] == {
    "TJA_BUTTON=1,MODE_X=0,MODE_Y=0,CAN_OFF=0,RES=0,SET_P=0,SET_M=0": 1,
  }
  assert report["related_can"]["can_src_2:CAM_LKAS"]["state_signatures"] == {
    "ERR_BIT_1=0,ERR_BIT_2=0,LINE_NOT_VISIBLE=0,LDW=0,LKAS_REQUEST=100": 1,
  }
  laneinfo_signature = ",".join((
    "TJA=0", "TJA_TRANSITION=2", "LANE_LINES=1", "LDW_WARN_LL=0", "LDW_WARN_RL=0",
    "HANDS_ON_STEER_WARN=0", "HANDS_ON_STEER_WARN_2=0", "HANDS_WARN_3_BITS=0",
    "ERR_BIT=0", "NO_ERR_BIT=0",
  ))
  assert report["related_can"]["can_src_2:CAM_LANEINFO"]["state_signatures"] == {laneinfo_signature: 1}
