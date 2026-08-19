#!/usr/bin/env python3
"""Build a read-only evidence report for Mazda FSC/cluster HUD traffic.

This tool never transmits CAN and never modifies params. It scans rlogs, records
every FSC bus-2 CAM_LANEINFO (0x440), correlates it with the latest driving
context, and reports what openpilot sent back on 0x440.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field
import json
from pathlib import Path
import re
from typing import Any

from opendbc.can import CANParser
from opendbc.car.mazda.mazdacan import (OEM_LL1_HUD_FAMILY, OEM_LL1_HUD_GREEN,
                                       OEM_LL1_HUD_GREEN_VARIANTS)
from openpilot.tools.lib.logreader import LogReader


CAM_LANEINFO_ADDR = 0x440
FSC_BUS = 2
MAX_CONTIGUOUS_GAP_S = 1.25  # FSC is nominally ~2 Hz; tolerate one delayed sample.
MAX_RELATED_PAYLOAD_SAMPLES = 128
IGNORED_SAFETY_MODELS = {"silent", "noOutput", "elm327"}
RELATED_CAN_MESSAGES = {
  0x09D: "CRZ_BTNS",     # physical TJA/MODE/CANCEL and ICBM button traffic
  0x243: "CAM_LKAS",     # FSC health/visibility and openpilot steering request
  0x440: "CAM_LANEINFO", # FSC and cluster HUD state
}
RELATED_STATE_FIELDS = {
  "CRZ_BTNS": ("TJA_BUTTON", "MODE_X", "MODE_Y", "CAN_OFF", "RES", "SET_P", "SET_M"),
  "CAM_LKAS": ("ERR_BIT_1", "ERR_BIT_2", "LINE_NOT_VISIBLE", "LDW", "LKAS_REQUEST"),
  "CAM_LANEINFO": ("TJA", "TJA_TRANSITION", "LANE_LINES", "LDW_WARN_LL", "LDW_WARN_RL",
                     "HANDS_ON_STEER_WARN", "HANDS_ON_STEER_WARN_2", "HANDS_WARN_3_BITS",
                     "ERR_BIT", "NO_ERR_BIT"),
}

WARNING_SIGNALS = (
  "LDW_WARN_LL",
  "LDW_WARN_RL",
  "HANDS_WARN_3_BITS",
  "HANDS_ON_STEER_WARN",
  "HANDS_ON_STEER_WARN_2",
)


def decode_can_payload(message_name: str, address: int, dat: bytes) -> dict[str, int]:
  if len(dat) != 8:
    raise ValueError(f"{message_name} must be 8 bytes, got {len(dat)}")
  parser = CANParser("mazda_2017", [(message_name, float("nan"))], 0)
  parser.update([(0, [(address, dat, 0)])])
  return {name: int(value) for name, value in parser.vl[message_name].items()}


def decode_cam_laneinfo(dat: bytes) -> dict[str, int]:
  return decode_can_payload("CAM_LANEINFO", CAM_LANEINFO_ADDR, dat)


def classify_payload(dat: bytes, decoded: dict[str, int] | None = None) -> tuple[str, list[str]]:
  """Conservatively classify a payload without declaring unknown states safe."""
  if dat in OEM_LL1_HUD_FAMILY:
    return "baseline_family", []
  if dat in OEM_LL1_HUD_GREEN_VARIANTS:
    return "experimental_variant", []
  if dat == OEM_LL1_HUD_GREEN:
    return "known_green_template", []

  signals = decoded if decoded is not None else decode_cam_laneinfo(dat)
  reasons: list[str] = []
  if signals.get("NO_ERR_BIT", 0):
    reasons.append("boot_marker")
  if signals.get("ERR_BIT", 0):
    reasons.append("fsc_error")
  for signal in WARNING_SIGNALS:
    if signals.get(signal, 0):
      reasons.append(signal.lower())

  if reasons:
    return "warning_or_fault", reasons
  return "unreviewed_no_known_warning", []


def _bool_key(value: bool | None) -> str:
  return "unknown" if value is None else str(value).lower()


@dataclass
class DrivingContext:
  controls_enabled: bool | None = None
  lat_active: bool | None = None
  requested_torque: float | None = None
  mads_enabled: bool | None = None
  mads_active: bool | None = None
  mads_available: bool | None = None
  mads_state: str | None = None
  lateral_auth: bool | None = None
  panda_controls_allowed: bool | None = None
  panda_faults: tuple[str, ...] = ()
  v_ego: float | None = None
  cruise_enabled: bool | None = None
  left_blinker: bool | None = None
  right_blinker: bool | None = None
  steering_pressed: bool | None = None
  steer_fault_temporary: bool | None = None
  steer_fault_permanent: bool | None = None
  invalid_lkas_setting: bool | None = None
  low_speed_alert: bool | None = None
  left_lane_prob: float | None = None
  right_lane_prob: float | None = None
  left_lane_y0: float | None = None
  right_lane_y0: float | None = None
  left_lane_departure: bool | None = None
  right_lane_departure: bool | None = None
  hud_left_lane_departure: bool | None = None
  hud_right_lane_departure: bool | None = None
  hud_lanes_visible: bool | None = None
  hud_left_lane_visible: bool | None = None
  hud_right_lane_visible: bool | None = None
  hud_visual_alert: str | None = None
  active_events: tuple[str, ...] = ()


@dataclass
class PayloadEvidence:
  dat: bytes
  decoded: dict[str, int]
  classification: str
  classification_reasons: list[str]
  count: int = 0
  routes: set[str] = field(default_factory=set)
  first_seen_s: dict[str, float] = field(default_factory=dict)
  last_seen_s: dict[str, float] = field(default_factory=dict)
  longest_run_s: float = 0.0
  speed_count: int = 0
  speed_sum: float = 0.0
  speed_min: float | None = None
  speed_max: float | None = None
  requested_torque_count: int = 0
  requested_torque_sum: float = 0.0
  requested_torque_min: float | None = None
  requested_torque_max: float | None = None
  left_lane_prob_count: int = 0
  left_lane_prob_sum: float = 0.0
  right_lane_prob_count: int = 0
  right_lane_prob_sum: float = 0.0
  left_lane_y0_count: int = 0
  left_lane_y0_sum: float = 0.0
  right_lane_y0_count: int = 0
  right_lane_y0_sum: float = 0.0
  contexts: dict[str, Counter[str]] = field(default_factory=lambda: defaultdict(Counter))

  def observe(self, route: str, timestamp_s: float, context: DrivingContext) -> None:
    self.count += 1
    self.routes.add(route)
    self.first_seen_s.setdefault(route, timestamp_s)
    self.last_seen_s[route] = timestamp_s

    for name in (
      "controls_enabled", "lat_active", "mads_enabled", "mads_active", "mads_available", "lateral_auth",
      "panda_controls_allowed", "cruise_enabled",
      "left_blinker", "right_blinker", "steering_pressed", "steer_fault_temporary",
      "steer_fault_permanent", "invalid_lkas_setting", "low_speed_alert",
      "left_lane_departure", "right_lane_departure", "hud_left_lane_departure",
      "hud_right_lane_departure", "hud_lanes_visible", "hud_left_lane_visible", "hud_right_lane_visible",
    ):
      self.contexts[name][_bool_key(getattr(context, name))] += 1
    self.contexts["mads_state"][context.mads_state or "unknown"] += 1
    self.contexts["hud_visual_alert"][context.hud_visual_alert or "unknown"] += 1
    for event in context.active_events or ("none",):
      self.contexts["active_event"][event] += 1
    for fault in context.panda_faults or ("none",):
      self.contexts["panda_fault"][fault] += 1

    if context.v_ego is not None:
      speed = float(context.v_ego)
      self.speed_count += 1
      self.speed_sum += speed
      self.speed_min = speed if self.speed_min is None else min(self.speed_min, speed)
      self.speed_max = speed if self.speed_max is None else max(self.speed_max, speed)
    if context.requested_torque is not None:
      torque = float(context.requested_torque)
      self.requested_torque_count += 1
      self.requested_torque_sum += torque
      self.requested_torque_min = torque if self.requested_torque_min is None else min(self.requested_torque_min, torque)
      self.requested_torque_max = torque if self.requested_torque_max is None else max(self.requested_torque_max, torque)
    if context.left_lane_prob is not None:
      self.left_lane_prob_count += 1
      self.left_lane_prob_sum += float(context.left_lane_prob)
    if context.right_lane_prob is not None:
      self.right_lane_prob_count += 1
      self.right_lane_prob_sum += float(context.right_lane_prob)
    if context.left_lane_y0 is not None:
      self.left_lane_y0_count += 1
      self.left_lane_y0_sum += float(context.left_lane_y0)
    if context.right_lane_y0 is not None:
      self.right_lane_y0_count += 1
      self.right_lane_y0_sum += float(context.right_lane_y0)

  def to_dict(self) -> dict[str, Any]:
    speed = None
    if self.speed_count:
      speed = {
        "min_mps": self.speed_min,
        "max_mps": self.speed_max,
        "mean_mps": self.speed_sum / self.speed_count,
      }
    requested_torque = None
    if self.requested_torque_count:
      requested_torque = {
        "min": self.requested_torque_min,
        "max": self.requested_torque_max,
        "mean": self.requested_torque_sum / self.requested_torque_count,
      }
    lane_prob = {
      "left_mean": self.left_lane_prob_sum / self.left_lane_prob_count if self.left_lane_prob_count else None,
      "right_mean": self.right_lane_prob_sum / self.right_lane_prob_count if self.right_lane_prob_count else None,
    }
    lane_position = {
      "left_y0_mean_m": self.left_lane_y0_sum / self.left_lane_y0_count if self.left_lane_y0_count else None,
      "right_y0_mean_m": self.right_lane_y0_sum / self.right_lane_y0_count if self.right_lane_y0_count else None,
    }
    return {
      "payload": self.dat.hex(),
      "classification": self.classification,
      "classification_reasons": self.classification_reasons,
      "count": self.count,
      "routes": sorted(self.routes),
      "first_seen_s": dict(sorted(self.first_seen_s.items())),
      "last_seen_s": dict(sorted(self.last_seen_s.items())),
      "longest_contiguous_run_s": self.longest_run_s,
      "decoded": dict(sorted(self.decoded.items())),
      "speed": speed,
      "requested_torque": requested_torque,
      "lane_probability": lane_prob,
      "lane_position": lane_position,
      "contexts": {name: dict(sorted(counts.items())) for name, counts in sorted(self.contexts.items())},
    }


@dataclass
class CanStreamEvidence:
  message_name: str
  address: int
  count: int = 0
  payloads: Counter[str] = field(default_factory=Counter)
  unsampled_payload_count: int = 0
  state_signatures: Counter[str] = field(default_factory=Counter)
  first_seen_s: dict[str, float] = field(default_factory=dict)
  last_seen_s: dict[str, float] = field(default_factory=dict)
  interval_count: int = 0
  interval_sum_s: float = 0.0
  interval_min_s: float | None = None
  interval_max_s: float | None = None
  _last_timestamp_s: dict[str, float] = field(default_factory=dict)
  _parser: Any = field(init=False, repr=False)

  def __post_init__(self) -> None:
    self._parser = CANParser("mazda_2017", [(self.message_name, float("nan"))], 0)

  def observe(self, route: str, timestamp_s: float, dat: bytes) -> None:
    self.count += 1
    payload = dat.hex()
    if payload in self.payloads or len(self.payloads) < MAX_RELATED_PAYLOAD_SAMPLES:
      self.payloads[payload] += 1
    else:
      self.unsampled_payload_count += 1
    self._parser.update([(0, [(self.address, dat, 0)])])
    decoded = self._parser.vl[self.message_name]
    signature = ",".join(f"{name}={int(decoded[name])}" for name in RELATED_STATE_FIELDS[self.message_name])
    self.state_signatures[signature] += 1
    self.first_seen_s.setdefault(route, timestamp_s)
    self.last_seen_s[route] = timestamp_s
    if route in self._last_timestamp_s:
      interval = timestamp_s - self._last_timestamp_s[route]
      if interval >= 0:
        self.interval_count += 1
        self.interval_sum_s += interval
        self.interval_min_s = interval if self.interval_min_s is None else min(self.interval_min_s, interval)
        self.interval_max_s = interval if self.interval_max_s is None else max(self.interval_max_s, interval)
    self._last_timestamp_s[route] = timestamp_s

  def to_dict(self) -> dict[str, Any]:
    timing = None
    if self.interval_count:
      timing = {
        "mean_interval_s": self.interval_sum_s / self.interval_count,
        "min_interval_s": self.interval_min_s,
        "max_interval_s": self.interval_max_s,
      }
    decoded_payloads = {
      payload: decode_can_payload(self.message_name, self.address, bytes.fromhex(payload))
      for payload in sorted(self.payloads)
    }
    return {
      "message": self.message_name,
      "address": hex(self.address),
      "count": self.count,
      "payloads": dict(sorted(self.payloads.items())),
      "payload_sample_limit": MAX_RELATED_PAYLOAD_SAMPLES,
      "unsampled_payload_count": self.unsampled_payload_count,
      "decoded_payloads": decoded_payloads,
      "state_signatures": dict(sorted(self.state_signatures.items())),
      "first_seen_s": dict(sorted(self.first_seen_s.items())),
      "last_seen_s": dict(sorted(self.last_seen_s.items())),
      "timing": timing,
    }


class EvidenceCollector:
  def __init__(self) -> None:
    self.context = DrivingContext()
    self.payloads: dict[bytes, PayloadEvidence] = {}
    self.outgoing = Counter[str]()
    self.input_to_output: dict[str, Counter[str]] = defaultdict(Counter)
    self.transitions = Counter[str]()
    self.hud_logs = Counter[str]()
    self.related_can: dict[str, CanStreamEvidence] = {}
    self.bookmarks: list[dict[str, Any]] = []
    self.builds: dict[str, dict[str, Any]] = {}
    self.car_metadata: dict[str, dict[str, Any]] = {}
    self.files: list[str] = []
    self._route: str | None = None
    self._last_input: bytes | None = None
    self._last_input_time_s: float | None = None
    self._run_start_s: float | None = None

  def start_file(self, path: Path, route: str) -> None:
    self._finish_run()
    self.files.append(str(path))
    self._route = route
    self._last_input = None
    self._last_input_time_s = None
    self._run_start_s = None

  def _finish_run(self) -> None:
    if self._last_input is not None and self._last_input_time_s is not None and self._run_start_s is not None:
      evidence = self.payloads[self._last_input]
      evidence.longest_run_s = max(evidence.longest_run_s, self._last_input_time_s - self._run_start_s)

  def observe_input(self, dat: bytes, timestamp_s: float) -> None:
    if self._route is None:
      raise RuntimeError("start_file must be called before observing messages")
    if dat not in self.payloads:
      decoded = decode_cam_laneinfo(dat)
      classification, reasons = classify_payload(dat, decoded)
      self.payloads[dat] = PayloadEvidence(dat, decoded, classification, reasons)
    self.payloads[dat].observe(self._route, timestamp_s, self.context)

    contiguous = self._last_input == dat and self._last_input_time_s is not None and \
                 timestamp_s - self._last_input_time_s <= MAX_CONTIGUOUS_GAP_S
    if not contiguous:
      self._finish_run()
      if self._last_input is not None:
        self.transitions[f"{self._last_input.hex()}->{dat.hex()}"] += 1
      self._run_start_s = timestamp_s
    self._last_input = dat
    self._last_input_time_s = timestamp_s

  def observe_output(self, dat: bytes) -> None:
    payload = dat.hex()
    self.outgoing[payload] += 1
    if self._last_input is not None:
      self.input_to_output[self._last_input.hex()][payload] += 1

  def observe_related_can(self, channel: str, address: int, dat: bytes, timestamp_s: float) -> None:
    if self._route is None or address not in RELATED_CAN_MESSAGES or len(dat) != 8:
      return
    message_name = RELATED_CAN_MESSAGES[address]
    key = f"{channel}:{message_name}"
    if key not in self.related_can:
      self.related_can[key] = CanStreamEvidence(message_name, address)
    self.related_can[key].observe(self._route, timestamp_s, dat)

  def process_message(self, message: Any) -> None:
    which = message.which()
    if which == "carControl":
      control = message.carControl
      hud = control.hudControl
      self.context.controls_enabled = bool(control.enabled)
      self.context.lat_active = bool(control.latActive)
      self.context.requested_torque = float(control.actuators.torque)
      self.context.hud_left_lane_departure = bool(hud.leftLaneDepart)
      self.context.hud_right_lane_departure = bool(hud.rightLaneDepart)
      self.context.hud_lanes_visible = bool(hud.lanesVisible)
      self.context.hud_left_lane_visible = bool(hud.leftLaneVisible)
      self.context.hud_right_lane_visible = bool(hud.rightLaneVisible)
      self.context.hud_visual_alert = str(hud.visualAlert)
    elif which == "carControlSP":
      mads = message.carControlSP.mads
      self.context.mads_enabled = bool(mads.enabled)
      self.context.mads_active = bool(mads.active)
      self.context.mads_available = bool(mads.available)
      self.context.mads_state = str(mads.state)
    elif which == "carState":
      state = message.carState
      self.context.v_ego = float(state.vEgo)
      self.context.cruise_enabled = bool(state.cruiseState.enabled)
      self.context.left_blinker = bool(state.leftBlinker)
      self.context.right_blinker = bool(state.rightBlinker)
      self.context.steering_pressed = bool(state.steeringPressed)
      self.context.steer_fault_temporary = bool(state.steerFaultTemporary)
      self.context.steer_fault_permanent = bool(state.steerFaultPermanent)
      self.context.invalid_lkas_setting = bool(state.invalidLkasSetting)
      self.context.low_speed_alert = bool(state.lowSpeedAlert)
    elif which == "modelV2":
      model = message.modelV2
      probs = list(model.laneLineProbs)
      if len(probs) >= 3:
        self.context.left_lane_prob = float(probs[1])
        self.context.right_lane_prob = float(probs[2])
      lane_lines = list(model.laneLines)
      if len(lane_lines) >= 3 and len(lane_lines[1].y) and len(lane_lines[2].y):
        self.context.left_lane_y0 = float(lane_lines[1].y[0])
        self.context.right_lane_y0 = float(lane_lines[2].y[0])
    elif which == "driverAssistance":
      assistance = message.driverAssistance
      self.context.left_lane_departure = bool(assistance.leftLaneDeparture)
      self.context.right_lane_departure = bool(assistance.rightLaneDeparture)
    elif which == "pandaStates":
      relevant = [p for p in message.pandaStates if str(p.safetyModel) not in IGNORED_SAFETY_MODELS]
      self.context.lateral_auth = bool(relevant) and all(bool(p.controlsAllowedLateral) for p in relevant)
      self.context.panda_controls_allowed = bool(relevant) and all(bool(p.controlsAllowed) for p in relevant)
      self.context.panda_faults = tuple(sorted({str(fault) for panda in relevant for fault in panda.faults}))
    elif which == "onroadEvents":
      self.context.active_events = tuple(sorted(str(event.name) for event in message.onroadEvents))
    elif which == "can":
      for frame in message.can:
        self.observe_related_can(f"can_src_{frame.src}", frame.address, bytes(frame.dat), message.logMonoTime / 1e9)
        if frame.address == CAM_LANEINFO_ADDR and frame.src == FSC_BUS and len(frame.dat) == 8:
          self.observe_input(bytes(frame.dat), message.logMonoTime / 1e9)
    elif which == "sendcan":
      for frame in message.sendcan:
        self.observe_related_can(f"sendcan_bus_{frame.src}", frame.address, bytes(frame.dat), message.logMonoTime / 1e9)
        if frame.address == CAM_LANEINFO_ADDR and len(frame.dat) == 8:
          self.observe_output(bytes(frame.dat))
    elif which == "carParams" and self._route is not None:
      params = message.carParams
      self.car_metadata.setdefault(self._route, {}).update({
        "brand": str(params.brand),
        "fingerprint": str(params.carFingerprint),
        "flags": int(params.flags),
        "openpilot_longitudinal": bool(params.openpilotLongitudinalControl),
        "safety_configs": [
          {"model": str(config.safetyModel), "param": int(config.safetyParam)} for config in params.safetyConfigs
        ],
      })
    elif which == "carParamsSP" and self._route is not None:
      self.car_metadata.setdefault(self._route, {})["flags_sp"] = int(message.carParamsSP.flags)
    elif which == "userBookmark" and self._route is not None:
      bookmark = message.userBookmark.to_dict() if hasattr(message.userBookmark, "to_dict") else {}
      self.bookmarks.append({"route": self._route, "timestamp_s": message.logMonoTime / 1e9, "data": bookmark})
    elif which == "logMessage":
      try:
        log_record = json.loads(message.logMessage)
        log_text = log_record.get("msg", message.logMessage)
        context = log_record.get("ctx", {})
        if self._route is not None and context:
          self.builds[self._route] = {
            key: context.get(key) for key in ("version", "origin", "branch", "commit", "dirty", "device")
          }
      except json.JSONDecodeError:
        log_text = message.logMessage
      if "mads_hud" in str(log_text):
        self.hud_logs[str(log_text)] += 1

  def finish(self) -> None:
    self._finish_run()

  def to_dict(self) -> dict[str, Any]:
    self.finish()
    classifications = Counter(e.classification for e in self.payloads.values())
    return {
      "schema_version": 1,
      "files": self.files,
      "summary": {
        "unique_fsc_payloads": len(self.payloads),
        "total_fsc_frames": sum(e.count for e in self.payloads.values()),
        "classifications": dict(sorted(classifications.items())),
      },
      "fsc_payloads": [self.payloads[dat].to_dict() for dat in sorted(self.payloads)],
      "outgoing_payloads": dict(sorted(self.outgoing.items())),
      "input_to_output": {
        source: dict(sorted(outputs.items())) for source, outputs in sorted(self.input_to_output.items())
      },
      "input_transitions": dict(sorted(self.transitions.items())),
      "hud_transition_logs": dict(sorted(self.hud_logs.items())),
      "related_can": {key: value.to_dict() for key, value in sorted(self.related_can.items())},
      "bookmarks": self.bookmarks,
      "builds": dict(sorted(self.builds.items())),
      "car_metadata": dict(sorted(self.car_metadata.items())),
    }


def _route_label(path: Path) -> str:
  parent = path.parent.name
  match = re.match(r"(.+--[^-]+)--\d+$", parent)
  if match:
    return match.group(1)
  return parent or path.name


def expand_log_paths(inputs: Iterable[str]) -> list[Path]:
  paths: set[Path] = set()
  for value in inputs:
    path = Path(value).expanduser()
    if path.is_file():
      paths.add(path.resolve())
    elif path.is_dir():
      paths.update(p.resolve() for pattern in ("rlog.zst", "rlog.bz2") for p in path.rglob(pattern))
    else:
      raise FileNotFoundError(path)
  return sorted(paths)


def analyze_paths(paths: Iterable[Path]) -> dict[str, Any]:
  collector = EvidenceCollector()
  for path in paths:
    collector.start_file(path, _route_label(path))
    for message in LogReader(str(path)):
      collector.process_message(message)
  return collector.to_dict()


def print_summary(report: dict[str, Any]) -> None:
  summary = report["summary"]
  print(f"FSC 0x440: {summary['total_fsc_frames']} frames, {summary['unique_fsc_payloads']} unique payloads")
  print("payload           count  class                         decoded highlights")
  for item in report["fsc_payloads"]:
    decoded = item["decoded"]
    highlights = " ".join((
      f"TJA={decoded.get('TJA')} TR={decoded.get('TJA_TRANSITION')}",
      f"LL={decoded.get('LANE_LINES')} LDW={decoded.get('LDW_WARN_LL')}/{decoded.get('LDW_WARN_RL')}",
      f"ERR={decoded.get('ERR_BIT')} BOOT={decoded.get('NO_ERR_BIT')}",
    ))
    print(f"{item['payload']}  {item['count']:5d}  {item['classification']:<28}  {highlights}")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("paths", nargs="+", help="rlog files or directories containing route segment rlogs")
  parser.add_argument("--json", type=Path, dest="json_path", help="write the complete evidence report as JSON")
  args = parser.parse_args()

  paths = expand_log_paths(args.paths)
  if not paths:
    parser.error("no rlog.zst or rlog.bz2 files found")
  report = analyze_paths(paths)
  print_summary(report)
  if args.json_path is not None:
    args.json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"wrote {args.json_path}")


if __name__ == "__main__":
  main()
