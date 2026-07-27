#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import traceback
from collections import Counter, defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

DEFAULT_REALDATA_ROOT = Path("/data/media/0/realdata")
DEFAULT_OUTPUT_ROOT = Path("/data")
DEFAULT_REPOSITORY_ROOT = Path("/data/openpilot")
DEFAULT_ASSISTANCE_ROOT = Path("/data/gps_assistance")
BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
SEGMENT_PATTERN = re.compile(r"^(?P<route>.+--[0-9a-fA-F]+)--(?P<segment>[0-9]+)$")
LOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog")
PARAM_KEYS = ("GitCommit", "GitBranch", "Version", "PublicYumaAlmanacEnabled", "IsOnroad", "IsOffroad")
STATE_FILES = (
  "navigation_cache.json",
  "navigation_cache_previous.json",
  "public_yuma_almanac.json",
  "public_yuma_last_outcome.json",
  "provisional_yuma_last_decision.json",
  "trusted_time_anchor.json",
  "trusted_time_anchor_previous.json",
)
EVENT_KEYWORDS = (
  "gps acquisition milestone",
  "gps acquisition status",
  "gps receiver cycle",
  "gps receiver utc provenance",
  "time assistance",
  "trusted time",
  "navigation assistance restore",
  "receiver cycle initialization",
  "gps public yuma",
  "requested gps navigation database",
  "saved gps navigation assistance cache",
  "watchdog",
  "receiver reset",
  "pigeond",
  "ubloxd",
  "mga",
)


def safe_get(obj: object, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def as_float(value: object, default: float | None = None) -> float | None:
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def as_int(value: object, default: int | None = None) -> int | None:
  try:
    return int(value)
  except (TypeError, ValueError):
    return default


def decode_text(value: object) -> str:
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="replace")
  return str(value)


def extract_event_message(text: str) -> str:
  try:
    payload = json.loads(text)
  except (json.JSONDecodeError, TypeError):
    return text
  if isinstance(payload, dict) and isinstance(payload.get("msg"), str):
    return payload["msg"]
  return text


def is_independent_receiver_utc_event(text: str) -> bool:
  message = extract_event_message(text).lower()
  return (
    "gps receiver utc provenance" in message
    and "classification=receiver_utc_unassisted_gnss" in message
    and "independent=true" in message
  )


def haversine_m(latitude_1: float, longitude_1: float, latitude_2: float, longitude_2: float) -> float:
  radius = 6_371_000.0
  phi_1 = math.radians(latitude_1)
  phi_2 = math.radians(latitude_2)
  delta_phi = math.radians(latitude_2 - latitude_1)
  delta_lambda = math.radians(longitude_2 - longitude_1)
  value = (
    math.sin(delta_phi / 2.0) ** 2
    + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
  )
  return 2.0 * radius * math.asin(min(1.0, math.sqrt(value)))


@dataclass(frozen=True)
class RouteSelection:
  route: str
  segments: tuple[tuple[int, Path], ...]
  newest_mtime: float


@dataclass
class RouteMetrics:
  route: str
  route_start: float | None = None
  route_end: float | None = None
  message_count: int = 0
  service_counts: Counter[str] = field(default_factory=Counter)
  errors: list[str] = field(default_factory=list)
  events: list[tuple[float, str]] = field(default_factory=list)
  used_logs: list[str] = field(default_factory=list)
  gps_samples: int = 0
  fix_samples: int = 0
  positive_timestamp_samples: int = 0
  rawx_reports: int = 0
  first_rawx: float | None = None
  first_nonempty_rawx: float | None = None
  first_gps_measurement: float | None = None
  first_glonass_measurement: float | None = None
  first_valid_gps_week: float | None = None
  first_valid_leap_second: float | None = None
  first_receiver_utc: float | None = None
  first_fix: float | None = None
  first_25m: float | None = None
  first_10m: float | None = None
  first_5m: float | None = None
  last_fix: float | None = None
  best_accuracy: float | None = None
  max_satellites: int = 0
  gps_distance_m: float = 0.0
  vehicle_distance_m: float = 0.0
  previous_gps: tuple[float, float, float] | None = None
  previous_car: tuple[float, float] | None = None

  def note_time(self, monotonic_time: float) -> None:
    if self.route_start is None or monotonic_time < self.route_start:
      self.route_start = monotonic_time
    if self.route_end is None or monotonic_time > self.route_end:
      self.route_end = monotonic_time

  def process_log_message(self, monotonic_time: float, value: object) -> None:
    text = decode_text(value)
    lowered = text.lower()
    if any(keyword in lowered for keyword in EVENT_KEYWORDS):
      self.events.append((monotonic_time, text))
    if self.first_receiver_utc is None and is_independent_receiver_utc_event(text):
      self.first_receiver_utc = monotonic_time

  def process_car_state(self, monotonic_time: float, car_state: object) -> None:
    speed = abs(float(safe_get(car_state, "vEgo", 0.0)))
    if self.previous_car is not None:
      previous_time, previous_speed = self.previous_car
      delta = monotonic_time - previous_time
      if 0.0 < delta <= 2.0:
        self.vehicle_distance_m += ((previous_speed + speed) / 2.0) * delta
    self.previous_car = (monotonic_time, speed)

  def process_gps(self, monotonic_time: float, gps: object) -> None:
    self.gps_samples += 1
    flags = as_int(safe_get(gps, "flags", 0), 0) or 0
    has_fix = bool(flags & 1) or bool(safe_get(gps, "hasFix", False))
    timestamp_ms = as_int(safe_get(gps, "unixTimestampMillis", 0), 0) or 0
    if timestamp_ms > 0:
      self.positive_timestamp_samples += 1

    satellites = as_int(safe_get(gps, "satelliteCount", 0), 0) or 0
    self.max_satellites = max(self.max_satellites, satellites)

    accuracy = as_float(safe_get(gps, "horizontalAccuracy", None), None)
    latitude = as_float(safe_get(gps, "latitude", None), None)
    longitude = as_float(safe_get(gps, "longitude", None), None)

    if not has_fix:
      return

    self.fix_samples += 1
    self.last_fix = monotonic_time
    if self.first_fix is None:
      self.first_fix = monotonic_time

    if accuracy is not None and math.isfinite(accuracy) and accuracy >= 0.0:
      if self.best_accuracy is None or accuracy < self.best_accuracy:
        self.best_accuracy = accuracy
      if accuracy <= 25.0 and self.first_25m is None:
        self.first_25m = monotonic_time
      if accuracy <= 10.0 and self.first_10m is None:
        self.first_10m = monotonic_time
      if accuracy <= 5.0 and self.first_5m is None:
        self.first_5m = monotonic_time

    valid_position = (
      latitude is not None
      and longitude is not None
      and math.isfinite(latitude)
      and math.isfinite(longitude)
      and -90.0 <= latitude <= 90.0
      and -180.0 <= longitude <= 180.0
      and not (latitude == 0.0 and longitude == 0.0)
    )
    if not valid_position:
      return

    assert latitude is not None
    assert longitude is not None
    if self.previous_gps is not None:
      previous_time, previous_latitude, previous_longitude = self.previous_gps
      delta = monotonic_time - previous_time
      if 0.0 < delta <= 5.0:
        distance = haversine_m(previous_latitude, previous_longitude, latitude, longitude)
        if distance / delta <= 100.0:
          self.gps_distance_m += distance
    self.previous_gps = (monotonic_time, latitude, longitude)

  def process_rawx(self, monotonic_time: float, report: object) -> None:
    self.rawx_reports += 1
    if self.first_rawx is None:
      self.first_rawx = monotonic_time

    measurements = tuple(safe_get(report, "measurements", ()) or ())
    if not measurements:
      return

    if self.first_nonempty_rawx is None:
      self.first_nonempty_rawx = monotonic_time

    gnss_ids = {as_int(safe_get(measurement, "gnssId", None), None) for measurement in measurements}
    if 0 in gnss_ids and self.first_gps_measurement is None:
      self.first_gps_measurement = monotonic_time
    if 6 in gnss_ids and self.first_glonass_measurement is None:
      self.first_glonass_measurement = monotonic_time

    gps_week = as_int(safe_get(report, "gpsWeek", None), None)
    if gps_week is not None and gps_week > 0 and self.first_valid_gps_week is None:
      self.first_valid_gps_week = monotonic_time

    leap_seconds = as_int(safe_get(report, "leapSeconds", None), None)
    if leap_seconds is not None and leap_seconds != 0 and self.first_valid_leap_second is None:
      self.first_valid_leap_second = monotonic_time

  def relative(self, value: float | None) -> float | None:
    if value is None or self.route_start is None:
      return None
    return value - self.route_start

  def summary_lines(self, segment_count: int) -> list[str]:
    duration = None if self.route_start is None or self.route_end is None else self.route_end - self.route_start
    fix_span = None
    if self.first_fix is not None and self.last_fix is not None:
      fix_span = max(0.0, self.last_fix - self.first_fix)

    lines = [
      f"route={self.route}",
      f"segments={segment_count}",
      f"log_files={len(self.used_logs)}",
      f"duration_seconds={duration}",
      f"estimated_vehicle_distance_miles={self.vehicle_distance_m / 1609.344:.4f}",
      f"gps_fixed_distance_miles={self.gps_distance_m / 1609.344:.4f}",
      f"fix_span_seconds={fix_span}",
      f"message_count={self.message_count}",
      f"gps_samples={self.gps_samples}",
      f"fix_samples={self.fix_samples}",
      f"positive_timestamp_samples={self.positive_timestamp_samples}",
      f"rawx_reports={self.rawx_reports}",
      f"first_rawx_seconds={self.relative(self.first_rawx)}",
      f"first_nonempty_rawx_seconds={self.relative(self.first_nonempty_rawx)}",
      f"first_gps_measurement_seconds={self.relative(self.first_gps_measurement)}",
      f"first_glonass_measurement_seconds={self.relative(self.first_glonass_measurement)}",
      f"first_valid_gps_week_seconds={self.relative(self.first_valid_gps_week)}",
      f"first_valid_leap_second_seconds={self.relative(self.first_valid_leap_second)}",
      f"first_receiver_utc_seconds={self.relative(self.first_receiver_utc)}",
      f"first_fix_seconds={self.relative(self.first_fix)}",
      f"first_25m_seconds={self.relative(self.first_25m)}",
      f"first_10m_seconds={self.relative(self.first_10m)}",
      f"first_5m_seconds={self.relative(self.first_5m)}",
      f"best_accuracy_m={self.best_accuracy}",
      f"max_satellites={self.max_satellites}",
      "",
      "===== LOG FILES =====",
      *self.used_logs,
      "",
      "===== SERVICE COUNTS =====",
    ]
    lines.extend(f"{service}={count}" for service, count in sorted(self.service_counts.items()))
    lines.extend(("", "===== ERRORS ====="))
    lines.extend(self.errors if self.errors else ["none"])
    return lines


def choose_log(segment_dir: Path) -> Path | None:
  for name in LOG_NAMES:
    candidate = segment_dir / name
    if candidate.is_file():
      return candidate
  return None


def discover_routes(realdata_root: Path) -> list[RouteSelection]:
  grouped: dict[str, list[tuple[int, Path]]] = defaultdict(list)
  if not realdata_root.is_dir():
    return []

  for entry in realdata_root.iterdir():
    if not entry.is_dir():
      continue
    match = SEGMENT_PATTERN.match(entry.name)
    if match is None:
      continue
    grouped[match.group("route")].append((int(match.group("segment")), entry))

  selections = []
  for route, segments in grouped.items():
    segments.sort(key=lambda item: item[0])
    newest = max(path.stat().st_mtime for _, path in segments)
    selections.append(RouteSelection(route, tuple(segments), newest))
  return sorted(selections, key=lambda selection: selection.newest_mtime, reverse=True)


def select_routes(discovered: Iterable[RouteSelection], route: str | None, latest: int | None) -> list[RouteSelection]:
  available = list(discovered)
  if route is not None:
    matches = [selection for selection in available if selection.route == route]
    if not matches:
      raise RuntimeError(f"Requested route not found: {route}")
    return matches
  count = latest if latest is not None else 1
  if count < 1:
    raise RuntimeError("--latest must be at least 1")
  selected = available[:count]
  if not selected:
    raise RuntimeError("No segmented routes were found")
  return selected


def _load_log_reader() -> Any:
  from openpilot.tools.lib.logreader import LogReader

  return LogReader


def analyze_route(selection: RouteSelection, output_root: Path) -> list[str]:
  log_reader = _load_log_reader()
  metrics = RouteMetrics(selection.route)

  for segment_number, segment_dir in selection.segments:
    log_path = choose_log(segment_dir)
    if log_path is None:
      metrics.errors.append(f"segment={segment_number}: no rlog/qlog found in {segment_dir}")
      continue
    metrics.used_logs.append(str(log_path))

    try:
      for message in log_reader(str(log_path)):
        metrics.message_count += 1
        monotonic_time = as_float(safe_get(message, "logMonoTime", None), None)
        if monotonic_time is None:
          continue
        monotonic_time *= 1e-9
        metrics.note_time(monotonic_time)

        try:
          service = message.which()
        except Exception:
          continue
        metrics.service_counts[service] += 1

        try:
          if service == "logMessage":
            metrics.process_log_message(monotonic_time, message.logMessage)
          elif service == "carState":
            metrics.process_car_state(monotonic_time, message.carState)
          elif service == "gpsLocationExternal":
            metrics.process_gps(monotonic_time, message.gpsLocationExternal)
          elif service == "ubloxGnss":
            ublox = message.ubloxGnss
            if ublox.which() == "measurementReport":
              metrics.process_rawx(monotonic_time, ublox.measurementReport)
        except Exception as exc:
          metrics.errors.append(f"{service} parse: {type(exc).__name__}: {exc}")
    except Exception as exc:
      metrics.errors.append(f"{log_path}: {type(exc).__name__}: {exc}\n{traceback.format_exc(limit=3)}")

  route_dir = output_root / "routes" / selection.route
  route_dir.mkdir(parents=True, exist_ok=True)
  summary = metrics.summary_lines(len(selection.segments))
  (route_dir / "summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")

  event_lines = []
  for event_time, text in sorted(metrics.events):
    elapsed = event_time - metrics.route_start if metrics.route_start is not None else 0.0
    event_lines.append(f"[{elapsed:.3f}s] {text}")
  (route_dir / "assistance_events.txt").write_text(
    "\n".join(event_lines) + ("\n" if event_lines else ""),
    encoding="utf-8",
  )
  return summary


def decode_param(value: object) -> str:
  if value is None:
    return "<missing>"
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="replace")
  return str(value)


def collect_params(destination: Path) -> None:
  from openpilot.common.params import Params

  params = Params()
  lines = []
  for key in PARAM_KEYS:
    try:
      lines.append(f"{key}={decode_param(params.get(key))}")
    except Exception as exc:
      raise RuntimeError(f"Params read failed for {key}: {type(exc).__name__}: {exc}") from exc
  destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_capture(*args: str, cwd: Path | None = None) -> str:
  try:
    result = subprocess.run(args, cwd=cwd, check=False, capture_output=True, text=True, timeout=30)
  except Exception as exc:
    return f"ERROR:{type(exc).__name__}:{exc}\n"
  output = result.stdout
  if result.stderr:
    output += result.stderr
  if result.returncode != 0:
    output += f"returncode={result.returncode}\n"
  return output


def collect_git_state(repository_root: Path, destination: Path) -> None:
  sections = (
    ("HEAD", ("git", "rev-parse", "HEAD")),
    ("BRANCH", ("git", "branch", "--show-current")),
    ("STATUS", ("git", "status", "--short", "--branch")),
    ("RECENT GPS COMMITS", ("git", "log", "-20", "--oneline", "--decorate")),
  )
  lines = []
  for title, command in sections:
    lines.extend((f"===== {title} =====", run_capture(*command, cwd=repository_root).rstrip(), ""))
  destination.write_text("\n".join(lines), encoding="utf-8")


def read_boot_id() -> str:
  try:
    value = BOOT_ID_PATH.read_text(encoding="utf-8").strip()
  except OSError as exc:
    return f"ERROR:{type(exc).__name__}:{exc}"
  return value or "<empty>"


def copy_state_files(assistance_root: Path, destination: Path, current_boot_id: str) -> None:
  destination.mkdir(parents=True, exist_ok=True)
  lines = [
    "State files are current-device snapshots, not route-contained evidence.",
    f"capture_boot_id={current_boot_id}",
    f"captured_at_utc={datetime.now(UTC).isoformat()}",
    "",
  ]

  for name in STATE_FILES:
    source = assistance_root / name
    target = destination / name
    if not source.is_file():
      lines.append(f"{name}: missing")
      continue
    shutil.copy2(source, target)
    lines.append(f"{name}: copied bytes={target.stat().st_size}")

  (destination / "STATE_SCOPE.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_system_snapshot(destination: Path, current_boot_id: str) -> None:
  lines = [
    f"captured_at_utc={datetime.now(UTC).isoformat()}",
    f"current_boot_id={current_boot_id}",
    "",
    "===== UNAME =====",
    run_capture("uname", "-a").rstrip(),
    "",
    "===== UPTIME =====",
    run_capture("cat", "/proc/uptime").rstrip(),
    "",
    "===== PIGEOND/MANAGER PROCESSES =====",
    run_capture("pgrep", "-af", "system\\.ubloxd\\.pigeond|manager\\.py").rstrip(),
  ]
  destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_checksums(bundle_root: Path) -> None:
  checksum_path = bundle_root / "SHA256SUMS.txt"
  entries = []
  for path in sorted(bundle_root.rglob("*")):
    if not path.is_file() or path == checksum_path:
      continue
    relative = path.relative_to(bundle_root).as_posix()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    entries.append(f"{digest}  {relative}")
  checksum_path.write_text("\n".join(entries) + "\n", encoding="utf-8")


def verify_checksums(bundle_root: Path) -> None:
  checksum_path = bundle_root / "SHA256SUMS.txt"
  if not checksum_path.is_file():
    raise RuntimeError("SHA256SUMS.txt is missing")
  for line in checksum_path.read_text(encoding="utf-8").splitlines():
    digest, separator, relative = line.partition("  ")
    if not separator or not relative:
      raise RuntimeError(f"Invalid checksum entry: {line!r}")
    if relative.startswith("/") or "/data/" in relative:
      raise RuntimeError(f"Absolute checksum path is forbidden: {relative}")
    path = bundle_root / relative
    if not path.is_file():
      raise RuntimeError(f"Checksummed file is missing: {relative}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != digest:
      raise RuntimeError(f"Checksum mismatch for {relative}")


def validate_bundle(bundle_root: Path, selected_routes: Iterable[RouteSelection]) -> None:
  summary_path = bundle_root / "LATEST_ROUTE_SUMMARY.txt"
  params_path = bundle_root / "selected_params.txt"
  if not summary_path.is_file():
    raise RuntimeError("LATEST_ROUTE_SUMMARY.txt is missing")
  summary = summary_path.read_text(encoding="utf-8")
  for selection in selected_routes:
    if f"route={selection.route}" not in summary:
      raise RuntimeError(f"Requested route missing from summary: {selection.route}")
  if not params_path.is_file() or "ERROR:" in params_path.read_text(encoding="utf-8"):
    raise RuntimeError("Params collection failed")
  verify_checksums(bundle_root)


def build_bundle(
  selected_routes: list[RouteSelection],
  output_root: Path,
  repository_root: Path,
  assistance_root: Path,
) -> tuple[Path, Path]:
  timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  final_directory = output_root / f"gps_drive_audit_{timestamp}"
  final_bundle = output_root / f"gps_drive_audit_{timestamp}.tar.gz"
  if final_directory.exists() or final_bundle.exists():
    raise RuntimeError(f"Audit output already exists for timestamp {timestamp}")

  output_root.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix="gps_drive_audit_", dir=output_root) as temporary:
    bundle_root = Path(temporary) / final_directory.name
    bundle_root.mkdir()

    combined = [
      "================================================================",
      "LATEST ROUTE GPS SUMMARY",
      "================================================================",
      f"requested_routes={','.join(selection.route for selection in selected_routes)}",
      f"discovered_route_count={len(selected_routes)}",
      "",
    ]
    for selection in selected_routes:
      combined.extend(analyze_route(selection, bundle_root))
      combined.extend(("", "----------------------------------------------------------------", ""))
    (bundle_root / "LATEST_ROUTE_SUMMARY.txt").write_text("\n".join(combined) + "\n", encoding="utf-8")

    current_boot_id = read_boot_id()
    (bundle_root / "current_boot_id.txt").write_text(current_boot_id + "\n", encoding="utf-8")
    (bundle_root / "state_capture_utc.txt").write_text(datetime.now(UTC).isoformat() + "\n", encoding="utf-8")
    evidence_scope = "\n".join((
      "Route summaries and assistance_events are derived from selected route logs.",
      "Files under state/current_boot are snapshots from the boot active when collection ran.",
      "A current state file must not be attributed to a route unless its boot identity is independently matched.",
    ))
    (bundle_root / "EVIDENCE_SCOPE.txt").write_text(evidence_scope + "\n", encoding="utf-8")
    collect_params(bundle_root / "selected_params.txt")
    collect_git_state(repository_root, bundle_root / "git_state.txt")
    write_system_snapshot(bundle_root / "system_snapshot.txt", current_boot_id)
    copy_state_files(assistance_root, bundle_root / "state" / "current_boot", current_boot_id)
    shutil.copy2(Path(__file__), bundle_root / "gps_drive_audit.py")

    generate_checksums(bundle_root)
    validate_bundle(bundle_root, selected_routes)
    shutil.move(str(bundle_root), final_directory)

  with tarfile.open(final_bundle, "w:gz") as archive:
    archive.add(final_directory, arcname=final_directory.name)
  return final_directory, final_bundle


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Collect a boot-aware GPS cold-start route audit bundle")
  selection = parser.add_mutually_exclusive_group(required=True)
  selection.add_argument("--route", help="Exact route name, for example 00000093--a1ef00c9c2")
  selection.add_argument("--latest", type=int, help="Collect the newest N locally recorded routes")
  parser.add_argument("--realdata-root", type=Path, default=DEFAULT_REALDATA_ROOT)
  parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
  parser.add_argument("--repository-root", type=Path, default=DEFAULT_REPOSITORY_ROOT)
  parser.add_argument("--assistance-root", type=Path, default=DEFAULT_ASSISTANCE_ROOT)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    discovered = discover_routes(args.realdata_root)
    selected = select_routes(discovered, args.route, args.latest)
    directory, bundle = build_bundle(selected, args.output_root, args.repository_root, args.assistance_root)
  except Exception as exc:
    print(f"RESULT: GPS_DRIVE_AUDIT_FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
    return 1

  print("================================================================")
  print("RESULT: GPS_DRIVE_AUDIT_BUNDLE_CREATED")
  print("================================================================")
  print(f"Directory: {directory}")
  print(f"Bundle:    {bundle}")
  print("Routes:")
  for selection in selected:
    print(selection.route)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
