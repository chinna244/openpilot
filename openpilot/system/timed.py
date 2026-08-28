#!/usr/bin/env python3
import datetime
import json
import math
import subprocess
import time
from typing import NoReturn

import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, MAX_DATE, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service

MODEM_STATE_PATH = "/dev/shm/modem"
MAX_CELLULAR_SAMPLE_AGE = 120.0
GPS_TIME_MAX_AGE = 2.0


def set_time(new_time) -> bool:
  diff = datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return True

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
    return True
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")
    return False


def gps_time_if_valid(gps, updated: bool, age_s: float) -> datetime.datetime | None:
  if not updated or age_s > GPS_TIME_MAX_AGE:
    return None
  if not getattr(gps, "hasFix", False):
    return None
  gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000., datetime.UTC).replace(tzinfo=None)
  if gps_time < min_date() or gps_time > MAX_DATE:
    return None
  return gps_time


def parse_cellular_time(state: dict, now_mono: float | None = None) -> datetime.datetime | None:
  raw = state.get("network_time_utc") or ""
  if not isinstance(raw, str) or not raw:
    return None
  try:
    sample = datetime.datetime.strptime(raw, "%Y-%m-%dT%H:%M:%SZ")
  except ValueError:
    return None

  mono = state.get("network_time_monotonic")
  if isinstance(mono, bool) or not isinstance(mono, (int, float)) or not math.isfinite(mono):
    return None

  now = time.monotonic() if now_mono is None else now_mono
  age = now - float(mono)
  if age < 0 or age > MAX_CELLULAR_SAMPLE_AGE:
    return None

  cellular_time = sample + datetime.timedelta(seconds=age)
  if cellular_time < min_date() or cellular_time > MAX_DATE:
    return None
  return cellular_time


def read_cellular_time(path: str = MODEM_STATE_PATH, now_mono: float | None = None) -> datetime.datetime | None:
  try:
    with open(path) as f:
      state = json.load(f)
  except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError):
    return None
  if not isinstance(state, dict):
    return None
  return parse_cellular_time(state, now_mono=now_mono)


def apply_clock_sources(gps_time: datetime.datetime | None, cellular_time: datetime.datetime | None,
                       cellular_applied: bool = False, set_time_fn=set_time) -> tuple[bool, bool]:
  """Apply GPS if present, otherwise one-shot cellular NITZ.

  Returns (gps_used, cellular_applied). Cellular NITZ is an initial/boot source only;
  after the first processed sample it is not applied again in this process.
  """
  if gps_time is not None:
    set_time_fn(gps_time)
    return True, cellular_applied
  if cellular_time is not None and not cellular_applied:
    return False, bool(set_time_fn(cellular_time))
  return False, cellular_applied


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS and cellular network (NITZ)
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  cellular_initial_time_applied = False
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_age = time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9
    gps_time = gps_time_if_valid(gps, sm.updated[gps_location_service], gps_age)
    gps_used, cellular_initial_time_applied = apply_clock_sources(
      gps_time, read_cellular_time(), cellular_initial_time_applied)
    if gps_used:
      time.sleep(10)

if __name__ == "__main__":
  main()
