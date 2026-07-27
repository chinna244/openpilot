#!/usr/bin/env python3
import datetime
import subprocess
import time
from typing import NoReturn

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import (
  HostTimeSource,
  MAX_DATE,
  mark_time_synced,
  min_date,
  set_system_time,
  system_time_valid,
)
from openpilot.common.gps_time import ublox_gps_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def set_time(new_time: datetime.datetime) -> bool:
  cloudlog.debug(f"Setting time from trusted source: {new_time}")

  try:
    return set_system_time(new_time)
  except (OSError, subprocess.CalledProcessError):
    cloudlog.exception("timed.failed_setting_time")
    return False


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(
      gps.unixTimestampMillis / 1000.,
      tz=datetime.UTC,
    )
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if gps.source == log.GpsLocationData.SensorSource.ublox:
      if not ublox_gps_time_valid(gps.flags):
        continue
    elif not gps.hasFix:
      continue

    minimum_time = min_date().replace(tzinfo=datetime.UTC)
    maximum_time = MAX_DATE.replace(tzinfo=datetime.UTC)
    if gps_time < minimum_time or gps_time > maximum_time:
      continue

    if set_time(gps_time):
      if not mark_time_synced(HostTimeSource.RECEIVER_DERIVED):
        cloudlog.warning("Failed to write trusted GPS time marker")
      time.sleep(10)

if __name__ == "__main__":
  main()
