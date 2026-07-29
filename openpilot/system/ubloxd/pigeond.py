#!/usr/bin/env python3
import re
import sys
import time
import signal
from openpilot.common.serial import Serial
import requests
import urllib.parse
from collections import deque
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace
from datetime import datetime, timedelta, UTC
from enum import StrEnum
from math import ceil, isfinite

from openpilot.cereal import messaging
from openpilot.common.time_helpers import (
  HostTimeObservation,
  HostTimeSource,
  read_host_time_observation,
)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware import HARDWARE, TICI
from openpilot.common.gpio import gpio_init, gpio_set
from openpilot.common.hardware.tici.pins import GPIO
from openpilot.system.ubloxd.ubx import Ubx
from openpilot.system.ubloxd.gps_assistance import (
  CacheAgeEvidence,
  CacheValidationError,
  CachePromotionStatus,
  CacheQualityTier,
  CaptureQualityTracker,
  MAXIMUM_NAV_PVT_GAP_SECONDS,
  MAXIMUM_NAV_SAT_AGE_SECONDS,
  MINIMUM_GLONASS_EPHEMERIS,
  MINIMUM_GPS_EPHEMERIS,
  MINIMUM_ORBIT_QUALITY_SECONDS,
  MINIMUM_RELIABLE_FIX_SECONDS,
  MINIMUM_SATELLITES_USED,
  MINIMUM_TOTAL_EPHEMERIS,
  NAVX5_MASK1_ACK_AIDING,
  NAVX5_MASK1_AOP,
  GPS_ASSISTANCE_CACHE_PATH,
  MAX_RTC_ASSISTANCE_ELAPSED_SECONDS,
  GnssConfig,
  MgaAck,
  MonVerInfo,
  NavAopStatus,
  NavSatQuality,
  Navx5Config,
  NavPvtFix,
  NavigationQuality,
  NavigationCacheStore,
  NavigationDatabaseDumpCollector,
  Pm2Config,
  PortConfig,
  RateConfig,
  Nav5Config,
  OdoConfig,
  ItfmConfig,
  MessageRateConfig,
  ReliableFixTracker,
  RestoredNavigationQuality,
  RtcEstimateRejection,
  RtcEstimateRejectionReason,
  RxmConfig,
  UbxStreamParser,
  build_cfg_gnss_poll_message,
  build_cfg_itfm_poll_message,
  build_cfg_msg_poll_message,
  build_cfg_nav5_poll_message,
  build_cfg_odo_poll_message,
  build_cfg_pm2_poll_message,
  build_cfg_prt_poll_message,
  build_cfg_rate_poll_message,
  build_cfg_rxm_poll_message,
  build_database_poll_message,
  build_nav_aopstatus_poll_message,
  build_navx5_ack_aiding_enable_message,
  build_navx5_aop_enable_message,
  build_navx5_poll_message,
  build_position_assistance_message,
  build_time_assistance_message,
  capture_eligible,
  conservative_navigation_quality,
  create_cache,
  effective_restored_navigation_quality,
  load_cache,
  parse_mga_ack,
  parse_cfg_gnss,
  parse_cfg_itfm,
  parse_cfg_msg,
  parse_cfg_nav5,
  parse_cfg_odo,
  parse_cfg_pm2,
  parse_cfg_prt,
  parse_cfg_rate,
  parse_cfg_rxm,
  parse_mon_ver,
  parse_nav_aopstatus,
  parse_nav_pvt,
  parse_nav_sat,
  parse_navx5,
  parse_upd_sos_response,
  read_rtc_counter_seconds,
  select_rtc_estimate,
  normalized_receiver_identity,
  navx5_unrelated_fields_unchanged,
  navigation_quality_strictly_better,
  navigation_quality_tier,
  validate_ubx_frame,
)
from openpilot.system.ubloxd.navigation_database_restore import (
  NavigationDatabaseRestoreDisposition,
  is_current_independent_network_time,
)
from openpilot.system.ubloxd.navigation_database_restore_runtime import (
  NavigationDatabaseRestoreExecution,
  NavigationDatabaseRestoreFrameFailureKind,
  NavigationDatabaseRestoreRuntime,
)
from openpilot.system.ubloxd.receiver_time_provenance import (
  ReceiverTimeProvenanceTracker,
  ReceiverUtcClassification,
  ReceiverUtcObservation,
  is_mga_time_assistance_message,
)
from openpilot.system.ubloxd.provisional_yuma_reference import (
  PROVISIONAL_YUMA_DISABLE_REASON_VALIDATION_DISAGREES,
  ProvisionalYumaReferenceDecision,
  ProvisionalYumaReferenceTime,
  ProvisionalYumaTransmissionOutcome,
  evaluate_provisional_yuma_reference,
  load_provisional_yuma_boot_disable_state,
  store_provisional_yuma_boot_disable_state,
  store_provisional_yuma_decision_event,
  transmit_provisional_yuma_reference,
)
from openpilot.system.ubloxd.rtc_time_observation import (
  CrossBootRtcObservation,
  RtcObservationState,
)
from openpilot.system.ubloxd.trusted_time_anchor import (
  TimeProvenance,
  TrustedTimeSource,
  read_boot_id,
  read_boottime_seconds,
)
from openpilot.system.ubloxd.trusted_time_authority import (
  AnchorWriteStatus,
  AuthorizedTime,
  TimeAuthority,
  TimeAuthorityEvaluation,
)
from openpilot.system.ubloxd.trusted_time_validation import (
  CrossBootRtcValidation,
  CrossBootRtcValidationStatus,
  IndependentTimeObservation,
  ReceiverCorrectionDecision,
  evaluate_receiver_correction,
  validate_cross_boot_rtc,
)
from openpilot.system.ubloxd.yuma_almanac_plan import (
  YumaDatabaseRestoreState,
  YumaSupplementationReason,
)
from openpilot.system.ubloxd.yuma_almanac_runtime import (
  YumaSupplementationRuntime,
  YumaSupplementationRuntimeOutcome,
)
from openpilot.system.ubloxd.yuma_almanac_transmit import (
  MgaReceiverNackError,
  MgaTransactionError,
  MgaWriteError,
)
from openpilot.system.ubloxd.yuma_almanac_config import (
  PUBLIC_YUMA_ALMANAC_PARAM_POLL_SECONDS,
  public_yuma_almanac_enabled,
)
from openpilot.system.ubloxd.yuma_almanac_outcome import (
  YUMA_LAST_OUTCOME_PATH,
  save_yuma_supplementation_outcome,
)


UBLOX_TTY = "/dev/ttyHS0"

UBLOX_ACK = b"\xb5\x62\x05\x01\x02\x00"
UBLOX_NACK = b"\xb5\x62\x05\x00\x02\x00"
UBLOX_ASSIST_ACK = b"\xb5\x62\x13\x60\x08\x00"

TIME_SYNC_CHECK_INTERVAL = 5.0
HOST_TIME_PERSISTENCE_RETRY_INTERVAL = 5.0
TIME_ASSISTANCE_RETRY_INTERVAL = 30.0

GPS_ASSISTANCE_ACK_TIMEOUT = 0.75
GPS_ASSISTANCE_FRAME_RETRY_DELAY = 0.25
MON_VER_POLL_TIMEOUT = 0.5
NAVX5_POLL_TIMEOUT = 0.5
CFG_ACK_TIMEOUT = 1.1
NAVX5_ACK_TIMEOUT = CFG_ACK_TIMEOUT
AOP_STATUS_POLL_TIMEOUT = 0.1
AOP_IDLE_WAIT_TIMEOUT = 0.3
AOP_IDLE_POLL_INTERVAL = 0.05
ACQUISITION_CONFIG_POLL_TIMEOUT = 0.25
GPS_ASSISTANCE_CAPTURE_RETRY_INTERVAL = 60.0
GPS_ASSISTANCE_QUALIFIED_UPGRADE_COOLDOWN = 5.0 * 60.0
GPS_ACQUISITION_STATUS_INTERVAL = 30.0
PRE_TRANSACTION_DRAIN_MAX_BYTES = 64 * 1024
CONTROLLED_GNSS_STOP_MESSAGE = b"\xB5\x62\x06\x04\x04\x00\x00\x00\x08\x00\x16\x74"
CONTROLLED_GNSS_START_MESSAGE = b"\xB5\x62\x06\x04\x04\x00\x00\x00\x09\x00\x17\x76"
CONTROLLED_GNSS_TRANSITION_DELAY = 0.05
NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS = 5.0
NAVIGATION_DATABASE_TRUSTED_TIME_POLL_SECONDS = 0.25
# A validated MGA-DBD cache is capped at 64 KiB. Frequent dispatch between
# transactions is the primary bound; these limits retain four cache volumes or
# 512 small navigation frames if a dispatcher is temporarily unavailable.
PENDING_FRAME_MAX_COUNT = 512
PENDING_FRAME_MAX_BYTES = 256 * 1024

NAVX5_ACK_AIDING_SOFTWARE_VERSION_PATTERN = re.compile(
  r"EXT CORE 3\.01(?: \([A-Z0-9][A-Z0-9._-]{0,31}\))?"
)


class ReceiverConfigurationError(RuntimeError):
  pass


class CfgNakError(ReceiverConfigurationError):
  def __init__(
    self,
    message: str,
    retry_not_before: float | None = None,
  ) -> None:
    self.retry_not_before = retry_not_before
    super().__init__(message)


class ResponseTransactionError(RuntimeError):
  pass


class PendingFrameOverflowError(ResponseTransactionError):
  def __init__(
    self,
    frame_count: int,
    byte_count: int,
    operation: str,
    receiver_cycle: int,
    exceeded: str,
  ) -> None:
    self.frame_count = frame_count
    self.byte_count = byte_count
    self.operation = operation
    self.receiver_cycle = receiver_cycle
    self.exceeded = exceeded
    super().__init__(
      f"Pending u-blox frame queue exceeded {exceeded}: "
      + f"frame_count={frame_count}, byte_count={byte_count}, "
      + f"operation={operation}, receiver_cycle={receiver_cycle}"
    )


class RawPublicationError(ResponseTransactionError):
  pass


class CfgPollTimeoutError(TimeoutError):
  pass


@dataclass
class ResponseTransaction:
  parser: UbxStreamParser
  request: bytes = b""
  operation: str = "response_transaction"
  sent_at: float = 0.0

def set_power(enabled: bool) -> None:
  gpio_init(GPIO.UBLOX_SAFEBOOT_N, True)
  gpio_init(GPIO.GNSS_PWR_EN, True)
  gpio_init(GPIO.UBLOX_RST_N, True)

  gpio_set(GPIO.UBLOX_SAFEBOOT_N, True)
  gpio_set(GPIO.GNSS_PWR_EN, enabled)
  gpio_set(GPIO.UBLOX_RST_N, enabled)

def add_ubx_checksum(msg: bytes) -> bytes:
  A = B = 0
  for b in msg[2:]:
    A = (A + b) % 256
    B = (B + A) % 256
  return msg + bytes([A, B])

def get_assistnow_messages(token: str) -> list[bytes]:
  # make request
  # TODO: implement adding the last known location
  r = requests.get("https://online-live2.services.u-blox.com/GetOnlineData.ashx", params=urllib.parse.urlencode({
    'token': token,
    'gnss': 'gps,glo',
    'datatype': 'eph,alm,aux',
  }, safe=':,'), timeout=5)
  assert r.status_code == 200, "Got invalid status code"
  dat = r.content

  # split up messages
  msgs = []
  while len(dat) > 0:
    assert dat[:2] == b"\xB5\x62"
    msg_len = 6 + (dat[5] << 8 | dat[4]) + 2
    msgs.append(dat[:msg_len])
    dat = dat[msg_len:]
  return msgs


class TTYPigeon:
  def __init__(
    self,
    raw_publisher: Callable[[bytes], None] | None = None,
    frame_dispatcher: Callable[[list[bytes]], None] | None = None,
  ):
    self.tty = Serial(UBLOX_TTY, baudrate=9600, timeout=0)
    self._stream_parser = UbxStreamParser()
    self._pending_frames: deque[bytes] = deque()
    self._pending_frame_bytes = 0
    # A kernel-read chunk remains here until ubloxRaw publication succeeds, so
    # it stays accounted for while this process is alive. An unrecoverable
    # process-level messaging failure cannot provide the same guarantee.
    self._pending_unpublished: bytes | None = None
    self._raw_publisher = raw_publisher
    self._frame_dispatcher = frame_dispatcher
    self._receiver_cycle = 0
    self.time_provenance: ReceiverTimeProvenanceTracker | None = None

  def send(self, dat: bytes) -> None:
    self.tty.write(dat)

  @property
  def receiver_cycle(self) -> int:
    return self._receiver_cycle

  def _receive_tty(self) -> bytes:
    dat = b''
    while len(dat) < 0x1000:
      d = self.tty.read(0x40)
      dat += d
      if len(d) == 0:
        break
    return dat

  def receive(self) -> bytes:
    data, _ = self.receive_normal()
    return data

  def receive_normal(self) -> tuple[bytes, list[bytes]]:
    if self._pending_frames:
      frames = list(self._pending_frames)
      self._pending_frames.clear()
      self._pending_frame_bytes = 0
      return b"", frames

    return self._read_stream()

  def _publish_raw(self, data: bytes) -> None:
    if data and self._raw_publisher is not None:
      try:
        self._raw_publisher(data)
      except Exception as exc:
        raise RawPublicationError(
          f"Failed to publish retained u-blox UART chunk: byte_count={len(data)}"
        ) from exc

  def _read_stream(self) -> tuple[bytes, list[bytes]]:
    data = self._pending_unpublished
    if data is None:
      data = self._receive_tty()
      if data:
        self._pending_unpublished = data
    self._publish_raw(data)
    if data:
      self._pending_unpublished = None
    return data, self._stream_parser.feed(data)

  def receive_transaction_data(
    self,
    transaction: ResponseTransaction,
  ) -> tuple[bytes, list[bytes], list[bytes]]:
    data, stream_frames = self._read_stream()
    return data, stream_frames, transaction.parser.feed(data)

  def queue_pending_frames(
    self,
    frames: list[bytes],
    operation: str = "response_transaction",
  ) -> None:
    added_bytes = sum(len(frame) for frame in frames)
    self._pending_frames.extend(frames)
    self._pending_frame_bytes += added_bytes
    exceeded = None
    if len(self._pending_frames) > PENDING_FRAME_MAX_COUNT:
      exceeded = "frame_limit"
    elif self._pending_frame_bytes > PENDING_FRAME_MAX_BYTES:
      exceeded = "byte_limit"
    if exceeded is not None:
      error = PendingFrameOverflowError(
        len(self._pending_frames),
        self._pending_frame_bytes,
        operation,
        self._receiver_cycle,
        exceeded,
      )
      cloudlog.error(f"GPS synchronous frame queue overflow: {error}")
      raise error

  def dispatch_pending_frames(self) -> None:
    if not self._pending_frames or self._frame_dispatcher is None:
      return
    frames = list(self._pending_frames)
    self._frame_dispatcher(frames)
    self._pending_frames.clear()
    self._pending_frame_bytes = 0

  def drain_before_transaction(
    self,
    operation: str = "pre_transaction_drain",
  ) -> None:
    self.dispatch_pending_frames()
    drained_bytes = 0
    while True:
      data, frames = self._read_stream()
      self.queue_pending_frames(frames, operation)
      self.dispatch_pending_frames()
      if not data:
        return
      drained_bytes += len(data)
      if drained_bytes >= PRE_TRANSACTION_DRAIN_MAX_BYTES:
        raise ResponseTransactionError(
          "Pre-transaction u-blox input drain exceeded its deterministic bound"
        )

  def begin_response_transaction(
    self,
    data: bytes,
    operation: str = "response_transaction",
  ) -> ResponseTransaction:
    self.drain_before_transaction(operation)
    parser = UbxStreamParser()
    self.send(data)
    sent_at = time.monotonic()
    return ResponseTransaction(parser, data, operation, sent_at)

  def reset_response_state(self) -> None:
    if self._pending_unpublished is not None:
      self._publish_raw(self._pending_unpublished)
      data = self._pending_unpublished
      self._pending_unpublished = None
      self.queue_pending_frames(
        self._stream_parser.feed(data), "receiver_cycle_reset",
      )
    self.dispatch_pending_frames()
    self._stream_parser.reset()
    self._pending_frames.clear()
    self._pending_frame_bytes = 0
    self._receiver_cycle += 1

  def set_baud(self, baud: int) -> None:
    self.tty.baudrate = baud

  def wait_for_ack(
    self,
    transaction: ResponseTransaction,
    ack: bytes = UBLOX_ACK,
    nack: bytes = UBLOX_NACK,
    timeout: float = 0.5,
  ) -> bool:
    ubx_ack_response = ack == UBLOX_ACK and nack == UBLOX_NACK
    acknowledged_key = transaction.request[2:4]
    st = time.monotonic()

    while True:
      result: bool | None = None
      _, stream_frames, transaction_frames = _receive_transaction_data(
        self,
        transaction,
      )

      for frame in transaction_frames:
        if ubx_ack_response:
          if (
            result is None
            and len(frame) == 10
            and frame[2] == 0x05
            and frame[3] in (0x00, 0x01)
            and frame[6:8] == acknowledged_key
          ):
            result = frame[3] == 0x01
            continue
        elif (
          result is None
          and frame.startswith((ack, nack))
        ):
          result = frame.startswith(ack)
          continue

      _queue_unrelated_frames(
        self,
        stream_frames,
        lambda frame: (
          len(frame) == 10
          and frame[2] == 0x05
          and frame[3] in (0x00, 0x01)
          and frame[6:8] == acknowledged_key
        ) if ubx_ack_response else frame.startswith((ack, nack)),
        transaction.operation,
      )

      if result is not None:
        if result:
          cloudlog.debug("Received ACK from ublox")
        else:
          cloudlog.error("Received NACK from ublox")

        return result

      if time.monotonic() - st > timeout:
        cloudlog.error("No response from ublox")
        raise TimeoutError("No response from ublox")

      time.sleep(0.001)

  def send_with_ack(self, dat: bytes, ack: bytes = UBLOX_ACK, nack: bytes = UBLOX_NACK) -> None:
    if (
      ack == UBLOX_ACK
      and nack == UBLOX_NACK
      and len(dat) >= 4
      and dat[:3] == b"\xB5\x62\x06"
    ):
      if not validate_ubx_frame(dat):
        raise ReceiverConfigurationError("Attempted to send an invalid UBX CFG frame")
      transaction = self.begin_response_transaction(
        dat, f"cfg_write_{dat[2]:02x}_{dat[3]:02x}",
      )
      acknowledgment = wait_for_cfg_ack(self, transaction, dat[2], dat[3])
      if acknowledgment is False:
        raise CfgNakError(
          f"u-blox rejected CFG message 0x{dat[2]:02X} 0x{dat[3]:02X}",
          transaction.sent_at + CFG_ACK_TIMEOUT,
        )
      if acknowledgment is None:
        raise TimeoutError(f"No matching acknowledgment for CFG message 0x{dat[2]:02X} 0x{dat[3]:02X}")
      return
    if ack == UBLOX_ASSIST_ACK:
      send_mga_with_strict_ack(
        self,
        dat,
        timeout=GPS_ASSISTANCE_ACK_TIMEOUT,
        time_provenance=getattr(
          self,
          "time_provenance",
          None,
        ),
        time_assistance_source="assistnow_online",
      )
      return
    transaction = self.begin_response_transaction(
      dat, f"ubx_write_{dat[2]:02x}_{dat[3]:02x}",
    )
    if not self.wait_for_ack(
      transaction, ack, nack,
      timeout=CFG_ACK_TIMEOUT if ack == UBLOX_ACK else 0.5,
    ):
      raise CfgNakError(
        f"u-blox rejected message 0x{dat[2]:02X} 0x{dat[3]:02X}",
        transaction.sent_at + CFG_ACK_TIMEOUT,
      )

  def wait_for_backup_restore_status(
    self,
    transaction: ResponseTransaction,
    timeout: float = 1.,
  ) -> int:
    st = time.monotonic()
    while True:
      status = None
      _, stream_frames, transaction_frames = _receive_transaction_data(self, transaction)
      for frame in transaction_frames:
        response = parse_upd_sos_response(frame)
        if (
          status is None
          and response is not None
          and response.command == 3
          and response.response in (1, 2, 3)
        ):
          status = response.response
          continue
      _queue_unrelated_frames(
        self,
        stream_frames,
        lambda frame: (
          (response := parse_upd_sos_response(frame)) is not None
          and response.command == 3
        ),
        transaction.operation,
      )
      if status is not None:
        return status
      if time.monotonic() - st > timeout:
        cloudlog.error("No backup restore response from ublox")
        raise TimeoutError('No response from ublox')
      time.sleep(0.001)

  def poll_backup_restore_status(self, timeout: float = 1.) -> int:
    transaction = self.begin_response_transaction(
      b"\xB5\x62\x09\x14\x00\x00\x1D\x60",
      "upd_sos_restore_status_poll",
    )
    return self.wait_for_backup_restore_status(transaction, timeout)

  def reset_device(self) -> bool:
    # deleting the backup does not always work on first try (mostly on second try)
    for _ in range(5):
      # device cold start
      self.send(b"\xb5\x62\x06\x04\x04\x00\xff\xff\x00\x00\x0c\x5d")
      time.sleep(1) # wait for cold start
      init_baudrate(self)

      # clear configuration
      self.send_with_ack(b"\xb5\x62\x06\x09\x0d\x00\x1f\x1f\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x17\x71\xd7")

      # clear flash memory (almanac backup)
      self.send_with_ack(b"\xB5\x62\x09\x14\x04\x00\x01\x00\x00\x00\x22\xf0")

      # try restoring backup to verify it got deleted
      # 1: failed to restore, 2: could restore, 3: no backup
      status = self.poll_backup_restore_status()
      if status == 1 or status == 3:
        return True
    return False


def _receive_transaction_data(
  pigeon: TTYPigeon,
  transaction: ResponseTransaction,
) -> tuple[bytes, list[bytes], list[bytes]]:
  if hasattr(pigeon, "_stream_parser"):
    return pigeon.receive_transaction_data(transaction)
  data = pigeon.receive()
  frames = transaction.parser.feed(data)
  return data, frames, frames


def _queue_unrelated_frames(
  pigeon: TTYPigeon,
  frames: list[bytes],
  is_response: Callable[[bytes], bool],
  operation: str = "response_transaction",
) -> None:
  if hasattr(pigeon, "_pending_frames"):
    pigeon.queue_pending_frames([
      frame for frame in frames if not is_response(frame)
    ], operation)
    pigeon.dispatch_pending_frames()


def _begin_response_transaction(
  pigeon: TTYPigeon,
  message: bytes,
  operation: str | None = None,
) -> ResponseTransaction:
  operation = operation or f"ubx_{message[2]:02x}_{message[3]:02x}"
  if hasattr(pigeon, "begin_response_transaction"):
    return pigeon.begin_response_transaction(message, operation)
  pigeon.send(message)
  return ResponseTransaction(
    UbxStreamParser(), message, operation, time.monotonic(),
  )


def build_mon_ver_poll_message() -> bytes:
  return add_ubx_checksum(b"\xb5\x62\x0a\x04\x00\x00")


def _wait_for_parsed_response[Response](
  pigeon: TTYPigeon,
  transaction: ResponseTransaction,
  response_parser: Callable[[bytes], Response | None],
  message_class: int,
  message_id: int,
  timeout: float,
  response_matches: Callable[[Response], bool] | None = None,
) -> Response | None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    result = None
    _, stream_frames, transaction_frames = _receive_transaction_data(pigeon, transaction)
    for frame in transaction_frames:
      parsed = response_parser(frame)
      if parsed is not None:
        matches = response_matches is None or response_matches(parsed)
        if result is None and matches:
          result = parsed
    _queue_unrelated_frames(
      pigeon,
      stream_frames,
      lambda frame: (
        frame[2:4] == bytes((message_class, message_id))
        and (parsed := response_parser(frame)) is not None
        and (response_matches is None or response_matches(parsed))
      ),
      transaction.operation,
    )
    if result is not None:
      return result
    time.sleep(0.001)
  return None


def poll_mon_ver(
  pigeon: TTYPigeon,
  timeout: float = MON_VER_POLL_TIMEOUT,
) -> MonVerInfo | None:
  transaction = _begin_response_transaction(pigeon, build_mon_ver_poll_message())
  return _wait_for_parsed_response(
    pigeon, transaction, parse_mon_ver, 0x0A, 0x04, timeout,
  )


def _poll_acquisition_config[AcquisitionConfig](
  pigeon: TTYPigeon,
  poll_message: bytes,
  response_parser: Callable[[bytes], AcquisitionConfig | None],
  timeout: float,
) -> AcquisitionConfig | None:
  transaction = _begin_response_transaction(pigeon, poll_message)
  return _wait_for_parsed_response(
    pigeon,
    transaction,
    response_parser,
    poll_message[2],
    poll_message[3],
    timeout,
  )


def poll_cfg_gnss(
  pigeon: TTYPigeon,
  timeout: float = ACQUISITION_CONFIG_POLL_TIMEOUT,
) -> GnssConfig | None:
  return _poll_acquisition_config(
    pigeon,
    build_cfg_gnss_poll_message(),
    parse_cfg_gnss,
    timeout,
  )


def poll_cfg_rxm(
  pigeon: TTYPigeon,
  timeout: float = ACQUISITION_CONFIG_POLL_TIMEOUT,
) -> RxmConfig | None:
  return _poll_acquisition_config(
    pigeon,
    build_cfg_rxm_poll_message(),
    parse_cfg_rxm,
    timeout,
  )


def poll_cfg_pm2(
  pigeon: TTYPigeon,
  timeout: float = ACQUISITION_CONFIG_POLL_TIMEOUT,
) -> Pm2Config | None:
  return _poll_acquisition_config(
    pigeon,
    build_cfg_pm2_poll_message(),
    parse_cfg_pm2,
    timeout,
  )


GNSS_NAMES = {
  0: "GPS",
  1: "SBAS",
  2: "Galileo",
  3: "BeiDou",
  4: "IMES",
  5: "QZSS",
  6: "GLONASS",
}


def _log_cfg_gnss(config: GnssConfig, info: MonVerInfo | None) -> None:
  cloudlog.info(", ".join((
    "GPS acquisition configuration CFG-GNSS",
    f"protocol_versions={list(info.protocol_versions) if info is not None else []}",
    f"message_version={config.version}",
    f"hardware_tracking_channels={config.hardware_tracking_channels}",
    f"configured_tracking_channels={config.configured_tracking_channels}",
    f"block_count={len(config.blocks)}",
  )))
  for block in config.blocks:
    cloudlog.info(", ".join((
      "GPS acquisition configuration CFG-GNSS block",
      f"gnss_id={block.gnss_id}",
      f"gnss_name={GNSS_NAMES.get(block.gnss_id, 'unknown')}",
      f"enabled={str(block.enabled).lower()}",
      f"reserved_tracking_channels={block.reserved_tracking_channels}",
      f"maximum_tracking_channels={block.maximum_tracking_channels}",
      f"signal_configuration_mask=0x{block.signal_configuration_mask:02X}",
      f"raw_flags=0x{block.flags:08X}",
    )))


def _log_cfg_rxm(config: RxmConfig) -> None:
  interpretation = {
    0: "continuous",
    1: "power_save",
    4: "continuous",
  }.get(config.low_power_mode, "unknown")
  cloudlog.info(", ".join((
    "GPS acquisition configuration CFG-RXM",
    f"low_power_mode={config.low_power_mode}",
    f"low_power_mode_interpretation={interpretation}",
  )))


def _log_cfg_pm2(config: Pm2Config) -> None:
  limit_peak_current = {
    0: "disabled",
    1: "enabled",
    2: "reserved",
    3: "reserved",
  }[(config.flags >> 8) & 0x03]
  power_mode = {
    0: "on_off",
    1: "cyclic_tracking",
    2: "reserved",
    3: "reserved",
  }[(config.flags >> 17) & 0x03]
  fields = [
    "GPS acquisition configuration CFG-PM2",
    f"message_version={config.version}",
    f"raw_flags=0x{config.flags:08X}",
    f"maximum_startup_state_duration_s={config.maximum_startup_state_duration_s}",
    f"update_period_ms={config.update_period_ms}",
    f"search_period_ms={config.search_period_ms}",
    f"grid_offset_ms={config.grid_offset_ms}",
    f"on_time_s={config.on_time_s}",
    f"minimum_acquisition_time_s={config.minimum_acquisition_time_s}",
    f"external_interrupt_pin={'EXTINT1' if config.flags & (1 << 4) else 'EXTINT0'}",
    f"external_interrupt_wake={str(bool(config.flags & (1 << 5))).lower()}",
    f"external_interrupt_backup={str(bool(config.flags & (1 << 6))).lower()}",
    f"limit_peak_current={limit_peak_current}",
    f"wait_for_time_fix={str(bool(config.flags & (1 << 10))).lower()}",
    f"update_rtc={str(bool(config.flags & (1 << 11))).lower()}",
    f"update_ephemeris={str(bool(config.flags & (1 << 12))).lower()}",
    f"do_not_enter_off={str(bool(config.flags & (1 << 16))).lower()}",
    f"power_mode={power_mode}",
  ]
  if config.version == 2:
    fields.extend((
      f"external_interrupt_inactive={str(bool(config.flags & (1 << 7))).lower()}",
      f"external_interrupt_inactivity_ms={config.external_interrupt_inactivity_ms}",
    ))
  cloudlog.info(", ".join(fields))


def _is_hpg_product(info: MonVerInfo | None) -> bool:
  if info is None:
    return False

  for firmware_version in info.firmware_versions:
    fields = firmware_version.removeprefix("FWVER=").split(maxsplit=1)
    if fields and fields[0] == "HPG":
      return True
  return False


def log_acquisition_configuration_diagnostics(
  pigeon: TTYPigeon,
  info: MonVerInfo | None,
) -> None:
  diagnostics = (
    ("CFG-GNSS", poll_cfg_gnss, lambda config: _log_cfg_gnss(config, info)),
    ("CFG-RXM", poll_cfg_rxm, _log_cfg_rxm),
    ("CFG-PM2", poll_cfg_pm2, _log_cfg_pm2),
  )
  for name, poll, log_config in diagnostics:
    if name == "CFG-PM2" and _is_hpg_product(info):
      cloudlog.info(
        "GPS acquisition configuration CFG-PM2 skipped, supported=false, reason=hpg_product_unsupported"
      )
      continue
    try:
      config = poll(pigeon)
      if config is None:
        cloudlog.warning(f"GPS acquisition configuration {name} response unavailable or malformed")
        continue
      log_config(config)
    except Exception:
      cloudlog.exception(f"GPS acquisition configuration {name} diagnostic poll failed")


def log_mon_ver_diagnostics(pigeon: TTYPigeon) -> MonVerInfo | None:
  try:
    info = poll_mon_ver(pigeon)
  except Exception:
    cloudlog.exception("GPS MON-VER diagnostic poll failed")
    return None
  if info is None:
    cloudlog.warning("GPS MON-VER diagnostic response unavailable or malformed")
    return None
  cloudlog.info(", ".join((
    "GPS MON-VER diagnostics",
    f"software_version={info.software_version}",
    f"hardware_version={info.hardware_version}",
    f"protocol_versions={list(info.protocol_versions)}",
    f"firmware_extensions={list(info.firmware_versions)}",
    f"module_identifiers={list(info.module_identifiers)}",
    f"supported_gnss={list(info.supported_gnss)}",
    f"extensions={list(info.extensions)}",
    f"diagnostic_identity={normalized_receiver_identity(info)}",
  )))
  return info


def navx5_ack_aiding_compatibility(info: MonVerInfo | None) -> tuple[bool, str]:
  if info is None:
    return False, "mon_ver_unavailable"

  software_supported = NAVX5_ACK_AIDING_SOFTWARE_VERSION_PATTERN.fullmatch(
    info.software_version.strip().upper()
  ) is not None
  protocol_supported = any(value.strip().upper() == "PROTVER=20.30" for value in info.protocol_versions)
  firmware_supported = any(value.strip().upper() == "FWVER=HPG 1.40ROV" for value in info.firmware_versions)
  if not software_supported:
    return False, "unsupported_software_version"
  if not protocol_supported:
    return False, "unsupported_protocol_version"
  if not firmware_supported:
    return False, "unsupported_firmware_version"
  return True, "m8_hpg_1_40_protver_20_30"


def assistnow_autonomous_compatibility(info: MonVerInfo | None) -> tuple[bool, str]:
  navx5_supported, reason = navx5_ack_aiding_compatibility(info)
  if not navx5_supported:
    return False, reason

  # u-blox HPG release documentation explicitly excludes AssistNow
  # Autonomous even though the generic protocol exposes its NAVX5 fields.
  return False, "hpg_1_40_rover_assistnow_autonomous_unsupported"


def log_navx5_ack_aiding_support(info: MonVerInfo | None) -> bool:
  supported, reason = navx5_ack_aiding_compatibility(info)
  message = ", ".join((
    "GPS NAVX5 ACK aiding support",
    f"supported={str(supported).lower()}",
    f"reason={reason}",
  ))
  if supported:
    cloudlog.info(message)
  else:
    cloudlog.warning(message)
  return supported


def log_assistnow_autonomous_support(info: MonVerInfo | None) -> bool:
  supported, reason = assistnow_autonomous_compatibility(info)
  message = ", ".join((
    "GPS AssistNow Autonomous support",
    f"supported={str(supported).lower()}",
    f"reason={reason}",
  ))
  if supported:
    cloudlog.info(message)
  else:
    cloudlog.warning(message)
  return supported


def poll_navx5_config(
  pigeon: TTYPigeon,
  timeout: float = NAVX5_POLL_TIMEOUT,
) -> Navx5Config | None:
  transaction = _begin_response_transaction(pigeon, build_navx5_poll_message())
  return _wait_for_parsed_response(
    pigeon, transaction, parse_navx5, 0x06, 0x23, timeout,
  )


def wait_for_cfg_ack(
  pigeon: TTYPigeon,
  transaction: ResponseTransaction,
  message_class: int,
  message_id: int,
  timeout: float = NAVX5_ACK_TIMEOUT,
) -> bool | None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    result: bool | None = None
    _, stream_frames, transaction_frames = _receive_transaction_data(pigeon, transaction)
    for frame in transaction_frames:
      if (
        result is None
        and len(frame) == 10
        and frame[2] == 0x05
        and frame[3] in (0x00, 0x01)
        and frame[6:8] == bytes((message_class, message_id))
      ):
        result = frame[3] == 0x01
        continue
    _queue_unrelated_frames(
      pigeon,
      stream_frames,
      lambda frame: (
        len(frame) == 10
        and frame[2] == 0x05
        and frame[3] in (0x00, 0x01)
        and frame[6:8] == bytes((message_class, message_id))
      ),
      transaction.operation,
    )
    if result is not None:
      return result
    time.sleep(0.001)
  return None


class Navx5AckAidingConfigurationResult(StrEnum):
  ENABLED_AND_VERIFIED = "enabled_and_verified"
  ALREADY_ENABLED = "already_enabled"
  UNSUPPORTED = "unsupported"
  POLL_UNAVAILABLE = "poll_unavailable"
  UNSUPPORTED_NAVX5_VERSION = "unsupported_navx5_version"
  WRITE_REJECTED = "write_rejected"
  WRITE_TIMED_OUT = "write_timed_out"
  READBACK_UNAVAILABLE = "readback_unavailable"
  READBACK_ACK_AIDING_FALSE = "readback_ack_aiding_false"
  READBACK_AOP_FIELD_CHANGED = "readback_aop_field_changed"
  READBACK_UNRELATED_FIELDS_CHANGED = "readback_unrelated_fields_changed"
  ERROR = "error"


class AssistNowAutonomousConfigurationResult(StrEnum):
  ENABLED_AND_VERIFIED = "enabled_and_verified"
  ALREADY_ENABLED = "already_enabled"
  UNSUPPORTED = "unsupported"
  POLL_UNAVAILABLE = "poll_unavailable"
  UNSUPPORTED_NAVX5_VERSION = "unsupported_navx5_version"
  WRITE_REJECTED = "write_rejected"
  WRITE_TIMED_OUT = "write_timed_out"
  READBACK_UNAVAILABLE = "readback_unavailable"
  READBACK_USE_AOP_FALSE = "readback_use_aop_false"
  READBACK_ORBIT_ERROR_THRESHOLD_CHANGED = "readback_orbit_error_threshold_changed"
  READBACK_UNRELATED_FIELDS_CHANGED = "readback_unrelated_fields_changed"
  ERROR = "error"


def configure_navx5_ack_aiding(
  pigeon: TTYPigeon,
  info: MonVerInfo | None,
) -> Navx5AckAidingConfigurationResult:
  supported, support_reason = navx5_ack_aiding_compatibility(info)
  if not supported:
    cloudlog.warning(f"GPS NAVX5 ACK aiding configuration skipped, reason={support_reason}")
    return Navx5AckAidingConfigurationResult.UNSUPPORTED

  try:
    current = poll_navx5_config(pigeon)
    if current is None:
      cloudlog.warning("GPS NAVX5 ACK aiding configuration failed, result=poll_unavailable")
      return Navx5AckAidingConfigurationResult.POLL_UNAVAILABLE
    if current.version != 2:
      cloudlog.warning(", ".join((
        "GPS NAVX5 ACK aiding configuration failed",
        f"navx5_version={current.version}",
        "result=unsupported_navx5_version",
      )))
      return Navx5AckAidingConfigurationResult.UNSUPPORTED_NAVX5_VERSION

    cloudlog.info(", ".join((
      "GPS NAVX5 ACK aiding configuration before",
      f"navx5_version={current.version}",
      f"ackAiding={str(current.ack_aiding).lower()}",
      f"useAOP={str(current.use_aop).lower()}",
      f"aop_orbit_max_error_m={current.aop_orbit_max_error_m}",
    )))
    if current.ack_aiding:
      cloudlog.info("GPS NAVX5 ACK aiding configuration unchanged, ackAiding=true, result=already_enabled")
      return Navx5AckAidingConfigurationResult.ALREADY_ENABLED

    transaction = _begin_response_transaction(
      pigeon, build_navx5_ack_aiding_enable_message(current),
    )
    acknowledgment = wait_for_cfg_ack(pigeon, transaction, 0x06, 0x23)
    if acknowledgment is False:
      cloudlog.warning(f"GPS NAVX5 ACK aiding configuration rejected, mask1=0x{NAVX5_MASK1_ACK_AIDING:04X}, result=write_rejected")
      return Navx5AckAidingConfigurationResult.WRITE_REJECTED
    if acknowledgment is None:
      cloudlog.warning(f"GPS NAVX5 ACK aiding configuration timed out, mask1=0x{NAVX5_MASK1_ACK_AIDING:04X}, result=write_timed_out")
      return Navx5AckAidingConfigurationResult.WRITE_TIMED_OUT

    resulting = poll_navx5_config(pigeon)
    if resulting is None:
      cloudlog.warning("GPS NAVX5 ACK aiding verification failed, result=readback_unavailable")
      return Navx5AckAidingConfigurationResult.READBACK_UNAVAILABLE
    if not resulting.ack_aiding:
      cloudlog.warning("GPS NAVX5 ACK aiding verification failed, ackAiding=false, result=readback_ack_aiding_false")
      return Navx5AckAidingConfigurationResult.READBACK_ACK_AIDING_FALSE
    if (
      resulting.use_aop != current.use_aop
      or resulting.aop_orbit_max_error_m != current.aop_orbit_max_error_m
    ):
      cloudlog.warning("GPS NAVX5 ACK aiding verification failed, result=readback_aop_field_changed")
      return Navx5AckAidingConfigurationResult.READBACK_AOP_FIELD_CHANGED
    if not navx5_unrelated_fields_unchanged(current, resulting, enabling_ack_aiding=True):
      cloudlog.warning("GPS NAVX5 ACK aiding verification failed, result=readback_unrelated_fields_changed")
      return Navx5AckAidingConfigurationResult.READBACK_UNRELATED_FIELDS_CHANGED

    cloudlog.info(", ".join((
      "GPS NAVX5 ACK aiding configuration accepted and verified",
      "previous_ackAiding=false",
      "resulting_ackAiding=true",
      f"useAOP={str(resulting.use_aop).lower()}",
      f"aop_orbit_max_error_m={resulting.aop_orbit_max_error_m}",
      f"mask1=0x{NAVX5_MASK1_ACK_AIDING:04X}",
      "result=enabled_and_verified",
    )))
    return Navx5AckAidingConfigurationResult.ENABLED_AND_VERIFIED
  except Exception:
    cloudlog.exception("GPS NAVX5 ACK aiding configuration failed, reason=unexpected_error")
    return Navx5AckAidingConfigurationResult.ERROR


def configure_assistnow_autonomous(
  pigeon: TTYPigeon,
  info: MonVerInfo | None,
) -> AssistNowAutonomousConfigurationResult:
  supported, support_reason = assistnow_autonomous_compatibility(info)
  if not supported:
    cloudlog.warning(", ".join((
      "GPS AssistNow Autonomous configuration skipped",
      f"reason={support_reason}",
    )))
    return AssistNowAutonomousConfigurationResult.UNSUPPORTED

  try:
    current = poll_navx5_config(pigeon)
    if current is None:
      cloudlog.warning("GPS AssistNow Autonomous configuration failed, result=poll_unavailable")
      return AssistNowAutonomousConfigurationResult.POLL_UNAVAILABLE
    if current.version != 2:
      cloudlog.warning(", ".join((
        "GPS AssistNow Autonomous configuration failed",
        f"navx5_version={current.version}",
        "result=unsupported_navx5_version",
      )))
      return AssistNowAutonomousConfigurationResult.UNSUPPORTED_NAVX5_VERSION

    cloudlog.info(", ".join((
      "GPS AssistNow Autonomous configuration before",
      f"navx5_version={current.version}",
      f"ackAiding={str(current.ack_aiding).lower()}",
      f"useAOP={str(current.use_aop).lower()}",
      f"aop_orbit_max_error_m={current.aop_orbit_max_error_m}",
    )))
    if current.use_aop:
      cloudlog.info("GPS AssistNow Autonomous configuration unchanged: useAOP=true, result=already_enabled")
      return AssistNowAutonomousConfigurationResult.ALREADY_ENABLED

    transaction = _begin_response_transaction(
      pigeon, build_navx5_aop_enable_message(current),
    )
    acknowledgment = wait_for_cfg_ack(pigeon, transaction, 0x06, 0x23)
    if acknowledgment is False:
      cloudlog.warning(f"GPS AssistNow Autonomous configuration rejected, mask1=0x{NAVX5_MASK1_AOP:04X}, result=write_rejected")
      return AssistNowAutonomousConfigurationResult.WRITE_REJECTED
    if acknowledgment is None:
      cloudlog.warning(f"GPS AssistNow Autonomous configuration timed out, mask1=0x{NAVX5_MASK1_AOP:04X}, result=write_timed_out")
      return AssistNowAutonomousConfigurationResult.WRITE_TIMED_OUT

    resulting = poll_navx5_config(pigeon)
    if resulting is None:
      cloudlog.warning("GPS AssistNow Autonomous verification failed, result=readback_unavailable")
      return AssistNowAutonomousConfigurationResult.READBACK_UNAVAILABLE
    if not resulting.use_aop:
      cloudlog.warning("GPS AssistNow Autonomous verification failed, useAOP=false, result=readback_use_aop_false")
      return AssistNowAutonomousConfigurationResult.READBACK_USE_AOP_FALSE
    if resulting.aop_orbit_max_error_m != current.aop_orbit_max_error_m:
      cloudlog.warning(", ".join((
        "GPS AssistNow Autonomous verification failed",
        "reason=orbit_error_threshold_changed",
        f"previous_aop_orbit_max_error_m={current.aop_orbit_max_error_m}",
        f"resulting_aop_orbit_max_error_m={resulting.aop_orbit_max_error_m}",
      )))
      return AssistNowAutonomousConfigurationResult.READBACK_ORBIT_ERROR_THRESHOLD_CHANGED
    if not navx5_unrelated_fields_unchanged(current, resulting):
      cloudlog.warning("GPS AssistNow Autonomous verification failed, result=readback_unrelated_fields_changed")
      return AssistNowAutonomousConfigurationResult.READBACK_UNRELATED_FIELDS_CHANGED

    cloudlog.info(", ".join((
      "GPS AssistNow Autonomous configuration accepted and verified",
      f"previous_useAOP={str(current.use_aop).lower()}",
      f"resulting_useAOP={str(resulting.use_aop).lower()}",
      f"ackAiding={str(resulting.ack_aiding).lower()}",
      f"aop_orbit_max_error_m={resulting.aop_orbit_max_error_m}",
      f"mask1=0x{NAVX5_MASK1_AOP:04X}",
      "result=enabled_and_verified",
    )))
    return AssistNowAutonomousConfigurationResult.ENABLED_AND_VERIFIED
  except Exception:
    cloudlog.exception("GPS AssistNow Autonomous configuration failed: reason=unexpected_error")
    return AssistNowAutonomousConfigurationResult.ERROR


def poll_nav_aopstatus(
  pigeon: TTYPigeon,
  timeout: float = AOP_STATUS_POLL_TIMEOUT,
) -> NavAopStatus | None:
  transaction = _begin_response_transaction(pigeon, build_nav_aopstatus_poll_message())
  return _wait_for_parsed_response(
    pigeon, transaction, parse_nav_aopstatus, 0x01, 0x60, timeout,
  )


class AopCaptureState(StrEnum):
  IDLE = "idle"
  BUSY = "busy"
  UNKNOWN = "unknown"
  UNSUPPORTED = "unsupported"


def wait_for_aop_idle(
  pigeon: TTYPigeon,
  timeout: float = AOP_IDLE_WAIT_TIMEOUT,
  poll_interval: float = AOP_IDLE_POLL_INTERVAL,
) -> AopCaptureState:
  deadline = time.monotonic() + timeout
  observed_busy = False
  while time.monotonic() < deadline:
    remaining = deadline - time.monotonic()
    try:
      status = poll_nav_aopstatus(
        pigeon,
        timeout=min(AOP_STATUS_POLL_TIMEOUT, remaining),
      )
    except Exception:
      cloudlog.exception("GPS AssistNow Autonomous status unavailable: reason=poll_error")
      return AopCaptureState.UNKNOWN
    if status is None:
      cloudlog.warning("GPS AssistNow Autonomous status unavailable")
      return AopCaptureState.UNKNOWN
    if status.idle:
      cloudlog.info(f"GPS AssistNow Autonomous status: state=idle, enabled={str(status.enabled).lower()}")
      return AopCaptureState.IDLE
    observed_busy = True
    cloudlog.info(f"GPS AssistNow Autonomous status: state=running, status={status.status}")
    remaining = deadline - time.monotonic()
    if remaining > 0:
      time.sleep(min(poll_interval, remaining))

  if observed_busy:
    cloudlog.warning("GPS AssistNow Autonomous status remained running through bounded wait")
    return AopCaptureState.BUSY
  cloudlog.warning("GPS AssistNow Autonomous status unavailable through bounded wait")
  return AopCaptureState.UNKNOWN

def init_baudrate(pigeon: TTYPigeon):
  # ublox default setting on startup is 9600 baudrate. Stop GNSS before
  # changing baud so no synchronous startup transaction can race the
  # trusted-age database decision.
  pigeon.set_baud(9600)
  pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)
  time.sleep(CONTROLLED_GNSS_TRANSITION_DELAY)

  # $PUBX,41,1,0007,0003,460800,0*15\r\n
  pigeon.send(b"\x24\x50\x55\x42\x58\x2C\x34\x31\x2C\x31\x2C\x30\x30\x30\x37\x2C\x30\x30\x30\x33\x2C\x34\x36\x30\x38\x30\x30\x2C\x30\x2A\x31\x35\x0D\x0A")
  time.sleep(0.1)
  pigeon.set_baud(460800)
  if hasattr(pigeon, "_stream_parser"):
    pigeon.reset_response_state()


def _poll_cfg[Config](
  pigeon: TTYPigeon,
  poll_message: bytes,
  response_parser: Callable[[bytes], Config | None],
  timeout: float = 0.5,
  response_matches: Callable[[Config], bool] | None = None,
) -> Config:
  transaction = _begin_response_transaction(pigeon, poll_message)
  config = _wait_for_parsed_response(
    pigeon,
    transaction,
    response_parser,
    poll_message[2],
    poll_message[3],
    timeout,
    response_matches,
  )
  if config is None:
    raise CfgPollTimeoutError(
      f"No valid CFG response for message 0x{poll_message[2]:02X} 0x{poll_message[3]:02X}"
    )
  return config


def poll_cfg_rate(pigeon: TTYPigeon, timeout: float = 0.5) -> RateConfig:
  return _poll_cfg(
    pigeon, build_cfg_rate_poll_message(), parse_cfg_rate, timeout,
  )


def poll_cfg_nav5(pigeon: TTYPigeon, timeout: float = 0.5) -> Nav5Config:
  return _poll_cfg(
    pigeon, build_cfg_nav5_poll_message(), parse_cfg_nav5, timeout,
  )


def poll_cfg_odo(pigeon: TTYPigeon, timeout: float = 0.5) -> OdoConfig:
  return _poll_cfg(
    pigeon, build_cfg_odo_poll_message(), parse_cfg_odo, timeout,
  )


def poll_cfg_itfm(pigeon: TTYPigeon, timeout: float = 0.5) -> ItfmConfig:
  return _poll_cfg(
    pigeon, build_cfg_itfm_poll_message(), parse_cfg_itfm, timeout,
  )


def poll_cfg_msg(
  pigeon: TTYPigeon,
  message_class: int,
  message_id: int,
  timeout: float = 0.5,
) -> MessageRateConfig:
  poll_message = build_cfg_msg_poll_message(message_class, message_id)
  return _poll_cfg(
    pigeon,
    poll_message,
    parse_cfg_msg,
    timeout,
    lambda config: (
      config.message_class == message_class
      and config.message_id == message_id
    ),
  )


def poll_cfg_prt(
  pigeon: TTYPigeon,
  port_id: int,
  timeout: float = 0.5,
) -> PortConfig:
  poll_message = build_cfg_prt_poll_message(port_id)
  return _poll_cfg(
    pigeon,
    poll_message,
    parse_cfg_prt,
    timeout,
    lambda response: response.port_id == port_id,
  )


def verify_cfg_prt_config(
  actual: PortConfig,
  expected: PortConfig,
) -> None:
  fields_by_port = {
    0: ("tx_ready", "mode", "input_protocol_mask", "output_protocol_mask", "flags"),
    1: ("tx_ready", "mode", "baud_rate", "input_protocol_mask", "output_protocol_mask", "flags"),
    3: ("tx_ready", "input_protocol_mask", "output_protocol_mask"),
    4: ("tx_ready", "mode", "input_protocol_mask", "output_protocol_mask", "flags"),
  }
  fields = fields_by_port.get(expected.port_id)
  if actual.port_id != expected.port_id or fields is None:
    raise ReceiverConfigurationError(
      f"CFG-PRT readback port mismatch: expected={expected.port_id}, actual={actual.port_id}"
    )
  mismatches = [
    field for field in fields
    if getattr(actual, field) != getattr(expected, field)
  ]
  if mismatches:
    raise ReceiverConfigurationError(
      f"CFG-PRT readback mismatch for port {expected.port_id}: fields={mismatches}"
    )


def verify_startup_configuration(
  rate: RateConfig,
  nav5: Nav5Config,
  odo: OdoConfig,
  itfm: ItfmConfig,
  nav_pvt: MessageRateConfig,
  rawx: MessageRateConfig,
) -> None:
  if rate != RateConfig(100, 1, 0):
    raise ReceiverConfigurationError(f"CFG-RATE readback mismatch: {rate}")
  if nav5.dynamic_model != 4 or nav5.fix_mode != 3:
    raise ReceiverConfigurationError(f"CFG-NAV5 readback mismatch: {nav5}")
  if (odo.flags & 0x0F) != 0x01 or odo.profile != 3:
    raise ReceiverConfigurationError(f"CFG-ODO readback mismatch: {odo}")
  if itfm != ItfmConfig(0xAD62ADFF, 0x0000631E):
    raise ReceiverConfigurationError(f"CFG-ITFM readback mismatch: {itfm}")
  if nav_pvt.rates[1] != 1:
    raise ReceiverConfigurationError(f"CFG-MSG NAV-PVT readback mismatch: {nav_pvt}")
  if rawx.rates[1] != 1:
    raise ReceiverConfigurationError(f"CFG-MSG RXM-RAWX readback mismatch: {rawx}")


def log_startup_configuration(
  rate: RateConfig,
  nav5: Nav5Config,
  odo: OdoConfig,
  itfm: ItfmConfig,
  nav_pvt: MessageRateConfig,
  rawx: MessageRateConfig,
) -> None:
  cloudlog.info(", ".join((
    "GPS startup configuration CFG-RATE effective",
    f"measurement_period_ms={rate.measurement_period_ms}",
    f"navigation_rate={rate.navigation_rate}",
    f"time_reference={rate.time_reference}",
  )))
  cloudlog.info(", ".join((
    "GPS startup configuration CFG-NAV5 effective",
    f"dynamic_model={nav5.dynamic_model}",
    f"fix_mode={nav5.fix_mode}",
  )))
  cloudlog.info(", ".join((
    "GPS startup configuration CFG-ODO effective",
    f"version={odo.version}",
    f"flags=0x{odo.flags:02X}",
    f"profile={odo.profile}",
  )))
  cloudlog.info(", ".join((
    "GPS startup configuration CFG-ITFM effective",
    f"config=0x{itfm.config:08X}",
    f"config2=0x{itfm.config2:08X}",
  )))
  for name, config in (("NAV-PVT", nav_pvt), ("RXM-RAWX", rawx)):
    cloudlog.info(", ".join((
      f"GPS startup configuration CFG-MSG {name} effective",
      f"message_class=0x{config.message_class:02X}",
      f"message_id=0x{config.message_id:02X}",
      f"rates={list(config.rates)}",
      f"uart1_rate={config.rates[1]}",
    )))


def init_pigeon(pigeon: TTYPigeon) -> bool:
  # try initializing a few times
  for _ in range(10):
    try:

      # setup port config
      port_configuration_messages = (
        b"\xb5\x62\x06\x00\x14\x00\x03\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x00\x00\x00\x00\x00\x1E\x7F",
        b"\xb5\x62\x06\x00\x14\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x19\x35",
        b"\xb5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x01\x00\x00\x00\x00\x00\xF4\x80",
        b"\xb5\x62\x06\x00\x14\x00\x04\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1D\x85",
      )
      expected_port_configurations = {}
      for message in port_configuration_messages:
        pigeon.send_with_ack(message)
        config = parse_cfg_prt(message)
        if config is None:
          raise ReceiverConfigurationError("Invalid built-in CFG-PRT configuration message")
        expected_port_configurations[config.port_id] = config
      for port_id in (0, 1, 3, 4):
        verify_cfg_prt_config(
          poll_cfg_prt(pigeon, port_id),
          expected_port_configurations[port_id],
        )

      # UBX-CFG-RATE (0x06 0x08)
      pigeon.send_with_ack(b"\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10")
      rate = poll_cfg_rate(pigeon)

      # UBX-CFG-NAV5 (0x06 0x24)
      pigeon.send_with_ack(b"\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63")

      # UBX-CFG-ODO (0x06 0x1E)
      pigeon.send_with_ack(b"\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37")
      pigeon.send_with_ack(b"\xB5\x62\x06\x39\x08\x00\xFF\xAD\x62\xAD\x1E\x63\x00\x00\x83\x0C")

      nav5 = poll_cfg_nav5(pigeon)
      odo = poll_cfg_odo(pigeon)
      itfm = poll_cfg_itfm(pigeon)

      # UBX-CFG-MSG (set message rate)
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x01\x07\x01\x13\x51")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x02\x15\x01\x22\x70")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x02\x13\x01\x20\x6C")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x0A\x09\x01\x1E\x70")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x0A\x0B\x01\x20\x74")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x01\x35\x01\x41\xAD")
      nav_pvt = poll_cfg_msg(pigeon, 0x01, 0x07)
      rawx = poll_cfg_msg(pigeon, 0x02, 0x15)
      verify_startup_configuration(rate, nav5, odo, itfm, nav_pvt, rawx)
      log_startup_configuration(rate, nav5, odo, itfm, nav_pvt, rawx)
      cloudlog.debug("pigeon configured")

      # try restoring almanac backup
      restore_status = pigeon.poll_backup_restore_status()
      if restore_status == 2:
        cloudlog.warning("almanac backup restored")
      elif restore_status == 3:
        cloudlog.warning("no almanac backup found")
      else:
        cloudlog.error(f"failed to restore almanac backup, status: {restore_status}")

      # try getting AssistNow if we have a token
      token = Params().get('AssistNowToken')
      if token is not None:
        try:
          for msg in get_assistnow_messages(token):
            pigeon.send_with_ack(msg, ack=UBLOX_ASSIST_ACK)
          cloudlog.warning("AssistNow messages sent")
        except Exception:
          cloudlog.warning("failed to get AssistNow messages")

      cloudlog.warning("Pigeon GPS on!")
      break
    except (CfgPollTimeoutError, ResponseTransactionError) as exc:
      if hasattr(pigeon, "dispatch_pending_frames"):
        pigeon.dispatch_pending_frames()
      cloudlog.warning(f"Receiver cycle initialization aborted: {exc}")
      return False
    except (ReceiverConfigurationError, TimeoutError) as exc:
      # UBX-ACK-ACK/NAK is documented to arrive within one second. The
      # retry boundary below expires that response window even when a matching
      # NAK classified the attempt immediately. The next transaction then
      # performs a bounded input drain before retrying.
      if isinstance(exc, CfgNakError) and exc.retry_not_before is not None:
        retry_delay = exc.retry_not_before - time.monotonic()
        if retry_delay > 0:
          time.sleep(retry_delay)
      if hasattr(pigeon, "dispatch_pending_frames"):
        pigeon.dispatch_pending_frames()
      cloudlog.warning(f"Initialization failed, trying again: {exc}")
  else:
    cloudlog.warning("Failed to initialize pigeon")
    return False
  return True

def deinitialize_and_exit(pigeon: TTYPigeon | None):
  if pigeon is not None:
    # controlled GNSS stop
    pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)

  # turn off power and exit cleanly
  set_power(False)
  sys.exit(0)

@dataclass
class PreAcquisitionInitialization:
  callback: Callable[[], None]
  executed: bool = False

  def run(self) -> None:
    if self.executed:
      return
    self.executed = True
    self.callback()


_ACTIVE_PRE_ACQUISITION_INITIALIZATION: (
  PreAcquisitionInitialization | None
) = None


@contextmanager
def install_pre_acquisition_initialization(
  callback: Callable[[], None],
) -> Iterator[PreAcquisitionInitialization]:
  global _ACTIVE_PRE_ACQUISITION_INITIALIZATION
  if _ACTIVE_PRE_ACQUISITION_INITIALIZATION is not None:
    raise RuntimeError("pre-acquisition initialization is already active")
  state = PreAcquisitionInitialization(callback)
  _ACTIVE_PRE_ACQUISITION_INITIALIZATION = state
  try:
    yield state
  finally:
    _ACTIVE_PRE_ACQUISITION_INITIALIZATION = None


def start_pigeon_transport(pigeon: TTYPigeon) -> None:
  signal.signal(signal.SIGINT, lambda sig, frame: deinitialize_and_exit(pigeon))
  if hasattr(pigeon, "_stream_parser"):
    pigeon.reset_response_state()
  set_power(False)
  time.sleep(0.1)
  set_power(True)
  # STOP is the first possible receiver command after power-on. It is
  # repeated by init_baudrate after the receiver boot interval.
  pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)
  time.sleep(0.5)
  init_baudrate(pigeon)


@contextmanager
def paused_gnss_acquisition(pigeon: TTYPigeon) -> Iterator[None]:
  # UBX-CFG-RST resetMode 0x08/0x09 stops/starts GNSS tasks without
  # clearing the hot-start BBR data. Newer firmware does not ACK these
  # commands, so the transition is bounded by a short deterministic delay.
  pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)
  time.sleep(CONTROLLED_GNSS_TRANSITION_DELAY)
  try:
    yield
  except BaseException:
    # A failed pre-acquisition claim must leave GNSS stopped.
    raise
  else:
    pigeon.send(CONTROLLED_GNSS_START_MESSAGE)
    time.sleep(CONTROLLED_GNSS_TRANSITION_DELAY)


def finish_pigeon_initialization(pigeon: TTYPigeon) -> None:
  if not init_pigeon(pigeon):
    raise RuntimeError("Failed to initialize pigeon")


def init(pigeon: TTYPigeon) -> None:
  start_pigeon_transport(pigeon)
  initialization = _ACTIVE_PRE_ACQUISITION_INITIALIZATION
  if initialization is not None:
    with paused_gnss_acquisition(pigeon):
      initialization.run()
  finish_pigeon_initialization(pigeon)


def send_time_assistance(
  pigeon: TTYPigeon,
  assistance_time: datetime | None = None,
  accuracy_seconds: int = 30,
  source: str = "synchronized",
  diagnostic_context: str | None = None,
  ack_timeout: float = GPS_ASSISTANCE_ACK_TIMEOUT,
  time_provenance: ReceiverTimeProvenanceTracker | None = None,
  assistance_boottime_seconds: float | None = None,
  independent: bool | None = None,
  source_provenance: TimeProvenance | None = None,
  correction: bool = False,
) -> bool:
  """Send trusted UTC or an explicit RTC-derived estimate."""
  time_tracker = time_provenance or getattr(
    pigeon,
    "time_provenance",
    None,
  )
  if assistance_time is None:
    host_time = read_host_time_observation()
    if host_time is None or not host_time.independent:
      return False

    assistance_time = host_time.utc
    accuracy_seconds = min(
      65_535,
      max(0, ceil(host_time.uncertainty_seconds)),
    )
    assistance_boottime_seconds = (
      host_time.observed_boottime_seconds
    )
    independent = True
    source_provenance = TimeProvenance.NETWORK_INDEPENDENT
    source = host_time.source.value

  msg = build_time_assistance_message(
    assistance_time,
    accuracy_seconds=accuracy_seconds,
  )
  context_suffix = (
    f", {diagnostic_context}"
    if diagnostic_context is not None
    else ""
  )
  message_fields = (
    f"source={source}",
    f"uncertainty_seconds={accuracy_seconds}",
    f"correction={str(correction).lower()}",
    f"mga_message_id=0x{msg[3]:02X}",
    f"mga_message_type=0x{msg[6]:02X}",
  )

  try:
    transaction = _begin_response_transaction(pigeon, msg)
    if time_tracker is not None:
      written_boottime = assistance_boottime_seconds
      if written_boottime is None:
        written_boottime = read_boottime_seconds()
      time_tracker.note_time_assistance_written(
        source=source,
        assistance_utc=assistance_time,
        uncertainty_seconds=accuracy_seconds,
        now=transaction.sent_at,
        written_boottime_seconds=written_boottime,
        independent=independent,
        provenance=source_provenance,
        correction=correction,
      )
  except Exception:
    cloudlog.exception(
      ", ".join((
        "Time assistance serial write failed",
        *message_fields,
        "write_result=failed",
        "ack_result=not_attempted",
      )) + context_suffix
    )
    return False

  try:
    acknowledgment = wait_for_matching_mga_ack(
      pigeon,
      transaction,
      msg,
      timeout=ack_timeout,
    )
  except Exception:
    cloudlog.exception(
      ", ".join((
        "Time assistance written; ublox ACK observation failed",
        *message_fields,
        "write_result=succeeded",
        "ack_result=observation_failed",
      )) + context_suffix
    )
    return False

  if acknowledgment is None:
    cloudlog.warning(
      ", ".join((
        "Time assistance written; matching ublox ACK timed out",
        *message_fields,
        "write_result=succeeded",
        "ack_result=timed_out",
      )) + context_suffix
    )
    return False
  else:
    ack_fields = (
      f"ack_type={acknowledgment.acknowledgment_type}",
      f"ack_version={acknowledgment.version}",
      f"ack_infoCode={acknowledgment.info_code}",
      f"ack_message_id=0x{acknowledgment.message_id:02X}",
    )
    if acknowledgment.accepted:
      cloudlog.info(
        ", ".join((
          "Time assistance written and accepted by ublox",
          *message_fields,
          "write_result=succeeded",
          "ack_result=accepted",
          *ack_fields,
        )) + context_suffix
      )
    else:
      cloudlog.warning(
        ", ".join((
          "Time assistance written but rejected by ublox",
          *message_fields,
          "write_result=succeeded",
          "ack_result=rejected",
          *ack_fields,
        )) + context_suffix
      )
      return False

  return True


def evaluate_time_authority(
  time_authority: TimeAuthority,
  host_time_observation: HostTimeObservation | None,
) -> TimeAuthorityEvaluation:
  evaluation = time_authority.current_authorized_time(
    host_time_observation=host_time_observation,
  )
  authorized = evaluation.authorized_time
  fields = [
    "GPS trusted time authority evaluation",
    (
      f"authorized={str(authorized is not None).lower()}"
    ),
    (
      f"evidence={authorized.evidence.value}"
      if authorized is not None
      else "evidence=none"
    ),
    (
      f"source={authorized.source.value}"
      if authorized is not None
      else "source=none"
    ),
    (
      f"independent={str(authorized.independent).lower()}"
      if authorized is not None
      else "independent=false"
    ),
    (
      f"uncertainty_seconds={authorized.uncertainty_seconds}"
      if authorized is not None
      else "uncertainty_seconds=none"
    ),
    (
      "host_source="
      + (
        host_time_observation.source.value
        if host_time_observation is not None
        else "none"
      )
    ),
    (
      "host_independent="
      + (
        str(host_time_observation.independent).lower()
        if host_time_observation is not None
        else "false"
      )
    ),
    (
      "host_generation="
      + (
        host_time_observation.generation
        if host_time_observation is not None
        else "none"
      )
    ),
    (
      f"rejection_reason={evaluation.rejection_reason.value}"
      if evaluation.rejection_reason is not None
      else "rejection_reason=none"
    ),
    f"anchor_write_status={evaluation.anchor_write_status.value}",
    (
      f"anchor_write_error={evaluation.anchor_write_error}"
      if evaluation.anchor_write_error is not None
      else "anchor_write_error=none"
    ),
    (
      "selected_anchor_generation="
      + (
        evaluation.selected_anchor_generation
        if evaluation.selected_anchor_generation is not None
        else "none"
      )
    ),
    (
      "selected_anchor_sequence="
      + str(evaluation.selected_anchor_sequence)
    ),
    (
      "anchor_write_reason="
      + (
        evaluation.anchor_write_reason.value
        if evaluation.anchor_write_reason is not None
        else "none"
      )
    ),
    (
      "anchor_comparison_status="
      + (
        evaluation.anchor_comparison.status.value
        if evaluation.anchor_comparison is not None
        else "none"
      )
    ),
    (
      "anchor_error_seconds="
      + (
        str(evaluation.anchor_comparison.error_seconds)
        if evaluation.anchor_comparison is not None
        else "none"
      )
    ),
    (
      "anchor_allowed_error_seconds="
      + (
        str(
          evaluation
          .anchor_comparison
          .allowed_error_seconds
        )
        if evaluation.anchor_comparison is not None
        else "none"
      )
    ),
  ]
  message = ", ".join(fields)
  if authorized is not None:
    cloudlog.info(message)
  else:
    cloudlog.warning(message)
  return evaluation


def cached_rtc_time_assistance(
  receiver_fingerprint: str,
) -> tuple[datetime, int] | None:
  """Choose the freshest defensible RTC estimate from either fixed cache."""
  store = NavigationCacheStore(GPS_ASSISTANCE_CACHE_PATH, loader=load_cache)
  cleanup_failure = store.remove_stale_candidate()
  if cleanup_failure is not None:
    cloudlog.warning(f"GPS stale cache candidate cleanup failed: reason={cleanup_failure}")
  inventory = store.inspect(receiver_fingerprint, None)
  current_rtc = read_rtc_counter_seconds()
  selected, evaluations = select_rtc_estimate(inventory, current_rtc)
  for inspection, result in evaluations:
    if result is None:
      cloudlog.info(
        f"GPS RTC anchor generation: generation={inspection.generation}, status={inspection.state.name.lower()}, reason={inspection.error or 'unavailable'}"
      )
      continue
    if isinstance(result, RtcEstimateRejection):
      level = cloudlog.warning if result.reason is RtcEstimateRejectionReason.RTC_ROLLBACK else cloudlog.info
      reason_text = {
        RtcEstimateRejectionReason.MISSING_CACHED_RTC_ANCHOR: "cache has no RTC anchor",
        RtcEstimateRejectionReason.CURRENT_RTC_UNAVAILABLE: "current RTC unavailable",
        RtcEstimateRejectionReason.RTC_ROLLBACK: "RTC rollback detected",
        RtcEstimateRejectionReason.ELAPSED_TIME_ABOVE_MAXIMUM: "elapsed time above maximum",
        RtcEstimateRejectionReason.UTC_BEFORE_SUPPORTED_MINIMUM: "estimated UTC before supported minimum",
        RtcEstimateRejectionReason.UTC_AFTER_SUPPORTED_MAXIMUM: "estimated UTC after supported maximum",
        RtcEstimateRejectionReason.INVALID_RTC_ESTIMATE: "RTC estimate invalid",
      }[result.reason]
      level(", ".join((
        "GPS RTC anchor generation rejected",
        f"generation={inspection.generation}",
        f"reason={reason_text}",
        f"saved_rtc_seconds={inspection.cache.rtc_counter_seconds}",
        f"current_rtc_seconds={current_rtc}",
        f"elapsed_seconds={result.elapsed_seconds}",
        f"maximum_elapsed_seconds={MAX_RTC_ASSISTANCE_ELAPSED_SECONDS}",
      )))

  if selected is None:
    cloudlog.info("GPS RTC time assistance skipped: no valid fixed-file RTC anchor")
    return None
  cloudlog.info(", ".join((
    "GPS RTC time assistance ready",
    f"generation={selected.generation}",
    f"elapsed_seconds={selected.estimate.elapsed_seconds}",
    f"uncertainty_seconds={selected.estimate.uncertainty_seconds}",
  )))
  return selected.estimate.estimated_utc, selected.estimate.uncertainty_seconds


def gps_assistance_receiver_fingerprint(
  params: Params,
) -> str:
  hardware_serial = (
    params.get("HardwareSerial")
    or HARDWARE.get_serial()
  )

  return f"{hardware_serial}|ublox-m8-prot20.30"


def wait_for_matching_mga_ack(
  pigeon: TTYPigeon,
  transaction: ResponseTransaction,
  message: bytes,
  timeout: float = GPS_ASSISTANCE_ACK_TIMEOUT,
) -> MgaAck | None:
  if len(message) < 8:
    raise CacheValidationError("MGA message is truncated")

  expected_message_id = message[3]
  expected_payload_start = message[6:10].ljust(
    4,
    b"\x00",
  )
  deadline = time.monotonic() + timeout

  while time.monotonic() < deadline:
    result = None
    _, stream_frames, transaction_frames = _receive_transaction_data(pigeon, transaction)
    for frame in transaction_frames:
      acknowledgment = parse_mga_ack(frame)

      if (
        result is None
        and acknowledgment is not None
        and (
          acknowledgment.message_id != expected_message_id
          or acknowledgment.message_payload_start
          != expected_payload_start
        )
      ):
        continue
      if result is None and acknowledgment is not None:
        result = acknowledgment
        continue
    _queue_unrelated_frames(
      pigeon,
      stream_frames,
      lambda frame: (
        (acknowledgment := parse_mga_ack(frame)) is not None
        and acknowledgment.message_id == expected_message_id
        and acknowledgment.message_payload_start == expected_payload_start
      ),
      transaction.operation,
    )

    if result is not None:
      return result
    time.sleep(0.001)

  return None


def send_mga_with_strict_ack(
  pigeon: TTYPigeon,
  message: bytes,
  timeout: float = GPS_ASSISTANCE_ACK_TIMEOUT,
  database_frame_index: int | None = None,
  time_provenance: ReceiverTimeProvenanceTracker | None = None,
  time_assistance_source: str = "mga_time_assistance",
) -> None:
  if len(message) < 8:
    raise CacheValidationError("MGA message is truncated")

  expected_message_id = message[3]
  payload_length = int.from_bytes(message[4:6], "little")
  mga_message_type = (
    f"0x{message[6]:02X}"
    if payload_length > 0
    else "unavailable"
  )

  try:
    transaction = _begin_response_transaction(pigeon, message)
    if (
      time_provenance is not None
      and is_mga_time_assistance_message(message)
    ):
      time_provenance.note_time_assistance_written(
        source=time_assistance_source,
        assistance_utc=None,
        uncertainty_seconds=None,
        now=transaction.sent_at,
      )
  except OSError as exc:
    raise MgaWriteError(
      f"Failed to write MGA message 0x{expected_message_id:02X}: {type(exc).__name__}: {exc}"
    ) from exc
  except ResponseTransactionError as exc:
    raise MgaTransactionError(
      f"Failed to start MGA acknowledgment transaction for message 0x{expected_message_id:02X}: {type(exc).__name__}: {exc}"
    ) from exc

  try:
    acknowledgment = wait_for_matching_mga_ack(
      pigeon,
      transaction,
      message,
      timeout=timeout,
    )
  except TimeoutError:
    raise
  except (OSError, ResponseTransactionError) as exc:
    raise MgaTransactionError(
      f"MGA acknowledgment transaction failed for message 0x{expected_message_id:02X}: {type(exc).__name__}: {exc}"
    ) from exc

  if acknowledgment is None:
    raise TimeoutError(f"No matching MGA acknowledgment for message 0x{expected_message_id:02X}")

  if not acknowledgment.accepted:
    rejection_fields = [
      f"mga_message_type={mga_message_type}",
      f"message_id=0x{expected_message_id:02X}",
      f"ack_type={acknowledgment.acknowledgment_type}",
      f"ack_version={acknowledgment.version}",
      f"ack_infoCode={acknowledgment.info_code}",
      f"rejected_message_id=0x{acknowledgment.message_id:02X}",
    ]
    if database_frame_index is not None:
      rejection_fields.append(
        f"database_frame_index={database_frame_index}"
      )
    raise MgaReceiverNackError(
      "u-blox rejected MGA message: "
      + ", ".join(rejection_fields)
    )


class NavigationAssistanceRestoreStatus(StrEnum):
  COMPLETE = "complete"
  PARTIAL = "partial"
  FAILED = "failed"


class NavigationAssistanceRestoreFailurePhase(StrEnum):
  CACHE_LOAD = "cache_load"
  POSITION_ASSISTANCE_BUILD = "position_assistance_build"
  POSITION_ASSISTANCE_WRITE = "position_assistance_write"
  POSITION_ASSISTANCE_ACK_REJECTED = "position_assistance_ack_rejected"
  POSITION_ASSISTANCE_ACK_TIMEOUT = "position_assistance_ack_timeout"
  DATABASE_FRAME_RESTORE = "database_frame_restore"


class NavigationAssistanceCacheResult(StrEnum):
  SAVED = "saved"
  PRESERVED_EXISTING = "preserved_existing"
  FAILED = "failed"


def navigation_cache_phase_completed(result: NavigationAssistanceCacheResult) -> bool:
  return result in (
    NavigationAssistanceCacheResult.SAVED,
    NavigationAssistanceCacheResult.PRESERVED_EXISTING,
  )


@dataclass
class NavigationCaptureState:
  drive_cache_saved: bool = False
  post_drive_refresh_pending: bool = False
  durable_baseline_quality: NavigationQuality | None = None
  durable_cache_ready: bool = False
  readiness_log_key: tuple[object, ...] | None = None
  last_successful_qualified_upgrade: float | None = None
  capture_fix: NavPvtFix | None = None
  capture_quality: NavigationQuality | None = None
  capture_reason: str | None = None
  capture_receiver_cycle: int | None = None
  capture_is_upgrade: bool = False
  next_capture_attempt: float = 0.0

  @property
  def frozen(self) -> bool:
    return (
      self.capture_fix is not None
      or self.capture_quality is not None
      or self.capture_reason is not None
      or self.capture_receiver_cycle is not None
      or self.capture_is_upgrade
    )

  def road_state_changed(self, started: bool) -> None:
    if started:
      self.drive_cache_saved = False
      self.post_drive_refresh_pending = False
      self.durable_baseline_quality = None
      self.durable_cache_ready = False
      self.readiness_log_key = None
      self.last_successful_qualified_upgrade = None
      self.reset_receiver_cycle()
    else:
      self.post_drive_refresh_pending = True

  def reset_receiver_cycle(self) -> None:
    self.capture_fix = None
    self.capture_quality = None
    self.capture_reason = None
    self.capture_receiver_cycle = None
    self.capture_is_upgrade = False
    self.next_capture_attempt = 0.0

  def fail(self, now: float) -> None:
    self.capture_fix = None
    self.capture_quality = None
    self.capture_reason = None
    self.capture_receiver_cycle = None
    self.capture_is_upgrade = False
    self.next_capture_attempt = now + GPS_ASSISTANCE_CAPTURE_RETRY_INTERVAL

  def request(
    self,
    now: float,
    started: bool | None,
    collector_active: bool,
    tracker: CaptureQualityTracker,
    receiver_cycle: int | None = None,
    stable_fix: NavPvtFix | None = None,
  ) -> bool:
    if collector_active or self.frozen or now < self.next_capture_attempt:
      return False
    if started is True:
      context = "onroad" if not self.drive_cache_saved else "onroad_refresh"
    elif started is False and self.post_drive_refresh_pending:
      context = "post_drive"
    else:
      return False

    quality = tracker.quality(
      now,
      "onroad" if context == "onroad_refresh" else context,
    )
    fix = tracker.latest_fix
    if not capture_eligible(quality, stable_fix, fix):
      return False

    is_upgrade = self.drive_cache_saved
    if is_upgrade:
      if (
        quality is None
        or not quality.passes_policy
        or self.durable_baseline_quality is None
        or not navigation_quality_strictly_better(
          quality, self.durable_baseline_quality,
        )
      ):
        return False
      if (
        self.last_successful_qualified_upgrade is not None
        and now - self.last_successful_qualified_upgrade
        < GPS_ASSISTANCE_QUALIFIED_UPGRADE_COOLDOWN
      ):
        return False

    assert quality is not None and fix is not None
    self.capture_fix = fix
    self.capture_quality = quality
    self.capture_reason = context
    self.capture_receiver_cycle = 0 if receiver_cycle is None else receiver_cycle
    self.capture_is_upgrade = is_upgrade
    return True

  def complete(
    self,
    result: NavigationAssistanceCacheResult,
    now: float,
    durable_quality: NavigationQuality | None = None,
    finalized_quality: NavigationQuality | None = None,
  ) -> str | None:
    readiness_message = None
    if navigation_cache_phase_completed(result):
      durable_tier = navigation_quality_tier(durable_quality)
      durable_confirmed = durable_tier in (
        CacheQualityTier.USABLE,
        CacheQualityTier.QUALIFIED,
      )
      if not durable_confirmed:
        self.fail(now)
        return None

      self.durable_cache_ready = True
      self.drive_cache_saved = True
      self.durable_baseline_quality = durable_quality
      if self.capture_reason == "post_drive":
        self.post_drive_refresh_pending = False
      if (
        self.capture_is_upgrade
        and result is NavigationAssistanceCacheResult.SAVED
        and finalized_quality is not None
        and finalized_quality.passes_policy
      ):
        self.last_successful_qualified_upgrade = now
      if result is NavigationAssistanceCacheResult.PRESERVED_EXISTING:
        action = "existing_cache_preserved"
      elif finalized_quality == durable_quality:
        action = "candidate_saved_selected"
      else:
        action = "candidate_saved_existing_selected"
      context = self.capture_reason or "unknown"
      readiness_message = self.completion_readiness_message(
        action,
        context,
        finalized_quality,
        self.durable_baseline_quality,
      )
      self.capture_fix = None
      self.capture_quality = None
      self.capture_reason = None
      self.capture_receiver_cycle = None
      self.capture_is_upgrade = False
      self.next_capture_attempt = 0.0
    else:
      self.fail(now)
    return readiness_message

  def readiness_message(
    self,
    ready: bool,
    reason: str,
    quality: NavigationQuality | None = None,
  ) -> str | None:
    key = (ready, reason, self.quality_log_signature(quality))
    if self.readiness_log_key == key:
      return None
    self.readiness_log_key = key
    fields = [
      "GPS navigation cache power-removal readiness",
      f"ready={ready}",
      f"reason={reason}",
    ]
    if quality is not None:
      tier = navigation_quality_tier(quality)
      fields.extend((
        f"quality_tier={tier.value if tier is not None else 'invalid'}",
        f"gps_ephemeris={quality.gps_ephemeris_available}",
        f"glonass_ephemeris={quality.glonass_ephemeris_available}",
        f"total_ephemeris={quality.total_ephemeris_available}",
        f"satellites_used={quality.satellites_used}",
      ))
    return ", ".join(fields)

  @staticmethod
  def quality_log_signature(
    quality: NavigationQuality | None,
  ) -> tuple[object, ...] | None:
    if quality is None:
      return None
    return (
      navigation_quality_tier(quality),
      quality.continuous_reliable_fix_seconds,
      quality.continuous_orbit_quality_seconds,
      quality.gps_ephemeris_available,
      quality.glonass_ephemeris_available,
      quality.satellites_used,
      quality.gps_almanac_available,
      quality.glonass_almanac_available,
      quality.assistnow_offline_available,
    )

  def completion_readiness_message(
    self,
    action: str,
    context: str,
    candidate_quality: NavigationQuality | None,
    selected_quality: NavigationQuality,
  ) -> str | None:
    candidate_tier = navigation_quality_tier(candidate_quality)
    selected_tier = navigation_quality_tier(selected_quality)
    key = (
      True,
      action,
      context,
      self.quality_log_signature(candidate_quality),
      self.quality_log_signature(selected_quality),
    )
    if self.readiness_log_key == key:
      return None
    self.readiness_log_key = key
    return ", ".join((
      "GPS navigation cache power-removal readiness",
      "ready=True",
      f"candidate_quality_tier={candidate_tier.value if candidate_tier is not None else 'unavailable'}",
      f"selected_quality_tier={selected_tier.value if selected_tier is not None else 'invalid'}",
      f"action={action}",
      f"context={context}",
      f"selected_gps_ephemeris={selected_quality.gps_ephemeris_available}",
      f"selected_glonass_ephemeris={selected_quality.glonass_ephemeris_available}",
      f"selected_total_ephemeris={selected_quality.total_ephemeris_available}",
      f"selected_satellites_used={selected_quality.satellites_used}",
    ))

  def drive_end_readiness_message(self) -> str | None:
    if self.durable_cache_ready:
      return None
    return self.readiness_message(
      False,
      "no_usable_cache_completed",
    )


def request_navigation_database_capture(
  pigeon: TTYPigeon,
  dump_collector: NavigationDatabaseDumpCollector,
  capture_state: NavigationCaptureState,
  now: float,
  assistnow_autonomous_supported: bool,
) -> AopCaptureState:
  aop_state = (
    wait_for_aop_idle(pigeon)
    if assistnow_autonomous_supported
    else AopCaptureState.UNSUPPORTED
  )
  cloudlog.info(", ".join((
    "GPS navigation cache capture AOP state",
    f"aop_state={aop_state.value}",
    f"capture_reason={capture_state.capture_reason}",
    "action=proceed",
  )))
  dump_collector.start(now)
  pigeon.send(build_database_poll_message())
  cloudlog.info(f"Requested GPS navigation database for {capture_state.capture_reason} cache: frozen_quality={capture_state.capture_quality}")
  return aop_state


@dataclass
class AutonomousOrbitDiagnostics:
  logged_state_mask: int = 0

  def note_nav_sat(self, nav_sat: NavSatQuality) -> None:
    available = nav_sat.assistnow_autonomous_available
    used = nav_sat.orbit_source_counts.get("assistnow_autonomous", 0)
    state_index = (int(available > 0) << 1) | int(used > 0)
    state_bit = 1 << state_index
    if self.logged_state_mask & state_bit:
      return
    self.logged_state_mask |= state_bit
    cloudlog.info(", ".join((
      "GPS AssistNow Autonomous orbit diagnostics",
      f"available_satellites={available}",
      f"used_as_orbit_source_satellites={used}",
      f"autonomous_orbit_data_present={str(available > 0).lower()}",
      f"autonomous_orbit_data_used={str(used > 0).lower()}",
    )))


def finalized_capture_quality(
  state: NavigationCaptureState,
  tracker: CaptureQualityTracker,
  now: float,
  active_receiver_cycle: int | None = None,
  stable_fix: NavPvtFix | None = None,
) -> NavigationQuality | None:
  if state.capture_reason is None:
    return None
  if (
    active_receiver_cycle is not None
    and state.capture_receiver_cycle != active_receiver_cycle
  ):
    return None
  live_quality = tracker.quality(
    now,
    "onroad" if state.capture_reason == "onroad_refresh" else state.capture_reason,
  )
  if (
    not capture_eligible(live_quality, stable_fix, tracker.latest_fix)
    or state.capture_quality is None
    or not state.capture_quality.usable_for_capture
  ):
    return None
  assert live_quality is not None
  conservative_quality = conservative_navigation_quality(
    state.capture_quality, live_quality,
  )
  if conservative_quality is None or not conservative_quality.usable_for_capture:
    return None
  if not state.capture_is_upgrade:
    return conservative_quality
  if (
    conservative_quality.passes_policy
    and state.durable_baseline_quality is not None
    and navigation_quality_strictly_better(
      conservative_quality,
      state.durable_baseline_quality,
    )
  ):
    return conservative_quality
  return None


def capture_quality_remains_valid(
  state: NavigationCaptureState,
  tracker: CaptureQualityTracker,
  now: float,
  active_receiver_cycle: int | None = None,
  stable_fix: NavPvtFix | None = None,
) -> bool:
  return finalized_capture_quality(
    state, tracker, now, active_receiver_cycle, stable_fix,
  ) is not None


def durable_quality_after_cache_result(
  result: NavigationAssistanceCacheResult,
  receiver_fingerprint: str,
  trusted_now: datetime | None = None,
) -> NavigationQuality | None:
  if not navigation_cache_phase_completed(result) or trusted_now is None:
    return None
  try:
    selection, _ = NavigationCacheStore(
      GPS_ASSISTANCE_CACHE_PATH, loader=load_cache,
    ).select_best(receiver_fingerprint, trusted_now)
  except Exception:
    cloudlog.exception(
      "Failed to resolve selected GPS navigation cache generation",
    )
    return None
  return None if selection is None else selection.cache.quality


@dataclass(frozen=True)
class NavigationAssistanceRestoreResult:
  status: NavigationAssistanceRestoreStatus
  total_frame_count: int
  accepted_frame_count: int
  initially_rejected_indexes: tuple[int, ...] = ()
  initially_timed_out_indexes: tuple[int, ...] = ()
  retry_accepted_indexes: tuple[int, ...] = ()
  permanently_rejected_indexes: tuple[int, ...] = ()
  permanently_timed_out_indexes: tuple[int, ...] = ()
  failure_phase: NavigationAssistanceRestoreFailurePhase | None = None
  cache_saved_at_utc: datetime | None = None
  restored_cache_generation: str | None = None
  restored_cache_selection_reason: str | None = None
  restored_cache_age_seconds: float | None = None
  restored_cache_age_evidence: str | None = None
  restored_cache_age_verified: bool = False
  captured_gps_ephemeris_available: int | None = None
  captured_glonass_ephemeris_available: int | None = None
  captured_gps_startup_ready: bool | None = None
  restored_gps_ephemeris_fresh: bool | None = None
  restored_glonass_ephemeris_fresh: bool | None = None
  restored_quality_expiration_reasons: tuple[str, ...] = ()
  restored_navigation_quality: RestoredNavigationQuality | None = None
  restored_gps_almanac_available: int | None = None
  restored_glonass_almanac_available: int | None = None
  restored_gps_ephemeris_available: int | None = None
  restored_glonass_ephemeris_available: int | None = None
  restored_satellites_used: int | None = None
  restored_gps_startup_ready: bool | None = None
  restored_gps_almanac_satellite_ids: tuple[int, ...] | None = None
  captured_gps_almanac_available: int | None = None
  captured_glonass_almanac_available: int | None = None
  captured_satellites_used: int | None = None
  captured_gps_almanac_satellite_ids: tuple[int, ...] | None = None
  database_restore_disposition: NavigationDatabaseRestoreDisposition | None = None
  database_frames_attempted_count: int = 0
  database_restore_boot_id: str | None = None
  database_restore_state_error: str | None = None
  database_restore_recovered_interrupted_attempt: bool = False
  database_restore_initial_failure_kinds: tuple[str, ...] = ()
  database_restore_permanent_failure_kinds: tuple[str, ...] = ()
  database_restore_execution_error: str | None = None
  database_restore_runtime_phase: str | None = None

  @property
  def usable(self) -> bool:
    if self.database_restore_disposition is not None:
      return self.database_restore_disposition.database_available
    return self.status in (
      NavigationAssistanceRestoreStatus.COMPLETE,
      NavigationAssistanceRestoreStatus.PARTIAL,
    )


def format_navigation_assistance_restore_summary(
  result: NavigationAssistanceRestoreResult | None,
  *,
  attempted: bool,
  time_assistance_source: str | None,
  diagnostic_context: str | None = None,
) -> str:
  if result is None:
    fields = (
      "GPS navigation assistance restore result",
      f"restore_attempted={attempted}",
      "total_frames=0",
      "accepted_frames=0",
      "rejected_frames=0",
      "retry_attempts=0",
      "timeout_events=0",
      "failure_phase=none",
      "terminal_result=not_attempted",
      "database_restore_disposition=not_attempted",
      "database_frames_attempted=0",
      "database_terminal_ack_count_matched=not_applicable_per_frame_restore",
      f"time_assistance_source={time_assistance_source or 'none'}",
    )
  else:
    fields = (
    "GPS navigation assistance restore result",
    f"restore_attempted={attempted}",
    f"restore_status={result.status.value}",
    f"total_frames={result.total_frame_count}",
    f"accepted_frames={result.accepted_frame_count}",
    f"total_frame_count={result.total_frame_count}",
    f"accepted_frame_count={result.accepted_frame_count}",
    f"rejected_frames={len(result.permanently_rejected_indexes)}",
    f"retry_attempts={len(result.initially_rejected_indexes) + len(result.initially_timed_out_indexes)}",
    f"timeout_events={len(result.initially_timed_out_indexes) + len(result.permanently_timed_out_indexes)}",
    f"failure_phase={result.failure_phase.value if result.failure_phase is not None else 'none'}",
    f"terminal_result={result.status.value}",
    "database_restore_disposition="
    + (
      result.database_restore_disposition.value
      if result.database_restore_disposition is not None
      else "legacy"
    ),
    f"database_frames_attempted={result.database_frames_attempted_count}",
    f"database_restore_boot_id={result.database_restore_boot_id or 'none'}",
    f"database_restore_state_error={result.database_restore_state_error or 'none'}",
    "database_restore_recovered_interrupted_attempt="
    + str(result.database_restore_recovered_interrupted_attempt).lower(),
    "database_restore_initial_failure_kinds="
    + str(list(result.database_restore_initial_failure_kinds)),
    "database_restore_permanent_failure_kinds="
    + str(list(result.database_restore_permanent_failure_kinds)),
    "database_restore_execution_error="
    + (result.database_restore_execution_error or "none"),
    "database_restore_runtime_phase="
    + (result.database_restore_runtime_phase or "none"),
    "database_terminal_ack_count_matched=not_applicable_per_frame_restore",
    f"time_assistance_source={time_assistance_source or 'unknown'}",
    f"restored_cache_generation={result.restored_cache_generation or 'none'}",
    f"restored_cache_selection_reason={result.restored_cache_selection_reason or 'none'}",
    f"restored_cache_age_seconds={result.restored_cache_age_seconds}",
    f"restored_cache_age_evidence={result.restored_cache_age_evidence or 'none'}",
    f"restored_cache_age_verified={str(result.restored_cache_age_verified).lower()}",
    f"captured_gps_ephemeris_available={result.captured_gps_ephemeris_available}",
    f"captured_glonass_ephemeris_available={result.captured_glonass_ephemeris_available}",
    f"captured_gps_startup_ready={result.captured_gps_startup_ready}",
    f"captured_gps_almanac_available={result.captured_gps_almanac_available}",
    f"captured_glonass_almanac_available={result.captured_glonass_almanac_available}",
    f"captured_satellites_used={result.captured_satellites_used}",
    f"captured_gps_almanac_satellite_ids={result.captured_gps_almanac_satellite_ids}",
    f"restored_gps_ephemeris_fresh={result.restored_gps_ephemeris_fresh}",
    f"restored_glonass_ephemeris_fresh={result.restored_glonass_ephemeris_fresh}",
    f"restored_quality_expiration_reasons={list(result.restored_quality_expiration_reasons)}",
    f"restored_gps_almanac_available={result.restored_gps_almanac_available}",
    f"restored_glonass_almanac_available={result.restored_glonass_almanac_available}",
    f"restored_gps_ephemeris_available={result.restored_gps_ephemeris_available}",
    f"restored_glonass_ephemeris_available={result.restored_glonass_ephemeris_available}",
    f"restored_satellites_used={result.restored_satellites_used}",
    f"restored_gps_startup_ready={result.restored_gps_startup_ready}",
    f"restored_gps_almanac_satellite_ids={result.restored_gps_almanac_satellite_ids}",
    f"initially_rejected_indexes={list(result.initially_rejected_indexes)}",
    f"initially_timed_out_indexes={list(result.initially_timed_out_indexes)}",
    f"retry_accepted_indexes={list(result.retry_accepted_indexes)}",
    f"permanently_rejected_indexes={list(result.permanently_rejected_indexes)}",
    f"permanently_timed_out_indexes={list(result.permanently_timed_out_indexes)}",
    )
  message = ", ".join(fields)
  if diagnostic_context is not None:
    message += f", {diagnostic_context}"
  return message


def log_navigation_assistance_restore_result(
  result: NavigationAssistanceRestoreResult,
  diagnostic_context: str | None,
  time_assistance_source: str | None = None,
) -> None:
  message = format_navigation_assistance_restore_summary(
    result,
    attempted=True,
    time_assistance_source=time_assistance_source,
    diagnostic_context=diagnostic_context,
  )

  if result.status is NavigationAssistanceRestoreStatus.FAILED:
    cloudlog.error(message)
  elif result.status is NavigationAssistanceRestoreStatus.PARTIAL:
    cloudlog.warning(message)
  else:
    cloudlog.info(message)


def navigation_assistance_result_from_database_execution(
  execution: NavigationDatabaseRestoreExecution,
) -> NavigationAssistanceRestoreResult:
  disposition = execution.disposition
  if disposition.database_available:
    status = (
      NavigationAssistanceRestoreStatus.COMPLETE
      if execution.position_assistance_succeeded
      else NavigationAssistanceRestoreStatus.PARTIAL
    )
    failure_phase = (
      None
      if execution.position_assistance_succeeded
      else NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE
    )
  elif execution.position_assistance_succeeded:
    status = NavigationAssistanceRestoreStatus.PARTIAL
    failure_phase = (
      NavigationAssistanceRestoreFailurePhase.DATABASE_FRAME_RESTORE
      if disposition.write_failed
      else None
    )
  else:
    status = NavigationAssistanceRestoreStatus.FAILED
    failure_phase = (
      NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE
      if execution.position_assistance_attempted
      else NavigationAssistanceRestoreFailurePhase.CACHE_LOAD
    )

  evaluated_quality = execution.effective_quality
  restored_quality = (
    evaluated_quality if disposition.database_available else None
  )
  captured_quality = execution.captured_quality
  return NavigationAssistanceRestoreResult(
    status=status,
    total_frame_count=execution.total_frame_count,
    accepted_frame_count=execution.accepted_frame_count,
    initially_rejected_indexes=execution.initial_indexes(
      NavigationDatabaseRestoreFrameFailureKind.REJECTED,
      NavigationDatabaseRestoreFrameFailureKind.VALIDATION_ERROR,
    ),
    initially_timed_out_indexes=execution.initial_indexes(
      NavigationDatabaseRestoreFrameFailureKind.TIMED_OUT,
    ),
    retry_accepted_indexes=execution.retry_accepted_indexes,
    permanently_rejected_indexes=execution.permanent_indexes(
      NavigationDatabaseRestoreFrameFailureKind.REJECTED,
      NavigationDatabaseRestoreFrameFailureKind.VALIDATION_ERROR,
    ),
    permanently_timed_out_indexes=execution.permanent_indexes(
      NavigationDatabaseRestoreFrameFailureKind.TIMED_OUT,
    ),
    failure_phase=failure_phase,
    cache_saved_at_utc=execution.cache_saved_at_utc,
    restored_cache_generation=execution.cache_generation,
    restored_cache_selection_reason=execution.cache_selection_reason,
    restored_cache_age_seconds=execution.cache_age_seconds,
    restored_cache_age_evidence=(
      evaluated_quality.age_evidence.value
      if evaluated_quality is not None
      else None
    ),
    restored_cache_age_verified=(
      evaluated_quality.age_verified
      if evaluated_quality is not None
      else False
    ),
    captured_gps_ephemeris_available=(
      evaluated_quality.captured_gps_ephemeris_available
      if evaluated_quality is not None
      else None
    ),
    captured_glonass_ephemeris_available=(
      evaluated_quality.captured_glonass_ephemeris_available
      if evaluated_quality is not None
      else None
    ),
    captured_gps_startup_ready=(
      evaluated_quality.captured_gps_startup_ready
      if evaluated_quality is not None
      else None
    ),
    captured_gps_almanac_available=(
      getattr(captured_quality, "gps_almanac_available", None)
      if captured_quality is not None
      else None
    ),
    captured_glonass_almanac_available=(
      getattr(captured_quality, "glonass_almanac_available", None)
      if captured_quality is not None
      else None
    ),
    captured_satellites_used=(
      getattr(captured_quality, "satellites_used", None)
      if captured_quality is not None
      else None
    ),
    captured_gps_almanac_satellite_ids=(
      getattr(captured_quality, "gps_almanac_satellite_ids", None)
      if captured_quality is not None
      else None
    ),
    restored_gps_ephemeris_fresh=(
      restored_quality.gps_ephemeris_fresh
      if restored_quality is not None
      else None
    ),
    restored_glonass_ephemeris_fresh=(
      restored_quality.glonass_ephemeris_fresh
      if restored_quality is not None
      else None
    ),
    restored_quality_expiration_reasons=(
      evaluated_quality.expiration_reasons
      if evaluated_quality is not None
      else ()
    ),
    restored_navigation_quality=restored_quality,
    restored_gps_almanac_available=(
      getattr(captured_quality, "gps_almanac_available", None)
      if restored_quality is not None and captured_quality is not None
      else None
    ),
    restored_glonass_almanac_available=(
      getattr(captured_quality, "glonass_almanac_available", None)
      if restored_quality is not None and captured_quality is not None
      else None
    ),
    restored_gps_ephemeris_available=(
      restored_quality.effective_gps_ephemeris_available
      if restored_quality is not None
      else None
    ),
    restored_glonass_ephemeris_available=(
      restored_quality.effective_glonass_ephemeris_available
      if restored_quality is not None
      else None
    ),
    restored_satellites_used=(
      getattr(captured_quality, "satellites_used", None)
      if restored_quality is not None and captured_quality is not None
      else None
    ),
    restored_gps_startup_ready=(
      restored_quality.effective_gps_startup_ready
      if restored_quality is not None
      else None
    ),
    restored_gps_almanac_satellite_ids=(
      getattr(captured_quality, "gps_almanac_satellite_ids", None)
      if restored_quality is not None and captured_quality is not None
      else None
    ),
    database_restore_disposition=disposition,
    database_frames_attempted_count=(
      execution.database_write_attempt_count
    ),
    database_restore_boot_id=execution.boot_id,
    database_restore_state_error=execution.state_persistence_error,
    database_restore_recovered_interrupted_attempt=(
      execution.recovered_interrupted_attempt
    ),
    database_restore_initial_failure_kinds=tuple(
      f"{failure.frame_index}:{failure.kind.value}"
      for failure in execution.initial_failures
    ),
    database_restore_permanent_failure_kinds=tuple(
      f"{failure.frame_index}:{failure.kind.value}"
      for failure in execution.permanent_failures
    ),
    database_restore_execution_error=execution.execution_error,
    database_restore_runtime_phase=execution.failure_phase,
  )


def restore_navigation_assistance(
  pigeon: TTYPigeon,
  receiver_fingerprint: str,
  diagnostic_context: str | None = None,
  time_assistance_source: str | None = None,
  trusted_now: datetime | None = None,
  *,
  navigation_database_runtime: NavigationDatabaseRestoreRuntime | None = None,
  authorized_time: AuthorizedTime | None = None,
  allow_legacy_direct_restore: bool = False,
) -> NavigationAssistanceRestoreResult:
  if navigation_database_runtime is not None:
    navigation_database_runtime.prepare()
    execution = navigation_database_runtime.evaluate(
      authorized_time=authorized_time,
      reliable_fix_available=False,
      yuma_already_sent=False,
      send_database_message=(
        lambda message, frame_index: send_mga_with_strict_ack(
          pigeon,
          message,
          database_frame_index=frame_index,
        )
      ),
    )
    if not navigation_database_runtime.acquisition_started:
      navigation_database_runtime.send_position_once(
        lambda message: send_mga_with_strict_ack(pigeon, message)
      )
      execution = navigation_database_runtime.execution
    result = navigation_assistance_result_from_database_execution(execution)
    log_navigation_assistance_restore_result(
      result,
      diagnostic_context,
      time_assistance_source,
    )
    return result

  if not allow_legacy_direct_restore:
    raise RuntimeError(
      "live navigation assistance restore requires a boot-scoped runtime"
    )

  if trusted_now is None:
    host_time = read_host_time_observation()
    if host_time is not None and host_time.independent:
      trusted_now = host_time.utc
      if time_assistance_source is None:
        time_assistance_source = host_time.source.value

  store = NavigationCacheStore(GPS_ASSISTANCE_CACHE_PATH, loader=load_cache)
  cleanup_failure = store.remove_stale_candidate()
  if cleanup_failure is not None:
    cloudlog.warning(f"GPS stale cache candidate cleanup failed: reason={cleanup_failure}")
  normalized_time_source = (
    time_assistance_source or ""
  ).casefold().replace("-", "_")

  if trusted_now is None:
    cache_age_evidence = CacheAgeEvidence.UNVERIFIED
  elif normalized_time_source == "rtc_estimate":
    cache_age_evidence = CacheAgeEvidence.RTC_ESTIMATE
  else:
    cache_age_evidence = CacheAgeEvidence.TRUSTED_UTC

  selection, inventory = store.select_best(
    receiver_fingerprint,
    trusted_now,
    age_evidence=cache_age_evidence,
  )
  cloudlog.info(", ".join((
    "GPS navigation cache startup generation selection",
    f"primary_status={inventory.primary.state.name.lower()}",
    f"previous_status={inventory.previous.state.name.lower()}",
    f"selected_generation={selection.generation if selection is not None else 'none'}",
    f"selection_reason={selection.reason if selection is not None else 'no_eligible_cache'}",
    f"age_evidence={cache_age_evidence.value}",
    f"age_verified={str(cache_age_evidence.verified).lower()}",
    f"trusted_time_source={time_assistance_source or 'unavailable'}",
    f"primary_quality={getattr(inventory.primary.cache, 'quality', None)}",
    f"previous_quality={getattr(inventory.previous.cache, 'quality', None)}",
  )))
  if selection is None:
    cloudlog.info(
      "GPS assistance cache load rejected: no eligible primary or previous cache"
    )
    result = NavigationAssistanceRestoreResult(
      status=NavigationAssistanceRestoreStatus.FAILED,
      total_frame_count=0,
      accepted_frame_count=0,
      failure_phase=NavigationAssistanceRestoreFailurePhase.CACHE_LOAD,
    )
    log_navigation_assistance_restore_result(
      result,
      diagnostic_context,
      time_assistance_source,
    )
    return result
  cache = selection.cache
  restored_quality = getattr(cache, "quality", None)
  effective_restored_quality = effective_restored_navigation_quality(
    restored_quality,
    cache.saved_at_utc,
    trusted_now,
    cache_age_evidence,
  )

  cloudlog.info(
    ", ".join((
      f"GPS assistance cache loaded: saved_at_utc={cache.saved_at_utc.isoformat()}",
      f"generation={selection.generation}",
      f"rtc_anchor_present={cache.rtc_counter_seconds is not None}",
      f"database_messages={len(cache.database_frames)}",
    ))
  )

  total_frame_count = len(cache.database_frames)
  accepted_indexes: set[int] = set()
  initially_rejected_indexes: list[int] = []
  initially_timed_out_indexes: list[int] = []
  retry_accepted_indexes: list[int] = []
  permanently_rejected_indexes: list[int] = []
  permanently_timed_out_indexes: list[int] = []
  failed_frames: list[tuple[int, bytes]] = []
  active_phase = NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_BUILD
  failure_phase = None

  try:
    position_message = build_position_assistance_message(
      latitude_e7=cache.latitude_e7,
      longitude_e7=cache.longitude_e7,
      altitude_cm=cache.altitude_cm,
      position_accuracy_cm=cache.position_accuracy_cm,
    )

    active_phase = NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE
    send_mga_with_strict_ack(
      pigeon,
      position_message,
    )

    active_phase = NavigationAssistanceRestoreFailurePhase.DATABASE_FRAME_RESTORE
    for database_frame_index, database_message in enumerate(
      cache.database_frames
    ):
      try:
        send_mga_with_strict_ack(
          pigeon,
          database_message,
          database_frame_index=database_frame_index,
        )
        accepted_indexes.add(database_frame_index)
      except CacheValidationError as exc:
        initially_rejected_indexes.append(
          database_frame_index
        )
        failed_frames.append((
          database_frame_index,
          database_message,
        ))
        cloudlog.warning(
          f"GPS navigation database frame rejected on initial pass: {exc}"
        )
      except (TimeoutError, MgaTransactionError) as exc:
        initially_timed_out_indexes.append(
          database_frame_index
        )
        failed_frames.append((
          database_frame_index,
          database_message,
        ))
        cloudlog.warning(
          f"GPS navigation database frame timed out on initial pass: database_frame_index={database_frame_index}, {exc}"
        )

    if failed_frames:
      time.sleep(GPS_ASSISTANCE_FRAME_RETRY_DELAY)

    for database_frame_index, database_message in failed_frames:
      try:
        send_mga_with_strict_ack(
          pigeon,
          database_message,
          database_frame_index=database_frame_index,
        )
        accepted_indexes.add(database_frame_index)
        retry_accepted_indexes.append(database_frame_index)
      except CacheValidationError as exc:
        permanently_rejected_indexes.append(
          database_frame_index
        )
        cloudlog.warning(
          f"GPS navigation database frame rejected on retry: {exc}"
        )
      except (TimeoutError, MgaTransactionError) as exc:
        permanently_timed_out_indexes.append(
          database_frame_index
        )
        cloudlog.warning(
          f"GPS navigation database frame timed out on retry: database_frame_index={database_frame_index}, {exc}"
        )

  except (
    CacheValidationError,
    MgaTransactionError,
    MgaWriteError,
    OSError,
    ResponseTransactionError,
    TimeoutError,
  ) as exc:
    if active_phase is NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE:
      if isinstance(exc, TimeoutError):
        failure_phase = NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_TIMEOUT
      elif isinstance(exc, CacheValidationError):
        failure_phase = NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_REJECTED
      else:
        failure_phase = NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE
    else:
      failure_phase = active_phase
    cloudlog.exception(
      f"Failed to restore GPS navigation assistance cache, failure_phase={failure_phase.value}"
    )
    status = NavigationAssistanceRestoreStatus.FAILED
  except Exception:
    failure_phase = active_phase
    cloudlog.exception(
      f"Unexpected failure while restoring GPS navigation assistance cache, failure_phase={failure_phase.value}"
    )
    status = NavigationAssistanceRestoreStatus.FAILED
  else:
    if not accepted_indexes:
      status = NavigationAssistanceRestoreStatus.FAILED
    elif len(accepted_indexes) == total_frame_count:
      status = NavigationAssistanceRestoreStatus.COMPLETE
    else:
      status = NavigationAssistanceRestoreStatus.PARTIAL
    if status is not NavigationAssistanceRestoreStatus.COMPLETE:
      failure_phase = NavigationAssistanceRestoreFailurePhase.DATABASE_FRAME_RESTORE

  result = NavigationAssistanceRestoreResult(
    status=status,
    total_frame_count=total_frame_count,
    accepted_frame_count=len(accepted_indexes),
    initially_rejected_indexes=tuple(
      initially_rejected_indexes
    ),
    initially_timed_out_indexes=tuple(
      initially_timed_out_indexes
    ),
    retry_accepted_indexes=tuple(retry_accepted_indexes),
    permanently_rejected_indexes=tuple(
      permanently_rejected_indexes
    ),
    permanently_timed_out_indexes=tuple(
      permanently_timed_out_indexes
    ),
    failure_phase=failure_phase,
    cache_saved_at_utc=cache.saved_at_utc,
    restored_cache_generation=selection.generation,
    restored_cache_selection_reason=selection.reason,
    restored_cache_age_seconds=(
      effective_restored_quality.cache_age_seconds
    ),
    restored_cache_age_evidence=(
      effective_restored_quality.age_evidence.value
    ),
    restored_cache_age_verified=(
      effective_restored_quality.age_verified
    ),
    captured_gps_ephemeris_available=(
      effective_restored_quality.captured_gps_ephemeris_available
    ),
    captured_glonass_ephemeris_available=(
      effective_restored_quality.captured_glonass_ephemeris_available
    ),
    captured_gps_startup_ready=(
      effective_restored_quality.captured_gps_startup_ready
    ),
    restored_gps_ephemeris_fresh=(
      effective_restored_quality.gps_ephemeris_fresh
    ),
    restored_glonass_ephemeris_fresh=(
      effective_restored_quality.glonass_ephemeris_fresh
    ),
    restored_quality_expiration_reasons=(
      effective_restored_quality.expiration_reasons
    ),
    restored_navigation_quality=effective_restored_quality,
    restored_gps_almanac_available=getattr(
      restored_quality,
      "gps_almanac_available",
      None,
    ),
    restored_glonass_almanac_available=getattr(
      restored_quality,
      "glonass_almanac_available",
      None,
    ),
    restored_gps_ephemeris_available=(
      effective_restored_quality.effective_gps_ephemeris_available
    ),
    restored_glonass_ephemeris_available=(
      effective_restored_quality.effective_glonass_ephemeris_available
    ),
    restored_satellites_used=getattr(
      restored_quality,
      "satellites_used",
      None,
    ),
    restored_gps_startup_ready=(
      effective_restored_quality.effective_gps_startup_ready
    ),
    restored_gps_almanac_satellite_ids=getattr(
      restored_quality,
      "gps_almanac_satellite_ids",
      None,
    ),
  )
  log_navigation_assistance_restore_result(
    result,
    diagnostic_context,
    time_assistance_source,
  )
  return result


def cache_promotion_trusted_now(
  receiver_utc: datetime | None,
  capture_receiver_cycle: int | None,
  active_receiver_cycle: int | None,
  *,
  receiver_utc_fresh: bool,
  synchronized_utc: datetime | None = None,
  receiver_utc_independent: bool = False,
  authorized_utc: datetime | None = None,
) -> datetime | None:
  if authorized_utc is not None:
    try:
      if (
        authorized_utc.tzinfo is None
        or authorized_utc.utcoffset() is None
      ):
        return None
      return authorized_utc.astimezone(UTC)
    except Exception:
      return None

  host_time = read_host_time_observation()
  if host_time is not None and host_time.independent:
    if synchronized_utc is None:
      return host_time.utc
    try:
      if (
        synchronized_utc.tzinfo is None
        or synchronized_utc.utcoffset() is None
      ):
        return None
      synchronized_now = synchronized_utc.astimezone(UTC)
    except Exception:
      return None
    if abs(
      (host_time.utc - synchronized_now).total_seconds()
    ) > 1.0:
      return None
    return synchronized_now

  if (
    not receiver_utc_fresh
    or not receiver_utc_independent
    or receiver_utc is None
    or receiver_utc.tzinfo is None
    or capture_receiver_cycle is None
    or active_receiver_cycle is None
    or capture_receiver_cycle != active_receiver_cycle
  ):
    return None
  try:
    if receiver_utc.utcoffset() is None:
      return None
    return receiver_utc.astimezone(UTC)
  except Exception:
    return None


def write_navigation_assistance_cache(
  receiver_fingerprint: str,
  fix: NavPvtFix,
  database_frames: tuple[bytes, ...],
  quality: NavigationQuality,
  source: str = "unknown",
  receiver_cycle: int | None = None,
  receiver_utc_now: datetime | None = None,
  active_receiver_cycle: int | None = None,
  receiver_utc_fresh: bool | None = None,
  trusted_promotion_utc: datetime | None = None,
  receiver_utc_independent: bool = False,
) -> NavigationAssistanceCacheResult:
  quality_tier = navigation_quality_tier(quality)
  if quality_tier not in (CacheQualityTier.USABLE, CacheQualityTier.QUALIFIED):
    cloudlog.warning("GPS navigation assistance candidate rejected: usable capture policy failed")
    cloudlog.warning("GPS navigation assistance cache outcome: failed")
    return NavigationAssistanceCacheResult.FAILED

  # Compatibility defaults apply only to direct callers that predate explicit
  # receiver-cycle plumbing. Production always supplies both cycle values.
  capture_cycle = 0 if receiver_cycle is None else receiver_cycle
  active_cycle = capture_cycle if active_receiver_cycle is None else active_receiver_cycle
  receiver_time = fix.utc_time if receiver_utc_now is None else receiver_utc_now
  fresh = receiver_time is not None if receiver_utc_fresh is None else receiver_utc_fresh
  normalized_promotion_utc = None
  if trusted_promotion_utc is not None:
    try:
      if trusted_promotion_utc.tzinfo is None or trusted_promotion_utc.utcoffset() is None:
        raise ValueError("Trusted promotion UTC has no UTC offset")
      normalized_promotion_utc = trusted_promotion_utc.astimezone(UTC)
    except Exception:
      cloudlog.warning("GPS assistance cache not saved because trusted promotion UTC is invalid")
      cloudlog.warning("GPS navigation assistance cache outcome: failed")
      return NavigationAssistanceCacheResult.FAILED
  trusted_now = cache_promotion_trusted_now(
    receiver_time,
    capture_cycle,
    active_cycle,
    receiver_utc_fresh=fresh,
    receiver_utc_independent=receiver_utc_independent,
    authorized_utc=normalized_promotion_utc,
  )
  if (
    trusted_now is None
    or (
      normalized_promotion_utc is not None
      and trusted_now != normalized_promotion_utc
    )
  ):
    cloudlog.warning("GPS assistance cache not saved because no trusted UTC time is available")
    cloudlog.warning("GPS navigation assistance cache outcome: failed")
    return NavigationAssistanceCacheResult.FAILED

  try:
    cache = create_cache(
      receiver_fingerprint=receiver_fingerprint,
      fix=fix,
      database_frames=database_frames,
      saved_at_utc=trusted_now,
      rtc_counter_seconds=read_rtc_counter_seconds(),
      quality=quality,
      receiver_cycle=capture_cycle,
    )
    promotion = NavigationCacheStore(
      GPS_ASSISTANCE_CACHE_PATH, loader=load_cache,
    ).promote(
      cache, receiver_fingerprint, trusted_now, active_cycle,
    )
  except Exception:
    cloudlog.exception(
      "Failed to save GPS navigation assistance cache"
    )
    cloudlog.warning("GPS navigation assistance cache outcome: failed")
    return NavigationAssistanceCacheResult.FAILED

  primary_quality = promotion.inventory.primary.cache.quality if promotion.inventory.primary.cache else None
  previous_quality = promotion.inventory.previous.cache.quality if promotion.inventory.previous.cache else None
  cloudlog.info(", ".join((
    "GPS navigation cache promotion result",
    f"source={source}",
    f"quality_tier={quality_tier.value}",
    f"receiver_cycle={capture_cycle}",
    f"generation={promotion.selected.generation if promotion.selected is not None else 'none'}",
    f"promotion_stage={promotion.stage.value}",
    f"fallback_generation={promotion.fallback_generation or 'none'}",
    f"selection_reason={promotion.selection_reason or 'none'}",
    f"terminal_result={promotion.status.name.lower()}",
    f"reason={promotion.reason}",
    f"candidate_quality={quality}",
    f"primary_quality={primary_quality}",
    f"previous_quality={previous_quality}",
  )))
  if promotion.cleanup_failure is not None:
    cloudlog.warning(", ".join((
      "GPS navigation cache candidate cleanup failed",
      f"source={source}",
      f"receiver_cycle={capture_cycle}",
      f"reason={promotion.cleanup_failure}",
    )))
  if promotion.status is CachePromotionStatus.PRESERVED_EXISTING:
    cloudlog.info("GPS navigation assistance cache outcome: existing preserved")
    return NavigationAssistanceCacheResult.PRESERVED_EXISTING
  if promotion.status is CachePromotionStatus.FAILED:
    cloudlog.warning("GPS navigation assistance cache outcome: failed")
    return NavigationAssistanceCacheResult.FAILED

  cloudlog.warning(f"Saved GPS navigation assistance cache: {len(database_frames)} database messages")
  cloudlog.info("GPS navigation assistance cache outcome: saved")
  return NavigationAssistanceCacheResult.SAVED


def is_all_zero_ublox_data(data: bytes) -> bool:
  return bool(data) and not any(data)


class UbloxDataWatchdog:
  def __init__(
    self,
    timeout: float = 10.0,
    max_recoveries: int = 1,
    start_time: float | None = None,
  ):
    self.timeout = timeout
    self.max_recoveries = max_recoveries
    self.last_data_time = (
      time.monotonic()
      if start_time is None
      else start_time
    )
    self.recoveries = 0

  def note_data(self, now: float) -> None:
    self.last_data_time = now
    self.recoveries = 0

  def check(self, now: float) -> bool:
    if now - self.last_data_time < self.timeout:
      return False

    if self.recoveries >= self.max_recoveries:
      raise RuntimeError(
        "No data from ublox after watchdog recovery"
      )

    self.recoveries += 1
    return True

  def recovery_completed(self, now: float) -> None:
    self.last_data_time = now


class GpsStartupDiagnostics:
  def __init__(
    self,
    process_start_time: float,
    status_interval: float = GPS_ACQUISITION_STATUS_INTERVAL,
  ) -> None:
    self.process_start_time = process_start_time
    self.status_interval = status_interval
    self.cycle_number = 0
    self.cycle_reason = ""
    self.cycle_start_time = process_start_time
    self._reset_cycle_state(process_start_time)

  def _reset_cycle_state(self, now: float) -> None:
    self.first_nav_pvt_logged = False
    self.first_fix_ok_logged = False
    self.first_receiver_utc_logged = False
    self.reliable_fix_observed = False
    self.first_rawx_after_initialization_logged = False
    self.first_nonempty_rawx_logged = False
    self.first_valid_gps_week_logged = False
    self.first_valid_leap_second_logged = False
    self.first_gps_measurement_logged = False
    self.first_glonass_measurement_logged = False
    self.latest_fix: NavPvtFix | None = None
    self.latest_fix_time: float | None = None
    self.next_status_time = now + self.status_interval

  def _timing_fields(self, now: float) -> tuple[str, ...]:
    return (
      f"cycle={self.cycle_number}",
      f"reason={self.cycle_reason}",
      f"process_elapsed_seconds={now - self.process_start_time:.1f}",
      f"cycle_elapsed_seconds={now - self.cycle_start_time:.1f}",
    )

  def _fix_fields(self, fix: NavPvtFix) -> tuple[str, ...]:
    return (
      f"fix_ok={fix.fix_ok}",
      f"satellites={fix.satellites}",
      f"horizontal_accuracy_cm={fix.horizontal_accuracy_cm}",
      f"receiver_utc_valid={fix.utc_time is not None}",
    )

  def start_cycle(self, reason: str, now: float) -> None:
    self.cycle_number += 1
    self.cycle_reason = reason
    self.cycle_start_time = now
    self._reset_cycle_state(now)
    cloudlog.info(", ".join((
      "GPS receiver cycle started",
      *self._timing_fields(now)[:3],
    )))

  def initialization_complete(self, now: float) -> None:
    cloudlog.info(", ".join((
      "GPS receiver cycle initialization complete",
      *self._timing_fields(now)[:3],
      f"cycle_initialization_elapsed_seconds={now - self.cycle_start_time:.1f}",
    )))

  def time_assistance_context(self, now: float) -> str:
    return ", ".join(self._timing_fields(now))

  def _log_milestone(
    self,
    milestone: str,
    fix: NavPvtFix,
    now: float,
  ) -> None:
    cloudlog.info(", ".join((
      f"GPS acquisition milestone={milestone}",
      *self._timing_fields(now),
      *self._fix_fields(fix),
    )))

  def note_nav_pvt(self, fix: NavPvtFix, now: float) -> None:
    self.latest_fix = fix
    self.latest_fix_time = now

    if not self.first_nav_pvt_logged:
      self._log_milestone("first_nav_pvt", fix, now)
      self.first_nav_pvt_logged = True

    if fix.fix_ok and not self.first_fix_ok_logged:
      self._log_milestone("first_fix_ok", fix, now)
      self.first_fix_ok_logged = True

    if fix.utc_time is not None and not self.first_receiver_utc_logged:
      self._log_milestone("first_receiver_utc", fix, now)
      self.first_receiver_utc_logged = True

    if fix.reliable and not self.reliable_fix_observed:
      self._log_milestone("first_reliable_fix", fix, now)
      self.reliable_fix_observed = True

  def _log_rawx_milestone(
    self,
    milestone: str,
    rawx: Ubx.RxmRawx,
    now: float,
  ) -> None:
    measurement_counts: dict[int, int] = {}
    maximum_cno: dict[int, int] = {}
    for measurement in rawx.meas:
      gnss_id = int(measurement.gnss_id)
      measurement_counts[gnss_id] = (
        measurement_counts.get(gnss_id, 0) + 1
      )
      maximum_cno[gnss_id] = max(
        maximum_cno.get(gnss_id, 0),
        measurement.cno,
      )

    cloudlog.info(", ".join((
      f"GPS acquisition milestone={milestone}",
      *self._timing_fields(now),
      f"gps_week_valid={rawx.week != 0}",
      f"leap_second_valid={bool(rawx.rec_stat & 0x01)}",
      f"measurement_count={rawx.num_meas}",
      f"measurement_counts_by_gnss={measurement_counts}",
      f"maximum_cno_by_gnss={maximum_cno}",
    )))

  def note_rawx(self, frame: bytes, now: float) -> None:
    if frame[2:4] != b"\x02\x15":
      return

    try:
      rawx = Ubx.RxmRawx.from_bytes(frame[6:-2])
    except Exception:
      return

    if not self.first_rawx_after_initialization_logged:
      self._log_rawx_milestone(
        "first_rawx_after_initialization",
        rawx,
        now,
      )
      self.first_rawx_after_initialization_logged = True

    if rawx.num_meas > 0 and not self.first_nonempty_rawx_logged:
      self._log_rawx_milestone("first_nonempty_rawx", rawx, now)
      self.first_nonempty_rawx_logged = True

    if rawx.week != 0 and not self.first_valid_gps_week_logged:
      self._log_rawx_milestone("first_valid_gps_week", rawx, now)
      self.first_valid_gps_week_logged = True

    if (
      rawx.rec_stat & 0x01
      and not self.first_valid_leap_second_logged
    ):
      self._log_rawx_milestone(
        "first_valid_leap_second",
        rawx,
        now,
      )
      self.first_valid_leap_second_logged = True

    measurement_gnss_ids = {
      int(measurement.gnss_id) for measurement in rawx.meas
    }
    if (
      int(Ubx.GnssType.gps) in measurement_gnss_ids
      and not self.first_gps_measurement_logged
    ):
      self._log_rawx_milestone("first_gps_measurement", rawx, now)
      self.first_gps_measurement_logged = True

    if (
      int(Ubx.GnssType.glonass) in measurement_gnss_ids
      and not self.first_glonass_measurement_logged
    ):
      self._log_rawx_milestone(
        "first_glonass_measurement",
        rawx,
        now,
      )
      self.first_glonass_measurement_logged = True

  def log_acquisition_status(self, now: float) -> None:
    if (
      self.reliable_fix_observed
      or now < self.next_status_time
    ):
      return

    fields = [
      "GPS acquisition status",
      *self._timing_fields(now),
      f"nav_pvt_seen={self.latest_fix is not None}",
    ]

    if self.latest_fix is not None:
      fields.extend(self._fix_fields(self.latest_fix))

    cloudlog.info(", ".join(fields))
    self.next_status_time = now + self.status_interval


@dataclass(frozen=True)
class ReceiverCycleInitialization:
  trusted_time_assistance_sent: bool
  next_time_assistance_attempt: float
  navigation_assistance_restore_attempted: bool
  mon_ver_info: MonVerInfo | None
  ack_aiding_configuration_attempted: bool
  assistnow_autonomous_supported: bool
  assistnow_autonomous_configuration_attempted: bool
  completed_at: float
  navigation_assistance_restore_result: (
    NavigationAssistanceRestoreResult | None
  ) = None
  time_assistance_utc: datetime | None = None
  time_assistance_source: str | None = None
  yuma_time_anchor_utc: datetime | None = None
  yuma_time_anchor_source: str | None = None
  yuma_time_anchor_monotonic: float | None = None
  authorized_time: AuthorizedTime | None = None
  host_time_observation: HostTimeObservation | None = None
  authority_evaluation: TimeAuthorityEvaluation | None = None


def wait_for_current_independent_network_time(
  authority: TimeAuthority,
  host_time_observation: HostTimeObservation | None,
  authority_evaluation: TimeAuthorityEvaluation,
  *,
  timeout_seconds: float = NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS,
  poll_seconds: float = NAVIGATION_DATABASE_TRUSTED_TIME_POLL_SECONDS,
  observation_reader: Callable[[], HostTimeObservation | None] = read_host_time_observation,
  evaluator: Callable[[TimeAuthority, HostTimeObservation | None], TimeAuthorityEvaluation] = evaluate_time_authority,
  monotonic: Callable[[], float] = time.monotonic,
  sleeper: Callable[[float], None] = time.sleep,
) -> tuple[HostTimeObservation | None, TimeAuthorityEvaluation]:
  authorized = authority_evaluation.authorized_time
  if authorized is not None and is_current_independent_network_time(authorized):
    return host_time_observation, authority_evaluation
  deadline = monotonic() + timeout_seconds
  latest_observation = host_time_observation
  latest_evaluation = authority_evaluation
  while True:
    remaining = deadline - monotonic()
    if remaining <= 0.0:
      return latest_observation, latest_evaluation
    sleeper(min(poll_seconds, remaining))
    latest_observation = observation_reader()
    latest_evaluation = evaluator(authority, latest_observation)
    authorized = latest_evaluation.authorized_time
    if authorized is not None and is_current_independent_network_time(authorized):
      return latest_observation, latest_evaluation


def send_yuma_with_durable_claim(
  navigation_database_runtime: NavigationDatabaseRestoreRuntime,
  send_message: Callable[[bytes], object],
  message: bytes,
) -> object:
  if not navigation_database_runtime.claim_yuma_transmission():
    raise RuntimeError("YUMA claim persistence failed")
  return send_message(message)


def initialize_receiver_cycle(
  pigeon: TTYPigeon,
  receiver_fingerprint: str,
  startup_diagnostics: GpsStartupDiagnostics,
  reason: str,
  collect_mon_ver_diagnostics: bool = False,
  time_authority: TimeAuthority | None = None,
  time_provenance: ReceiverTimeProvenanceTracker | None = None,
  navigation_database_runtime: NavigationDatabaseRestoreRuntime | None = None,
) -> ReceiverCycleInitialization:
  cycle_started_at = time.monotonic()
  startup_diagnostics.start_cycle(reason, cycle_started_at)
  provenance = time_provenance or ReceiverTimeProvenanceTracker()
  cycle_id = getattr(
    startup_diagnostics,
    "cycle_number",
    provenance.cycle_id + 1,
  )
  if type(cycle_id) is not int or cycle_id < 1:
    cycle_id = provenance.cycle_id + 1
  provenance.start_cycle(
    cycle_id,
    cycle_started_at,
    observations_enabled=False,
  )
  try:
    pigeon.time_provenance = provenance
  except (AttributeError, TypeError):
    pass

  authority = time_authority or TimeAuthority()
  host_time_observation = read_host_time_observation()
  authority_evaluation = evaluate_time_authority(
    authority,
    host_time_observation,
  )
  authorized_time = authority_evaluation.authorized_time
  database_runtime = (
    navigation_database_runtime
    or NavigationDatabaseRestoreRuntime(receiver_fingerprint)
  )
  database_runtime.prepare()

  mon_ver_info: MonVerInfo | None = None
  ack_aiding_configuration_attempted = False
  trusted_time_assistance_sent = False
  time_assistance_source = None
  time_assistance_utc = None
  yuma_time_anchor_utc = None
  yuma_time_anchor_source = None
  yuma_time_anchor_monotonic = None
  diagnostic_context = None
  navigation_assistance_restore_result: (
    NavigationAssistanceRestoreResult | None
  ) = None
  navigation_assistance_restore_attempted = False
  acquisition_start_claimed = False
  next_time_assistance_attempt = (
    cycle_started_at + TIME_SYNC_CHECK_INTERVAL
  )

  def pre_acquisition_initialization() -> None:
    nonlocal mon_ver_info
    nonlocal ack_aiding_configuration_attempted
    nonlocal trusted_time_assistance_sent
    nonlocal time_assistance_source
    nonlocal time_assistance_utc
    nonlocal yuma_time_anchor_utc
    nonlocal yuma_time_anchor_source
    nonlocal yuma_time_anchor_monotonic
    nonlocal diagnostic_context
    nonlocal navigation_assistance_restore_result
    nonlocal navigation_assistance_restore_attempted
    nonlocal acquisition_start_claimed
    nonlocal next_time_assistance_attempt
    nonlocal host_time_observation
    nonlocal authority_evaluation
    nonlocal authorized_time

    if collect_mon_ver_diagnostics:
      mon_ver_info = log_mon_ver_diagnostics(pigeon)
    else:
      try:
        mon_ver_info = poll_mon_ver(pigeon)
      except Exception:
        cloudlog.exception("GPS MON-VER compatibility poll failed")
        mon_ver_info = None
    log_navx5_ack_aiding_support(mon_ver_info)
    configure_navx5_ack_aiding(pigeon, mon_ver_info)
    ack_aiding_configuration_attempted = True
    if (
      database_runtime.controller.pending
      and (
        authorized_time is None
        or not is_current_independent_network_time(authorized_time)
      )
    ):
      host_time_observation, authority_evaluation = (
        wait_for_current_independent_network_time(
          authority,
          host_time_observation,
          authority_evaluation,
        )
      )
      authorized_time = authority_evaluation.authorized_time
    if (
      database_runtime.controller.pending
      and (
        authorized_time is None
        or not is_current_independent_network_time(authorized_time)
      )
    ):
      if not database_runtime.close_restore_window_unverified():
        raise RuntimeError("DBD timeout decision persistence failed")
    attempt_started_at = time.monotonic()
    if authorized_time is not None:
      yuma_time_anchor_utc = authorized_time.utc
      yuma_time_anchor_source = authorized_time.evidence.value
      yuma_time_anchor_monotonic = time.monotonic()
      diagnostic_context = startup_diagnostics.time_assistance_context(
        yuma_time_anchor_monotonic
      )

    navigation_assistance_restore_result = restore_navigation_assistance(
      pigeon,
      receiver_fingerprint,
      diagnostic_context=diagnostic_context,
      time_assistance_source=(
        authorized_time.evidence.value
        if authorized_time is not None
        else None
      ),
      trusted_now=(
        authorized_time.utc if authorized_time is not None else None
      ),
      navigation_database_runtime=database_runtime,
      authorized_time=authorized_time,
    )
    navigation_assistance_restore_attempted = True

    if authorized_time is not None:
      time_assistance_utc = authorized_time.utc
      trusted_time_assistance_sent = send_time_assistance(
        pigeon,
        assistance_time=authorized_time.utc,
        accuracy_seconds=authorized_time.mga_accuracy_seconds,
        source=authorized_time.evidence.value,
        diagnostic_context=diagnostic_context,
        time_provenance=provenance,
        assistance_boottime_seconds=getattr(
          authorized_time,
          "observed_boottime_seconds",
          None,
        ),
        independent=authorized_time.independent,
        source_provenance=authorized_time.provenance,
      )
      if trusted_time_assistance_sent:
        time_assistance_source = authorized_time.evidence.value
      next_time_assistance_attempt = (
        attempt_started_at + TIME_ASSISTANCE_RETRY_INTERVAL
      )
    else:
      next_time_assistance_attempt = (
        attempt_started_at + TIME_SYNC_CHECK_INTERVAL
      )

    if not acquisition_start_claimed:
      if not database_runtime.claim_acquisition_start():
        raise RuntimeError("GNSS START claim persistence failed")
      acquisition_start_claimed = True
  with install_pre_acquisition_initialization(
    pre_acquisition_initialization
  ) as initialization:
    init(pigeon)

  if not initialization.executed:
    # A custom initializer did not execute the controlled pre-acquisition
    # hook. Production init() always executes it before GNSS START. This
    # compatibility path sends no controlled START itself, so close the
    # in-process DBD window before running the callback without treating
    # unavailable test storage as a receiver-action persistence failure.
    database_runtime.note_acquisition_started()
    acquisition_start_claimed = True
    initialization.run()
  provenance.enable_receiver_observations(time.monotonic())

  assistnow_autonomous_configuration_attempted = False
  assistnow_autonomous_supported = log_assistnow_autonomous_support(
    mon_ver_info
  )
  if not assistnow_autonomous_supported:
    configure_assistnow_autonomous(pigeon, mon_ver_info)
    assistnow_autonomous_configuration_attempted = True
  elif authorized_time is not None:
    configure_assistnow_autonomous(pigeon, mon_ver_info)
    assistnow_autonomous_configuration_attempted = True
  else:
    cloudlog.info(
      "GPS AssistNow Autonomous configuration deferred: "
      + "reason=absolute_time_unavailable"
    )

  if collect_mon_ver_diagnostics:
    try:
      log_acquisition_configuration_diagnostics(pigeon, mon_ver_info)
    except Exception:
      cloudlog.exception(
        "GPS acquisition configuration diagnostics failed"
      )

  return ReceiverCycleInitialization(
    trusted_time_assistance_sent=trusted_time_assistance_sent,
    next_time_assistance_attempt=next_time_assistance_attempt,
    navigation_assistance_restore_attempted=(
      navigation_assistance_restore_attempted
    ),
    mon_ver_info=mon_ver_info,
    ack_aiding_configuration_attempted=ack_aiding_configuration_attempted,
    assistnow_autonomous_supported=assistnow_autonomous_supported,
    assistnow_autonomous_configuration_attempted=(
      assistnow_autonomous_configuration_attempted
    ),
    completed_at=time.monotonic(),
    navigation_assistance_restore_result=(
      navigation_assistance_restore_result
    ),
    time_assistance_utc=(
      time_assistance_utc if trusted_time_assistance_sent else None
    ),
    time_assistance_source=(
      time_assistance_source if trusted_time_assistance_sent else None
    ),
    yuma_time_anchor_utc=yuma_time_anchor_utc,
    yuma_time_anchor_source=yuma_time_anchor_source,
    yuma_time_anchor_monotonic=yuma_time_anchor_monotonic,
    authorized_time=authorized_time,
    host_time_observation=host_time_observation,
    authority_evaluation=authority_evaluation,
  )


def publish_ublox_raw(pm: messaging.PubMaster, data: bytes) -> None:
  message = messaging.new_message(
    "ubloxRaw",
    len(data),
    valid=True,
  )
  message.ubloxRaw = data
  pm.send("ubloxRaw", message)


def receiver_frames_show_gnss_acquisition(
  frames: list[bytes],
) -> bool:
  for frame in frames:
    if len(frame) >= 8 and frame[2:4] == b"\x02\x15":
      try:
        if Ubx.RxmRawx.from_bytes(frame[6:-2]).num_meas > 0:
          return True
      except Exception:
        pass
    fix = parse_nav_pvt(frame)
    if fix is not None and fix.fix_ok:
      return True
    nav_sat = parse_nav_sat(frame)
    if nav_sat is not None and nav_sat.satellites_used > 0:
      return True
  return False


def process_receiver_frames(
  frames: list[bytes],
  frame_time: float,
  startup_diagnostics: GpsStartupDiagnostics,
  fix_tracker: ReliableFixTracker,
  capture_quality_tracker: CaptureQualityTracker,
  autonomous_orbit_diagnostics: AutonomousOrbitDiagnostics,
  dump_collector: NavigationDatabaseDumpCollector,
  capture_state: NavigationCaptureState,
  time_provenance: ReceiverTimeProvenanceTracker | None = None,
) -> tuple[bytes, ...] | None:
  completed_database = None
  for frame in frames:
    startup_diagnostics.note_rawx(frame, frame_time)
    if time_provenance is not None:
      time_provenance.note_rawx(frame, frame_time)
    fix = parse_nav_pvt(frame)

    if fix is not None:
      startup_diagnostics.note_nav_pvt(fix, frame_time)
      if time_provenance is not None:
        time_provenance.note_nav_pvt(fix, frame_time)
      fix_tracker.update(fix, frame_time)
      reset_reason = capture_quality_tracker.update_fix(fix, frame_time)
      if reset_reason is not None:
        cloudlog.info(f"GPS navigation assistance quality gate reset: reason={reset_reason}")

    nav_sat = parse_nav_sat(frame)
    if nav_sat is not None:
      autonomous_orbit_diagnostics.note_nav_sat(nav_sat)
      orbit_was_eligible = capture_quality_tracker.orbit_eligible(frame_time)
      reset_reason = capture_quality_tracker.update_nav_sat(nav_sat, frame_time)
      if reset_reason is not None:
        cloudlog.info(f"GPS navigation assistance quality gate reset: reason={reset_reason}")
      if not orbit_was_eligible and capture_quality_tracker.orbit_eligible(frame_time):
        cloudlog.info("GPS navigation assistance orbit-quality gate eligible")

    if dump_collector.active:
      try:
        result = dump_collector.feed(frame)
        if result is not None:
          completed_database = result
      except CacheValidationError:
        cloudlog.exception("GPS navigation database capture failed")
        dump_collector.cancel()
        capture_state.fail(frame_time)

  return completed_database


def yuma_database_restore_state(
  result: NavigationAssistanceRestoreResult | None,
) -> YumaDatabaseRestoreState:
  if result is None:
    return YumaDatabaseRestoreState.FAILED
  disposition = result.database_restore_disposition
  if disposition is None:
    try:
      return YumaDatabaseRestoreState(result.status.value)
    except (AttributeError, ValueError):
      return YumaDatabaseRestoreState.FAILED
  if disposition is NavigationDatabaseRestoreDisposition.PENDING:
    return YumaDatabaseRestoreState.PENDING
  if disposition.database_available:
    return YumaDatabaseRestoreState.COMPLETE
  return YumaDatabaseRestoreState.FAILED


def log_cross_boot_rtc_observation(
  observation: CrossBootRtcObservation,
) -> None:
  candidate = observation.candidate
  fields = [
    "GPS cross-boot RTC observation",
    f"state={observation.state.value}",
    f"reason={observation.reason.value}",
    f"authorized={str(observation.authorized).lower()}",
    f"operational={str(observation.operational).lower()}",
    (
      "candidate_utc="
      + (
        candidate.candidate_utc.isoformat()
        if candidate is not None
        else "none"
      )
    ),
    (
      "candidate_uncertainty_seconds="
      + (
        str(candidate.uncertainty_seconds)
        if candidate is not None
        else "none"
      )
    ),
    (
      "anchor_generation="
      + (
        candidate.anchor_generation
        if candidate is not None
        else "none"
      )
    ),
    (
      "anchor_sequence="
      + (
        str(candidate.anchor_sequence)
        if candidate is not None
        else "none"
      )
    ),
    (
      "anchor_boot_id="
      + (
        candidate.anchor_boot_id
        if candidate is not None
        else "none"
      )
    ),
    (
      "current_boot_id="
      + (
        candidate.current_boot_id
        if candidate is not None
        else "none"
      )
    ),
    (
      "anchor_rtc_epoch_seconds="
      + (
        str(candidate.anchor_rtc_epoch_seconds)
        if candidate is not None
        else "none"
      )
    ),
    (
      "current_rtc_epoch_seconds="
      + (
        str(candidate.current_rtc_epoch_seconds)
        if candidate is not None
        else "none"
      )
    ),
    (
      "rtc_elapsed_seconds="
      + (
        str(candidate.rtc_elapsed_seconds)
        if candidate is not None
        else "none"
      )
    ),
    (
      "current_boottime_seconds="
      + (
        str(candidate.current_boottime_seconds)
        if candidate is not None
        else "none"
      )
    ),
    (
      "elapsed_covers_uptime="
      + (
        str(candidate.elapsed_covers_uptime).lower()
        if candidate is not None
        else "false"
      )
    ),
    (
      "rtc_voltage_status_supported="
      + (
        str(
          candidate.rtc_voltage_status_supported
        ).lower()
        if candidate is not None
        else "false"
      )
    ),
    (
      "rtc_voltage_status_flags="
      + (
        str(candidate.rtc_voltage_status_flags)
        if (
          candidate is not None
          and candidate.rtc_voltage_status_flags is not None
        )
        else "none"
      )
    ),
    (
      "rtc_tick_delta_seconds="
      + (
        str(observation.rtc_tick_delta_seconds)
        if observation.rtc_tick_delta_seconds is not None
        else "none"
      )
    ),
    (
      "boottime_tick_delta_seconds="
      + (
        str(observation.boottime_tick_delta_seconds)
        if (
          observation.boottime_tick_delta_seconds
          is not None
        )
        else "none"
      )
    ),
    (
      "tick_consistent="
      + (
        str(observation.tick_consistent).lower()
        if observation.tick_consistent is not None
        else "unknown"
      )
    ),
  ]
  message = ", ".join(fields)
  if observation.state is RtcObservationState.REJECTED:
    cloudlog.warning(message)
  else:
    cloudlog.info(message)


def log_receiver_utc_observation(
  observation: ReceiverUtcObservation,
) -> None:
  fields = (
    "GPS receiver UTC provenance",
    f"cycle={observation.cycle_id}",
    f"classification={observation.classification.value}",
    f"reason={observation.reason}",
    f"independent={str(observation.independent).lower()}",
    (
      "time_assistance_written="
      + str(observation.time_assistance_written).lower()
    ),
    (
      "time_assistance_source="
      + (
        observation.time_assistance_source
        if observation.time_assistance_source is not None
        else "none"
      )
    ),
    (
      "time_accuracy_ns="
      + (
        str(observation.time_accuracy_ns)
        if observation.time_accuracy_ns is not None
        else "none"
      )
    ),
    (
      "rawx_measurement_count="
      + str(observation.rawx_measurement_count)
    ),
    (
      "gps_week_valid="
      + str(observation.gps_week_valid).lower()
    ),
    (
      "leap_second_valid="
      + str(observation.leap_second_valid).lower()
    ),
  )
  message = ", ".join(fields)
  if observation.classification is (
    ReceiverUtcClassification.UNASSISTED_GNSS
  ):
    cloudlog.info(message)
  else:
    cloudlog.warning(message)


@dataclass(frozen=True)
class HostTimeProcessingState:
  generation: str | None = None
  source: HostTimeSource | None = None
  persistence_complete: bool = True
  next_retry_at: float = 0.0


def host_time_persistence_complete(
  evaluation: TimeAuthorityEvaluation,
) -> bool:
  return evaluation.anchor_write_status in {
    AnchorWriteStatus.SAVED,
    AnchorWriteStatus.PRESERVED_CURRENT_BOOT,
  }


def host_time_processing_state(
  observation: HostTimeObservation | None,
  evaluation: TimeAuthorityEvaluation | None,
  *,
  now: float,
) -> HostTimeProcessingState:
  if observation is None:
    return HostTimeProcessingState()
  persistence_complete = (
    not observation.independent
    or (
      evaluation is not None
      and host_time_persistence_complete(evaluation)
    )
  )
  return HostTimeProcessingState(
    generation=observation.generation,
    source=observation.source,
    persistence_complete=persistence_complete,
    next_retry_at=(
      now
      if persistence_complete
      else now + HOST_TIME_PERSISTENCE_RETRY_INTERVAL
    ),
  )


def host_time_requires_processing(
  state: HostTimeProcessingState,
  observation: HostTimeObservation | None,
  *,
  now: float,
) -> bool:
  if observation is None:
    return False
  if observation.generation != state.generation:
    return True
  return (
    observation.independent
    and not state.persistence_complete
    and now >= state.next_retry_at
  )


def independent_time_observation(
  authorized: AuthorizedTime | object | None,
) -> IndependentTimeObservation | None:
  if (
    authorized is None
    or getattr(authorized, "independent", False) is not True
  ):
    return None
  boottime = getattr(
    authorized,
    "observed_boottime_seconds",
    None,
  )
  if boottime is None:
    return None
  try:
    return IndependentTimeObservation(
      utc=authorized.utc,
      observed_boottime_seconds=boottime,
      uncertainty_seconds=authorized.uncertainty_seconds,
      source=authorized.source,
      provenance=authorized.provenance,
    )
  except (AttributeError, TypeError, ValueError):
    return None


def authorize_independent_receiver_utc(
  time_authority: TimeAuthority,
  observation: ReceiverUtcObservation,
  *,
  now: float | None = None,
) -> TimeAuthorityEvaluation | None:
  if (
    observation.classification
    is not ReceiverUtcClassification.UNASSISTED_GNSS
    or not observation.independent
    or observation.utc is None
    or observation.observed_at is None
    or observation.time_accuracy_ns is None
  ):
    return None
  current_monotonic = time.monotonic() if now is None else now
  if (
    type(current_monotonic) not in (int, float)
    or isinstance(current_monotonic, bool)
    or not isfinite(current_monotonic)
    or current_monotonic < observation.observed_at
  ):
    cloudlog.warning(
      "GPS independent receiver UTC rejected: reason=observation_time_invalid"
    )
    return None
  observed_boottime = read_boottime_seconds()
  if observed_boottime is None:
    cloudlog.warning(
      "GPS independent receiver UTC rejected: reason=boottime_unavailable"
    )
    return None
  receiver_utc_at_boottime = (
    observation.utc
    + timedelta(
      seconds=float(current_monotonic) - observation.observed_at
    )
  )
  evaluation = time_authority.observe_independent_time(
    utc=receiver_utc_at_boottime,
    uncertainty_seconds=(
      observation.time_accuracy_ns / 1_000_000_000
    ),
    source=(
      TrustedTimeSource.RECEIVER_UTC_UNASSISTED_GNSS
    ),
    provenance=TimeProvenance.GNSS_INDEPENDENT,
    observed_boottime_seconds=observed_boottime,
  )
  authorized = evaluation.authorized_time
  fields = (
    "GPS independent receiver UTC authority",
    f"authorized={str(authorized is not None).lower()}",
    (
      "anchor_write_status="
      + evaluation.anchor_write_status.value
    ),
    (
      "anchor_write_reason="
      + (
        evaluation.anchor_write_reason.value
        if evaluation.anchor_write_reason is not None
        else "none"
      )
    ),
    (
      "anchor_error_seconds="
      + (
        str(evaluation.anchor_comparison.error_seconds)
        if evaluation.anchor_comparison is not None
        else "none"
      )
    ),
  )
  message = ", ".join(fields)
  if authorized is not None:
    cloudlog.info(message)
  else:
    cloudlog.warning(message)
  return evaluation


def log_cross_boot_rtc_validation(
  validation: CrossBootRtcValidation,
) -> None:
  fields = (
    "GPS cross-boot RTC independent validation",
    f"status={validation.status.value}",
    f"reason={validation.reason}",
    f"authorized={str(validation.authorized).lower()}",
    f"operational={str(validation.operational).lower()}",
    f"validation_source={validation.validation_source.value}",
    (
      "candidate_error_seconds="
      + (
        str(validation.candidate_error_seconds)
        if validation.candidate_error_seconds is not None
        else "none"
      )
    ),
    (
      "allowed_error_seconds="
      + (
        str(validation.allowed_error_seconds)
        if validation.allowed_error_seconds is not None
        else "none"
      )
    ),
    (
      "anchor_generation="
      + (
        validation.anchor_generation
        if validation.anchor_generation is not None
        else "none"
      )
    ),
    (
      "anchor_sequence="
      + str(validation.anchor_sequence)
    ),
    (
      "rtc_elapsed_seconds="
      + str(validation.rtc_elapsed_seconds)
    ),
    (
      "current_uptime_seconds="
      + str(validation.current_uptime_seconds)
    ),
    (
      "tick_consistent="
      + (
        str(validation.tick_consistent).lower()
        if validation.tick_consistent is not None
        else "unknown"
      )
    ),
  )
  message = ", ".join(fields)
  if validation.status is (
    CrossBootRtcValidationStatus.DISAGREES
  ):
    cloudlog.warning(message)
  else:
    cloudlog.info(message)


def validate_observed_cross_boot_rtc(
  observation: CrossBootRtcObservation | None,
  independent: IndependentTimeObservation,
) -> CrossBootRtcValidation | None:
  if (
    observation is None
    or observation.state is not RtcObservationState.OBSERVED
  ):
    return None
  validation = validate_cross_boot_rtc(
    observation,
    independent,
  )
  log_cross_boot_rtc_validation(validation)
  return validation


def log_receiver_correction_decision(
  decision: ReceiverCorrectionDecision,
  *,
  write_observed: bool,
  ack_accepted: bool,
) -> None:
  fields = (
    "GPS receiver UTC correction",
    f"cycle={decision.receiver_cycle}",
    f"decision={decision.reason.value}",
    f"should_correct={str(decision.should_correct).lower()}",
    f"source={decision.source.value}",
    (
      "delta_seconds="
      + (
        str(decision.delta_seconds)
        if decision.delta_seconds is not None
        else "none"
      )
    ),
    (
      "minimum_delta_seconds="
      + str(decision.minimum_delta_seconds)
    ),
    (
      "materially_better="
      + str(decision.materially_better).lower()
    ),
    f"write_observed={str(write_observed).lower()}",
    f"ack_accepted={str(ack_accepted).lower()}",
  )
  message = ", ".join(fields)
  if decision.should_correct and not write_observed:
    cloudlog.warning(message)
  else:
    cloudlog.info(message)


def maybe_send_receiver_time_correction(
  pigeon: TTYPigeon,
  time_provenance: ReceiverTimeProvenanceTracker,
  independent: IndependentTimeObservation,
  *,
  diagnostic_context: str | None = None,
) -> tuple[ReceiverCorrectionDecision, bool]:
  decision = evaluate_receiver_correction(
    time_provenance.time_assistance_observation,
    independent,
  )
  if not decision.should_correct:
    log_receiver_correction_decision(
      decision,
      write_observed=False,
      ack_accepted=False,
    )
    return decision, False

  accepted = send_time_assistance(
    pigeon,
    assistance_time=independent.utc,
    accuracy_seconds=min(
      65_535,
      max(0, ceil(independent.uncertainty_seconds)),
    ),
    source=independent.source.value,
    diagnostic_context=diagnostic_context,
    time_provenance=time_provenance,
    assistance_boottime_seconds=(
      independent.observed_boottime_seconds
    ),
    independent=True,
    source_provenance=independent.provenance,
    correction=True,
  )
  write_observed = time_provenance.correction_written
  log_receiver_correction_decision(
    decision,
    write_observed=write_observed,
    ack_accepted=accepted,
  )
  return decision, accepted


def fresh_receiver_utc_time_anchor(
  diagnostics: GpsStartupDiagnostics,
  now: float,
) -> tuple[datetime, float] | None:
  fix = getattr(diagnostics, "latest_fix", None)
  observed_at = getattr(diagnostics, "latest_fix_time", None)
  utc_time = getattr(fix, "utc_time", None)
  if (
    utc_time is None
    or observed_at is None
    or now < observed_at
    or now - observed_at > MAXIMUM_NAV_PVT_GAP_SECONDS
  ):
    return None
  return utc_time, observed_at


def fresh_independent_receiver_utc_time_anchor(
  time_provenance: ReceiverTimeProvenanceTracker,
  now: float,
) -> tuple[datetime, float] | None:
  observation = time_provenance.current_observation(now)
  if (
    observation.classification
    is not ReceiverUtcClassification.UNASSISTED_GNSS
    or not observation.independent
    or observation.utc is None
    or observation.observed_at is None
  ):
    return None
  return observation.utc, observation.observed_at


def create_yuma_supplementation_runtime(
  initialization: ReceiverCycleInitialization,
  *,
  started_at: float | None = None,
  time_anchor_utc: datetime | None = None,
  time_anchor_monotonic: float | None = None,
  time_anchor_source: str | None = None,
) -> YumaSupplementationRuntime:
  restore_result = getattr(
    initialization,
    "navigation_assistance_restore_result",
    None,
  )
  completed_at = getattr(
    initialization,
    "completed_at",
    time.monotonic(),
  )
  runtime_started_at = (
    completed_at
    if started_at is None
    else started_at
  )
  anchor_utc = (
    getattr(
      initialization,
      "yuma_time_anchor_utc",
      getattr(initialization, "time_assistance_utc", None),
    )
    if time_anchor_utc is None
    else time_anchor_utc
  )
  anchor_monotonic = (
    getattr(
      initialization,
      "yuma_time_anchor_monotonic",
      completed_at,
    )
    if time_anchor_monotonic is None
    else time_anchor_monotonic
  )
  if anchor_monotonic is None:
    anchor_monotonic = completed_at
  anchor_source = (
    getattr(
      initialization,
      "yuma_time_anchor_source",
      getattr(initialization, "time_assistance_source", None),
    )
    if time_anchor_source is None
    else time_anchor_source
  )
  return YumaSupplementationRuntime(
    database_state=yuma_database_restore_state(
      restore_result
    ),
    database_saved_at_utc=(
      getattr(restore_result, "cache_saved_at_utc", None)
    ),
    restored_cache_generation=(
      getattr(
        restore_result,
        "restored_cache_generation",
        None,
      )
    ),
    restored_cache_selection_reason=(
      getattr(
        restore_result,
        "restored_cache_selection_reason",
        None,
      )
    ),
    restored_gps_almanac_available=(
      getattr(
        restore_result,
        "restored_gps_almanac_available",
        None,
      )
    ),
    restored_glonass_almanac_available=(
      getattr(
        restore_result,
        "restored_glonass_almanac_available",
        None,
      )
    ),
    restored_gps_ephemeris_available=(
      getattr(
        restore_result,
        "restored_gps_ephemeris_available",
        None,
      )
    ),
    restored_glonass_ephemeris_available=(
      getattr(
        restore_result,
        "restored_glonass_ephemeris_available",
        None,
      )
    ),
    restored_satellites_used=(
      getattr(
        restore_result,
        "restored_satellites_used",
        None,
      )
    ),
    restored_gps_startup_ready=(
      getattr(
        restore_result,
        "restored_gps_startup_ready",
        None,
      )
    ),
    restored_gps_almanac_satellite_ids=(
      getattr(
        restore_result,
        "restored_gps_almanac_satellite_ids",
        None,
      )
    ),
    restored_navigation_quality=(
      getattr(
        restore_result,
        "restored_navigation_quality",
        None,
      )
    ),
    started_at=runtime_started_at,
    time_anchor_utc=anchor_utc,
    time_anchor_source=anchor_source,
    time_anchor_monotonic=anchor_monotonic,
  )


class YumaSupplementationFeature:
  def __init__(
    self,
    params: Params,
    initialization: ReceiverCycleInitialization,
    receiver_cycle: int,
  ) -> None:
    if (
      isinstance(receiver_cycle, bool)
      or not isinstance(receiver_cycle, int)
      or receiver_cycle < 0
    ):
      raise ValueError(
        "receiver_cycle must be a non-negative integer"
      )
    self._params = params
    self._receiver_cycle = receiver_cycle
    self._enabled: bool | None = None
    self._next_param_check = 0.0
    self._runtime: YumaSupplementationRuntime | None = None
    self._cycle_injection_consumed = False
    self._provisional_reference: ProvisionalYumaReferenceTime | None = None
    self._provisional_reference_used: ProvisionalYumaReferenceTime | None = None
    self._provisional_attempted = False
    self._current_boot_id = read_boot_id()
    disable_state = load_provisional_yuma_boot_disable_state(
      self._current_boot_id
    )
    self._provisional_disabled_for_boot = disable_state.disabled
    cloudlog.info(", ".join((
      "GPS provisional YUMA boot disable state",
      f"disabled={str(disable_state.disabled).lower()}",
      "current_boot_id=" + (disable_state.current_boot_id or "none"),
      "stored_boot_id=" + (disable_state.stored_boot_id or "none"),
      "reason=" + (disable_state.reason or "none"),
      "error=" + (disable_state.error or "none"),
    )))
    self._pending_outcomes: deque[YumaSupplementationRuntimeOutcome] = deque()
    self._initialization = initialization
    self._time_anchor_utc: datetime | None = None
    self._time_anchor_source: str | None = None
    self._time_anchor_monotonic = getattr(
      initialization,
      "completed_at",
      time.monotonic(),
    )
    self.reset_receiver_cycle(
      initialization,
      receiver_cycle,
    )

  @property
  def runtime_active(self) -> bool:
    return self._runtime is not None

  @property
  def time_anchor_source(self) -> str | None:
    return self._time_anchor_source

  @property
  def cycle_injection_consumed(self) -> bool:
    return self._cycle_injection_consumed

  def update_navigation_assistance_restore_result(
    self,
    result: NavigationAssistanceRestoreResult,
    now: float,
  ) -> bool:
    self._initialization = replace(
      self._initialization,
      navigation_assistance_restore_result=result,
      navigation_assistance_restore_attempted=True,
    )
    if self._cycle_injection_consumed:
      return False
    self._runtime = None
    self._next_param_check = 0.0
    self._refresh_enabled(now, force=True)
    return True

  def persist_provisional_telemetry(
    self,
    event: str,
    *,
    now: float,
    observation: object | None = None,
    authority: object | None = None,
    decision: object | None = None,
    accepted: bool | None = None,
    outcome: object | None = None,
    validation: object | None = None,
  ) -> None:
    try:
      if (
        not isinstance(self._current_boot_id, str)
        or not self._current_boot_id.strip()
      ):
        raise ValueError("current Linux boot ID is unavailable")
      store_provisional_yuma_decision_event(
        event,
        current_boot_id=self._current_boot_id,
        receiver_cycle=self._receiver_cycle,
        observed_at=now,
        observation=observation,
        authority=authority,
        decision=decision,
        accepted=accepted,
        outcome=outcome,
        validation=validation,
      )
    except Exception:
      cloudlog.exception(
        "Failed to persist provisional YUMA decision telemetry"
      )

  def _contextualize_outcome(
    self,
    outcome: YumaSupplementationRuntimeOutcome,
  ) -> YumaSupplementationRuntimeOutcome:
    return replace(
      outcome,
      receiver_cycle=self._receiver_cycle,
      feature_enabled=(
        outcome.plan.reason
        is not YumaSupplementationReason.FEATURE_DISABLED
      ),
    )

  def _queue_cancellation(
    self,
    now: float,
    reason: YumaSupplementationReason,
  ) -> None:
    if self._runtime is None:
      return
    outcome = self._runtime.cancel(
      now=now,
      reason=reason,
    )
    if outcome is not None:
      self._pending_outcomes.append(
        self._contextualize_outcome(outcome)
      )

  def reset_receiver_cycle(
    self,
    initialization: ReceiverCycleInitialization,
    receiver_cycle: int,
  ) -> None:
    if (
      isinstance(receiver_cycle, bool)
      or not isinstance(receiver_cycle, int)
      or receiver_cycle < 0
    ):
      raise ValueError(
        "receiver_cycle must be a non-negative integer"
      )
    completed_at = getattr(
      initialization,
      "completed_at",
      time.monotonic(),
    )
    self._queue_cancellation(
      completed_at,
      YumaSupplementationReason.RECEIVER_CYCLE_RESET,
    )
    self._receiver_cycle = receiver_cycle
    self._initialization = initialization
    self._runtime = None
    self._cycle_injection_consumed = False
    self._provisional_reference = None
    self._provisional_reference_used = None
    self._provisional_attempted = False
    self._time_anchor_utc = getattr(
      initialization,
      "yuma_time_anchor_utc",
      getattr(initialization, "time_assistance_utc", None),
    )
    self._time_anchor_source = getattr(
      initialization,
      "yuma_time_anchor_source",
      getattr(initialization, "time_assistance_source", None),
    )
    self._time_anchor_monotonic = getattr(
      initialization,
      "yuma_time_anchor_monotonic",
      completed_at,
    )
    if self._time_anchor_monotonic is None:
      self._time_anchor_monotonic = completed_at
    self._next_param_check = 0.0
    self._refresh_enabled(completed_at, force=True)

  def set_time_anchor(
    self,
    anchor_utc: datetime,
    anchor_monotonic: float,
    source: str,
  ) -> None:
    if not isinstance(source, str) or not source.strip():
      raise ValueError("source must be a non-empty string")
    self._time_anchor_utc = anchor_utc
    self._time_anchor_source = source.strip()
    self._time_anchor_monotonic = anchor_monotonic
    if not self._provisional_attempted:
      self._provisional_reference = None
    if self._runtime is not None:
      self._runtime.set_time_anchor(
        anchor_utc,
        anchor_monotonic,
        self._time_anchor_source,
      )

  def set_receiver_time_anchor(
    self,
    anchor_utc: datetime,
    anchor_monotonic: float,
    source: str = "receiver_utc",
  ) -> bool:
    # Preserve an RTC or synchronized host anchor. Receiver UTC is a fallback
    # only for a cycle that began without usable absolute time.
    if self._time_anchor_utc is not None:
      return False
    self.set_time_anchor(
      anchor_utc,
      anchor_monotonic,
      source,
    )
    return True

  def set_provisional_reference(
    self,
    reference: ProvisionalYumaReferenceTime,
  ) -> bool:
    if not isinstance(reference, ProvisionalYumaReferenceTime):
      raise ValueError(
        "reference must be a ProvisionalYumaReferenceTime"
      )
    if (
      self._provisional_disabled_for_boot
      or self._provisional_attempted
      or self._cycle_injection_consumed
      or self._time_anchor_utc is not None
      or reference.receiver_cycle != self._receiver_cycle
      or reference.current_boot_id != self._current_boot_id
    ):
      return False
    self._provisional_reference = reference
    return True

  def note_cross_boot_validation(
    self,
    validation: CrossBootRtcValidation | None,
  ) -> None:
    if validation is None or self._provisional_reference_used is None:
      return
    if validation.status is CrossBootRtcValidationStatus.DISAGREES:
      self._provisional_disabled_for_boot = True
      self._provisional_reference = None
      try:
        current_boot_id = self._current_boot_id
        if not isinstance(current_boot_id, str) or not current_boot_id.strip():
          raise ValueError("current Linux boot ID is unavailable")
        store_provisional_yuma_boot_disable_state(
          current_boot_id,
          PROVISIONAL_YUMA_DISABLE_REASON_VALIDATION_DISAGREES,
        )
      except Exception:
        cloudlog.exception(
          "Failed to persist provisional YUMA boot disable state"
        )
    self.persist_provisional_telemetry(
      "independent_validation",
      now=time.monotonic(),
      validation=validation,
    )
    cloudlog.info(", ".join((
      "GPS provisional YUMA independent validation",
      f"status={validation.status.value}",
      f"reason={validation.reason}",
      f"validation_source={validation.validation_source.value}",
      "candidate_error_seconds="
      + (
        str(validation.candidate_error_seconds)
        if validation.candidate_error_seconds is not None
        else "none"
      ),
      "allowed_error_seconds="
      + (
        str(validation.allowed_error_seconds)
        if validation.allowed_error_seconds is not None
        else "none"
      ),
      "disabled_for_boot="
      + str(self._provisional_disabled_for_boot).lower(),
    )))

  def evaluate_provisional(
    self,
    send_message: Callable[[bytes], bool],
    *,
    now: float,
    reliable_fix_available: bool,
  ) -> ProvisionalYumaTransmissionOutcome | None:
    if not isinstance(reliable_fix_available, bool):
      raise ValueError(
        "reliable_fix_available must be a bool"
      )
    enabled = self._refresh_enabled(now)
    reference = self._provisional_reference
    if reliable_fix_available and reference is not None:
      self._provisional_attempted = True
      self._provisional_reference = None
      return None
    if (
      not enabled
      or reference is None
      or self._provisional_disabled_for_boot
      or self._provisional_attempted
      or self._cycle_injection_consumed
      or self._time_anchor_utc is not None
    ):
      return None

    self._provisional_attempted = True
    self._provisional_reference = None
    outcome = transmit_provisional_yuma_reference(
      reference,
      send_message,
    )
    if outcome.receiver_write_attempted:
      self._provisional_reference_used = reference
      self._cycle_injection_consumed = True
      self._runtime = None
    return outcome

  def _refresh_enabled(
    self,
    now: float,
    *,
    force: bool = False,
  ) -> bool:
    if (
      not force
      and now < self._next_param_check
    ):
      return (
        self._enabled is True
        and self._runtime is not None
      )

    enabled = public_yuma_almanac_enabled(self._params)
    self._next_param_check = (
      now + PUBLIC_YUMA_ALMANAC_PARAM_POLL_SECONDS
    )

    if enabled != self._enabled:
      cloudlog.info(f"GPS public YUMA feature, enabled={str(enabled).lower()}")

    self._enabled = enabled

    if not enabled:
      self._queue_cancellation(
        now,
        YumaSupplementationReason.FEATURE_DISABLED,
      )
      self._runtime = None
      return False

    if self._cycle_injection_consumed and self._runtime is None:
      return False

    if self._runtime is None:
      self._runtime = create_yuma_supplementation_runtime(
        self._initialization,
        started_at=now,
        time_anchor_utc=self._time_anchor_utc,
        time_anchor_monotonic=(
          self._time_anchor_monotonic
        ),
        time_anchor_source=self._time_anchor_source,
      )

    return True

  def evaluate(
    self,
    send_message: Callable[[bytes], bool],
    *,
    now: float,
    nav_sat: NavSatQuality | None,
    nav_sat_time: float | None,
    reliable_fix_available: bool,
  ) -> YumaSupplementationRuntimeOutcome | None:
    enabled = self._refresh_enabled(now)
    if self._pending_outcomes:
      return self._pending_outcomes.popleft()
    if not enabled:
      return None

    assert self._runtime is not None
    outcome = self._runtime.evaluate(
      send_message,
      now=now,
      nav_sat=nav_sat,
      nav_sat_time=nav_sat_time,
      reliable_fix_available=reliable_fix_available,
    )
    if outcome is None:
      return None
    if outcome.transmission_attempt > 0:
      self._cycle_injection_consumed = True
    return self._contextualize_outcome(outcome)


def log_provisional_yuma_reference_decision(
  decision: ProvisionalYumaReferenceDecision,
  *,
  accepted: bool,
) -> None:
  reference = decision.reference
  cloudlog.info(", ".join((
    "GPS provisional YUMA reference",
    f"eligible={str(decision.eligible).lower()}",
    f"accepted={str(accepted).lower()}",
    "rejection="
    + (
      decision.rejection.value
      if decision.rejection is not None
      else "none"
    ),
    "receiver_cycle="
    + (str(reference.receiver_cycle) if reference is not None else "none"),
    "reference_utc="
    + (reference.utc.isoformat() if reference is not None else "none"),
    "uncertainty_seconds="
    + (str(reference.uncertainty_seconds) if reference is not None else "none"),
    "anchor_generation="
    + (reference.anchor_generation if reference is not None else "none"),
    "anchor_sequence="
    + (str(reference.anchor_sequence) if reference is not None else "none"),
    "use=yuma_reference_only",
    "time_assistance_written=false",
    "cache_quality_changed=false",
  )))


def log_provisional_yuma_outcome(
  outcome: ProvisionalYumaTransmissionOutcome,
) -> None:
  result = outcome.transmit_result
  fields = [
    "GPS provisional YUMA transmission",
    f"receiver_cycle={outcome.reference.receiver_cycle}",
    f"reference_utc={outcome.reference.utc.isoformat()}",
    f"uncertainty_seconds={outcome.reference.uncertainty_seconds}",
    f"anchor_generation={outcome.reference.anchor_generation}",
    f"anchor_sequence={outcome.reference.anchor_sequence}",
    f"rtc_elapsed_seconds={outcome.reference.rtc_elapsed_seconds}",
    "selected_prns=" + _format_yuma_prns(outcome.satellite_ids),
    "snapshot_sha256=" + (outcome.snapshot_sha256 or "none"),
    "validated_reference_utc="
    + (
      outcome.validated_reference_utc.isoformat()
      if outcome.validated_reference_utc is not None
      else "none"
    ),
    f"elapsed_ms={outcome.elapsed_ms}",
    "receiver_write_attempted="
    + str(outcome.receiver_write_attempted).lower(),
    "error=" + (outcome.error or "none"),
    f"time_assistance_written={str(outcome.time_assistance_written).lower()}",
    f"cache_quality_changed={str(outcome.cache_quality_changed).lower()}",
    f"anchor_written={str(outcome.anchor_written).lower()}",
    f"system_clock_changed={str(outcome.system_clock_changed).lower()}",
    f"receiver_reset={str(outcome.receiver_reset).lower()}",
  ]
  if result is not None:
    fields.extend((
      f"transmit_status={result.status.value}",
      "requested_prns=" + _format_yuma_prns(result.requested_satellite_ids),
      "attempted_prns=" + _format_yuma_prns(result.attempted_satellite_ids),
      "accepted_prns=" + _format_yuma_prns(result.accepted_satellite_ids),
      "failed_prns=" + _format_yuma_prns(result.failed_satellite_ids),
      "rejected_prns=" + _format_yuma_prns(result.rejected_satellite_ids),
      "timed_out_prns=" + _format_yuma_prns(result.timed_out_satellite_ids),
      "deferred_prns=" + _format_yuma_prns(result.deferred_satellite_ids),
    ))
  cloudlog.info(", ".join(fields))


def _format_yuma_prns(
  satellite_ids: tuple[int, ...] | frozenset[int],
) -> str:
  return (
    ",".join(str(value) for value in satellite_ids)
    or "none"
  )


def log_yuma_supplementation_outcome(
  outcome: YumaSupplementationRuntimeOutcome,
) -> None:
  def optional(value: object | None) -> str:
    return "none" if value is None else str(value)

  def timestamp(value: datetime | None) -> str:
    return "none" if value is None else value.isoformat()

  database_state = (
    outcome.database_state.value
    if outcome.database_state is not None
    else "unknown"
  )
  fields = [
    "GPS public YUMA supplementation",
    f"enabled={str(outcome.feature_enabled).lower()}",
    f"terminal={str(outcome.terminal).lower()}",
    f"retry_pending={str(outcome.retry_pending).lower()}",
    "time_anchor_source=" + optional(outcome.time_anchor_source),
    f"trusted_now_utc={timestamp(outcome.trusted_now_utc)}",
    "trusted_time_wait_expired="
    + str(outcome.trusted_time_wait_expired).lower(),
    "cache_wait_expired="
    + str(outcome.cache_wait_expired).lower(),
    "nav_sat_observation_expired="
    + str(outcome.nav_sat_observation_expired).lower(),
    f"dbd_state={database_state}",
    f"dbd_age_seconds={optional(outcome.database_age_seconds)}",
    "restored_cache_generation="
    + optional(outcome.restored_cache_generation),
    "restored_cache_selection_reason="
    + optional(outcome.restored_cache_selection_reason),
    "restored_gps_almanac_available="
    + optional(outcome.restored_gps_almanac_available),
    "restored_glonass_almanac_available="
    + optional(outcome.restored_glonass_almanac_available),
    "restored_gps_ephemeris_available="
    + optional(outcome.restored_gps_ephemeris_available),
    "restored_glonass_ephemeris_available="
    + optional(outcome.restored_glonass_ephemeris_available),
    "restored_satellites_used="
    + optional(outcome.restored_satellites_used),
    "restored_gps_startup_ready="
    + optional(outcome.restored_gps_startup_ready),
    "restored_gps_almanac_satellite_ids="
    + optional(outcome.restored_gps_almanac_satellite_ids),
    "runtime_elapsed_seconds="
    + optional(outcome.runtime_elapsed_seconds),
    "time_anchor_elapsed_seconds="
    + optional(outcome.time_anchor_elapsed_seconds),
    "decision_ready_elapsed_seconds="
    + optional(outcome.decision_ready_elapsed_seconds),
    "nav_sat_observed_elapsed_seconds="
    + optional(outcome.nav_sat_observed_elapsed_seconds),
    "nav_sat_wait_seconds="
    + optional(outcome.nav_sat_wait_seconds),
    "completion_elapsed_seconds="
    + optional(outcome.completion_elapsed_seconds),
    "completion_utc=" + timestamp(outcome.completion_utc),
    "yuma_snapshot_sha256="
    + optional(outcome.yuma_snapshot_sha256),
    "cancellation_reason="
    + optional(outcome.cancellation_reason),
    f"action={outcome.plan.action.value}",
    f"reason={outcome.plan.reason.value}",
    "selected_prns="
    + _format_yuma_prns(outcome.plan.satellite_ids),
    "unavailable_plan_prns="
    + _format_yuma_prns(
      outcome.plan.unavailable_satellite_ids
    ),
    f"yuma_reference_utc={timestamp(outcome.yuma_reference_utc)}",
    "yuma_reference_age_seconds="
    + optional(outcome.yuma_reference_age_seconds),
    f"downloaded_at_utc={timestamp(outcome.downloaded_at_utc)}",
    f"cache_error={optional(outcome.cache_error)}",
    f"transmission_attempt={outcome.transmission_attempt}",
    f"attempt_history_count={len(outcome.attempt_history)}",
    "transmission_elapsed_ms="
    + optional(outcome.transmission_elapsed_ms),
  ]
  if outcome.transmit_result is not None:
    result = outcome.transmit_result
    fields.extend((
      f"transmit_status={result.status.value}",
      "requested_prns="
      + _format_yuma_prns(
        getattr(result, "requested_satellite_ids", ())
      ),
      "attempted_prns="
      + _format_yuma_prns(
        result.attempted_satellite_ids
      ),
      "accepted_prns="
      + _format_yuma_prns(
        result.accepted_satellite_ids
      ),
      "failed_prns="
      + _format_yuma_prns(
        result.failed_satellite_ids
      ),
      "rejected_prns="
      + _format_yuma_prns(
        getattr(result, "rejected_satellite_ids", ())
      ),
      "timed_out_prns="
      + _format_yuma_prns(
        getattr(result, "timed_out_satellite_ids", ())
      ),
      "deferred_prns="
      + _format_yuma_prns(
        result.deferred_satellite_ids
      ),
      "unavailable_cache_prns="
      + _format_yuma_prns(
        result.unavailable_satellite_ids
      ),
    ))
  if outcome.error is not None:
    fields.append(f"error={outcome.error}")
  cloudlog.info(", ".join(fields))


def read_param_text_compat(
  params: Params,
  key: str,
) -> str | None:
  try:
    value = params.get(key)
  except TypeError:
    value = params.get(key, "utf-8")
  if value is None:
    return None
  if isinstance(value, bytes):
    return value.decode("utf-8")
  return str(value)


def persist_yuma_supplementation_outcome(
  outcome: YumaSupplementationRuntimeOutcome,
  params: Params,
) -> None:
  try:
    commit = read_param_text_compat(params, "GitCommit")
  except Exception:
    cloudlog.exception(
      "GPS public YUMA commit metadata unavailable"
    )
    commit = None

  try:
    save_yuma_supplementation_outcome(
      YUMA_LAST_OUTCOME_PATH,
      outcome,
      commit=commit,
      receiver_cycle=outcome.receiver_cycle,
      recorded_at_utc=(
        outcome.completion_utc or outcome.trusted_now_utc
      ),
    )
  except Exception:
    cloudlog.exception(
      "GPS public YUMA outcome persistence failed"
    )


def run_receiving(duration: int = 0):
  diagnostic_process_start_time = time.monotonic()
  startup_diagnostics = GpsStartupDiagnostics(
    diagnostic_process_start_time
  )
  pm = messaging.PubMaster(['ubloxRaw'])
  sm = messaging.SubMaster(['deviceState'])

  params = Params()
  receiver_fingerprint = (
    gps_assistance_receiver_fingerprint(params)
  )

  fix_tracker = ReliableFixTracker()
  capture_quality_tracker = CaptureQualityTracker()
  dump_collector = NavigationDatabaseDumpCollector()
  # Automatic UPD-SOS backup creation is disabled for this receiver.
  # The M8 HPG 1.40 ROV firmware consistently rejects the command,
  # including after a controlled GNSS stop. Startup restore-status
  # polling remains unchanged.
  autonomous_orbit_diagnostics = AutonomousOrbitDiagnostics()
  capture_state = NavigationCaptureState()
  completed_databases: deque[tuple[bytes, ...]] = deque()
  receiver_time_provenance = (
    ReceiverTimeProvenanceTracker()
  )
  navigation_database_runtime = NavigationDatabaseRestoreRuntime(
    receiver_fingerprint
  )

  def dispatch_frames(frames: list[bytes]) -> None:
    if receiver_frames_show_gnss_acquisition(frames):
      if not navigation_database_runtime.note_acquisition_started():
        raise RuntimeError("acquisition latch persistence failed")
    completed = process_receiver_frames(
      frames,
      time.monotonic(),
      startup_diagnostics,
      fix_tracker,
      capture_quality_tracker,
      autonomous_orbit_diagnostics,
      dump_collector,
      capture_state,
      receiver_time_provenance,
    )
    if completed is not None:
      completed_databases.append(completed)

  pigeon = TTYPigeon(
    lambda data: publish_ublox_raw(pm, data),
    dispatch_frames,
  )
  def send_yuma_message(message: bytes) -> object:
    return send_yuma_with_durable_claim(
      navigation_database_runtime,
      lambda claimed_message: send_mga_with_strict_ack(
        pigeon, claimed_message
      ),
      message,
    )
  def reject_live_database_write(_message: bytes, _frame_index: int) -> None:
    raise RuntimeError("DBD restore is restricted to pre-acquisition initialization")
  time_authority = TimeAuthority()
  rtc_observer = (
    time_authority.create_cross_boot_rtc_observer()
  )
  cycle_initialization = initialize_receiver_cycle(
    pigeon,
    receiver_fingerprint,
    startup_diagnostics,
    "process_start",
    collect_mon_ver_diagnostics=True,
    time_authority=time_authority,
    time_provenance=receiver_time_provenance,
    navigation_database_runtime=navigation_database_runtime,
  )
  startup_diagnostics.initialization_complete(
    cycle_initialization.completed_at
  )
  yuma_feature = YumaSupplementationFeature(
    params,
    cycle_initialization,
    getattr(pigeon, "receiver_cycle", 0),
  )

  stream_parser = UbxStreamParser()
  started_state: bool | None = None

  start_time = time.monotonic()
  next_time_assistance_attempt = (
    cycle_initialization.next_time_assistance_attempt
  )
  trusted_time_assistance_sent = (
    cycle_initialization.trusted_time_assistance_sent
  )
  latest_cross_boot_rtc_observation: (
    CrossBootRtcObservation | None
  ) = None
  latest_independent_time = independent_time_observation(
    getattr(cycle_initialization, "authorized_time", None)
  )
  host_time_state = host_time_processing_state(
    getattr(
      cycle_initialization,
      "host_time_observation",
      None,
    ),
    getattr(
      cycle_initialization,
      "authority_evaluation",
      None,
    ),
    now=start_time,
  )
  latest_authority_evaluation: TimeAuthorityEvaluation | None = getattr(
    cycle_initialization,
    "authority_evaluation",
    None,
  )
  receiver_self_time_cycle: int | None = None
  mon_ver_info = cycle_initialization.mon_ver_info
  assistnow_autonomous_configuration_attempted = (
    cycle_initialization.assistnow_autonomous_configuration_attempted
  )
  assistnow_autonomous_supported = (
    cycle_initialization.assistnow_autonomous_supported
  )
  data_watchdog = UbloxDataWatchdog()
  cloudlog.info(", ".join((
    "GPS navigation assistance quality policy",
    f"reliable_fix_seconds={MINIMUM_RELIABLE_FIX_SECONDS:.0f}",
    f"gps_ephemeris={MINIMUM_GPS_EPHEMERIS}",
    f"glonass_ephemeris={MINIMUM_GLONASS_EPHEMERIS}",
    f"total_ephemeris={MINIMUM_TOTAL_EPHEMERIS}",
    f"satellites_used={MINIMUM_SATELLITES_USED}",
    f"orbit_quality_seconds={MINIMUM_ORBIT_QUALITY_SECONDS:.0f}",
    f"nav_pvt_max_gap_seconds={MAXIMUM_NAV_PVT_GAP_SECONDS:.0f}",
    f"nav_sat_max_age_seconds={MAXIMUM_NAV_SAT_AGE_SECONDS:.0f}",
  )))

  while (
    duration == 0
    or time.monotonic() - start_time < duration
  ):
    now = time.monotonic()
    authority_evaluation_for_loop: (
      TimeAuthorityEvaluation | None
    ) = None
    changed_rtc_observation = (
      rtc_observer.changed_observation(now)
    )
    if changed_rtc_observation is not None:
      log_cross_boot_rtc_observation(
        changed_rtc_observation
      )
      yuma_feature.persist_provisional_telemetry(
        "rtc_observation",
        now=now,
        observation=changed_rtc_observation,
        authority=latest_authority_evaluation,
      )
      if (
        changed_rtc_observation.state
        is RtcObservationState.OBSERVED
      ):
        latest_cross_boot_rtc_observation = (
          changed_rtc_observation
        )
        if latest_independent_time is not None:
          validation = validate_observed_cross_boot_rtc(
            latest_cross_boot_rtc_observation,
            latest_independent_time,
          )
          yuma_feature.note_cross_boot_validation(validation)
        authority_for_provisional = (
          latest_authority_evaluation
          or evaluate_time_authority(
            time_authority,
            read_host_time_observation(),
          )
        )
        latest_authority_evaluation = authority_for_provisional
        provisional_decision = evaluate_provisional_yuma_reference(
          latest_cross_boot_rtc_observation,
          authority_for_provisional,
          receiver_cycle=getattr(pigeon, "receiver_cycle", 0),
        )
        provisional_accepted = (
          provisional_decision.reference is not None
          and yuma_feature.set_provisional_reference(
            provisional_decision.reference
          )
        )
        log_provisional_yuma_reference_decision(
          provisional_decision,
          accepted=provisional_accepted,
        )
        yuma_feature.persist_provisional_telemetry(
          "reference_decision",
          now=now,
          observation=changed_rtc_observation,
          authority=authority_for_provisional,
          decision=provisional_decision,
          accepted=provisional_accepted,
        )
    sm.update(0)

    host_time_observation = read_host_time_observation()
    if host_time_requires_processing(
      host_time_state,
      host_time_observation,
      now=now,
    ):
      authority_evaluation_for_loop = evaluate_time_authority(
        time_authority,
        host_time_observation,
      )
      latest_authority_evaluation = authority_evaluation_for_loop
      host_time_state = host_time_processing_state(
        host_time_observation,
        authority_evaluation_for_loop,
        now=now,
      )
      host_authorized = (
        authority_evaluation_for_loop.authorized_time
      )
      host_independent = (
        independent_time_observation(host_authorized)
        if (
          host_time_observation is not None
          and host_time_observation.independent
        )
        else None
      )
      if host_independent is not None:
        latest_independent_time = host_independent
        yuma_feature.set_time_anchor(
          host_independent.utc,
          now,
          host_independent.source.value,
        )
        validation = validate_observed_cross_boot_rtc(
          latest_cross_boot_rtc_observation,
          host_independent,
        )
        yuma_feature.note_cross_boot_validation(validation)
        correction_context = (
          startup_diagnostics.time_assistance_context(
            now
          )
        )
        correction_decision, correction_accepted = (
          maybe_send_receiver_time_correction(
            pigeon,
            receiver_time_provenance,
            host_independent,
            diagnostic_context=correction_context,
          )
        )
        if (
          correction_decision.should_correct
          and receiver_time_provenance.correction_written
        ):
          trusted_time_assistance_sent = True
          next_time_assistance_attempt = (
            now + TIME_ASSISTANCE_RETRY_INTERVAL
          )
          if (
            correction_accepted
            and not (
              assistnow_autonomous_configuration_attempted
            )
          ):
            configure_assistnow_autonomous(
              pigeon,
              mon_ver_info,
            )
            assistnow_autonomous_configuration_attempted = True

    if sm.updated['deviceState']:
      current_started = sm['deviceState'].started

      if started_state is None:
        started_state = current_started

      elif current_started != started_state:
        if current_started and dump_collector.active:
          dump_collector.cancel()
        capture_state.road_state_changed(current_started)
        if current_started:
          cloudlog.info(
            "GPS assistance drive tracking started"
          )
        else:
          cloudlog.info(
            "GPS assistance post-drive refresh requested"
          )
          readiness_message = capture_state.drive_end_readiness_message()
          if readiness_message is not None:
            cloudlog.info(readiness_message)

        started_state = current_started

    if (
      not trusted_time_assistance_sent
      and now >= next_time_assistance_attempt
    ):
      authority_evaluation = (
        authority_evaluation_for_loop
        or evaluate_time_authority(
          time_authority,
          read_host_time_observation(),
        )
      )
      latest_authority_evaluation = authority_evaluation
      authorized_time = authority_evaluation.authorized_time
      if authorized_time is not None:
        receiver_self_source = (
          receiver_self_time_cycle
          == receiver_time_provenance.cycle_id
          and authorized_time.source
          is TrustedTimeSource.RECEIVER_UTC_UNASSISTED_GNSS
          and authorized_time.evidence.value
          == "same_boot_boottime"
        )
        if receiver_self_source:
          cloudlog.info(
            "GPS time assistance suppressed: reason=receiver_self_resolved_utc"
          )
          next_time_assistance_attempt = (
            now + TIME_SYNC_CHECK_INTERVAL
          )
        else:
          diagnostic_context = (
            startup_diagnostics.time_assistance_context(
              time.monotonic()
            )
          )
          anchor_monotonic = now
          yuma_feature.set_time_anchor(
            authorized_time.utc,
            anchor_monotonic,
            authorized_time.evidence.value,
          )
          trusted_time_assistance_sent = send_time_assistance(
            pigeon,
            assistance_time=authorized_time.utc,
            accuracy_seconds=(
              authorized_time.mga_accuracy_seconds
            ),
            source=authorized_time.evidence.value,
            diagnostic_context=diagnostic_context,
            time_provenance=receiver_time_provenance,
            assistance_boottime_seconds=getattr(
              authorized_time,
              "observed_boottime_seconds",
              None,
            ),
            independent=authorized_time.independent,
            source_provenance=authorized_time.provenance,
          )
          if (
            trusted_time_assistance_sent
            and not assistnow_autonomous_configuration_attempted
          ):
            configure_assistnow_autonomous(
              pigeon,
              mon_ver_info,
            )
            assistnow_autonomous_configuration_attempted = True
          next_time_assistance_attempt = (
            now + TIME_ASSISTANCE_RETRY_INTERVAL
          )
      else:
        next_time_assistance_attempt = (
          now + TIME_SYNC_CHECK_INTERVAL
        )

    startup_diagnostics.log_acquisition_status(
      time.monotonic()
    )

    raw_published_by_pigeon = hasattr(pigeon, "_stream_parser")
    try:
      if raw_published_by_pigeon:
        data, received_frames = pigeon.receive_normal()
      else:
        data = pigeon.receive()
        received_frames = stream_parser.feed(data)
    except RawPublicationError as exc:
      cloudlog.error(f"GPS raw publication deferred: {exc}")
      time.sleep(0.001)
      continue

    if not data and not received_frames:
      if not data_watchdog.check(now):
        time.sleep(0.001)
        continue

      cloudlog.error("No data from ublox for 10 seconds; power-cycling and reinitializing receiver")

      cycle_initialization = initialize_receiver_cycle(
        pigeon,
        receiver_fingerprint,
        startup_diagnostics,
        "no_data_watchdog",
        time_authority=time_authority,
        time_provenance=receiver_time_provenance,
        navigation_database_runtime=navigation_database_runtime,
      )
      initialization_completed_at = (
        cycle_initialization.completed_at
      )

      stream_parser.reset()
      fix_tracker.reset()
      capture_quality_tracker.reset()
      dump_collector.cancel()
      capture_state.reset_receiver_cycle()

      trusted_time_assistance_sent = (
        cycle_initialization.trusted_time_assistance_sent
      )
      next_time_assistance_attempt = (
        cycle_initialization.next_time_assistance_attempt
      )
      mon_ver_info = cycle_initialization.mon_ver_info
      assistnow_autonomous_configuration_attempted = (
        cycle_initialization.assistnow_autonomous_configuration_attempted
      )
      assistnow_autonomous_supported = (
        cycle_initialization.assistnow_autonomous_supported
      )
      autonomous_orbit_diagnostics.logged_state_mask = 0
      data_watchdog.recovery_completed(time.monotonic())
      startup_diagnostics.initialization_complete(
        initialization_completed_at
      )
      yuma_feature.reset_receiver_cycle(
        cycle_initialization,
        getattr(pigeon, "receiver_cycle", 0),
      )
      cycle_independent_time = independent_time_observation(
        getattr(cycle_initialization, "authorized_time", None)
      )
      if cycle_independent_time is not None:
        latest_independent_time = cycle_independent_time
      host_time_state = host_time_processing_state(
        getattr(
          cycle_initialization,
          "host_time_observation",
          None,
        ),
        getattr(
          cycle_initialization,
          "authority_evaluation",
          None,
        ),
        now=time.monotonic(),
      )
      latest_authority_evaluation = getattr(
        cycle_initialization,
        "authority_evaluation",
        None,
      )
      continue

    data_watchdog.note_data(now)

    if data:
      if is_all_zero_ublox_data(data):
        cloudlog.warning(
          "received all-zero data from ublox, re-initing!"
        )

        cycle_initialization = initialize_receiver_cycle(
          pigeon,
          receiver_fingerprint,
          startup_diagnostics,
          "all_zero_data",
          time_authority=time_authority,
          time_provenance=receiver_time_provenance,
          navigation_database_runtime=navigation_database_runtime,
        )
        initialization_completed_at = (
          cycle_initialization.completed_at
        )

        stream_parser.reset()
        fix_tracker.reset()
        capture_quality_tracker.reset()
        dump_collector.cancel()
        capture_state.reset_receiver_cycle()

        trusted_time_assistance_sent = (
          cycle_initialization.trusted_time_assistance_sent
        )
        next_time_assistance_attempt = (
          cycle_initialization.next_time_assistance_attempt
        )
        mon_ver_info = cycle_initialization.mon_ver_info
        assistnow_autonomous_configuration_attempted = (
          cycle_initialization.assistnow_autonomous_configuration_attempted
        )
        assistnow_autonomous_supported = (
          cycle_initialization.assistnow_autonomous_supported
        )
        autonomous_orbit_diagnostics.logged_state_mask = 0
        startup_diagnostics.initialization_complete(
          initialization_completed_at
        )
        yuma_feature.reset_receiver_cycle(
          cycle_initialization,
          getattr(pigeon, "receiver_cycle", 0),
        )
        cycle_independent_time = independent_time_observation(
          getattr(cycle_initialization, "authorized_time", None)
        )
        if cycle_independent_time is not None:
          latest_independent_time = cycle_independent_time
        host_time_state = host_time_processing_state(
          getattr(
            cycle_initialization,
            "host_time_observation",
            None,
          ),
          getattr(
            cycle_initialization,
            "authority_evaluation",
            None,
          ),
          now=time.monotonic(),
        )
        latest_authority_evaluation = getattr(
          cycle_initialization,
          "authority_evaluation",
          None,
        )
        continue

      if not raw_published_by_pigeon:
        publish_ublox_raw(pm, data)

    dispatch_frames(received_frames)
    completed_database = (
      completed_databases.popleft()
      if completed_databases
      else None
    )

    now = time.monotonic()
    changed_receiver_utc = (
      receiver_time_provenance.changed_observation(now)
    )
    if changed_receiver_utc is not None:
      log_receiver_utc_observation(
        changed_receiver_utc
      )
      receiver_evaluation = (
        authorize_independent_receiver_utc(
          time_authority,
          changed_receiver_utc,
          now=now,
        )
      )
      if receiver_evaluation is not None:
        latest_authority_evaluation = receiver_evaluation
      if (
        receiver_evaluation is not None
        and receiver_evaluation.authorized_time is not None
      ):
        receiver_independent = independent_time_observation(
          receiver_evaluation.authorized_time
        )
        if receiver_independent is not None:
          latest_independent_time = receiver_independent
          receiver_self_time_cycle = (
            changed_receiver_utc.cycle_id
          )
          yuma_feature.set_receiver_time_anchor(
            receiver_independent.utc,
            now,
            source=receiver_independent.source.value,
          )
          validation = validate_observed_cross_boot_rtc(
            latest_cross_boot_rtc_observation,
            receiver_independent,
          )
          yuma_feature.note_cross_boot_validation(validation)

    stable_fix = fix_tracker.stable_fix(now)
    previous_database_disposition = (
      navigation_database_runtime.controller.disposition
    )
    database_execution = navigation_database_runtime.evaluate(
      authorized_time=(
        latest_authority_evaluation.authorized_time
        if latest_authority_evaluation is not None
        else None
      ),
      reliable_fix_available=stable_fix is not None,
      yuma_already_sent=yuma_feature.cycle_injection_consumed,
      send_database_message=reject_live_database_write,
    )
    if (
      navigation_database_runtime.controller.disposition
      is not previous_database_disposition
    ):
      database_result = (
        navigation_assistance_result_from_database_execution(
          database_execution
        )
      )
      log_navigation_assistance_restore_result(
        database_result,
        startup_diagnostics.time_assistance_context(now),
        (
          latest_authority_evaluation.authorized_time.evidence.value
          if (
            latest_authority_evaluation is not None
            and latest_authority_evaluation.authorized_time is not None
          )
          else None
        ),
      )
      yuma_feature.update_navigation_assistance_restore_result(
        database_result,
        now,
      )

    # Provisional YUMA deliberately has first YUMA priority; the shared
    # wrapper durably consumes the DBD/YUMA boot choice before frame 0.
    provisional_yuma_outcome = yuma_feature.evaluate_provisional(
      send_yuma_message,
      now=now,
      reliable_fix_available=stable_fix is not None,
    )
    if provisional_yuma_outcome is not None:
      log_provisional_yuma_outcome(provisional_yuma_outcome)
      yuma_feature.persist_provisional_telemetry(
        "transmission",
        now=now,
        outcome=provisional_yuma_outcome,
      )

    yuma_outcome = yuma_feature.evaluate(
      send_yuma_message,
      now=now,
      nav_sat=capture_quality_tracker.latest_nav_sat,
      nav_sat_time=(
        capture_quality_tracker.latest_nav_sat_time
      ),
      reliable_fix_available=stable_fix is not None,
    )
    if yuma_outcome is not None:
      log_yuma_supplementation_outcome(yuma_outcome)
      persist_yuma_supplementation_outcome(
        yuma_outcome,
        params,
      )

    if dump_collector.expired(now):
      cloudlog.warning(
        "GPS navigation database capture timed out"
      )
      dump_collector.cancel()
      capture_state.fail(now)

    if (
      completed_database is not None
      and capture_state.capture_fix is not None
      and capture_state.capture_quality is not None
      and capture_state.capture_reason is not None
    ):
      finalized_quality = finalized_capture_quality(
        capture_state,
        capture_quality_tracker,
        now,
        getattr(pigeon, "receiver_cycle", 0),
        stable_fix,
      )
      capture_still_valid = finalized_quality is not None
      receiver_utc_observation = (
        receiver_time_provenance.current_observation(now)
      )
      promotion_authority = (
        time_authority.current_authorized_time(
          host_time_observation=(
            read_host_time_observation()
          ),
        )
      )
      authorized_promotion_utc = (
        promotion_authority.authorized_time.utc
        if promotion_authority.authorized_time is not None
        else None
      )
      trusted_promotion_utc = cache_promotion_trusted_now(
        (
          capture_quality_tracker.latest_fix.utc_time
          if capture_quality_tracker.latest_fix is not None
          else None
        ),
        capture_state.capture_receiver_cycle,
        getattr(pigeon, "receiver_cycle", 0),
        receiver_utc_fresh=capture_still_valid,
        receiver_utc_independent=(
          receiver_utc_observation.independent
        ),
        authorized_utc=authorized_promotion_utc,
      )
      if not capture_still_valid:
        cloudlog.warning("GPS navigation assistance candidate discarded because quality degraded during dump")
        result = NavigationAssistanceCacheResult.FAILED
      elif trusted_promotion_utc is None:
        cloudlog.warning("GPS navigation assistance candidate discarded because trusted promotion UTC is unavailable")
        result = NavigationAssistanceCacheResult.FAILED
      else:
        assert finalized_quality is not None
        source = {
          "onroad": "onroad_first",
          "onroad_refresh": "onroad_refresh",
          "post_drive": "postdrive",
        }[capture_state.capture_reason]
        result = write_navigation_assistance_cache(
          receiver_fingerprint,
          capture_state.capture_fix,
          completed_database,
          finalized_quality,
          source=source,
          receiver_cycle=capture_state.capture_receiver_cycle,
          receiver_utc_now=(
            capture_quality_tracker.latest_fix.utc_time
            if capture_quality_tracker.latest_fix is not None
            else None
          ),
          active_receiver_cycle=getattr(pigeon, "receiver_cycle", 0),
          receiver_utc_fresh=capture_still_valid,
          receiver_utc_independent=(
            receiver_utc_observation.independent
          ),
          trusted_promotion_utc=trusted_promotion_utc,
        )

      durable_quality = durable_quality_after_cache_result(
        result,
        receiver_fingerprint,
        trusted_promotion_utc,
      )
      readiness_message = capture_state.complete(
        result,
        now,
        durable_quality,
        finalized_quality,
      )
      if readiness_message is not None:
        cloudlog.info(readiness_message)

    if capture_state.request(
      now,
      started_state,
      dump_collector.active,
      capture_quality_tracker,
      getattr(pigeon, "receiver_cycle", 0),
      stable_fix,
    ):
      trigger_source = {
        "onroad": "onroad_first",
        "onroad_refresh": "onroad_refresh",
        "post_drive": "postdrive",
      }[capture_state.capture_reason]
      cloudlog.info(", ".join((
        "GPS navigation cache capture trigger",
        f"source={trigger_source}",
        f"quality_tier={navigation_quality_tier(capture_state.capture_quality).value}",
        f"receiver_cycle={capture_state.capture_receiver_cycle}",
        f"quality={capture_state.capture_quality}",
      )))
      try:
        request_navigation_database_capture(
          pigeon,
          dump_collector,
          capture_state,
          now,
          assistnow_autonomous_supported,
        )

      except Exception:
        cloudlog.exception(
          "Failed to request GPS navigation database"
        )
        dump_collector.cancel()
        capture_state.fail(now)


    if not data:
      time.sleep(0.001)


def main():
  assert TICI, "unsupported hardware for pigeond"
  run_receiving()

if __name__ == "__main__":
  main()
