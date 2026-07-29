"""Same-boot execution of the trusted-age MGA-DBD restore policy."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
import json
from math import isfinite
import os
from pathlib import Path
import tempfile
import time
from typing import Any

from openpilot.system.ubloxd.gps_assistance import (
  CacheAgeEvidence,
  GpsAssistanceCache,
  GPS_ASSISTANCE_CACHE_PATH,
  NavigationCacheStore,
  NavigationQuality,
  RestoredNavigationQuality,
  build_position_assistance_message,
  effective_restored_navigation_quality,
  load_cache,
)
from openpilot.system.ubloxd.navigation_database_restore import (
  NavigationDatabaseRestoreBootController,
  NavigationDatabaseRestoreDisposition,
  evaluate_navigation_database_restore,
  is_current_independent_network_time,
)
from openpilot.system.ubloxd.trusted_time_anchor import read_boot_id
from openpilot.system.ubloxd.trusted_time_authority import AuthorizedTime


NAVIGATION_DATABASE_RESTORE_STATE_VERSION = 1
NAVIGATION_DATABASE_RESTORE_STATE_PATH = Path("/data/gps_assistance/navigation_database_restore_state.json")


class NavigationDatabaseRestoreStateError(ValueError):
  pass


@dataclass(frozen=True)
class NavigationDatabaseRestoreSnapshot:
  saved_at_utc: datetime
  database_frames: tuple[bytes, ...]
  latitude_e7: int
  longitude_e7: int
  altitude_cm: int
  position_accuracy_cm: int
  quality: NavigationQuality | None
  generation: str
  selection_reason: str

  @classmethod
  def from_cache(
    cls,
    cache: GpsAssistanceCache,
    *,
    generation: str,
    selection_reason: str,
  ) -> NavigationDatabaseRestoreSnapshot:
    return cls(
      saved_at_utc=cache.saved_at_utc,
      database_frames=cache.database_frames,
      latitude_e7=cache.latitude_e7,
      longitude_e7=cache.longitude_e7,
      altitude_cm=cache.altitude_cm,
      position_accuracy_cm=cache.position_accuracy_cm,
      quality=getattr(cache, "quality", None),
      generation=generation,
      selection_reason=selection_reason,
    )


@dataclass(frozen=True)
class NavigationDatabaseRestoreExecution:
  disposition: NavigationDatabaseRestoreDisposition
  total_frame_count: int
  accepted_frame_count: int
  database_write_attempt_count: int
  initially_failed_indexes: tuple[int, ...] = ()
  retry_accepted_indexes: tuple[int, ...] = ()
  permanently_failed_indexes: tuple[int, ...] = ()
  position_assistance_attempted: bool = False
  position_assistance_succeeded: bool = False
  position_assistance_error: str | None = None
  cache_saved_at_utc: datetime | None = None
  cache_generation: str | None = None
  cache_selection_reason: str | None = None
  cache_age_seconds: float | None = None
  effective_quality: RestoredNavigationQuality | None = None
  captured_quality: NavigationQuality | None = None
  boot_id: str | None = None
  state_persistence_error: str | None = None
  recovered_interrupted_attempt: bool = False

  @property
  def database_available(self) -> bool:
    return self.disposition.database_available


@dataclass(frozen=True)
class NavigationDatabaseRestoreBootState:
  version: int
  boot_id: str
  receiver_fingerprint: str
  disposition: NavigationDatabaseRestoreDisposition
  restore_attempted: bool
  position_assistance_claimed: bool
  acquisition_started: bool
  yuma_sent: bool
  cache_generation: str | None = None
  cache_saved_at_utc: datetime | None = None

  def __post_init__(self) -> None:
    if self.version != NAVIGATION_DATABASE_RESTORE_STATE_VERSION:
      raise NavigationDatabaseRestoreStateError("unsupported state version")
    if not isinstance(self.boot_id, str) or not self.boot_id.strip():
      raise NavigationDatabaseRestoreStateError("boot_id is invalid")
    if not isinstance(self.receiver_fingerprint, str):
      raise NavigationDatabaseRestoreStateError("receiver_fingerprint is invalid")
    if not isinstance(self.disposition, NavigationDatabaseRestoreDisposition):
      raise NavigationDatabaseRestoreStateError("disposition is invalid")
    for name, value in (
      ("restore_attempted", self.restore_attempted),
      ("position_assistance_claimed", self.position_assistance_claimed),
      ("acquisition_started", self.acquisition_started),
      ("yuma_sent", self.yuma_sent),
    ):
      if not isinstance(value, bool):
        raise NavigationDatabaseRestoreStateError(f"{name} is invalid")
    if self.restore_attempted and self.disposition.intentionally_skipped:
      raise NavigationDatabaseRestoreStateError("attempted restore cannot have an intentional-skip disposition")
    if (
      self.disposition
      in (
        NavigationDatabaseRestoreDisposition.RESTORED,
        NavigationDatabaseRestoreDisposition.WRITE_FAILED,
      )
      and not self.restore_attempted
    ):
      raise NavigationDatabaseRestoreStateError("restore completion requires restore_attempted")
    if self.cache_generation is not None and not isinstance(self.cache_generation, str):
      raise NavigationDatabaseRestoreStateError("cache_generation is invalid")
    if self.cache_saved_at_utc is not None:
      if not isinstance(self.cache_saved_at_utc, datetime):
        raise NavigationDatabaseRestoreStateError("cache_saved_at_utc is invalid")
      if self.cache_saved_at_utc.tzinfo is None or self.cache_saved_at_utc.utcoffset() is None:
        raise NavigationDatabaseRestoreStateError("cache_saved_at_utc must be timezone-aware")

  def to_json_dict(self) -> dict[str, Any]:
    return {
      "version": self.version,
      "boot_id": self.boot_id,
      "receiver_fingerprint": self.receiver_fingerprint,
      "disposition": self.disposition.value,
      "restore_attempted": self.restore_attempted,
      "position_assistance_claimed": self.position_assistance_claimed,
      "acquisition_started": self.acquisition_started,
      "yuma_sent": self.yuma_sent,
      "cache_generation": self.cache_generation,
      "cache_saved_at_utc": (self.cache_saved_at_utc.astimezone(UTC).isoformat() if self.cache_saved_at_utc is not None else None),
    }

  @classmethod
  def from_json_dict(cls, value: object) -> NavigationDatabaseRestoreBootState:
    if not isinstance(value, dict):
      raise NavigationDatabaseRestoreStateError("state root is invalid")
    expected_keys = {
      "version",
      "boot_id",
      "receiver_fingerprint",
      "disposition",
      "restore_attempted",
      "position_assistance_claimed",
      "acquisition_started",
      "yuma_sent",
      "cache_generation",
      "cache_saved_at_utc",
    }
    if set(value) != expected_keys:
      raise NavigationDatabaseRestoreStateError("state keys are invalid")
    saved_at_raw = value["cache_saved_at_utc"]
    if saved_at_raw is None:
      saved_at = None
    elif isinstance(saved_at_raw, str):
      try:
        saved_at = datetime.fromisoformat(saved_at_raw)
      except ValueError as exc:
        raise NavigationDatabaseRestoreStateError("cache_saved_at_utc is invalid") from exc
    else:
      raise NavigationDatabaseRestoreStateError("cache_saved_at_utc is invalid")
    try:
      disposition = NavigationDatabaseRestoreDisposition(value["disposition"])
    except (TypeError, ValueError) as exc:
      raise NavigationDatabaseRestoreStateError("disposition is invalid") from exc
    return cls(
      version=value["version"],
      boot_id=value["boot_id"],
      receiver_fingerprint=value["receiver_fingerprint"],
      disposition=disposition,
      restore_attempted=value["restore_attempted"],
      position_assistance_claimed=value["position_assistance_claimed"],
      acquisition_started=value["acquisition_started"],
      yuma_sent=value["yuma_sent"],
      cache_generation=value["cache_generation"],
      cache_saved_at_utc=saved_at,
    )


def load_navigation_database_restore_boot_state(
  path: Path = NAVIGATION_DATABASE_RESTORE_STATE_PATH,
) -> NavigationDatabaseRestoreBootState | None:
  try:
    raw = path.read_text(encoding="utf-8")
  except FileNotFoundError:
    return None
  except OSError as exc:
    raise NavigationDatabaseRestoreStateError("state read failed") from exc
  try:
    value = json.loads(raw)
  except json.JSONDecodeError as exc:
    raise NavigationDatabaseRestoreStateError("state JSON is invalid") from exc
  return NavigationDatabaseRestoreBootState.from_json_dict(value)


def store_navigation_database_restore_boot_state(
  state: NavigationDatabaseRestoreBootState,
  path: Path = NAVIGATION_DATABASE_RESTORE_STATE_PATH,
) -> None:
  if not isinstance(state, NavigationDatabaseRestoreBootState):
    raise NavigationDatabaseRestoreStateError("state is invalid")
  parent = path.parent
  parent.mkdir(parents=True, exist_ok=True)
  descriptor, temporary_name = tempfile.mkstemp(
    prefix=f".{path.name}.",
    suffix=".tmp",
    dir=parent,
  )
  temporary = Path(temporary_name)
  try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
      json.dump(
        state.to_json_dict(),
        stream,
        sort_keys=True,
        separators=(",", ":"),
      )
      stream.write("\n")
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, path)
    directory_descriptor = os.open(parent, os.O_RDONLY)
    try:
      os.fsync(directory_descriptor)
    finally:
      os.close(directory_descriptor)
  finally:
    temporary.unlink(missing_ok=True)


def load_navigation_database_restore_snapshot(
  receiver_fingerprint: str,
) -> NavigationDatabaseRestoreSnapshot | None:
  store = NavigationCacheStore(GPS_ASSISTANCE_CACHE_PATH, loader=load_cache)
  store.remove_stale_candidate()
  selection, _ = store.select_best(
    receiver_fingerprint,
    None,
    age_evidence=CacheAgeEvidence.UNVERIFIED,
  )
  if selection is None:
    return None
  return NavigationDatabaseRestoreSnapshot.from_cache(
    selection.cache,
    generation=selection.generation,
    selection_reason=selection.reason,
  )


class NavigationDatabaseRestoreRuntime:
  """Persists one DBD decision and position claim for the current Linux boot."""

  def __init__(
    self,
    receiver_fingerprint: str,
    *,
    snapshot_loader: Callable[[str], NavigationDatabaseRestoreSnapshot | None] = load_navigation_database_restore_snapshot,
    retry_delay_seconds: float = 0.25,
    sleeper: Callable[[float], None] = time.sleep,
    state_path: Path = NAVIGATION_DATABASE_RESTORE_STATE_PATH,
    boot_id_reader: Callable[[], str | None] = read_boot_id,
    state_loader: Callable[[Path], NavigationDatabaseRestoreBootState | None] = load_navigation_database_restore_boot_state,
    state_storer: Callable[[NavigationDatabaseRestoreBootState, Path], None] = store_navigation_database_restore_boot_state,
  ) -> None:
    if not isinstance(receiver_fingerprint, str):
      raise ValueError("receiver_fingerprint must be a string")
    for name, dependency in (
      ("snapshot_loader", snapshot_loader),
      ("sleeper", sleeper),
      ("boot_id_reader", boot_id_reader),
      ("state_loader", state_loader),
      ("state_storer", state_storer),
    ):
      if not callable(dependency):
        raise ValueError(f"{name} must be callable")
    if (
      isinstance(retry_delay_seconds, bool)
      or not isinstance(retry_delay_seconds, (int, float))
      or not isfinite(float(retry_delay_seconds))
      or float(retry_delay_seconds) < 0.0
    ):
      raise ValueError("retry_delay_seconds must be finite and non-negative")
    if not isinstance(state_path, Path):
      raise ValueError("state_path must be a Path")

    self._receiver_fingerprint = receiver_fingerprint
    self._snapshot_loader = snapshot_loader
    self._retry_delay_seconds = float(retry_delay_seconds)
    self._sleeper = sleeper
    self._state_path = state_path
    self._state_storer = state_storer
    self._controller = NavigationDatabaseRestoreBootController()
    self._snapshot_loaded = False
    self._snapshot: NavigationDatabaseRestoreSnapshot | None = None
    self._position_claimed = False
    self._position_attempted = False
    self._position_succeeded = False
    self._position_error: str | None = None
    self._yuma_sent = False
    self._state_persistence_error: str | None = None
    self._recovered_interrupted_attempt = False
    self._persisted_cache_generation: str | None = None
    self._persisted_cache_saved_at_utc: datetime | None = None

    try:
      boot_id = boot_id_reader()
    except Exception:
      boot_id = None
    self._boot_id = boot_id if isinstance(boot_id, str) and boot_id.strip() else None

    if self._boot_id is None:
      self._fail_closed("boot_state:boot_id_unavailable")
    else:
      try:
        persisted = state_loader(self._state_path)
      except Exception as exc:
        persisted = None
        self._fail_closed(f"boot_state:{type(exc).__name__}:{exc}")
      if self._controller.pending:
        self._restore_persisted_state(persisted)
      if persisted is None and self._controller.pending:
        self._persist_state()

    self._execution = self._build_execution()

  @property
  def controller(self) -> NavigationDatabaseRestoreBootController:
    return self._controller

  @property
  def snapshot(self) -> NavigationDatabaseRestoreSnapshot | None:
    return self._snapshot

  @property
  def acquisition_started(self) -> bool:
    return self._controller.acquisition_started

  @property
  def yuma_sent(self) -> bool:
    return self._yuma_sent

  @property
  def execution(self) -> NavigationDatabaseRestoreExecution:
    return self._execution

  def _fail_closed(self, reason: str) -> None:
    self._position_claimed = True
    self._position_error = reason
    if self._controller.pending and not self._controller.restore_attempted:
      self._controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED)

  def _restore_persisted_state(
    self,
    state: NavigationDatabaseRestoreBootState | None,
  ) -> None:
    if state is None:
      return
    assert self._boot_id is not None
    if state.boot_id != self._boot_id:
      return
    if state.receiver_fingerprint != self._receiver_fingerprint:
      self._fail_closed("boot_state:receiver_fingerprint_mismatch")
      self._persist_state()
      return

    self._position_claimed = state.position_assistance_claimed
    self._yuma_sent = state.yuma_sent
    self._persisted_cache_generation = state.cache_generation
    self._persisted_cache_saved_at_utc = state.cache_saved_at_utc
    if state.acquisition_started:
      self._controller.note_acquisition_started()

    if state.restore_attempted:
      self._controller.begin_restore_attempt()
      if state.disposition is NavigationDatabaseRestoreDisposition.PENDING:
        self._controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)
        self._recovered_interrupted_attempt = True
        self._persist_state()
      else:
        self._controller.finish_restore(state.disposition)
    elif state.disposition.intentionally_skipped:
      self._controller.skip(state.disposition)
    elif state.disposition is not NavigationDatabaseRestoreDisposition.PENDING:
      self._fail_closed("boot_state:invalid_terminal_state")
      self._persist_state()

  def _state(self) -> NavigationDatabaseRestoreBootState:
    if self._boot_id is None:
      raise NavigationDatabaseRestoreStateError("boot_id is unavailable")
    snapshot = self._snapshot
    generation = snapshot.generation if snapshot is not None else self._persisted_cache_generation
    saved_at = snapshot.saved_at_utc if snapshot is not None else self._persisted_cache_saved_at_utc
    return NavigationDatabaseRestoreBootState(
      version=NAVIGATION_DATABASE_RESTORE_STATE_VERSION,
      boot_id=self._boot_id,
      receiver_fingerprint=self._receiver_fingerprint,
      disposition=self._controller.disposition,
      restore_attempted=self._controller.restore_attempted,
      position_assistance_claimed=self._position_claimed,
      acquisition_started=self._controller.acquisition_started,
      yuma_sent=self._yuma_sent,
      cache_generation=generation,
      cache_saved_at_utc=saved_at,
    )

  def _persist_state(self) -> bool:
    if self._boot_id is None:
      self._state_persistence_error = "boot_id_unavailable"
      return False
    try:
      self._state_storer(self._state(), self._state_path)
    except Exception as exc:
      self._state_persistence_error = f"{type(exc).__name__}:{exc}"
      return False
    self._state_persistence_error = None
    return True

  def prepare(self) -> NavigationDatabaseRestoreExecution:
    if self._snapshot_loaded:
      return self._execution
    self._snapshot_loaded = True
    try:
      snapshot = self._snapshot_loader(self._receiver_fingerprint)
    except Exception as exc:
      snapshot = None
      self._position_error = f"snapshot_load:{type(exc).__name__}:{exc}"
    if snapshot is not None and not isinstance(
      snapshot,
      NavigationDatabaseRestoreSnapshot,
    ):
      self._position_error = "snapshot_load:invalid_snapshot_type"
      snapshot = None
    self._snapshot = snapshot

    if snapshot is None or not snapshot.database_frames:
      if self._controller.pending:
        self._controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_NO_USABLE_CACHE)
        self._persist_state()
      self._execution = self._build_execution()
      return self._execution

    if self._persisted_cache_generation is not None:
      if snapshot.generation != self._persisted_cache_generation or snapshot.saved_at_utc != self._persisted_cache_saved_at_utc:
        self._position_claimed = True
        self._position_error = "snapshot_identity_changed_within_boot"
        if self._controller.pending:
          self._controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED)
        self._persist_state()
        self._execution = self._build_execution()
        return self._execution
    else:
      self._persisted_cache_generation = snapshot.generation
      self._persisted_cache_saved_at_utc = snapshot.saved_at_utc
      if not self._persist_state():
        self._fail_closed("boot_state:snapshot_identity_persist_failed")

    self._execution = self._build_execution()
    return self._execution

  def send_position_once(
    self,
    send_message: Callable[[bytes], object],
  ) -> NavigationDatabaseRestoreExecution:
    if not callable(send_message):
      raise ValueError("send_message must be callable")
    self.prepare()
    if self._snapshot is None or self._position_claimed:
      return self._execution
    self._position_claimed = True
    if not self._persist_state():
      self._position_error = "boot_state:position_claim_persist_failed"
      self._execution = self._build_execution()
      return self._execution

    self._position_attempted = True
    try:
      message = build_position_assistance_message(
        latitude_e7=self._snapshot.latitude_e7,
        longitude_e7=self._snapshot.longitude_e7,
        altitude_cm=self._snapshot.altitude_cm,
        position_accuracy_cm=self._snapshot.position_accuracy_cm,
      )
      send_message(message)
    except Exception as exc:
      self._position_error = f"{type(exc).__name__}:{exc}"
      self._position_succeeded = False
    else:
      self._position_succeeded = True
    self._execution = self._build_execution()
    return self._execution

  def note_acquisition_started(self) -> None:
    if self._controller.acquisition_started:
      return
    self._controller.note_acquisition_started()
    self._persist_state()
    self._execution = self._build_execution()

  def note_yuma_sent(self) -> None:
    if self._yuma_sent:
      return
    self._yuma_sent = True
    self._persist_state()
    self._execution = self._build_execution()

  def evaluate(
    self,
    *,
    authorized_time: AuthorizedTime | None,
    reliable_fix_available: bool,
    yuma_already_sent: bool,
    send_database_message: Callable[[bytes, int], object],
  ) -> NavigationDatabaseRestoreExecution:
    if not callable(send_database_message):
      raise ValueError("send_database_message must be callable")
    self.prepare()
    snapshot = self._snapshot
    if snapshot is None or self._controller.terminal:
      return self._execution

    cache_age_seconds = None
    if authorized_time is not None and is_current_independent_network_time(authorized_time):
      try:
        cache_age_seconds = (authorized_time.utc - snapshot.saved_at_utc).total_seconds()
      except (OverflowError, TypeError, ValueError):
        cache_age_seconds = None

    decision = evaluate_navigation_database_restore(
      reliable_fix_available=reliable_fix_available,
      yuma_already_sent=(yuma_already_sent or self._yuma_sent),
      authorized_time=authorized_time,
      cache_age_seconds=cache_age_seconds,
      gnss_acquisition_started=self._controller.acquisition_started,
    )
    if not self._controller.apply_decision(decision):
      self._execution = self._build_execution(authorized_time)
      return self._execution
    if not decision.should_restore:
      self._persist_state()
      self._execution = self._build_execution(authorized_time)
      return self._execution

    if not self._persist_state():
      self._controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)
      self._execution = self._build_execution(authorized_time)
      return self._execution

    accepted: set[int] = set()
    initially_failed: list[int] = []
    retry_accepted: list[int] = []
    permanently_failed: list[int] = []
    failed_frames: list[tuple[int, bytes]] = []
    write_attempts = 0
    succeeded = False
    try:
      for index, frame in enumerate(snapshot.database_frames):
        write_attempts += 1
        try:
          send_database_message(frame, index)
          accepted.add(index)
        except Exception:
          initially_failed.append(index)
          failed_frames.append((index, frame))
      if failed_frames and self._retry_delay_seconds:
        self._sleeper(self._retry_delay_seconds)
      for index, frame in failed_frames:
        write_attempts += 1
        try:
          send_database_message(frame, index)
          accepted.add(index)
          retry_accepted.append(index)
        except Exception:
          permanently_failed.append(index)
      succeeded = bool(snapshot.database_frames) and len(accepted) == len(snapshot.database_frames) and not permanently_failed
    except Exception:
      succeeded = False
    finally:
      self._controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED if succeeded else NavigationDatabaseRestoreDisposition.WRITE_FAILED)
      self._persist_state()

    self._execution = self._build_execution(
      authorized_time,
      accepted_frame_count=len(accepted),
      database_write_attempt_count=write_attempts,
      initially_failed_indexes=tuple(initially_failed),
      retry_accepted_indexes=tuple(retry_accepted),
      permanently_failed_indexes=tuple(permanently_failed),
    )
    return self._execution

  def _build_execution(
    self,
    authorized_time: AuthorizedTime | None = None,
    *,
    accepted_frame_count: int = 0,
    database_write_attempt_count: int = 0,
    initially_failed_indexes: tuple[int, ...] = (),
    retry_accepted_indexes: tuple[int, ...] = (),
    permanently_failed_indexes: tuple[int, ...] = (),
  ) -> NavigationDatabaseRestoreExecution:
    snapshot = self._snapshot
    cache_age_seconds = None
    effective_quality = None
    if snapshot is not None:
      current_network_time = authorized_time if (authorized_time is not None and is_current_independent_network_time(authorized_time)) else None
      age_evidence = CacheAgeEvidence.TRUSTED_UTC if current_network_time is not None else CacheAgeEvidence.UNVERIFIED
      if current_network_time is not None:
        try:
          cache_age_seconds = (current_network_time.utc - snapshot.saved_at_utc).total_seconds()
        except (OverflowError, TypeError, ValueError):
          cache_age_seconds = None
      effective_quality = effective_restored_navigation_quality(
        snapshot.quality,
        snapshot.saved_at_utc,
        (current_network_time.utc if current_network_time is not None else None),
        age_evidence,
      )

    return NavigationDatabaseRestoreExecution(
      disposition=self._controller.disposition,
      total_frame_count=(len(snapshot.database_frames) if snapshot else 0),
      accepted_frame_count=accepted_frame_count,
      database_write_attempt_count=database_write_attempt_count,
      initially_failed_indexes=initially_failed_indexes,
      retry_accepted_indexes=retry_accepted_indexes,
      permanently_failed_indexes=permanently_failed_indexes,
      position_assistance_attempted=self._position_attempted,
      position_assistance_succeeded=self._position_succeeded,
      position_assistance_error=self._position_error,
      cache_saved_at_utc=(snapshot.saved_at_utc if snapshot else None),
      cache_generation=(snapshot.generation if snapshot else None),
      cache_selection_reason=(snapshot.selection_reason if snapshot else None),
      cache_age_seconds=cache_age_seconds,
      effective_quality=effective_quality,
      captured_quality=(snapshot.quality if snapshot else None),
      boot_id=self._boot_id,
      state_persistence_error=self._state_persistence_error,
      recovered_interrupted_attempt=self._recovered_interrupted_attempt,
    )
