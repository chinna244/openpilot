"""Linux-boot-scoped execution of the trusted-age MGA-DBD restore policy."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from enum import StrEnum
from hashlib import sha256
import json
from math import isfinite
import os
from pathlib import Path
import tempfile
import time
from typing import Any

from openpilot.system.ubloxd.gps_assistance import (
  CacheAgeEvidence,
  CacheFileInspection,
  CacheFileState,
  CacheInventory,
  CacheValidationError,
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
  NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS,
  NavigationDatabaseRestoreBootController,
  NavigationDatabaseRestoreDisposition,
  evaluate_navigation_database_restore,
  is_current_independent_network_time,
)
from openpilot.system.ubloxd.trusted_time_anchor import (
  read_boot_id,
  read_boottime_seconds,
)
from openpilot.system.ubloxd.trusted_time_authority import AuthorizedTime
from openpilot.system.ubloxd.yuma_almanac_transmit import (
  MgaReceiverNackError,
  MgaTransactionError,
  MgaWriteError,
)


NAVIGATION_DATABASE_RESTORE_STATE_VERSION = 2
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

  @property
  def database_digest(self) -> str:
    digest = sha256()
    for frame in self.database_frames:
      digest.update(len(frame).to_bytes(4, "big"))
      digest.update(frame)
    return digest.hexdigest()


@dataclass(frozen=True)
class NavigationDatabaseRestoreFrozenCaches:
  position_snapshot: NavigationDatabaseRestoreSnapshot | None
  primary_snapshot: NavigationDatabaseRestoreSnapshot | None
  previous_snapshot: NavigationDatabaseRestoreSnapshot | None
  inventory: CacheInventory | None = None

  @property
  def database_candidates(self) -> tuple[NavigationDatabaseRestoreSnapshot, ...]:
    return tuple(snapshot for snapshot in (self.primary_snapshot, self.previous_snapshot) if snapshot is not None and snapshot.database_frames)


class NavigationDatabaseRestoreFrameFailureKind(StrEnum):
  REJECTED = "rejected"
  TIMED_OUT = "timed_out"
  WRITE_ERROR = "write_error"
  TRANSACTION_ERROR = "transaction_error"
  VALIDATION_ERROR = "validation_error"
  UNEXPECTED_ERROR = "unexpected_error"

  @property
  def retryable(self) -> bool:
    return self in (
      NavigationDatabaseRestoreFrameFailureKind.TIMED_OUT,
      NavigationDatabaseRestoreFrameFailureKind.WRITE_ERROR,
      NavigationDatabaseRestoreFrameFailureKind.TRANSACTION_ERROR,
    )


@dataclass(frozen=True)
class NavigationDatabaseRestoreFrameFailure:
  frame_index: int
  attempt: int
  kind: NavigationDatabaseRestoreFrameFailureKind
  error: str


@dataclass(frozen=True)
class NavigationDatabaseRestoreCandidateIdentity:
  generation: str
  saved_at_utc: datetime
  database_digest: str

  def __post_init__(self) -> None:
    if not isinstance(self.generation, str) or self.generation not in ("primary", "previous", "legacy"):
      raise NavigationDatabaseRestoreStateError("candidate generation is invalid")
    if not isinstance(self.saved_at_utc, datetime):
      raise NavigationDatabaseRestoreStateError("candidate timestamp is invalid")
    if self.saved_at_utc.tzinfo is None or self.saved_at_utc.utcoffset() is None:
      raise NavigationDatabaseRestoreStateError("candidate timestamp must be aware")
    if not isinstance(self.database_digest, str):
      raise NavigationDatabaseRestoreStateError("candidate digest is invalid")
    if len(self.database_digest) != 64 or any(character not in "0123456789abcdef" for character in self.database_digest):
      raise NavigationDatabaseRestoreStateError("candidate digest is invalid")

  @classmethod
  def from_snapshot(
    cls,
    snapshot: NavigationDatabaseRestoreSnapshot,
  ) -> NavigationDatabaseRestoreCandidateIdentity:
    return cls(
      generation=snapshot.generation,
      saved_at_utc=snapshot.saved_at_utc,
      database_digest=snapshot.database_digest,
    )

  def matches(self, snapshot: NavigationDatabaseRestoreSnapshot) -> bool:
    return self == NavigationDatabaseRestoreCandidateIdentity.from_snapshot(snapshot)

  def to_json_dict(self) -> dict[str, str]:
    return {
      "generation": self.generation,
      "saved_at_utc": self.saved_at_utc.astimezone(UTC).isoformat(),
      "database_digest": self.database_digest,
    }

  @classmethod
  def from_json_dict(
    cls,
    value: object,
  ) -> NavigationDatabaseRestoreCandidateIdentity:
    if not isinstance(value, dict) or set(value) != {
      "generation",
      "saved_at_utc",
      "database_digest",
    }:
      raise NavigationDatabaseRestoreStateError("candidate identity is invalid")
    try:
      saved_at = datetime.fromisoformat(value["saved_at_utc"])
    except (TypeError, ValueError) as exc:
      raise NavigationDatabaseRestoreStateError("candidate timestamp is invalid") from exc
    return cls(
      generation=value["generation"],
      saved_at_utc=saved_at,
      database_digest=value["database_digest"],
    )


@dataclass(frozen=True)
class NavigationDatabaseRestoreExecution:
  disposition: NavigationDatabaseRestoreDisposition
  total_frame_count: int
  accepted_frame_count: int
  database_write_attempt_count: int
  initial_failures: tuple[NavigationDatabaseRestoreFrameFailure, ...] = ()
  retry_accepted_indexes: tuple[int, ...] = ()
  permanent_failures: tuple[NavigationDatabaseRestoreFrameFailure, ...] = ()
  execution_error: str | None = None
  failure_phase: str | None = None
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

  @property
  def initially_failed_indexes(self) -> tuple[int, ...]:
    return tuple(failure.frame_index for failure in self.initial_failures)

  @property
  def permanently_failed_indexes(self) -> tuple[int, ...]:
    return tuple(failure.frame_index for failure in self.permanent_failures)

  def initial_indexes(
    self,
    *kinds: NavigationDatabaseRestoreFrameFailureKind,
  ) -> tuple[int, ...]:
    return tuple(failure.frame_index for failure in self.initial_failures if failure.kind in kinds)

  def permanent_indexes(
    self,
    *kinds: NavigationDatabaseRestoreFrameFailureKind,
  ) -> tuple[int, ...]:
    return tuple(failure.frame_index for failure in self.permanent_failures if failure.kind in kinds)


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
  candidate_identities: tuple[NavigationDatabaseRestoreCandidateIdentity, ...] = ()
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
    if not isinstance(self.candidate_identities, tuple) or not all(
      isinstance(identity, NavigationDatabaseRestoreCandidateIdentity) for identity in self.candidate_identities
    ):
      raise NavigationDatabaseRestoreStateError("candidate identities are invalid")
    generations = tuple(identity.generation for identity in self.candidate_identities)
    if len(set(generations)) != len(generations):
      raise NavigationDatabaseRestoreStateError("candidate generations are duplicated")
    if self.cache_generation is not None and not isinstance(self.cache_generation, str):
      raise NavigationDatabaseRestoreStateError("cache_generation is invalid")
    if self.cache_saved_at_utc is not None:
      if not isinstance(self.cache_saved_at_utc, datetime):
        raise NavigationDatabaseRestoreStateError("cache_saved_at_utc is invalid")
      if self.cache_saved_at_utc.tzinfo is None or self.cache_saved_at_utc.utcoffset() is None:
        raise NavigationDatabaseRestoreStateError("cache_saved_at_utc must be timezone-aware")
    if (self.cache_generation is None) != (self.cache_saved_at_utc is None):
      raise NavigationDatabaseRestoreStateError("selected cache generation and timestamp must be paired")

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
      "candidate_identities": [identity.to_json_dict() for identity in self.candidate_identities],
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
      "candidate_identities",
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
    identities_raw = value["candidate_identities"]
    if not isinstance(identities_raw, list):
      raise NavigationDatabaseRestoreStateError("candidate identities are invalid")
    return cls(
      version=value["version"],
      boot_id=value["boot_id"],
      receiver_fingerprint=value["receiver_fingerprint"],
      disposition=disposition,
      restore_attempted=value["restore_attempted"],
      position_assistance_claimed=value["position_assistance_claimed"],
      acquisition_started=value["acquisition_started"],
      yuma_sent=value["yuma_sent"],
      candidate_identities=tuple(NavigationDatabaseRestoreCandidateIdentity.from_json_dict(identity) for identity in identities_raw),
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


def _snapshot_from_inspection(
  inspection: CacheFileInspection,
  *,
  reason: str,
) -> NavigationDatabaseRestoreSnapshot | None:
  if inspection.cache is None:
    return None
  return NavigationDatabaseRestoreSnapshot.from_cache(
    inspection.cache,
    generation=inspection.generation,
    selection_reason=reason,
  )


def load_navigation_database_restore_frozen_caches(
  receiver_fingerprint: str,
) -> NavigationDatabaseRestoreFrozenCaches:
  store = NavigationCacheStore(GPS_ASSISTANCE_CACHE_PATH, loader=load_cache)
  store.remove_stale_candidate()
  inventory = store.inspect(receiver_fingerprint, None)
  position_selection = store.select_inventory(
    inventory,
    age_evidence=CacheAgeEvidence.UNVERIFIED,
  )
  position_snapshot = (
    None
    if position_selection is None
    else NavigationDatabaseRestoreSnapshot.from_cache(
      position_selection.cache,
      generation=position_selection.generation,
      selection_reason=f"position:{position_selection.reason}",
    )
  )
  return NavigationDatabaseRestoreFrozenCaches(
    position_snapshot=position_snapshot,
    primary_snapshot=_snapshot_from_inspection(
      inventory.primary,
      reason="frozen_primary",
    ),
    previous_snapshot=_snapshot_from_inspection(
      inventory.previous,
      reason="frozen_previous",
    ),
    inventory=inventory,
  )


def _bounded_error(exc: BaseException) -> str:
  return f"{type(exc).__name__}:{exc}"[:240]


def _classify_failure(
  exc: BaseException,
) -> NavigationDatabaseRestoreFrameFailureKind:
  if isinstance(exc, MgaReceiverNackError):
    return NavigationDatabaseRestoreFrameFailureKind.REJECTED
  if isinstance(exc, TimeoutError):
    return NavigationDatabaseRestoreFrameFailureKind.TIMED_OUT
  if isinstance(exc, MgaWriteError):
    return NavigationDatabaseRestoreFrameFailureKind.WRITE_ERROR
  if isinstance(exc, MgaTransactionError):
    return NavigationDatabaseRestoreFrameFailureKind.TRANSACTION_ERROR
  if isinstance(exc, CacheValidationError):
    return NavigationDatabaseRestoreFrameFailureKind.VALIDATION_ERROR
  return NavigationDatabaseRestoreFrameFailureKind.UNEXPECTED_ERROR


class NavigationDatabaseRestoreRuntime:
  """Persists one DBD decision and receiver-write claims per Linux boot."""

  def __init__(
    self,
    receiver_fingerprint: str,
    *,
    snapshot_loader: Callable[
      [str],
      NavigationDatabaseRestoreFrozenCaches | NavigationDatabaseRestoreSnapshot | None,
    ] = load_navigation_database_restore_frozen_caches,
    retry_delay_seconds: float = 0.25,
    sleeper: Callable[[float], None] = time.sleep,
    state_path: Path = NAVIGATION_DATABASE_RESTORE_STATE_PATH,
    boot_id_reader: Callable[[], str | None] = read_boot_id,
    boottime_reader: Callable[[], float | None] = read_boottime_seconds,
    state_loader: Callable[[Path], NavigationDatabaseRestoreBootState | None] = load_navigation_database_restore_boot_state,
    state_storer: Callable[[NavigationDatabaseRestoreBootState, Path], None] = store_navigation_database_restore_boot_state,
  ) -> None:
    if not isinstance(receiver_fingerprint, str):
      raise ValueError("receiver_fingerprint must be a string")
    for name, dependency in (
      ("snapshot_loader", snapshot_loader),
      ("sleeper", sleeper),
      ("boot_id_reader", boot_id_reader),
      ("boottime_reader", boottime_reader),
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
    self._boottime_reader = boottime_reader
    self._state_storer = state_storer
    self._controller = NavigationDatabaseRestoreBootController()
    self._caches_loaded = False
    self._frozen_caches: NavigationDatabaseRestoreFrozenCaches | None = None
    self._position_snapshot: NavigationDatabaseRestoreSnapshot | None = None
    self._database_snapshot: NavigationDatabaseRestoreSnapshot | None = None
    self._candidate_identities: tuple[NavigationDatabaseRestoreCandidateIdentity, ...] = ()
    self._position_claimed = False
    self._position_attempted = False
    self._position_succeeded = False
    self._position_error: str | None = None
    self._yuma_sent = False
    self._state_persistence_error: str | None = None
    self._recovered_interrupted_attempt = False
    self._persisted_candidate_identities: tuple[NavigationDatabaseRestoreCandidateIdentity, ...] = ()
    self._persisted_cache_generation: str | None = None
    self._persisted_cache_saved_at_utc: datetime | None = None
    self._last_authorized_time: AuthorizedTime | None = None
    self._last_initial_failures: tuple[NavigationDatabaseRestoreFrameFailure, ...] = ()
    self._last_retry_accepted_indexes: tuple[int, ...] = ()
    self._last_permanent_failures: tuple[NavigationDatabaseRestoreFrameFailure, ...] = ()
    self._last_execution_error: str | None = None
    self._last_failure_phase: str | None = None
    self._last_accepted_frame_count = 0
    self._last_write_attempt_count = 0
    self._last_cache_age_seconds: float | None = None

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
    return self._database_snapshot or self._position_snapshot

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
    self._persisted_candidate_identities = state.candidate_identities
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
    elif state.disposition is NavigationDatabaseRestoreDisposition.PENDING:
      # A same-boot PENDING state belongs to an earlier process. The prior
      # process may have observed acquisition but failed before durably
      # recording the latch, so never reopen the DBD window after restart.
      self._controller.skip(
        NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
      )
      self._persist_state()
    else:
      self._fail_closed("boot_state:invalid_terminal_state")
      self._persist_state()

  def _state(self) -> NavigationDatabaseRestoreBootState:
    if self._boot_id is None:
      raise NavigationDatabaseRestoreStateError("boot_id is unavailable")
    snapshot = self._database_snapshot
    generation = snapshot.generation if snapshot is not None else self._persisted_cache_generation
    saved_at = snapshot.saved_at_utc if snapshot is not None else self._persisted_cache_saved_at_utc
    identities = self._candidate_identities or self._persisted_candidate_identities
    return NavigationDatabaseRestoreBootState(
      version=NAVIGATION_DATABASE_RESTORE_STATE_VERSION,
      boot_id=self._boot_id,
      receiver_fingerprint=self._receiver_fingerprint,
      disposition=self._controller.disposition,
      restore_attempted=self._controller.restore_attempted,
      position_assistance_claimed=self._position_claimed,
      acquisition_started=self._controller.acquisition_started,
      yuma_sent=self._yuma_sent,
      candidate_identities=identities,
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
      self._state_persistence_error = _bounded_error(exc)
      return False
    self._state_persistence_error = None
    return True

  @staticmethod
  def _normalize_frozen_caches(
    loaded: NavigationDatabaseRestoreFrozenCaches | NavigationDatabaseRestoreSnapshot | None,
  ) -> NavigationDatabaseRestoreFrozenCaches:
    if loaded is None:
      return NavigationDatabaseRestoreFrozenCaches(None, None, None)
    if isinstance(loaded, NavigationDatabaseRestoreSnapshot):
      return NavigationDatabaseRestoreFrozenCaches(
        position_snapshot=loaded,
        primary_snapshot=(loaded if loaded.generation != "previous" else None),
        previous_snapshot=(loaded if loaded.generation == "previous" else None),
      )
    if not isinstance(loaded, NavigationDatabaseRestoreFrozenCaches):
      raise TypeError("snapshot loader returned an invalid type")
    return loaded

  def prepare(self) -> NavigationDatabaseRestoreExecution:
    if self._caches_loaded:
      return self._execution
    self._caches_loaded = True
    try:
      loaded = self._snapshot_loader(self._receiver_fingerprint)
      frozen = self._normalize_frozen_caches(loaded)
    except Exception as exc:
      frozen = NavigationDatabaseRestoreFrozenCaches(None, None, None)
      self._position_error = f"snapshot_load:{_bounded_error(exc)}"
    self._frozen_caches = frozen
    self._position_snapshot = frozen.position_snapshot
    self._candidate_identities = tuple(NavigationDatabaseRestoreCandidateIdentity.from_snapshot(candidate) for candidate in frozen.database_candidates)

    if self._persisted_candidate_identities:
      if self._candidate_identities != self._persisted_candidate_identities:
        self._fail_closed("snapshot_identity_changed_within_boot")
        self._persist_state()
        self._execution = self._build_execution()
        return self._execution
    elif self._candidate_identities:
      if not self._persist_state():
        self._fail_closed("boot_state:candidate_identity_persist_failed")

    if not frozen.database_candidates and self._controller.pending:
      self._controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_NO_USABLE_CACHE)
      self._persist_state()

    self._execution = self._build_execution()
    return self._execution

  def send_position_once(
    self,
    send_message: Callable[[bytes], object],
  ) -> NavigationDatabaseRestoreExecution:
    if not callable(send_message):
      raise ValueError("send_message must be callable")
    self.prepare()
    snapshot = self._position_snapshot
    if snapshot is None or self._position_claimed:
      return self._execution
    self._position_claimed = True
    if not self._persist_state():
      self._position_error = "boot_state:position_claim_persist_failed"
      self._execution = self._build_execution()
      return self._execution

    self._position_attempted = True
    try:
      message = build_position_assistance_message(
        latitude_e7=snapshot.latitude_e7,
        longitude_e7=snapshot.longitude_e7,
        altitude_cm=snapshot.altitude_cm,
        position_accuracy_cm=snapshot.position_accuracy_cm,
      )
      send_message(message)
    except Exception as exc:
      self._position_error = _bounded_error(exc)
      self._position_succeeded = False
    else:
      self._position_succeeded = True
    self._execution = self._build_execution()
    return self._execution

  def close_restore_window_unverified(self) -> bool:
    """Persist a terminal timeout skip while GNSS remains stopped."""
    if self._controller.pending and not self._controller.restore_attempted:
      self._controller.skip(
        NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
      )
    persisted = self._persist_state()
    self._execution = self._build_execution(self._last_authorized_time)
    return persisted

  def claim_acquisition_start(self) -> bool:
    """Durably close the DBD window before sending GNSS START."""
    if self._controller.acquisition_started:
      return self._state_persistence_error is None
    if self._controller.pending and not self._controller.restore_attempted:
      self._controller.skip(
        NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
      )
    self._controller.note_acquisition_started()
    persisted = self._persist_state()
    self._execution = self._build_execution(self._last_authorized_time)
    return persisted

  def note_acquisition_started(self) -> bool:
    return self.claim_acquisition_start()

  def claim_yuma_transmission(self) -> bool:
    """Durably consume the boot's YUMA/DBD choice before receiver I/O."""
    if self._yuma_sent:
      return self._state_persistence_error is None
    self._yuma_sent = True
    if self._controller.pending and not self._controller.restore_attempted:
      self._controller.skip(
        NavigationDatabaseRestoreDisposition.SKIPPED_YUMA_ALREADY_SENT
      )
    persisted = self._persist_state()
    self._execution = self._build_execution(self._last_authorized_time)
    return persisted

  def note_yuma_sent(self) -> bool:
    return self.claim_yuma_transmission()

  @staticmethod
  def _cache_age(
    snapshot: NavigationDatabaseRestoreSnapshot,
    now_utc: datetime,
  ) -> float | None:
    try:
      age = (now_utc - snapshot.saved_at_utc).total_seconds()
    except (OverflowError, TypeError, ValueError):
      return None
    return float(age) if isfinite(age) else None

  def _effective_cache_age(
    self,
    snapshot: NavigationDatabaseRestoreSnapshot,
    authorized_time: AuthorizedTime,
  ) -> float | None:
    nominal_age = self._cache_age(snapshot, authorized_time.utc)
    uncertainty = authorized_time.uncertainty_seconds
    observed_boottime = authorized_time.observed_boottime_seconds
    if (
      nominal_age is None
      or isinstance(uncertainty, bool)
      or not isinstance(uncertainty, (int, float))
      or not isfinite(float(uncertainty))
      or float(uncertainty) < 0.0
      or isinstance(observed_boottime, bool)
      or not isinstance(observed_boottime, (int, float))
      or not isfinite(float(observed_boottime))
      or float(observed_boottime) < 0.0
    ):
      return None
    try:
      current_boottime = self._boottime_reader()
    except Exception:
      return None
    if (
      isinstance(current_boottime, bool)
      or not isinstance(current_boottime, (int, float))
      or not isfinite(float(current_boottime))
      or float(current_boottime) < float(observed_boottime)
    ):
      return None
    elapsed = float(current_boottime) - float(observed_boottime)
    effective_age = nominal_age + float(uncertainty) + elapsed
    return effective_age if isfinite(effective_age) else None

  def _select_database_snapshot(
    self,
    authorized_time: AuthorizedTime,
  ) -> tuple[NavigationDatabaseRestoreSnapshot | None, float | None]:
    assert is_current_independent_network_time(authorized_time)
    assert self._frozen_caches is not None
    candidates = self._frozen_caches.database_candidates
    ages = {
      candidate.generation: self._effective_cache_age(candidate, authorized_time)
      for candidate in candidates
    }

    if self._persisted_cache_generation is not None:
      matching = [
        candidate
        for candidate in candidates
        if (candidate.generation == self._persisted_cache_generation and candidate.saved_at_utc == self._persisted_cache_saved_at_utc)
      ]
      if len(matching) != 1:
        return None, None
      selected = matching[0]
      return selected, ages[selected.generation]

    eligible = [
      candidate
      for candidate in candidates
      if (ages[candidate.generation] is not None and 0.0 <= ages[candidate.generation] <= NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS)
    ]
    if not eligible:
      valid_ages = [age for age in ages.values() if age is not None and age >= 0.0]
      if len(valid_ages) == len(candidates) and valid_ages:
        return None, min(valid_ages)
      return None, None

    if len(eligible) == 1:
      selected = eligible[0]
      return replace(
        selected,
        selection_reason=f"trusted_age_only_eligible:{selected.generation}",
      ), ages[selected.generation]

    inventory = self._frozen_caches.inventory
    if inventory is None:
      selected = next(
        (candidate for candidate in eligible if candidate.generation == "primary"),
        eligible[0],
      )
      return replace(
        selected,
        selection_reason="trusted_age_primary_tiebreak",
      ), ages[selected.generation]

    eligible_generations = {candidate.generation for candidate in eligible}

    def filtered(inspection: CacheFileInspection) -> CacheFileInspection:
      if inspection.generation in eligible_generations:
        return inspection
      return CacheFileInspection(
        generation=inspection.generation,
        path=inspection.path,
        state=CacheFileState.ABSENT,
      )

    selection = NavigationCacheStore.select_inventory(
      CacheInventory(
        primary=filtered(inventory.primary),
        previous=filtered(inventory.previous),
      ),
      age_evidence=CacheAgeEvidence.TRUSTED_UTC,
    )
    if selection is None:
      return None, None
    selected = next(candidate for candidate in eligible if candidate.generation == selection.generation)
    return replace(
      selected,
      selection_reason=f"trusted_age:{selection.reason}",
    ), ages[selected.generation]

  def validate_database_write_boundary(self, frame_index: int) -> None:
    """Revalidate trusted age and acquisition after the UART pre-send drain."""
    if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
      raise ValueError("database frame index must be a non-negative int")
    if not self._controller.restore_attempted or self._controller.terminal:
      raise CacheValidationError("DBD receiver write is outside an active restore attempt")
    if self._controller.acquisition_started:
      raise CacheValidationError("DBD receiver write blocked: GNSS acquisition already started")
    snapshot = self._database_snapshot
    authorized_time = self._last_authorized_time
    if snapshot is None or authorized_time is None or not is_current_independent_network_time(authorized_time):
      raise CacheValidationError("DBD receiver write blocked: trusted-age evidence is unavailable")
    cache_age_seconds = self._effective_cache_age(snapshot, authorized_time)
    self._last_cache_age_seconds = cache_age_seconds
    if cache_age_seconds is None or cache_age_seconds < 0.0:
      raise CacheValidationError("DBD receiver write blocked: trusted cache age is unverified")
    if cache_age_seconds > NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS:
      raise CacheValidationError("DBD receiver write blocked: trusted cache age expired")

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
    self._last_authorized_time = authorized_time
    if self._controller.terminal:
      self._execution = self._build_execution(authorized_time)
      return self._execution

    cache_age_seconds = None
    if authorized_time is not None and is_current_independent_network_time(authorized_time):
      selected, cache_age_seconds = self._select_database_snapshot(authorized_time)
      if selected is not None:
        self._database_snapshot = selected
        self._persisted_cache_generation = selected.generation
        self._persisted_cache_saved_at_utc = selected.saved_at_utc
      self._last_cache_age_seconds = cache_age_seconds

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

    snapshot = self._database_snapshot
    if snapshot is None:
      self._controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)
      self._last_execution_error = "eligible_database_snapshot_missing"
      self._last_failure_phase = "selection"
      self._persist_state()
      self._execution = self._build_execution(authorized_time)
      return self._execution

    if not self._persist_state():
      self._controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)
      self._last_execution_error = "restore_claim_persist_failed"
      self._last_failure_phase = "state_persistence"
      self._execution = self._build_execution(authorized_time)
      return self._execution

    accepted: set[int] = set()
    initial_failures: list[NavigationDatabaseRestoreFrameFailure] = []
    retry_accepted: list[int] = []
    permanent_failures: list[NavigationDatabaseRestoreFrameFailure] = []
    retry_frames: list[tuple[int, bytes]] = []
    write_attempts = 0
    execution_error = None
    failure_phase = None

    try:
      failure_phase = "initial_pass"
      for index, frame in enumerate(snapshot.database_frames):
        write_attempts += 1
        try:
          send_database_message(frame, index)
          accepted.add(index)
        except Exception as exc:
          kind = _classify_failure(exc)
          failure = NavigationDatabaseRestoreFrameFailure(
            frame_index=index,
            attempt=1,
            kind=kind,
            error=_bounded_error(exc),
          )
          initial_failures.append(failure)
          if kind.retryable:
            retry_frames.append((index, frame))
          else:
            permanent_failures.append(failure)

      if retry_frames:
        if self._retry_delay_seconds:
          failure_phase = "retry_delay"
          self._sleeper(self._retry_delay_seconds)

        failure_phase = "retry_pass"
        for index, frame in retry_frames:
          write_attempts += 1
          try:
            send_database_message(frame, index)
            accepted.add(index)
            retry_accepted.append(index)
          except Exception as exc:
            permanent_failures.append(
              NavigationDatabaseRestoreFrameFailure(
                frame_index=index,
                attempt=2,
                kind=_classify_failure(exc),
                error=_bounded_error(exc),
              )
            )
    except Exception as exc:
      execution_error = _bounded_error(exc)
    succeeded = execution_error is None and bool(snapshot.database_frames) and len(accepted) == len(snapshot.database_frames) and not permanent_failures
    self._controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED if succeeded else NavigationDatabaseRestoreDisposition.WRITE_FAILED)
    self._persist_state()

    self._last_initial_failures = tuple(initial_failures)
    self._last_retry_accepted_indexes = tuple(retry_accepted)
    self._last_permanent_failures = tuple(permanent_failures)
    self._last_execution_error = execution_error
    self._last_failure_phase = None if succeeded else failure_phase
    self._last_accepted_frame_count = len(accepted)
    self._last_write_attempt_count = write_attempts
    self._execution = self._build_execution(authorized_time)
    return self._execution

  def _build_execution(
    self,
    authorized_time: AuthorizedTime | None = None,
  ) -> NavigationDatabaseRestoreExecution:
    snapshot = self._database_snapshot
    cache_age_seconds = self._last_cache_age_seconds
    effective_quality = None
    if snapshot is not None:
      current_network_time = authorized_time if (authorized_time is not None and is_current_independent_network_time(authorized_time)) else None
      age_evidence = CacheAgeEvidence.TRUSTED_UTC if current_network_time is not None else CacheAgeEvidence.UNVERIFIED
      if current_network_time is not None:
        cache_age_seconds = self._effective_cache_age(snapshot, current_network_time)
      effective_quality = effective_restored_navigation_quality(
        snapshot.quality,
        snapshot.saved_at_utc,
        (current_network_time.utc if current_network_time is not None else None),
        age_evidence,
      )

    return NavigationDatabaseRestoreExecution(
      disposition=self._controller.disposition,
      total_frame_count=(len(snapshot.database_frames) if snapshot else 0),
      accepted_frame_count=self._last_accepted_frame_count,
      database_write_attempt_count=self._last_write_attempt_count,
      initial_failures=self._last_initial_failures,
      retry_accepted_indexes=self._last_retry_accepted_indexes,
      permanent_failures=self._last_permanent_failures,
      execution_error=self._last_execution_error,
      failure_phase=self._last_failure_phase,
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
