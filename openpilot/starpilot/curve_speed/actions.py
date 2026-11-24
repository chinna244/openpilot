"""Confirmed parked learning edits, shared by native settings and Galaxy."""

from collections.abc import Callable
from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import tempfile

from openpilot.starpilot.curve_speed import corrections as curve_corrections
from openpilot.starpilot.curve_speed.learning import LearnedCurve
from openpilot.starpilot.curve_speed.preferences import (
  DOCUMENT_KEY, LEGACY_KEY, MASTER_KEY, DOCUMENT_LIMIT,
  acquire_learning_lease, decode, learning_lock_path,
)
from openpilot.starpilot.saved_source import read_saved


ACTION_KEYS = frozenset(('curve_reset',))
SOURCE_KEYS = (DOCUMENT_KEY, LEGACY_KEY, MASTER_KEY)
# The Mazda Curve Speed Controller learns only its corrections: its page and reset own that document alone.
CORRECTIONS_KEY = curve_corrections.KEY
CORRECTIONS_LIMIT = curve_corrections.DOC_LIMIT
CORRECTIONS_SOURCE_KEYS = (CORRECTIONS_KEY, MASTER_KEY)
UNIT_CORRECTIONS_RAW = json.dumps({"version": 1, "scales": list(curve_corrections.UNIT)}, separators=(',', ':')).encode()


@dataclass(frozen=True)
class LearningSnapshot:
  sources: tuple[tuple[str, bytes | None], ...]
  readable: bool
  valid: bool
  reason: str
  progress: float | None
  comfort: float | None
  has_samples: bool
  resettable: bool
  corrections: tuple[float, ...] | None = None  # None: the saved corrections are malformed


@dataclass(frozen=True)
class ActionResult:
  committed: bool
  reason: str


def _sources(params) -> tuple[tuple[tuple[str, bytes | None], ...], bool]:
  reads = [(key, *read_saved(params, key, 8 if key == MASTER_KEY else DOCUMENT_LIMIT)) for key in SOURCE_KEYS]
  return tuple((key, raw) for key, raw, _readable in reads), all(readable for _key, _raw, readable in reads)


def _raw(sources, key):
  return dict(sources)[key]


def _loaded(sources):
  current, legacy = _raw(sources, DOCUMENT_KEY), _raw(sources, LEGACY_KEY)
  raw = current if current is not None else legacy
  try:
    document = decode(raw)
    if raw is not None and (document is None or current is not None and
                           (type(document) is not dict or document.get('version') != 1)):
      raise ValueError('invalid saved document')
    return LearnedCurve.load(document)
  except (ValueError, UnicodeDecodeError, RecursionError, OverflowError):
    return LearnedCurve.load(False)


def _idle(params) -> bool:
  """Read-only hint; the action must acquire its own exclusive lease."""
  try:
    fd = os.open(learning_lock_path(params), os.O_RDONLY)
  except FileNotFoundError:
    return True
  except OSError:
    return False
  try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return True
  except OSError:
    return False
  finally:
    os.close(fd)


def _corrections_sources(params) -> tuple[tuple[tuple[str, bytes | None], ...], bool, bool]:
  """Corrections first (the row's source), then the master. An unreadable corrections document stays resettable."""
  corrections_raw, corrections_readable = read_saved(params, CORRECTIONS_KEY, CORRECTIONS_LIMIT)
  master, master_readable = read_saved(params, MASTER_KEY, 8)
  return ((CORRECTIONS_KEY, corrections_raw), (MASTER_KEY, master)), master_readable, corrections_readable


def corrections_snapshot(params) -> LearningSnapshot:
  sources, readable, corrections_readable = _corrections_sources(params)
  corrections_raw, master = sources[0][1], sources[1][1]
  scales = curve_corrections.parse(corrections_raw) if corrections_readable else None
  learned = corrections_raw is not None and (scales is None or scales != curve_corrections.UNIT)
  idle = _idle(params)
  allowed = readable and master in (None, b'0', b'1') and idle
  reason = ('unreadable_learning' if not readable else 'invalid_curve_switch' if master not in (None, b'0', b'1') else
            'learning_active' if not idle else 'valid')
  return LearningSnapshot(sources, readable, readable, reason, None, None, learned, allowed and learned, scales)


def _reset_corrections(params, expected_sources, authorized) -> ActionResult:
  sources, readable, _corrections_readable = _corrections_sources(params)
  if not readable or sources != expected_sources:
    return ActionResult(False, 'source_changed')
  if sources[1][1] not in (None, b'0', b'1'):
    return ActionResult(False, 'invalid_curve_switch')
  if sources[0][1] is None:
    return ActionResult(False, 'no_saved_learning')

  def read_sources():
    actual, actual_readable, _ = _corrections_sources(params)
    return actual, actual_readable
  return _replace_document(params, CORRECTIONS_KEY, UNIT_CORRECTIONS_RAW, CORRECTIONS_LIMIT, read_sources, expected_sources,
                           authorized)


def learning_snapshot(params, *, corrections: bool = False) -> LearningSnapshot:
  if corrections:
    return corrections_snapshot(params)
  sources, readable = _sources(params)
  current, legacy, master = _raw(sources, DOCUMENT_KEY), _raw(sources, LEGACY_KEY), _raw(sources, MASTER_KEY)
  loaded = _loaded(sources)
  valid = readable and loaded.valid
  idle = _idle(params)
  allowed = readable and master in (None, b'0', b'1') and idle
  reason = ('unreadable_learning' if not readable else 'invalid_curve_switch' if master not in (None, b'0', b'1') else
            'learning_active' if not idle else loaded.reason)
  return LearningSnapshot(sources, readable, valid, reason,
                          loaded.curve.progress if valid else None,
                          loaded.curve.average_comfort if valid else None,
                          bool(loaded.curve.document()['buckets']) if valid else False,
                          allowed and (bool(loaded.curve.document()['buckets']) if valid else current is not None or legacy is not None))


def apply_learning(params, kind: str, expected_sources: tuple[tuple[str, bytes | None], ...], *,
                   authorized: Callable[[], bool], corrections: bool = False) -> ActionResult:
  """Caller owns confirmation. Fresh authority and exact sources are rechecked.
  corrections: the vehicle's Curve Speed Controller is the Mazda vision solver; only its corrections are reset.

  Lock ordering is learning lease, then the ordinary Params root lock. Both
  acquisitions are nonblocking, keeping a parked UI action bounded under load.
  Legacy bytes and the saved master are never changed. Reset writes an empty
  canonical document so preserved legacy learning cannot become active again.
  """
  if kind not in ACTION_KEYS or not authorized():
    return ActionResult(False, 'unavailable')
  if corrections:
    return _reset_corrections(params, expected_sources, authorized)
  sources, readable = _sources(params)
  if not readable or sources != expected_sources:
    return ActionResult(False, 'source_changed')
  current, legacy, master = _raw(sources, DOCUMENT_KEY), _raw(sources, LEGACY_KEY), _raw(sources, MASTER_KEY)
  if master not in (None, b'0', b'1'):
    return ActionResult(False, 'invalid_curve_switch')
  if kind == 'curve_reset' and current is None and legacy is None:
    return ActionResult(False, 'no_saved_learning')
  document = LearnedCurve().document()
  raw = json.dumps(document, allow_nan=False, separators=(',', ':')).encode()
  return _replace_document(params, DOCUMENT_KEY, raw, DOCUMENT_LIMIT, lambda: _sources(params), expected_sources, authorized)


def _replace_document(params, key: str, raw: bytes, limit: int, read_sources, expected_sources, authorized) -> ActionResult:
  lease = acquire_learning_lease(params)
  if lease is None:
    return ActionResult(False, 'learning_active')
  lock_fd = None
  temporary = None
  committed = False
  try:
    destination = Path(params.get_param_path(key))
    root = destination.parent.parent
    with tempfile.NamedTemporaryFile(prefix='.tmp_curve_action_', dir=root, delete=False) as staging:
      temporary = staging.name
      staging.write(raw)
      staging.flush()
      os.fsync(staging.fileno())
    lock_fd = os.open(root / '.lock', os.O_CREAT | os.O_RDONLY, 0o775)
    try:
      fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return ActionResult(False, 'storage_busy')
    actual, readable = read_sources()
    if not readable or actual != expected_sources:
      return ActionResult(False, 'source_changed')
    if not authorized():
      return ActionResult(False, 'unavailable')
    os.replace(temporary, destination)
    temporary = None
    committed = True
    directory_fd = os.open(destination.parent, os.O_RDONLY)
    try:
      os.fsync(directory_fd)
    finally:
      os.close(directory_fd)
    actual, readable = read_saved(params, key, limit)
    if not readable or actual != raw:
      return ActionResult(True, 'verification_failed')
    return ActionResult(True, 'saved')
  except (OSError, ValueError, TypeError, RuntimeError):
    return ActionResult(committed, 'sync_failed' if committed else 'write_failed')
  finally:
    if lock_fd is not None:
      os.close(lock_fd)
    os.close(lease)
    if temporary is not None:
      try:
        os.unlink(temporary)
      except FileNotFoundError:
        pass
