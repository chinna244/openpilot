from enum import StrEnum


class NavigationDatabaseRestoreDisposition(StrEnum):
  """Boot-scoped outcome for opaque MGA-DBD restoration."""

  PENDING = "pending"
  RESTORED = "restored"
  SKIPPED_EXPIRED = "skipped_expired"
  SKIPPED_UNVERIFIED = "skipped_unverified"
  SKIPPED_LATE_RECEIVER_TIME = "skipped_late_receiver_time"
  SKIPPED_ACQUISITION_ALREADY_STARTED = "skipped_acquisition_already_started"
  SKIPPED_RELIABLE_FIX = "skipped_reliable_fix"
  SKIPPED_YUMA_ALREADY_SENT = "skipped_yuma_already_sent"
  SKIPPED_NO_USABLE_CACHE = "skipped_no_usable_cache"
  WRITE_FAILED = "write_failed"

  @property
  def terminal(self) -> bool:
    return self is not NavigationDatabaseRestoreDisposition.PENDING

  @property
  def database_available(self) -> bool:
    return self is NavigationDatabaseRestoreDisposition.RESTORED

  @property
  def intentionally_skipped(self) -> bool:
    return self.name.startswith("SKIPPED_")

  @property
  def write_failed(self) -> bool:
    return self is NavigationDatabaseRestoreDisposition.WRITE_FAILED


class NavigationDatabaseRestoreBootController:
  def __init__(self) -> None:
    self._disposition = NavigationDatabaseRestoreDisposition.PENDING
    self._restore_attempted = False
    self._position_assistance_claimed = False
    self._acquisition_started = False

  @property
  def disposition(self) -> NavigationDatabaseRestoreDisposition:
    return self._disposition

  @property
  def pending(self) -> bool:
    return self._disposition is NavigationDatabaseRestoreDisposition.PENDING

  @property
  def terminal(self) -> bool:
    return self._disposition.terminal

  @property
  def restore_attempted(self) -> bool:
    return self._restore_attempted

  @property
  def acquisition_started(self) -> bool:
    return self._acquisition_started

  def claim_position_assistance(self) -> bool:
    if self._position_assistance_claimed:
      return False
    self._position_assistance_claimed = True
    return True

  def note_acquisition_started(self) -> None:
    self._acquisition_started = True

  def begin_restore_attempt(self) -> bool:
    if self.terminal or self._restore_attempted:
      return False
    self._restore_attempted = True
    return True

  def finish_restore(self, disposition: NavigationDatabaseRestoreDisposition) -> bool:
    if disposition not in (NavigationDatabaseRestoreDisposition.RESTORED, NavigationDatabaseRestoreDisposition.WRITE_FAILED):
      raise ValueError("restore completion must be RESTORED or WRITE_FAILED")
    if self.terminal or not self._restore_attempted:
      return False
    self._disposition = disposition
    return True

  def skip(self, disposition: NavigationDatabaseRestoreDisposition) -> bool:
    if not disposition.intentionally_skipped:
      raise ValueError("skip disposition must be intentional")
    if self.terminal or self._restore_attempted:
      return False
    self._disposition = disposition
    return True
