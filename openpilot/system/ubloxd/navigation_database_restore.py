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
