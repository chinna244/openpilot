from openpilot.system.ubloxd.navigation_database_restore import NavigationDatabaseRestoreDisposition


def test_disposition_values_are_stable() -> None:
  assert {item.name: item.value for item in NavigationDatabaseRestoreDisposition} == {
    "PENDING": "pending",
    "RESTORED": "restored",
    "SKIPPED_EXPIRED": "skipped_expired",
    "SKIPPED_UNVERIFIED": "skipped_unverified",
    "SKIPPED_LATE_RECEIVER_TIME": "skipped_late_receiver_time",
    "SKIPPED_ACQUISITION_ALREADY_STARTED": "skipped_acquisition_already_started",
    "SKIPPED_RELIABLE_FIX": "skipped_reliable_fix",
    "SKIPPED_YUMA_ALREADY_SENT": "skipped_yuma_already_sent",
    "SKIPPED_NO_USABLE_CACHE": "skipped_no_usable_cache",
    "WRITE_FAILED": "write_failed",
  }


def test_only_pending_is_nonterminal() -> None:
  assert not NavigationDatabaseRestoreDisposition.PENDING.terminal
  assert all(item.terminal for item in NavigationDatabaseRestoreDisposition if item is not NavigationDatabaseRestoreDisposition.PENDING)


def test_only_restored_makes_database_available() -> None:
  assert NavigationDatabaseRestoreDisposition.RESTORED.database_available
  assert all(not item.database_available for item in NavigationDatabaseRestoreDisposition if item is not NavigationDatabaseRestoreDisposition.RESTORED)


def test_terminal_meanings_are_unambiguous() -> None:
  for item in NavigationDatabaseRestoreDisposition:
    meanings = (item.database_available, item.intentionally_skipped, item.write_failed)
    assert sum(meanings) == int(item.terminal)
