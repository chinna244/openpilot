import pytest

from openpilot.system.ubloxd.navigation_database_restore import (
  NavigationDatabaseRestoreBootController,
  NavigationDatabaseRestoreDisposition,
)


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


def test_boot_controller_starts_pending() -> None:
  controller = NavigationDatabaseRestoreBootController()
  assert controller.pending
  assert not controller.terminal
  assert not controller.restore_attempted


def test_position_assistance_is_claimed_once() -> None:
  controller = NavigationDatabaseRestoreBootController()
  assert controller.claim_position_assistance()
  assert not controller.claim_position_assistance()


def test_acquisition_state_is_latched() -> None:
  controller = NavigationDatabaseRestoreBootController()
  controller.note_acquisition_started()
  assert controller.acquisition_started


def test_restore_attempt_is_one_shot() -> None:
  controller = NavigationDatabaseRestoreBootController()
  assert controller.begin_restore_attempt()
  assert not controller.begin_restore_attempt()
  assert controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED)
  assert controller.terminal


def test_started_attempt_cannot_become_skip() -> None:
  controller = NavigationDatabaseRestoreBootController()
  assert controller.begin_restore_attempt()
  assert not controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_RELIABLE_FIX)
  assert controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)


def test_finish_restore_requires_started_attempt() -> None:
  controller = NavigationDatabaseRestoreBootController()

  assert not controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED)
  assert controller.pending
  assert not controller.restore_attempted


def test_intentional_skip_is_terminal_and_persisted() -> None:
  controller = NavigationDatabaseRestoreBootController()

  assert controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED)
  assert controller.terminal
  assert controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED
  assert not controller.restore_attempted


def test_terminal_disposition_cannot_change() -> None:
  controller = NavigationDatabaseRestoreBootController()

  assert controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_RELIABLE_FIX)
  assert not controller.skip(NavigationDatabaseRestoreDisposition.SKIPPED_YUMA_ALREADY_SENT)
  assert not controller.begin_restore_attempt()
  assert not controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED)
  assert controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_RELIABLE_FIX


def test_finish_restore_rejects_noncompletion_disposition() -> None:
  controller = NavigationDatabaseRestoreBootController()
  assert controller.begin_restore_attempt()

  with pytest.raises(ValueError):
    controller.finish_restore(NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED)

  assert controller.pending
  assert controller.restore_attempted
  assert controller.finish_restore(NavigationDatabaseRestoreDisposition.WRITE_FAILED)


def test_skip_rejects_nonintentional_disposition() -> None:
  controller = NavigationDatabaseRestoreBootController()

  for disposition in (
    NavigationDatabaseRestoreDisposition.PENDING,
    NavigationDatabaseRestoreDisposition.RESTORED,
    NavigationDatabaseRestoreDisposition.WRITE_FAILED,
  ):
    with pytest.raises(ValueError):
      controller.skip(disposition)

  assert controller.pending
  assert not controller.restore_attempted


def test_terminal_state_remains_on_same_controller_instance() -> None:
  controller = NavigationDatabaseRestoreBootController()

  assert controller.begin_restore_attempt()
  assert controller.finish_restore(NavigationDatabaseRestoreDisposition.RESTORED)

  same_controller = controller
  assert same_controller.terminal
  assert same_controller.restore_attempted
  assert not same_controller.begin_restore_attempt()
  assert same_controller.disposition is NavigationDatabaseRestoreDisposition.RESTORED
