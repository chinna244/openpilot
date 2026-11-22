import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.starpilot.car.mazda.radar_status import notice, status_text
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert

NOW = 10_000_000_000


def cp():
  return CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], True, False, False)


def record(**changes):
  return dict(fingerprint=str(CAR.MAZDA_CX5_2022), state='ready', radarOff=True, alphaRequested=False, monoTime=NOW, **changes)


def test_failed_restore_is_visible_without_replacing_existing_alerts():
  current = cp()
  # develop shows the takeover as one-time events (selfdrived add_mazda_takeover_events), not a standing notice
  assert notice(OnroadAlert(), current, record(), False, NOW).size == AlertSize.NONE
  failed = notice(OnroadAlert(), current, None, True, NOW)
  assert 'restore' in failed.text1.lower() and 'restart' in failed.text2.lower()
  critical = OnroadAlert(AlertSize.FULL, 'TAKE CONTROL', 'Brake', True)
  assert notice(critical, current, record(), True, NOW) is critical
  assert 'failed' in status_text(None, True, NOW).lower()


@pytest.mark.parametrize('stamp', [0, NOW + 1, NOW - 3_000_000_001])
def test_missing_or_stale_status_cannot_claim_aeb_ownership(stamp):
  data = record()
  data['monoTime'] = stamp
  assert notice(OnroadAlert(), cp(), data, False, NOW).size == AlertSize.NONE


def test_requested_takeover_has_no_standing_notice():
  data = record()
  data.update(state='parkToTakeOver', radarOff=False, alphaRequested=True)
  assert notice(OnroadAlert(), cp(), data, False, NOW).size == AlertSize.NONE  # alphaLongNextStop says it once


def test_settings_status_is_bound_to_the_current_vehicle():
  assert 'Waiting' in status_text(record(), False, NOW, 'other-car')
  assert 'AEB off' in status_text(record(), False, NOW, str(CAR.MAZDA_CX5_2022))


def test_filter_handback_failure_does_not_claim_aeb_off():
  """The Smart CAN Filter keeps the radar online (radarOff False): a failed hand-back is a cruise problem, not AEB."""
  filtered = cp()
  filtered.safetyConfigs[-1].safetyParam |= 16
  alert = notice(OnroadAlert(), filtered, dict(record(), state='failed', radarOff=False), False, NOW)
  assert 'hand-back' in alert.text1.lower() and 'aeb' not in (alert.text1 + alert.text2).lower()
  # radar actually off: the AEB-off notice stands
  alert = notice(OnroadAlert(), filtered, dict(record(), state='failed', radarOff=True), False, NOW)
  assert 'aeb off' in alert.text2.lower()
  # unfiltered alpha long keeps the AEB-off notice
  alert = notice(OnroadAlert(), cp(), dict(record(), state='failed', radarOff=False), False, NOW)
  assert 'aeb off' in alert.text2.lower()
