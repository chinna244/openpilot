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


def test_aeb_off_and_failed_restore_are_visible_without_replacing_existing_alerts():
  current = cp()
  alert = notice(OnroadAlert(), current, record(), False, NOW)
  assert 'AEB' in alert.text1 and 'off' in alert.text1.lower()
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


def test_requested_takeover_explains_the_next_stop():
  data = record()
  data.update(state='parkToTakeOver', radarOff=False, alphaRequested=True)
  alert = notice(OnroadAlert(), cp(), data, False, NOW)
  assert 'stop' in (alert.text1 + alert.text2).lower()


def test_settings_status_is_bound_to_the_current_vehicle():
  assert 'Waiting' in status_text(record(), False, NOW, 'other-car')
  assert 'AEB off' in status_text(record(), False, NOW, str(CAR.MAZDA_CX5_2022))
