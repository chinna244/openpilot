"""Display-only radar ownership evidence; never grants driving authority."""
from opendbc.car.mazda.stock_ecu import ENGAGES_NORMALLY, StockEcuState
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert

KEY = 'MazdaStockEcuStatus'
MAX_AGE_NS = 3_000_000_000
LABELS = {
  'starting': 'Preparing alpha long', 'parkToTakeOver': 'Waiting for the next stop',
  'stockCruiseOn': 'Factory cruise has control', 'ready': 'Alpha long ready',
  'restoring': 'Restoring factory radar', 'restored': 'Factory radar restored',
  'failed': 'Radar transition failed', 'notNeeded': 'Factory radar has control',
}


def fresh(record, now_ns):
  return bool(isinstance(record, dict) and type(record.get('state')) is str and record['state'] in set(StockEcuState) and
              type(record.get('radarOff')) is bool and type(record.get('alphaRequested')) is bool and
              type(record.get('fingerprint')) is str and type(record.get('monoTime')) is int and
              0 < record['monoTime'] <= now_ns and now_ns - record['monoTime'] <= MAX_AGE_NS)


def read(params):
  return params.get(KEY)


def status_text(record, restore_failed, now_ns, fingerprint=None):
  if restore_failed:
    return 'Radar restore failed; restart the vehicle'
  if not fresh(record, now_ns) or fingerprint is not None and record['fingerprint'] != fingerprint:
    return 'Waiting for current radar status'
  label = LABELS[record['state']]
  return label + ('; factory AEB off' if record['radarOff'] else '')


def current(record, cp, now_ns):
  return bool(cp is not None and fresh(record, now_ns) and record['fingerprint'] == cp.carFingerprint)


def notice(native, cp, record, restore_failed, now_ns):
  if native.size != AlertSize.NONE or cp is None or getattr(cp, 'brand', '') != 'mazda' or cp.notCar or cp.dashcamOnly:
    return native
  if restore_failed or cp.openpilotLongitudinalControl and current(record, cp, now_ns) and record['state'] == StockEcuState.FAILED:
    # a failed radar is never openpilot's: whatever radarOff says, AEB is off and nothing owns it
    return OnroadAlert(AlertSize.SMALL, 'Mazda radar restore failed', 'Factory AEB off; restart the vehicle',
                       alert_type='mazdaRadarRestore/permanent')
  if not cp.openpilotLongitudinalControl or not current(record, cp, now_ns):
    return native
  if record['radarOff']:
    detail = 'Restoring factory radar' if record['state'] == 'restoring' else 'Mazda alpha longitudinal has radar control'
    return OnroadAlert(AlertSize.SMALL, 'Factory AEB off', detail, alert_type='mazdaRadarOwnership/permanent')
  if record['alphaRequested'] and record['state'] in ('starting', 'parkToTakeOver'):
    return OnroadAlert(AlertSize.SMALL, 'Alpha long requested', 'Takes over at the next stop',
                       alert_type='mazdaRadarTakeover/permanent')
  return native


# A SET/RES press the car will not engage on while alpha long's radar transition is not done.
SET_SPEED_BUTTONS = ('accelCruise', 'resumeCruise', 'decelCruise', 'setCruise')
NOT_READY_TEXT = {
  StockEcuState.STARTING: ('Alpha Long Not Ready', 'Stay stopped: factory AEB turning off'),
  StockEcuState.PARK_TO_TAKE_OVER: ('Alpha Long Not Ready', 'Stop first: takes over at the next stop'),
  StockEcuState.RESTORING: ('Factory Cruise Restarting', 'Wait about 12 s'),
  StockEcuState.FAILED: ('Factory Radar Not Restored', 'Restart the car'),
}
UNKNOWN_TEXT = ('Alpha Long Not Ready', 'Radar status unavailable')


def stock_ecu_events(cp, record, restore_failed, button_events, now_ns):
  """(failed, not_ready_state) for selfdrived: display evidence only, never driving authority.
  not_ready_state is the state to name (None when the press needs no explanation; 'unknown' when stale)."""
  if cp is None or cp.brand != 'mazda' or not cp.openpilotLongitudinalControl:
    return False, None
  state = record['state'] if current(record, cp, now_ns) else None
  failed = bool(restore_failed or state == StockEcuState.FAILED)
  pressed = any(be.pressed and str(be.type) in SET_SPEED_BUTTONS for be in button_events)
  if not pressed or state in ENGAGES_NORMALLY or state == StockEcuState.STOCK_CRUISE_ON:
    return failed, None
  return failed, state if state is not None else 'unknown'


def not_ready_text(state):
  return NOT_READY_TEXT.get(state, UNKNOWN_TEXT)
