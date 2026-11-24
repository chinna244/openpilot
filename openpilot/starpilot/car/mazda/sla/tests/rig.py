"""Offline rig for the Mazda SLA port: plannerd (MazdaSlaPlanner), card (MazdaSlaSession), selfdrived
(SelfdriveD.add_mazda_slc_events -> MAZDA_EVENTS callbacks -> AlertManager), in their production
order, on frame dicts. The factory SET is a model of the Mazda ECU: driver presses move it as given (a frame's 'dash'
delta while a driver button is down or just released), the SET walk's taps register every 0.2 s with every fourth
slot dropped, the hold stream at 4 mph/s.
"""
import gzip
import json
from pathlib import Path
from types import SimpleNamespace as NS

from openpilot.starpilot.car.mazda.sla import AssistState
from openpilot.starpilot.car.mazda.sla.planner import MazdaSlaPlanner
from openpilot.starpilot.car.mazda.sla.session import MazdaSlaSession
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import ET, MazdaEvents
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

MPH = 2.2369363
DT_NS = 10_000_000
CP = NS(brand='mazda', openpilotLongitudinalControl=True, pcmCruise=True, passive=False, dashcamOnly=False)
DRIVER_BUTTONS = ('accelCruise', 'decelCruise', 'setCruise', 'resumeCruise')


class _Params:
  def get_bool(self, key, **kw):
    return key == 'UbloxAvailable'


class FakeSM:
  def __init__(self):
    self.d, self.logMonoTime, self.seen, self.valid, self.alive = {}, {}, {}, {}, {}

  def __getitem__(self, k):
    return self.d[k]

  def put(self, k, v, t):
    self.d[k], self.logMonoTime[k], self.seen[k], self.valid[k], self.alive[k] = v, t, True, True, True


class Btn:
  __slots__ = ('type', 'pressed')

  def __init__(self, t, p):
    self.type, self.pressed = t, p


class Dash:
  def __init__(self):
    self.dash, self.held, self.release_t, self.last_rec = None, set(), -1e9, None
    self.last_send_t, self.tap_slot, self.stream_acc = -1e9, 0, 0.

  def step(self, f, send, hold):
    rec, ts = round(f['dash']), f['ts']
    for name, pressed in f['btn']:
      if name in DRIVER_BUTTONS:
        (self.held.add if pressed else self.held.discard)(name)
        if not pressed:
          self.release_t = ts
    if self.dash is None or not f['avail'] or rec <= 0:
      self.dash = self.last_rec = rec
      return
    if (self.held or ts - self.release_t < 0.6) and rec != self.last_rec:
      self.dash += rec - self.last_rec
    self.last_rec = rec
    if send != 'none' and not self.held:
      step = 1 if send == 'increase' else -1
      if hold:
        if ts - self.last_send_t > 0.1:
          self.last_send_t = ts
          self.stream_acc += 0.4
          if self.stream_acc >= 1.:
            self.stream_acc -= 1.
            self.dash += step
      elif ts - self.last_send_t > 0.2:
        self.tap_slot += 1
        if [1, 1, 0, None][self.tap_slot % 4] is not None:
          self.dash += step
          self.last_send_t = ts
    else:
      self.stream_acc = 0.


class Rig:
  def __init__(self, offset=lambda lim: lim * 0.2, enabled_setting=True, card=False):
    """card=True runs card's own Car.mazda_sla_update (slcState freshness, unfilled mazdaSla) instead of the session directly;
    a frame with stall=True has no plannerd tick (plannerd stalled or restarting)."""
    self.card = card
    self._sm_card = None
    self.planner = MazdaSlaPlanner(CP, _Params(), offset)
    self.session = MazdaSlaSession(CP)
    self.enabled_setting = enabled_setting
    self.sm = FakeSM()
    self.sd = NS(sm=FakeSM(), enabled=False, mazda_events=MazdaEvents(), mazda_slc_alerts=True, is_metric=False)
    self.AM = AlertManager()
    self.dash = Dash()
    self.sla = None
    self.session_msg = None
    self.last_vcruise = 255.
    self.frame = 0
    self.rows = []

  def step(self, f):
    self.frame += 1
    now = f['t']
    if f['tick'] and not f.get('stall', False):
      ml, ahead, ahead_d, mt = f['map']
      vl, vt = f['vision']
      if mt:
        self.sm.put('mapdOut', NS(speedLimit=ml, nextSpeedLimit=ahead, nextSpeedLimitDistance=ahead_d), mt)
      if vt:
        self.sm.put('slcVisionObservation', NS(vision=NS(status='valid' if vl > 0 else 'absent', speedMps=vl)), vt)
      self.sm.put('slcDashboardObservation', NS(status='valid' if f['car'] > 0 else 'absent', speedMps=f['car'],
                                                  validUntilMonoTime=now + 1_000_000_000), now)
      if f['gps'][1]:
        self.sm.put('gpsLocationExternal', NS(hasFix=f['gps'][0]), f['gps'][1])
      self.sm.put('carState', NS(vEgo=f['v'], vCruise=self.last_vcruise), now)
      if self.session_msg is not None:
        self.sm.put('starpilotCarState', NS(mazdaSlaSession=self.session_msg), now)
      for k in ('mapdOut', 'slcVisionObservation', 'gpsLocationExternal', 'starpilotCarState'):
        self.sm.seen.setdefault(k, False)
      cap = self.planner.step(self.sm, now)
      self.sla = NS()
      self.planner.fill(self.sla)
      self.cap = cap
      self.sd.sm.put('slcState', NS(mazdaSla=self.sla), now)
    if self.sla is None:
      return None
    sla, session = self.sla, self.session
    dash_ms = self.dash.dash / MPH if self.dash.dash else 0.
    cs = NS(available=f['avail'], enabled=f['cruise_on'], speed=dash_ms, speedCluster=dash_ms, standstill=f['cruise_standstill'])
    CS = NS(vEgo=f['v'], gasPressed=f['gas'], brakePressed=f['brake'], standstill=f['standstill'], steeringAngleDeg=f['steer'],
            cruiseState=cs, buttonEvents=[Btn(n, p) for n, p in f['btn']], vCruise=0., vCruiseCluster=0.)
    override, cancel, resume = f['cc_cruise']
    if self.card:
      send = self._card_step(CS, f, now)
    else:
      session.prompting_published = sla.assistState == AssistState.preActive
      send = session.update(CS, cc_enabled=f['cc_enabled'], override=override, cancel=cancel, resume=resume, resolver=sla,
                            servo_v_target=sla.servoVTarget, limiter_active=sla.limiterActive, enabled_setting=self.enabled_setting,
                            is_metric=False)
      if session.v_cruise_kph < 255:
        CS.vCruise, CS.vCruiseCluster = float(session.v_cruise_kph), float(session.v_cruise_cluster_kph)
    self.session_msg = NS()
    session.fill(self.session_msg)
    self.last_vcruise = CS.vCruise if session.v_cruise_kph < 255 else 255.
    self.dash.step(f, send, session.hold)
    sd = self.sd
    sd.enabled = f['cc_enabled']
    sd.mazda_events.clear()
    SelfdriveD.add_mazda_slc_events(sd, CS, now_ns=now)
    alerts = sd.mazda_events.create_alerts([ET.WARNING] if sd.enabled else [], callback_args=[CP, CS, sd.sm, False, 0, None])
    self.AM.add_many(self.frame, alerts)
    self.AM.process_alerts(self.frame, set() if sd.enabled else {ET.WARNING})
    a = self.AM.current_alert
    row = NS(ts=f['ts'], state=AssistState(session.arbiter.state), cap=session.arbiter.v_cap, target=sla.speedLimitFinalLast,
             offer=session.arbiter.target,
             plan_cap=self.cap, setpoint=session.v_cruise_kph, dash=self.dash.dash, send=send, hold=session.hold,
             text=(a.alert_text_1 + ' ' + a.alert_text_2).strip().replace('\n', ' '), size=a.alert_size, alert_type=a.alert_type,
             sound=a.audible_alert, duration=a.duration, prompting=session.arbiter.prompting, announce=session.arbiter.announce_counter,
             btn=f['btn'], enabled=f['cc_enabled'])
    self.rows.append(row)
    return row

  def _card_step(self, CS, f, now):
    from unittest.mock import patch
    from openpilot.selfdrive.car import card as card_mod

    class SM(dict):
      pass
    sm = SM(slcState=self.sd.sm['slcState'], carControl=NS(enabled=f['cc_enabled'], cruiseControl=NS(
      override=f['cc_cruise'][0], cancel=f['cc_cruise'][1], resume=f['cc_cruise'][2])))
    sm.seen = {'slcState': True}
    sm.logMonoTime = {'slcState': self.sd.sm.logMonoTime['slcState']}
    car = NS(sm=sm, mazda_sla=self.session, is_metric=False, slc_control_enabled=self.enabled_setting, can_log_mono_time=now)
    with patch.object(card_mod, 'REPLAY', True):
      card_mod.Car.mazda_sla_update(car, CS)
    return self.session.send_button

  def run(self, frames):
    for f in frames:
      self.step(f)
    return self.rows


def spans(rows, prefixes=('Press +', 'Press -', 'Adjusting to')):
  """Displayed SLA alerts: (start_s, end_s, text, alert_type)."""
  out, cur = [], None
  for r in rows:
    sla = r.text.startswith(prefixes)
    if cur is not None and (not sla or r.text != cur[2]):
      out.append((cur[0], r.ts, cur[2], cur[3]))
      cur = None
    if sla and cur is None:
      cur = (r.ts, None, r.text, r.alert_type)
  if cur is not None:
    out.append((cur[0], rows[-1].ts, cur[2], cur[3]))
  return out


def load_fixture(name='route08_inputs.json.gz'):
  fx = json.loads(gzip.decompress((Path(__file__).parent / name).read_bytes()))
  n = fx['n']

  def expand(col):
    out = [None] * n
    for j, (i, val) in enumerate(col):
      end = col[j + 1][0] if j + 1 < len(col) else n
      out[i:end] = [val] * (end - i)
    return out

  C = {k: expand(v) for k, v in fx['cols'].items()}
  ticks = set(fx['ticks'])
  btn: dict[int, list] = {}
  for i, button, p in fx['buttons']:
    btn.setdefault(i, []).append((button, p))

  def latest(lst):
    by, cur, out = {m[0]: m for m in lst}, None, []
    for i in range(n):
      cur = by.get(i, cur)
      out.append(cur)
    return out

  V, M, G = latest(fx['vision']), latest(fx['map']), latest(fx['gps'])
  t0 = 10**12
  frames = []
  for i in range(n):
    t = t0 + i * DT_NS
    v, m, g = V[i], M[i], G[i]
    frames.append({'t': t, 'ts': fx['start_s'] + i * 0.01, 'v': C['v'][i], 'standstill': C['standstill'][i], 'gas': C['gas'][i], 'brake': False,
                       'steer': C['steer'][i], 'dash': float(C['dash'][i]), 'cruise_on': C['cruise_on'][i], 'avail': C['avail'][i],
                       'btn': btn.get(i, []), 'tick': i in ticks, 'cc_enabled': C['cc_enabled'][i], 'cc_cruise': tuple(C['cc_cruise'][i]),
                       'cruise_standstill': C['cruise_standstill'][i],
                       'map': ((m[1] / MPH, m[2] / MPH, m[3], t0 + m[0] * DT_NS) if m else (0., 0., 0., 0)),
                       'vision': ((v[1] / MPH, t0 + v[0] * DT_NS) if v else (0., 0)), 'car': C['car'][i] / MPH,
                       'gps': ((g[1], t0 + g[0] * DT_NS) if g else (False, 0))})
  return frames


def frame(i, *, v_mph=25., dash=35, map_mph=0, vision_mph=0, car_mph=0, ahead_mph=0, ahead_m=0., enabled=True, btn=(), gas=False,
          steer=0., gps=True, t0=10**12, tick=None):
  """One synthetic frame at 10 ms; sources are fresh every tick (5th frame)."""
  t = t0 + i * DT_NS
  return {'t': t, 'ts': i * 0.01, 'v': v_mph / MPH, 'standstill': v_mph < 0.2, 'gas': gas, 'brake': False, 'steer': steer, 'dash': float(dash),
              'cruise_on': enabled, 'avail': True, 'btn': list(btn), 'tick': (i % 5 == 0) if tick is None else tick, 'cc_enabled': enabled,
              'cc_cruise': (False, False, False), 'cruise_standstill': False,
              'map': (map_mph / MPH, ahead_mph / MPH, ahead_m, t), 'vision': (vision_mph / MPH, t), 'car': car_mph / MPH, 'gps': (gps, t)}
