"""Route 00000002--c9d55e7093 t=493.160: ignition-off teardown raised controlsMismatch (immediate disable) because
pandad withdrew aolSafetyWire (empty, invalid) as both ignitions went off with nothing enabled or active."""
import inspect
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.selfdrived import native_missing_event

E = log.OnroadEvent.EventName


def panda(line=False, can=False):
  return SimpleNamespace(ignitionLine=line, ignitionCan=can)


def test_route_493s_parked_ignition_off_is_no_mismatch():
  assert native_missing_event(wire_seen=True, enabled=False, active=False, lost_active_aol=False,
                              panda_states=[panda()], device_started=False) is None


@pytest.mark.parametrize('started', [True, None])
def test_panda_ignition_off_alone_is_not_key_off(started):
  assert native_missing_event(wire_seen=True, enabled=False, active=False, lost_active_aol=False,
                              panda_states=[panda()], device_started=started) == E.controlsMismatch


@pytest.mark.parametrize('kw', [dict(enabled=True), dict(active=True), dict(lost_active_aol=True)])
def test_real_mismatch_with_ignition_off_still_disables(kw):
  base = dict(wire_seen=True, enabled=False, active=False, lost_active_aol=False, panda_states=[panda()], device_started=False)
  assert native_missing_event(**{**base, **kw}) == E.controlsMismatch


@pytest.mark.parametrize('states', [[panda(line=True)], [panda(can=True)], [panda(), panda(can=True)], [], None])
def test_ignition_on_or_unknown_still_disables(states):
  assert native_missing_event(wire_seen=True, enabled=False, active=False, lost_active_aol=False,
                              panda_states=states, device_started=False) == E.controlsMismatch


def test_startup_before_any_wire_stays_comm_issue():
  assert native_missing_event(wire_seen=False, enabled=False, active=False, lost_active_aol=False,
                              panda_states=[panda(line=True)], device_started=True) == E.commIssue
  assert native_missing_event(wire_seen=False, enabled=True, active=False, lost_active_aol=False,
                              panda_states=[panda(line=True)], device_started=True) == E.controlsMismatch


def test_selfdrived_uses_the_helper_for_a_missing_native():
  source = inspect.getsource(selfdrived.SelfdriveD)
  assert 'native_missing_event(' in source and 'native_missing_at_startup' not in source


# Through SelfdriveD's call site, with the freshness each key-off input needs.
MS = 1_000_000
T_PANDA, T_DEVICE, T_EVENT = 493_131 * MS, 493_146 * MS, 493_160 * MS  # route 00000002--c9d55e7093 recorded stamps


class FakeSM:
  def __init__(self):
    self.data = {'pandaStates': [panda()], 'deviceState': SimpleNamespace(started=False)}
    self.logMonoTime = {'pandaStates': T_PANDA, 'deviceState': T_DEVICE, 'aolSafetyWire': T_PANDA - 30 * MS}
    self.seen = {k: True for k in ('pandaStates', 'deviceState', 'aolSafetyWire')}
    self.valid = {k: True for k in self.seen}
    self.alive = {k: True for k in self.seen}

  def __getitem__(self, name):
    return self.data[name]


def call_site(sm, now_ns=T_EVENT, *, enabled=False, active=False, lat=False, lon=False):
  sd = SimpleNamespace(sm=sm, enabled=enabled, active=active)
  return selfdrived.SelfdriveD.missing_native_event(sd, now_ns, lost_active_aol=lat or lon)


def test_call_site_route_key_off_sequence_raises_nothing():
  assert call_site(FakeSM()) is None


def test_call_site_route_before_key_off_still_raises():
  sm = FakeSM()  # 492.934: deviceState started, panda CAN ignition still on
  sm.data = {'pandaStates': [panda(can=True)], 'deviceState': SimpleNamespace(started=True)}
  assert call_site(sm, now_ns=493_000 * MS) == E.controlsMismatch


@pytest.mark.parametrize('service', ['pandaStates', 'deviceState'])
@pytest.mark.parametrize('fault', ['stale', 'invalid', 'not_alive', 'future', 'never_seen'])
def test_call_site_needs_fresh_valid_key_off_evidence(service, fault):
  sm = FakeSM()
  if fault == 'stale':
    sm.logMonoTime[service] = T_EVENT - (int(3 * 1e9 / (10 if service == 'pandaStates' else 2)) + MS)
  elif fault == 'invalid':
    sm.valid[service] = False
  elif fault == 'not_alive':
    sm.alive[service] = False
  elif fault == 'future':
    sm.logMonoTime[service] = T_EVENT + MS
  else:
    sm.seen[service] = False
  assert call_site(sm) == E.controlsMismatch


@pytest.mark.parametrize('service', ['pandaStates', 'deviceState'])
def test_call_site_freshness_boundary(service):
  sm = FakeSM()
  sm.logMonoTime[service] = T_EVENT - int(3 * 1e9 / (10 if service == 'pandaStates' else 2))
  assert call_site(sm) is None


@pytest.mark.parametrize('key_on', ['panda_line', 'panda_can', 'device_started'])
def test_call_site_key_on_after_shutdown_raises(key_on):
  sm = FakeSM()
  if key_on == 'device_started':
    sm.data['deviceState'] = SimpleNamespace(started=True)
  else:
    sm.data['pandaStates'] = [panda(line=key_on == 'panda_line', can=key_on == 'panda_can')]
  assert call_site(sm, now_ns=T_EVENT + 10 * MS) == E.controlsMismatch


@pytest.mark.parametrize('kw', [dict(enabled=True), dict(active=True), dict(lat=True), dict(lon=True)])
def test_call_site_active_axis_mismatch_stays_fail_closed(kw):
  assert call_site(FakeSM(), **kw) == E.controlsMismatch


def test_call_site_startup_before_any_wire_is_comm_issue():
  sm = FakeSM()
  sm.seen['aolSafetyWire'] = False
  sm.data = {'pandaStates': [panda(line=True)], 'deviceState': SimpleNamespace(started=True)}
  assert call_site(sm) == E.commIssue


def test_step_uses_the_call_site():
  source = inspect.getsource(selfdrived.SelfdriveD)
  assert 'missing_event = self.missing_native_event(now_ns, lost_active_aol=bool(lost_active_aol))' in source
  # the missing-native frame keeps setting step()'s own flag, which the downstream fault handling reads
  call = source.index('missing_event = self.missing_native_event(')
  assert 'lost_active_aol = self.aol_axis_decision.lateral_active or self.aol_axis_decision.longitudinal_active' in \
    source[call - 400:call]


def test_route_04_key_off_mismatch_diagnostic_logs_once_and_changes_nothing(monkeypatch):
  """Route 00000004--0ed7a0ac2e t=434.068: log what the step saw (here: deviceState started=False not yet received)."""
  logged = []
  monkeypatch.setattr(selfdrived.cloudlog, 'event', lambda name, **kw: logged.append((name, kw)))
  sm = FakeSM()
  sm.data['deviceState'] = SimpleNamespace(started=True)
  sd = SimpleNamespace(sm=sm, enabled=False, active=False)
  for _ in range(3):
    assert selfdrived.SelfdriveD.missing_native_event(sd, T_EVENT, lost_active_aol=False) == E.controlsMismatch
  assert len(logged) == 1 and logged[0][0] == 'aol_native_missing_mismatch'
  kw = logged[0][1]
  assert kw['device_started'] is True and kw['panda_proof'] and kw['ignition'] == [(False, False)]
  assert kw['deviceState']['age_ms'] == (T_EVENT - T_DEVICE) / 1e6
  # key-off proven: nothing raised, episode ends; a later episode logs again
  sm.data['deviceState'] = SimpleNamespace(started=False)
  assert selfdrived.SelfdriveD.missing_native_event(sd, T_EVENT, lost_active_aol=False) is None
  sm.data['deviceState'] = SimpleNamespace(started=True)
  selfdrived.SelfdriveD.missing_native_event(sd, T_EVENT, lost_active_aol=False)
  assert len(logged) == 2


def test_key_off_diagnostic_skips_real_authority_loss(monkeypatch):
  logged = []
  monkeypatch.setattr(selfdrived.cloudlog, 'event', lambda name, **kw: logged.append(name))
  sd = SimpleNamespace(sm=FakeSM(), enabled=True, active=True)
  assert selfdrived.SelfdriveD.missing_native_event(sd, T_EVENT, lost_active_aol=True) == E.controlsMismatch
  assert not logged
