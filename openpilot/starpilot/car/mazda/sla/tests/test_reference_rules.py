"""filter-dev ce9225e178's Mazda SLA rules, to the frame, through the port's planner, card session and selfdrived alerts.

Frames are 10 ms, plannerd ticks every 5th. Map and camera agree unless a test says otherwise (Combined policy: two
agreeing sources decide), the offset is +20 % (25 -> 30, 30 -> 36). The session sees a new limit only after the
resolver has held it LIMIT_SETTLE (2 s); a prompt is open PRE_ACTIVE_GUARD_PERIOD (5 s); a press in its direction up
to LATE_ANSWER_PERIOD (1.5 s) after it closed is still its answer; an unanswered prompt asks again REPROMPT_COOLDOWN
(25 s) after it closed, at most REPROMPT_MAX (2) times.
"""
import pytest

from openpilot.cereal import log

from openpilot.starpilot.car.mazda.sla import AssistState
from openpilot.starpilot.car.mazda.sla import arbiter as A
from openpilot.starpilot.car.mazda.sla.mirror import CAP_FALL_RATE, SpeedLimitAssistMirror
from openpilot.starpilot.car.mazda.sla.resolver import Sources, SpeedLimitResolver
from openpilot.starpilot.car.mazda.sla.tests.rig import CP, MPH, Rig, frame, spans

KPH_PER_MPH = 1.609344


def drive(n, limit_at, *, dash=36, v_mph=33., presses=(), enabled_at=lambda i: True, vision_at=None, gas_at=lambda i: False):
  """Run n frames. limit_at(i) -> mph both sources read; presses: {frame: [(button, pressed)]}. A driver press steps the
  recorded dash by one on its press edge, as the Mazda ECU does for a tap; the SET walk moves it through the rig's model."""
  rig = Rig()
  presses = dict(presses)
  for i in range(n):
    lim = limit_at(i)
    btn = presses.get(i, ())
    for name, pressed in btn:  # the ECU steps the dash on the driver's press edge
      if pressed and name in ('accelCruise', 'decelCruise'):
        dash += 1 if name == 'accelCruise' else -1
    rig.step(frame(i, v_mph=v_mph, dash=dash, map_mph=lim, vision_mph=vision_at(i) if vision_at else lim, enabled=enabled_at(i),
                   btn=btn, gas=gas_at(i)))
  return rig.rows


def tap(at, button='decelCruise', hold=8):
  return {at: [(button, True)], at + hold: [(button, False)]}


def first(rows, pred, start=0):
  return next(i for i in range(start, len(rows)) if pred(rows[i]))


def prompt_frame(rows, start=0):
  return first(rows, lambda r: r.state == AssistState.preActive, start)


# ---------------------------------------------------------------- settle, prompt, timeout

def test_engage_on_the_limit_forms_silently():
  rows = drive(600, lambda i: 30, dash=36)
  assert rows[-1].state == AssistState.active and round(rows[-1].cap * MPH) == 36
  assert spans(rows) == []


def test_engage_off_the_limit_prompts_after_settle():
  rows = drive(800, lambda i: 25, dash=36)
  p = prompt_frame(rows)
  # the resolver's limit must have been seen on 200 card frames (2 s, frame 0 counts as the first) before the session uses it
  assert p == 199
  (start, end, text, kind), = spans(rows)
  assert text == 'Press - for 30 mph' and kind == 'slcConfirmPrompt/warning'
  assert rows[p].ts <= start <= rows[p].ts + 0.05   # selfdrived shows it at plannerd's next tick


@pytest.mark.parametrize('hold_frames, prompts', [(195, 0), (205, 1)])
def test_a_reading_shorter_than_settle_never_prompts(hold_frames, prompts):
  change = 1000
  rows = drive(2400, lambda i: 25 if change <= i < change + hold_frames else 30, dash=36)
  assert rows[change - 1].state == AssistState.active
  assert sum(1 for s in spans(rows) if s[2].startswith('Press')) == prompts


def open_prompt(n=3000, **kw):
  """Active at 36 on a 30, then the limit drops to 25 at frame 1000: a "Press - for 30" opens ~2 s later."""
  return drive(n, lambda i: 30 if i < 1000 else 25, dash=36, **kw)


def test_prompt_times_out_after_5s_exactly():
  rows = open_prompt()
  p = prompt_frame(rows, 1000)
  assert all(r.state == AssistState.preActive for r in rows[p:p + 500])
  assert rows[p + 500].state == AssistState.inactive
  (start, end, text, _), = [s for s in spans(rows) if s[2].startswith('Press')]
  assert text == 'Press - for 30 mph' and end - start == pytest.approx(5.0, abs=0.06)
  assert rows[-1].cap == 255. and rows[-1].plan_cap is None


@pytest.mark.parametrize('offset, confirmed', [
  (1, True), (499, True),          # inside the prompt (a press on the opening frame is judged against the pre-frame state)
  (500, True),                     # the timeout frame: the press is classified from the frame-start (prompt) state
  (501, True), (650, True),        # LATE_ANSWER_PERIOD: 1.5 s after the prompt closed
  (651, False),                    # one frame later: a plain - press
])
def test_minus_answer_window(offset, confirmed):
  rows0 = open_prompt()
  p = prompt_frame(rows0, 1000)
  rows = open_prompt(presses=tap(p + offset))
  end = rows[p + offset + 60]
  if confirmed:
    assert end.state == AssistState.active and round(end.cap * MPH) == 30
    assert round(end.setpoint / KPH_PER_MPH) == 30   # a - confirm adopts the target into the setpoint (ICBM)
    assert any(s[2] == 'Adjusting to 30 mph' for s in spans(rows))
  else:
    assert end.state == AssistState.inactive and end.cap == 255.
    assert round(end.setpoint / KPH_PER_MPH) == 35   # one plain step down from 36
    assert not any(s[2].startswith('Adjusting') for s in spans(rows))


def test_opposite_press_declines():
  rows0 = open_prompt()
  p = prompt_frame(rows0, 1000)
  rows = open_prompt(presses=tap(p + 100, 'accelCruise'))
  r = rows[p + 160]
  assert r.state == AssistState.inactive and r.cap == 255.
  assert round(r.setpoint / KPH_PER_MPH) == 37   # the + steps the setpoint as a plain press
  assert not any(s[2].startswith('Adjusting') for s in spans(rows))


# ---------------------------------------------------------------- re-asks

def test_unanswered_prompt_asks_twice_more_every_30s():
  rows = open_prompt(n=14000)
  starts = [s[0] for s in spans(rows) if s[2] == 'Press - for 30 mph']
  assert len(starts) == 3
  assert starts[1] - starts[0] == pytest.approx(30.0, abs=0.06)   # 5 s prompt + 25 s cooldown (the 2 s hold ran meanwhile)
  assert starts[2] - starts[1] == pytest.approx(30.0, abs=0.06)


def test_no_reask_while_slow_or_on_the_gas():
  # 15 mph is not near min(SET 36, 30) - 5; gas pressed blocks the re-ask too
  slow = open_prompt(n=8000, v_mph=15.)
  gas = open_prompt(n=8000, gas_at=lambda i: i > 1500)
  assert len([s for s in spans(slow) if s[2].startswith('Press')]) == 1
  assert len([s for s in spans(gas) if s[2].startswith('Press')]) == 1


# ---------------------------------------------------------------- confirm, follow, dismiss, flicker

def test_confirm_follow_press_is_still_the_confirm():
  rows0 = open_prompt()
  p = prompt_frame(rows0, 1000)
  presses = {**tap(p + 50), **tap(p + 250)}   # the second - while the dash still walks down: owned, no dismiss
  rows = open_prompt(presses=presses)
  assert rows[p + 320].state == AssistState.active and round(rows[p + 320].cap * MPH) == 30
  late = {**tap(p + 50), **tap(p + 50 + 520)}  # after CONFIRM_FOLLOW_PERIOD (5 s) a - is a dismiss
  rows = open_prompt(presses=late)
  assert rows[p + 600].state == AssistState.inactive and rows[p + 600].cap == 255.


def test_press_while_active_dismisses_and_clears_cap_same_frame():
  rows = drive(1500, lambda i: 30, dash=36, presses=tap(1000, 'accelCruise'))
  assert rows[999].state == AssistState.active
  assert rows[1000].state == AssistState.inactive and rows[1000].cap == 255.
  assert any(r.plan_cap is None for r in rows[1000:1006])
  assert round(rows[1100].setpoint / KPH_PER_MPH) == 37
  assert spans(rows) == []


def test_camera_flicker_keeps_the_offer_and_the_confirmed_walk():
  """A 1 s camera reading of 40 (the map's 25 stays) while the prompt is open, and again while the confirmed SET walk
  runs: the prompt keeps its text and target, the session its 30 cap, the walk keeps going down (LIMIT_SETTLE)."""
  def vision(i):
    return 40 if 1300 <= i < 1400 or 1480 <= i < 1580 else (30 if i < 1000 else 25)
  rows0 = open_prompt(vision_at=vision)
  p = prompt_frame(rows0, 1000)
  assert p < 1300 < 1400 < p + 500
  assert {s[2] for s in spans(rows0) if s[2].startswith('Press')} == {'Press - for 30 mph'}
  rows = open_prompt(vision_at=vision, presses=tap(1450))
  assert {s[2] for s in spans(rows) if s[2].startswith('Press')} == {'Press - for 30 mph'}
  assert all(r.state == AssistState.active and round(r.cap * MPH) == 30 for r in rows[1460:1700])
  walk = [r.send for r in rows[1460:1700] if r.send != 'none']
  assert walk and set(walk) == {'decrease'}
  assert rows[1700].dash == 30


def test_no_up_walk_while_prompting():
  """A + offer: the SET walk never presses + while the prompt is open (arbiter.gate_send_button)."""
  rows = drive(2000, lambda i: 30 if i < 1000 else 35, dash=36, v_mph=36.)
  prompt_frame(rows, 1000)   # a prompt opens
  assert spans(rows)[0][2] == 'Press + for 42 mph'
  assert all(r.send != 'increase' for r in rows if r.prompting)


def test_plus_confirm_walks_set_up_after_prompt_closes():
  rows0 = drive(2000, lambda i: 30 if i < 1000 else 35, dash=36, v_mph=36.)
  p = prompt_frame(rows0, 1000)
  rows = drive(3000, lambda i: 30 if i < 1000 else 35, dash=36, v_mph=36., presses=tap(p + 50, 'accelCruise'))
  assert rows[p + 60].state == AssistState.active and round(rows[p + 60].setpoint / KPH_PER_MPH) == 42
  assert rows[-1].dash == 42
  assert any(s[2] == 'Adjusting to 42 mph' for s in spans(rows))


def test_disengage_clears_the_prompt():
  rows0 = open_prompt()
  p = prompt_frame(rows0, 1000)
  rows = open_prompt(enabled_at=lambda i: not p + 100 <= i < p + 400)
  assert rows[p + 100].state == AssistState.disabled
  assert not rows[p + 101].text
  assert all(r.send == 'none' for r in rows[p + 101:p + 400])


def test_set_walk_never_goes_below_the_ecu_minimum():
  """A 15 mph zone with the offset off (target 15) and SET at 19: the walk stops at the ECU's 30 km/h floor (reference
  controller: v_cruise_cluster > v_cruise_min), it does not press - forever."""
  from openpilot.starpilot.car.mazda.sla.tests.rig import Rig as R
  rig = R(offset=lambda lim: 0.)
  for i in range(3000):
    lim = 25 if i < 1000 else 15
    rig.step(frame(i, v_mph=22., dash=26 if i == 0 else rig.dash.dash, map_mph=lim, vision_mph=lim))
  p = prompt_frame(rig.rows, 1000)
  rig2 = R(offset=lambda lim: 0.)
  presses = tap(p + 50)
  dash = 25
  for i in range(3000):
    lim = 25 if i < 1000 else 15
    dash -= sum(1 for _, pressed in presses.get(i, ()) if pressed)
    rig2.step(frame(i, v_mph=22., dash=dash, map_mph=lim, vision_mph=lim, btn=presses.get(i, ())))
  tail = rig2.rows[-600:]
  assert min(r.dash for r in rig2.rows) == 19
  assert all(r.send == 'none' for r in tail)


# ---------------------------------------------------------------- alert presentation and chimes

def test_alert_presentation():
  rows0 = open_prompt()
  p = prompt_frame(rows0, 1000)
  rows = open_prompt(presses=tap(p + 50))
  prompt = next(r for r in rows if r.text.startswith('Press'))
  adjust = next(r for r in rows if r.text.startswith('Adjusting'))
  small = log.SelfdriveState.AlertSize.small
  assert prompt.alert_type == 'slcConfirmPrompt/warning' and prompt.size == small and prompt.duration == 10  # frames: 0.1 s, re-raised every frame while open
  assert adjust.alert_type == 'slcAdjusting/warning' and adjust.size == small and adjust.duration == 500  # frames: 5 s
  adj = [s for s in spans(rows) if s[2].startswith('Adjusting')]
  assert len(adj) == 1 and adj[0][1] - adj[0][0] == pytest.approx(5.0, abs=0.02)


@pytest.mark.parametrize('pack', ['stock', 'starpilot', 'custom'])
def test_soundd_plays_the_selected_prompt_sound(tmp_path, pack):
  """Both SLA alerts play StarPilot's ordinary prompt: the selected sound pack's prompt file at Prompt Volume, sample for
  sample what any other prompt alert plays. The alerts come from the real selfdrived pipeline, the audio from soundd's
  real selection and callback."""
  import wave
  from pathlib import Path
  from types import SimpleNamespace as NS
  import numpy as np
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui import soundd
  from openpilot.starpilot.audio.sound_pack import read_wav

  rows0 = open_prompt()
  rows = open_prompt(presses=tap(prompt_frame(rows0, 1000) + 50))
  sla = [next(r for r in rows if r.text.startswith('Press')), next(r for r in rows if r.text.startswith('Adjusting'))]
  assert [r.alert_type.split('/')[0] for r in sla] == ['slcConfirmPrompt', 'slcAdjusting']

  params = Params(str(tmp_path / 'params'))
  params.put('SoundPack', pack, block=True)
  Path(params.get_param_path('PromptVolume')).write_bytes(b'40')
  custom = (np.sin(np.arange(9000) / 7.) * 9000).astype(np.int16)
  (tmp_path / 'packs/custom/sounds').mkdir(parents=True)
  with wave.open(str(tmp_path / 'packs/custom/sounds/prompt.wav'), 'wb') as target:
    target.setnchannels(1)
    target.setsampwidth(2)
    target.setframerate(48000)
    target.writeframes(custom.tobytes())
  assets = Path(soundd.__file__).resolve().parents[1] / 'assets'
  expected = {'stock': read_wav(assets / 'sounds/warning.wav'), 'starpilot': read_wav(assets / 'sounds_starpilot/warning_1.wav'),
              'custom': custom.astype(np.float32) / 32768}[pack]

  daemon = soundd.Soundd(params, pack_root=tmp_path / 'packs')
  daemon.refresh_saved_volumes(10.)

  class State:
    logMonoTime, valid, alive = {}, {'selfdriveState': True}, {'selfdriveState': True}

    def __init__(self, sound, alert_type):
      self.updated = {'selfdriveState': True}
      self.state = NS(alertSound=NS(raw=int(sound)), alertType=alert_type)

    def __getitem__(self, service):
      return self.state

  def play(sound, alert_type):
    daemon.current_alert = soundd.AudibleAlert.none
    daemon.get_audible_alert(State(sound, alert_type))
    out = np.zeros((4096, 1), np.float32)
    daemon.callback(out, 4096, None, None)
    return daemon.current_alert, out[:, 0].copy()

  ordinary_alert, ordinary = play(soundd.AudibleAlert.prompt, 'someOtherPrompt/warning')
  assert ordinary_alert == soundd.AudibleAlert.prompt
  n = min(4096, len(expected))
  np.testing.assert_array_equal(ordinary[:n], expected[:n] * np.float32(.4))
  for row in sla:
    assert row.sound == log.SelfdriveState.AudibleAlert.prompt
    alert, out = play(row.sound, row.alert_type)
    assert alert == soundd.AudibleAlert.prompt and soundd.ALERT_VOLUME_KEYS[alert] == 'PromptVolume'
    np.testing.assert_array_equal(out, ordinary)
  assert set(soundd.sound_list) <= set(soundd.AudibleAlert.schema.enumerants.values()) | {soundd.CRITICAL_MAX}


# ---------------------------------------------------------------- resolver lookahead and planner ramp

@pytest.mark.parametrize('distance, expect_mph', [(290., 40), (270., 25)])
def test_lower_limit_ahead_applies_inside_the_braking_distance(distance, expect_mph):
  """A lower map limit ahead replaces the current one once the distance is within what coasting at 0.35 m/s^2
  (COMMIT_FRAC x OP_LONG_COAST_A) needs: 40 -> 25 mph from 40 mph is 278 m."""
  res = SpeedLimitResolver(CP, offset_fn=lambda lim: 0.)
  t = 10**12
  for k in range(3):
    res.update(40 / MPH, Sources(t + k * 50_000_000, map_limit=40 / MPH, map_ahead=25 / MPH, map_ahead_distance=distance,
                                 map_ns=t + k * 50_000_000, gps_fix=True, gps_ns=t + k * 50_000_000))
  assert round(res.speed_limit * MPH) == expect_mph


@pytest.mark.parametrize('vision, map_, car, expect', [
  (40, 30, 30, 30),   # two of three agree: the map's 30 beats the camera ranked first
  (40, 30, 0, 40),    # no agreement: Vision, Map, Car order
  (0, 30, 40, 40),    # a new map limit the car contradicts waits NEW_MAP_WAIT_S (5 s): the car's reading stands meanwhile
  (25, 0, 25, 25),
])
def test_combined_vote(vision, map_, car, expect):
  res = SpeedLimitResolver(CP, offset_fn=lambda lim: 0.)
  t = 10**12
  for k in range(3):
    now = t + k * 50_000_000
    res.update(30 / MPH, Sources(now, car=car / MPH, vision=vision / MPH, vision_ns=now, map_limit=map_ / MPH, map_ns=now,
                                 gps_fix=True, gps_ns=now))
  assert round(res.speed_limit * MPH) == expect


def test_combined_vote_backs_a_lower_limit_ahead():
  """The map brakes early for a 25 ahead (inside the coast distance) and the car reads 25 too: the two agree, so the
  25 wins over the camera's 40 although the map's own limit (30) agrees with nobody."""
  res = SpeedLimitResolver(CP, offset_fn=lambda lim: 0.)
  t = 10**12
  for k in range(3):
    now = t + k * 50_000_000
    res.update(40 / MPH, Sources(now, car=25 / MPH, vision=40 / MPH, vision_ns=now, map_limit=30 / MPH, map_ahead=25 / MPH,
                                 map_ahead_distance=100., map_ns=now, gps_fix=True, gps_ns=now))
  assert round(res.speed_limit * MPH) == 25


def test_mirror_ramps_a_lower_cap():
  m = SpeedLimitAssistMirror()
  from types import SimpleNamespace as NS
  m.update(NS(state=5, vCap=36 / MPH, announceCounter=1, setpointKph=58., target=30 / MPH), v_ego=36 / MPH)
  m.update(NS(state=5, vCap=30 / MPH, announceCounter=2, setpointKph=58., target=30 / MPH), v_ego=36 / MPH)
  assert m.output_v_target == pytest.approx(30 / MPH)
  assert CAP_FALL_RATE == 0.35  # m/s^2 (assist_mirror)
  assert m.plan_v_target == pytest.approx(36 / MPH - 0.35 * 0.05)
  for _ in range(400):
    m.update(NS(state=5, vCap=30 / MPH, announceCounter=2, setpointKph=58., target=30 / MPH), v_ego=36 / MPH)
  assert m.plan_v_target == pytest.approx(30 / MPH)


def test_reference_constants():
  assert (A.LIMIT_SETTLE, A.PRE_ACTIVE_GUARD_PERIOD, A.LATE_ANSWER_PERIOD, A.CONFIRM_FOLLOW_PERIOD) == (2., 5., 1.5, 5.)
  assert (A.REPROMPT_COOLDOWN, A.REPROMPT_HOLD, A.REPROMPT_MAX, A.REPROMPT_MARGIN, A.REPROMPT_NEAR) == (25., 2., 2, 3, 5)
  assert (A.DISABLED_GUARD_PERIOD, A.TURN_REPROMPT_WINDOW, A.TURN_MAP_SETTLE) == (0.5, 15., 1.)
