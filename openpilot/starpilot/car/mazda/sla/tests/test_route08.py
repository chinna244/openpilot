"""Route 00000008--80148a3eeb (StarPilot 6d72f8516, 2026-11-24) through the Mazda SLA port, from the recorded source
readings and wheel presses to the alert, the session, the factory SET and the plan cap.

route08_inputs.json.gz holds the drive's SLA inputs on a 10 ms clock (times run ~0.4 s ahead of the log). Every value
asserted here is what pinned filter-dev (ce9225e178) produces on the same inputs (refsim2 replay, 0 frame differences).
On this drive StarPilot's mapd drops East Howard / South Iowa every 0.1-2 s and the camera reads 30 on South Iowa, so
the filter-dev resolver keeps the camera's 30 (+20 % offset: 36) and never settles the map's 25 for 2 s.
The deployed build (the log) showed 28 SLA alerts on these roads: offers flipping +36/-30 every 0.05-2 s, a 15 s prompt,
and a 30 cap that held the car at 29 mph after the driver set 35.
"""
import pytest

from openpilot.starpilot.car.mazda.sla import AssistState
from openpilot.starpilot.car.mazda.sla.tests.rig import MPH, Rig, load_fixture, spans

KPH_PER_MPH = 1.609344


@pytest.fixture(scope='module')
def route():
  frames = load_fixture()
  rows = Rig().run(frames)
  return frames, rows


def press_index(rows, name, after_s, pressed=True):
  return next(i for i, r in enumerate(rows) if r.ts >= after_s and (name, pressed) in r.btn)


def mph(ms):
  return round(ms * MPH)


def setpoint_mph(r):
  return round(r.setpoint / KPH_PER_MPH)


def test_whole_drive_alerts(route):
  """Three SLA alerts on the whole drive, as filter-dev: none on East Howard or South Iowa's conflicts."""
  _, rows = route
  got = [(round(a, 2), round(b - a, 2), text, kind.split('/')[0]) for a, b, text, kind in spans(rows)]
  assert got == [
    (65.47, 1.64, 'Press + for 36 mph', 'slcConfirmPrompt'),   # engaged at 33, confirmed by + after 1.64 s
    (67.16, 5.01, 'Adjusting to 36 mph', 'slcAdjusting'),
    (232.41, 5.01, 'Press + for 36 mph', 'slcConfirmPrompt'),  # re-engaged on South Iowa, unanswered: 5 s and gone
  ]


def test_east_howard_minus_is_a_manual_press(route):
  """144-149 s: Maps 25 (blanking) against Vision 30. The 30 holds (36 target), a 0.4 s 30-mph target flicker when
  the camera blinked never reaches the session (2 s settle), so no prompt. The driver's - dismisses the active 36
  session at the press frame and steps SET 36 -> 35 as a plain press."""
  _, rows = route
  i = press_index(rows, 'decelCruise', 147.)
  before, at = rows[i - 1], rows[i]
  assert before.state == AssistState.active and mph(before.cap) == 36 and mph(before.plan_cap) == 36
  assert not any(r.text.startswith('Press') for r in rows[i - 400:i + 400])
  assert at.state == AssistState.inactive and at.cap == 255.
  # plannerd's next tick drops the plan cap
  assert next(r for r in rows[i:i + 6] if r.plan_cap is None)
  after = rows[i + 20]
  assert after.state == AssistState.inactive and setpoint_mph(after) == 35 and after.dash == 35
  assert not any(r.text.startswith('Adjusting') for r in rows[i:i + 600])


def test_south_iowa_conflict_is_quiet_and_minus_presses_are_manual(route):
  """168-175 s: Vision 30 / Maps 25 flicker. No offer appears (the deployed build flashed "Press + for 36" four times
  for 0.07-0.23 s); the two - presses step the setpoint 35 -> 34 -> 33 and confirm nothing."""
  _, rows = route
  i1 = press_index(rows, 'decelCruise', 168.)
  i2 = press_index(rows, 'decelCruise', rows[i1].ts + 1.)
  window = [r for r in rows if 165. <= r.ts <= 175.]
  assert not any(r.text for r in window if r.text.startswith(('Press', 'Adjusting')))
  assert all(r.state == AssistState.inactive for r in window if r.enabled)
  assert setpoint_mph(rows[i1 + 20]) == 34 and setpoint_mph(rows[i2 + 20]) == 33


def test_south_iowa_reengage_prompt_times_out_once(route):
  """232-250 s: one "Press + for 36" for 5.0 s after the 0.5 s engage guard, unanswered, then quiet: the car is near
  standstill (the re-ask needs the car within 5 mph of the lower of SET 33 and 36, and waits 25 s). The deployed build
  showed it for 6.3 s and then flipped +36/-30 eight times."""
  _, rows = route
  i = press_index(rows, 'resumeCruise', 231.)
  prompts = [s for s in spans(rows) if rows[i].ts <= s[0] <= rows[i].ts + 40.]
  assert len(prompts) == 1
  start, end, text, _ = prompts[0]
  assert text == 'Press + for 36 mph'
  # the engage gate waits for the RES release with carControl enabled, then the 0.5 s guard, then plannerd's next tick
  released = press_index(rows, 'resumeCruise', rows[i].ts, pressed=False)
  engaged = next(r.ts for r in rows[released:] if r.enabled)
  assert 0.5 <= start - engaged <= 0.56
  assert end - start == pytest.approx(5.0, abs=0.02)
  after = [r for r in rows if end <= r.ts <= end + 20.]
  assert all(r.state == AssistState.inactive for r in after) and all(r.setpoint == rows[i + 100].setpoint for r in after)


def test_south_iowa_late_minus_walks_set_and_session_forms_silently(route):
  """272-283 s: the 30 (Vision 25 + Maps 25 agree) shows for 1.7 s and gives way to the held 36 until 281.9 s, so no
  offer is made; the driver's - at 282.25 is a plain press (no offer, nothing to confirm or answer late) and walks SET
  33 -> 30. Once the 30 has held 2 s the session forms on it with no alert (the dash already shows it) and caps the plan."""
  _, rows = route
  i = press_index(rows, 'decelCruise', 281.)
  assert not any(r.text.startswith(('Press', 'Adjusting')) for r in rows if 270. <= r.ts <= 292.)
  assert rows[i].state == AssistState.inactive
  j = next(k for k in range(i, i + 300) if rows[k].state == AssistState.active)
  assert setpoint_mph(rows[j]) == 30 and mph(rows[j].cap) == 30
  assert next(r for r in rows[j:j + 6] if r.plan_cap is not None and mph(r.plan_cap) == 30)


def test_howard_return_plus_takes_ownership(route):
  """330-368 s: session active at 30 on the 25 (+20 %). The driver's + is not toward the target (the dash is on it),
  so it dismisses the session at the press frame: the cap clears that frame, the plan cap at plannerd's next tick, and
  the setpoint follows the wheel to 35. The deployed build kept the 30 cap and held the car at 29 with SET 35."""
  _, rows = route
  i = press_index(rows, 'accelCruise', 329.)
  assert rows[i - 1].state == AssistState.active and mph(rows[i - 1].cap) == 30
  assert rows[i].state == AssistState.inactive and rows[i].cap == 255.
  k = next(k for k in range(i, i + 6) if rows[k].plan_cap is None)
  assert k - i <= 5
  assert setpoint_mph(rows[i + 100]) == 35
  # Howard's own limit (30 again on this side) flickers with the camera's 30/25 and never asks the dismissed session
  assert not any(r.text for r in rows[i:] if r.text.startswith(('Press', 'Adjusting')) and r.ts < 370.)
  assert all(r.plan_cap is None for r in rows[k:] if r.ts < 370.)
