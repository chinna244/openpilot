"""Mazda: one wheel press confirms an SLC offer and the factory SET walks to it.

Mazda keeps the factory cruise (pcmCruise): the dash SET is the body ECU's, moved only by
CRZ_BTNS presses. StarPilot's SLC answered a prompt press as SET+ = accept / SET- = reject
and then capped the planner only, so the dash kept the driver's SET (route
00000002--c9d55e7093: SET- rejected an offered 30 at 35, the ECU stepped 35->34; SET+ accepted
25 at 34 and the ECU stepped 34->35, planner capped at 25 under a 35 dash).

filter-dev (CruiseArbiter + ICBM) behaviour restored here:
- during an offer, a press TOWARD the offered target accepts it, a press away rejects it;
- after the accept shows up in slcState, the ICBM sender (opendbc mazda icbm.py, taps paced
  at the ECU's 200 ms registration floor) walks the factory SET onto the accepted target.
  The ECU's own one-step reaction to the confirming press only shortens the walk: the
  target is the accepted limit, never the SET that press produced.

A press is sent only while openpilot owns longitudinal control and slcState still carries
exactly the planner acknowledgment of the confirming press (acceptedMonoTime and value), and
ends as soon as the factory SET reaches the target. Any change of the accepted value (a new limit needs its own
confirm; a lower cap is the planner's), a physical cruise press after the confirm, cancel,
RES, disengagement, loss of longitudinal control, gas, brake, standstill, invalid CAN, a new
offer, a missing target or a dash that does not move ENDS the walk; nothing resumes it.
"""
from dataclasses import dataclass

from opendbc.car.structs import car
from openpilot.common.constants import CV

ButtonType = car.CarState.ButtonEvent.Type
CRUISE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise, ButtonType.cancel,
                  ButtonType.setCruise)

ACCEPT_BIND_NS = 2_000_000_000  # the accept must appear in slcState this soon after the press
REACT_NS = 300_000_000  # error must persist this long before the first press of a walk
STALL_NS = 2_000_000_000  # sending this long without the dash moving ends the session
MIN_SET_MPH = 20  # the body ECU's lowest SET; never walk below it


def eligible(CP) -> bool:
  return bool(CP.brand == 'mazda' and CP.pcmCruise and not CP.passive and not CP.dashcamOnly)


def display_conv(is_metric: bool) -> float:
  return CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH


def confirm_direction(offer_mps: float, set_mps: float, is_metric: bool) -> str | None:
  """'accel' when the offer is above the dash SET, 'decel' when below, None when equal or unknown."""
  if not (offer_mps > 0. and set_mps > 0.):
    return None
  conv = display_conv(is_metric)
  offer, current = round(offer_mps * conv), round(set_mps * conv)
  if offer > current:
    return 'accel'
  if offer < current:
    return 'decel'
  return None


def directional_answer(pressed: str, offer_mps: float, set_mps: float, is_metric: bool) -> str:
  """StarPilot SLC reads 'accel' as accept and 'decel' as reject. Map a Mazda press so that the
  press toward the offered target accepts and the press away rejects. With no direction (offer
  equals SET) the press keeps StarPilot's own meaning."""
  toward = confirm_direction(offer_mps, set_mps, is_metric)
  if toward is None:
    return pressed
  return 'accel' if pressed == toward else 'decel'


@dataclass
class _Armed:
  target: int  # display units
  press_ns: int


class MazdaSlaSetServo:
  NONE, INCREASE, DECREASE = 'none', 'increase', 'decrease'

  def __init__(self, is_metric: bool = False):
    self.is_metric = is_metric
    self.reset()

  def reset(self) -> None:
    self.armed: _Armed | None = None
    self.target: int | None = None
    self.press_ns = 0
    self.error_since_ns = 0
    self.sending_since_ns = 0
    self.last_set = None
    self.last_set_change_ns = 0
    self.confirmed: int | None = None
    self.bound: tuple[float, int] | None = None
    self.end_reason = ''

  @property
  def active(self) -> bool:
    return self.target is not None

  def arm(self, offer_mps: float, press_ns: int) -> None:
    """Card emitted an accept for this offer at press_ns: wait for slcState to carry it."""
    self.reset()
    self.armed = _Armed(round(offer_mps * display_conv(self.is_metric)), press_ns)
    self.press_ns = press_ns

  def _end(self, reason: str) -> None:
    self.reset()
    self.end_reason = reason

  def update(self, CS, *, enabled: bool, long_active: bool, cancel: bool, resume: bool, accepted_mps: float | None,
             accepted_ns: int, prompting: bool, now_ns: int, confirm_frame: bool = False) -> str:
    """One card frame. accepted_mps: slcState.acceptedAdjustedLimit when fresh and present, else None;
    accepted_ns: its acceptedMonoTime; confirm_frame: this frame's press is the confirm being armed.

    Any interruption ENDS the walk (review 2026-11-23: a walk paused through gas/standstill/invalid CAN resumed
    afterwards, and kept pressing for ~1 s after the accepted target disappeared). A press is only sent while
    openpilot owns longitudinal control and slcState still carries exactly the value the press confirmed."""
    conv = display_conv(self.is_metric)
    if self.armed is None and self.target is None:
      return self.NONE

    # physical cruise presses after the confirm: the driver takes the SET back
    if not confirm_frame and any(b.pressed and b.type in CRUISE_BUTTONS for b in CS.buttonEvents):
      self._end('driver_press')
      return self.NONE
    if cancel or resume or not enabled or not CS.cruiseState.enabled:
      self._end('disengaged' if not enabled or not CS.cruiseState.enabled else 'cancel_resume')
      return self.NONE
    interrupted = (not long_active or not CS.canValid or CS.canTimeout or CS.gasPressed or CS.brakePressed or
                   CS.standstill or CS.cruiseState.standstill)
    if interrupted:
      self._end('interrupted')
      return self.NONE
    if prompting and self.target is not None:
      self._end('new_offer')  # a new offer needs its own answer
      return self.NONE

    accepted = round(accepted_mps * conv) if accepted_mps is not None and accepted_mps > 0. else None

    if self.armed is not None:
      if accepted is not None and accepted == self.armed.target and accepted_ns >= self.armed.press_ns - 50_000_000:
        self.confirmed = accepted
        self.bound = (float(accepted_mps), int(accepted_ns))  # the exact planner acknowledgment of this press
        self.target = max(accepted, MIN_SET_MPH if not self.is_metric else round(MIN_SET_MPH * CV.MPH_TO_KPH))
        self.armed = None
      elif now_ns - self.armed.press_ns > ACCEPT_BIND_NS:
        self._end('accept_not_seen')
        return self.NONE
      else:
        return self.NONE

    # every press needs the confirmed value still accepted: gone or changed (either way) ends the walk
    if accepted is None:
      self._end('target_lost')
      return self.NONE
    if accepted != self.confirmed or (float(accepted_mps), int(accepted_ns)) != self.bound:
      self._end('target_changed')  # a newer acceptance, even one rounding to the same mph, needs its own press
      return self.NONE

    current = round(CS.cruiseState.speedCluster * conv)
    if current != self.last_set:
      self.last_set = current
      self.last_set_change_ns = now_ns

    error = self.target - current
    if error == 0:
      self._end('settled')  # reached: nothing re-arms it but a new confirm
      return self.NONE
    if current <= 0:
      self.error_since_ns = 0
      self.sending_since_ns = 0
      return self.NONE
    if not self.error_since_ns:
      self.error_since_ns = now_ns
    if now_ns - self.error_since_ns < REACT_NS:
      return self.NONE
    if not self.sending_since_ns:
      self.sending_since_ns = now_ns
    elif now_ns - max(self.sending_since_ns, self.last_set_change_ns) > STALL_NS:
      self._end('stalled')
      return self.NONE
    return self.INCREASE if error > 0 else self.DECREASE
