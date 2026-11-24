"""Mazda Speed Limit Assist, ported from filter-dev (mazda-smart-can-filter-dev ce9225e178).

One owner for the Mazda decision and the factory SET: the resolver (plannerd) picks the limit, the cruise
arbiter (card) runs the session and its prompts, the servo walks the factory SET, and the mirror (plannerd)
publishes the plan cap and the alert state. resolver.py and arbiter.py keep the reference's logic; only the
inputs and the message plumbing are StarPilot's.
"""
from enum import IntEnum

from openpilot.common.constants import CV

REFERENCE = 'ce9225e17816615d90503084f75674a569e4a050'

V_CRUISE_UNSET = 255.

CONFIRM_SPEED_THRESHOLD = {
  True: 80,   # km/h
  False: 50,  # mph
}


class Source(IntEnum):
  none = 0
  car = 1
  map = 2
  vision = 3


class AssistState(IntEnum):
  disabled = 0
  inactive = 1
  preActive = 2
  pending = 3
  adapting = 4
  active = 5


class CruiseIntent(IntEnum):
  none = 0
  increment = 1
  decrement = 2
  confirm = 3
  decline = 4
  dismiss = 5


ACTIVE_STATES = (AssistState.active, AssistState.adapting)
ENABLED_STATES = (AssistState.preActive, AssistState.pending, *ACTIVE_STATES)


def compare_cluster_target(v_cruise_cluster: float, target_set_speed: float, is_metric: bool) -> tuple[bool, bool]:
  speed_conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
  v_cruise_cluster_conv = round(v_cruise_cluster * speed_conv)
  target_set_speed_conv = round(target_set_speed * speed_conv)
  return v_cruise_cluster_conv < target_set_speed_conv, v_cruise_cluster_conv > target_set_speed_conv


def confirm_needed_for_change(cluster_conv: int, target_conv: int, is_metric: bool) -> bool:
  cst = CONFIRM_SPEED_THRESHOLD[is_metric]
  if cluster_conv < cst:
    return True
  return target_conv < cst


def settle_conv(v_cap: float, cluster_conv: int, is_metric: bool) -> int:
  return min(round(v_cap * (CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH)), cluster_conv)


def eligible(CP) -> bool:
  """Mazda factory cruise under openpilot longitudinal: the SET is reachable only through the wheel buttons."""
  return bool(CP.brand == 'mazda' and CP.openpilotLongitudinalControl and CP.pcmCruise and
              not getattr(CP, 'passive', False) and not CP.dashcamOnly)
