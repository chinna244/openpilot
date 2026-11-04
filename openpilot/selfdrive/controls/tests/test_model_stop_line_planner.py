"""The planner takes the model stop line's candidate (model_stop_line.py) into its min()."""
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.tests.test_cruise_handover import V_EGO, build_sm
from openpilot.selfdrive.controls.tests.test_cruise_handover import planner as any_planner


def planner():
  p = any_planner()
  p.CP.brand = "mazda"
  return p


def approach(p, stop_at=14., lines=(12., 16.), n=10, **kwargs):
  """n cycles at V_EGO with the model's plan stopping stop_at m ahead (None: not stopping) and the camera's bands,
  painted on the road: nearer by V_EGO each cycle."""
  for i in range(n):
    sm = build_sm(**kwargs)
    md = sm['modelV2'].as_builder()
    if stop_at is not None:
      md.velocity.x = [max(V_EGO * (1. - i / 10.), 0.) for i in range(33)]
      md.position.x = [min(stop_at * i / 10., stop_at) for i in range(33)]
    sm['modelV2'] = md.as_reader()
    cr = messaging.new_message('customReserved11')
    cr.customReserved11.lineDistances = [d - V_EGO * DT_MDL * i for d in lines]
    sm['customReserved11'] = cr.customReserved11
    p.update(sm)
  return p.output_a_target


def test_not_a_mazda_reads_nothing():
  p = any_planner()
  approach(p)
  assert not p.model_stop_line.locked


def test_a_model_stop_past_a_line_brakes_for_the_line():
  base = approach(planner(), lines=())
  p = planner()
  a = approach(p)
  assert p.model_stop_line.locked
  assert a < base and a < -0.5


@pytest.mark.parametrize("kwargs", [{"stop_at": None}, {"lines": ()}, {"lead": 20.}, {"gas": True, "override": True},
                                    {"brake": True, "override": True}])
def test_no_stop_no_paint_a_lead_or_the_driver_leave_the_plan_alone(kwargs):
  base = {k: v for k, v in kwargs.items() if k not in ("lines",)}
  assert approach(planner(), **kwargs) == pytest.approx(approach(planner(), **{**base, "lines": ()}))
