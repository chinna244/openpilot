"""Mazda: develop's lead forecast is the MPC's obstacle in StarPilot's planner; other brands keep StarPilot's path."""
import numpy as np

from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.longitudinal.lead_forecast import LEAD_T_IDXS, Inhibit
from openpilot.starpilot.longitudinal.tests.test_set_speed_assist_planner import mazda_frame, mazda_planner

V = 20.


def lead_frame(tick, *, d_rel=40., v_lead=15., model_v=None):
  sm, now = mazda_frame(tick, model_accel=0., plan_v=V)
  sm['carState'].aEgo = 0.
  lead = sm['radarState'].leadOne
  lead.present, lead.dRel, lead.vLead, lead.vRel, lead.aLeadK, lead.aLeadTau = True, d_rel, v_lead, v_lead - V, 0., 1.5
  lead.modelProb, lead.radar, lead.radarTrackId = 0.9, False, -1
  model_v = np.full(len(LEAD_T_IDXS), v_lead) if model_v is None else np.asarray(model_v, dtype=float)
  ml = sm['modelV2'].init('leadsV3', 2)
  for m in ml:
    m.prob, m.t = 0.9, LEAD_T_IDXS.tolist()
    m.x = (d_rel + 1.52 + np.concatenate(([0.], np.cumsum(np.diff(LEAD_T_IDXS) * (model_v[1:] + model_v[:-1]) / 2.)))).tolist()
    m.v, m.y, m.a = model_v.tolist(), [0.] * 6, [0.] * 6
  return sm, now


def test_mazda_mpc_takes_the_forecast():
  planner = mazda_planner()
  assert planner.lead_forecast is not None
  slowing = np.interp(LEAD_T_IDXS, [0., 4., 10.], [15., 9., 9.])
  for tick in range(40):
    sm, now = lead_frame(tick, model_v=slowing)
    planner.update(sm, now_ns=now, experimental_speed_nudge=True)
  forecast = planner.lead_forecast
  assert forecast.inhibits[0] == Inhibit.none and forecast.weights[0] == 1.
  # the MPC's obstacle slows as the model predicts (upstream's extrapolation would hold 15 m/s)
  assert forecast.lead_xv[0][-1, 1] < 10.


def test_other_brands_keep_starpilot_model_lead_path():
  params = HondaInterface.get_non_essential_params(HONDA.HONDA_CIVIC)
  params.openpilotLongitudinalControl = True
  planner = LongitudinalPlanner(params, init_v=V)
  assert planner.lead_forecast is None
  assert planner.mpc.process_lead.__func__ is type(planner.mpc).process_lead
