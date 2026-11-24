"""C4 rendering of the Mazda SLA alerts: filter-dev's mici AlertRenderer presentation, with its blinking +/- arrow."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.ui.raylib import rl
from openpilot.starpilot.ui import onroad_alerts
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert
from openpilot.starpilot.ui.presentation import Profile
from openpilot.selfdrive.ui.mici.onroad import alert_renderer as AR

MPH = 0.44704


@pytest.mark.parametrize('event, text', [('slcConfirmPrompt', 'Press - for\n30 mph'), ('slcAdjusting', 'Adjusting to\n30 mph')])
def test_sla_alerts_use_the_native_mici_renderer(event, text):
  fonts = Mock(profile=Profile.COMPACT)
  renderer = onroad_alerts.AlertRenderer(fonts)
  owner = Mock()
  owner.render_alert.return_value = True
  bounds = rl.Rectangle(0, 0, 476, 240)
  alert = OnroadAlert(AlertSize.SMALL, text, alert_type=event + '/warning')
  with patch.object(onroad_alerts, 'NativeAlertRenderer', return_value=owner):
    renderer.render(bounds, alert)
  owner.render_alert.assert_called_with(bounds, AR.Alert(alert.text1, alert.text2, size=1, alert_type=alert.alert_type),
                                        signal_direction=0)
  fonts.draw.assert_not_called()   # not StarPilot's lower-case compact banner


def native(state, offer_mph, set_mph):
  r = object.__new__(AR.AlertRenderer)
  r._txt_sla_arrow_up, r._txt_sla_arrow_down, r._txt_sla_arrow_blank = 'up', 'down', 'blank'
  r._sla_prompt_fade = AR.SlaPromptFade(60)
  sm = {'slcState': NS(mazdaSla=NS(assistState=state, offerTarget=offer_mph * MPH)), 'carState': NS(vCruiseCluster=set_mph * 1.609344)}
  return r, NS(sm=sm, is_metric=False)


@pytest.mark.parametrize('offer, set_speed, arrow', [(30, 35, 'down'), (36, 30, 'up'), (30, 30, 'blank')])
def test_prompt_arrow_points_to_the_offer(offer, set_speed, arrow):
  r, ui = native(2, offer, set_speed)
  with patch.object(AR, 'ui_state', ui):
    r._sla_prompt_fade.update(True)
    side, texture, alpha, mx, my = r._sla_prompt_icon()
  assert (side, texture, alpha, mx, my) == (AR.IconSide.right, arrow, 255.0, 10, 18)


def test_prompt_arrow_blinks_like_filter_dev():
  """On 0.75 s of every second (45 of 60 frames), then fading with rc 0.05."""
  fade = AR.SlaPromptFade(60)
  alphas = []
  for _ in range(120):
    fade.update(True)
    alphas.append(fade.alpha)
  assert all(a == 1.0 for a in alphas[:44]) and alphas[44] < 1.0
  assert alphas[58] < 0.02 and alphas[59] == 1.0   # frame 60 starts the next second
  for _ in range(50):
    fade.update(True)   # frame 110 (alpha decaying)
  low = fade.alpha
  fade.update(False)    # the prompt closed: back up toward 1, the blink counter restarts
  assert low < fade.alpha < 1.0
