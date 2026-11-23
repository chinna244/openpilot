"""card writes the SLC SET servo's choice as a plain string into control_context; the real
controller must turn it into paced SET+/SET- CRZ_BTNS frames on the car's bus (route
00000002--c9d55e7093: StarPilot never set sendButton, so the factory SET never moved)."""
import pytest

from opendbc.car.mazda.tests.conftest import CRZ_BTNS, car_control, car_controller, frames, mazda_car_state, parse_frame, set_car_state


@pytest.mark.parametrize('button,signal', [('increase', 'SET_P'), ('decrease', 'SET_M')])
def test_card_string_reaches_crz_btns(button, signal):
  cc = car_controller(alpha_long=True)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  sends = []
  for _ in range(100):  # 1 s
    set_car_state(cs, available=True, cruise_engaged=True, v_ego=13.)
    cc.control_context.intelligentCruiseButtonManagement.sendButton = button
    sends.extend(cc.update(car_control(long_active=True), cs, 0)[1])
    cc.control_context.intelligentCruiseButtonManagement.sendButton = 'none'
  presses = frames(sends, CRZ_BTNS, bus=0)
  assert 3 <= len(presses) <= 6  # taps at the ECU's 200 ms floor
  for dat in presses:
    sig = parse_frame(CRZ_BTNS, dat)
    assert sig[signal] == 1 and sig['SET_P' if signal == 'SET_M' else 'SET_M'] == 0 and sig['CAN_OFF'] == 0


def test_none_sends_nothing():
  cc = car_controller(alpha_long=True)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  sends = []
  for _ in range(100):
    set_car_state(cs, available=True, cruise_engaged=True, v_ego=13.)
    sends.extend(cc.update(car_control(long_active=True), cs, 0)[1])
  assert frames(sends, CRZ_BTNS, bus=0) == []
