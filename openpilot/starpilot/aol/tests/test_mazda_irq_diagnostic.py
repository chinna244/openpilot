"""The CAN2 interrupt-rate diagnostic latched at every Mazda boot does not block a controllable Mazda; nothing else changes."""
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.cereal import log
from openpilot.starpilot.aol.panda_faults import blocking_fault_list, blocking_fault_mask

CAN2 = log.PandaState.FaultType.interruptRateCan2


def mazda_cp(car=CAR.MAZDA_CX5_2022, alpha_long=False):
  return CarInterface.get_params(car, gen_empty_fingerprint(), [], alpha_long, False, False)


@pytest.mark.parametrize('car', [CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021])
@pytest.mark.parametrize('alpha_long', [False, True])
def test_boot_can2_diagnostic_does_not_block(car, alpha_long):
  cp = mazda_cp(car, alpha_long)
  assert not blocking_fault_list([CAN2], cp)
  assert not blocking_fault_mask(1 << CAN2, cp)


@pytest.mark.parametrize('name,value', tuple(log.PandaState.FaultType.schema.enumerants.items()))
def test_other_and_mixed_faults_still_block(name, value):
  cp = mazda_cp()
  expected = name != 'interruptRateCan2'
  assert blocking_fault_mask(1 << value, cp) == expected
  assert blocking_fault_mask((1 << value) | (1 << CAN2), cp) == expected
  assert blocking_fault_list([value], cp) == expected
  assert blocking_fault_list([CAN2, value], cp) == expected


@pytest.mark.parametrize('mutation', ['dashcam', 'other_mazda', 'extra', 'safety_model'])
def test_other_configurations_cannot_borrow_the_diagnostic(mutation):
  cp = mazda_cp()
  if mutation == 'dashcam':
    cp.dashcamOnly = True
  elif mutation == 'other_mazda':
    cp = mazda_cp(CAR.MAZDA_3)
  elif mutation == 'extra':
    cp.safetyConfigs = [cp.safetyConfigs[0], cp.safetyConfigs[0]]
  else:
    cp.safetyConfigs[0].safetyModel = 'noOutput'
  assert blocking_fault_list([CAN2], cp)
  assert blocking_fault_mask(1 << CAN2, cp)
