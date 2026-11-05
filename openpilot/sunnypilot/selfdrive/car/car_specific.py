"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.mads import SET_SPEED_BUTTONS
from opendbc.sunnypilot.car.stock_ecu import ENGAGES_NORMALLY
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import FAILED_KEY
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName
GearShifter = structs.CarState.GearShifter
StockEcuState = custom.CarStateZP.StockEcuState



class CarEventsSP(EventsSP):
  """selfdrived takes these through to_msg -> add_from_msg, so zoompilot's ride along beside sunnypilot's."""
  def to_msg(self):
    return self.to_msg_with_zp()


class CarSpecificEventsSP:
  # the camera's fault holds lateral this long past its clear, so a fault flickering as it clears
  # cannot toggle steering or replay the onset alert
  CAMERA_FAULT_CLEAR_FRAMES = int(1.0 / DT_CTRL)

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params=None):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False
    self.stock_ecu_prev = StockEcuState.notNeeded
    self.stock_ecu_off_prev = False
    self.alpha_long_announced = False
    self.camera_fault_clear_frames = self.CAMERA_FAULT_CLEAR_FRAMES
    # set by the stop that ended the last session; selfdrived starts fresh every session
    self.stock_cruise_failed = (params or Params()).get_bool(FAILED_KEY)

  def update(self, CS: structs.CarState, events: Events, CS_SP, mads_enabled_toggle: bool = False, mads_enabled: bool = False):
    events_sp = CarEventsSP()

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    elif self.CP.brand == 'mazda':
      if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS and events.has(EventName.steerTempUnavailable):
        # steerFaultTemporary on this EPS is the non-delivery latch reporting a sustained
        # road-speed block. The latch has already zeroed the command, so there is nothing
        # for a soft disable to protect; it only costs MADS its lateral after 3 s and
        # shouts at the driver. Keep the banner, drop the escalation.
        events.remove(EventName.steerTempUnavailable)
        events.add(EventName.steerTempUnavailableSilent)
      if mads_enabled_toggle and CS.invalidLkasSetting:
        # MADS on: the car's own lane-keep setting off (CAM_SETTINGS or LANE_LINES 0) is the
        # driver's choice about steering, not about cruise. The selfdrive machine still
        # engages on the stock cruise; the MADS machine alone refuses lateral. With MADS off
        # upstream's no-entry stands.
        events.remove(EventName.invalidLkasSetting)
        events_sp.zp.add(EventNameZP.stockLkasOff)
      self.camera_fault_clear_frames = 0 if CS.steerFaultPermanent else self.camera_fault_clear_frames + 1
      if mads_enabled_toggle and self.camera_fault_clear_frames < self.CAMERA_FAULT_CLEAR_FRAMES:
        # MADS on: the camera's fault (CAM_LKAS ERR_BIT_1, the only source of steerFaultPermanent
        # here) is glare, fog or a blocked windshield as often as a failure, and it can clear
        # mid-drive (route 00000172: 46 s). It takes lateral, which controlsd already gates on
        # steerFaultPermanent and the EPS sees in our command, not cruise: MADS pauses on it and
        # resumes 1 s after it clears (mads.py). With MADS off upstream's immediate disable stands.
        events.remove(EventName.steerUnavailable)
        events_sp.zp.add(EventNameZP.cameraLkasFault)
      if mads_enabled and CS_SP.zoompilot.lkasArming:
        # LKA back on with lateral coming back, the EPS not delivering yet
        events_sp.zp.add(EventNameZP.stockLkasArming)

    self.update_stock_ecu(CS, CS_SP, events_sp)
    return events_sp

  def update_stock_ecu(self, CS: structs.CarState, CS_SP, events_sp: EventsSP) -> None:
    """Also run by selfdrived before it initializes: card reports the takeover from its first
    frame, up to 6 s before the init gate lets car events through."""
    # A SET/RES press before the stock ECU openpilot stands in for is owned lands on a body
    # that will not engage (Mazda route 0000020d: six presses, nothing shown): name what the
    # driver has to do. Alert-only; the press itself does nothing.
    if self.stock_cruise_failed:
      events_sp.zp.add(EventNameZP.stockCruiseFailed)
    stock_ecu = CS_SP.zoompilot.stockEcu
    # The car's own cruise on the stock radar: SET works there as on a stock car, so no press alert.
    stock_cruise = stock_ecu == StockEcuState.stockCruiseOn
    if str(stock_ecu) not in ENGAGES_NORMALLY and not stock_cruise and \
       any(be.pressed and be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents):
      events_sp.zp.add(EventNameZP.stockEcuNotReady)
    # The car plays nothing when its own cruise engages or ends (route 0000012e): openpilot's
    # chimes, sound-only, as for a stock-longitudinal car. They change no state.
    stock_cruise_prev = self.stock_ecu_prev == StockEcuState.stockCruiseOn
    if stock_cruise and not stock_cruise_prev:
      events_sp.zp.add(EventNameZP.longitudinalEnableChime)
    elif stock_cruise_prev and not stock_cruise and stock_ecu != StockEcuState.restored:
      # restored here is the end-of-drive hand-back answered under the car's still-engaged
      # cruise: nothing ended, so no chime
      events_sp.zp.add(EventNameZP.longitudinalDisableChime)
    # Unprompted only where waiting changes the outcome: stopped with the takeover still
    # starting (route 0000021b pulled away 5 s short of it). Rolling, the press alert carries
    # "stop to engage"; a standing banner would nag a whole drive. One line on the ready edge.
    if stock_ecu == StockEcuState.starting and CS.standstill:
      events_sp.zp.add(EventNameZP.stockEcuInitializing)
    if stock_ecu == StockEcuState.ready and self.stock_ecu_prev != StockEcuState.ready:
      events_sp.zp.add(EventNameZP.stockEcuReady)
    self.stock_ecu_prev = stock_ecu
    # The stock ECU's own functions counted as back (Mazda AEB on). Said once, and not while the
    # driver's own setting keeps AEB off.
    off = CS_SP.zoompilot.stockEcuOff
    if self.stock_ecu_off_prev and not off and not CS_SP.zoompilot.aebSettingOff:
      events_sp.zp.add(EventNameZP.stockEcuHandedBack)
    self.stock_ecu_off_prev = off
    # The request registered, said once per request while driving: the takeover waits for the
    # next stop. Pressed at a stop, it is said on pulling away if no takeover started first.
    if not CS_SP.zoompilot.alphaLongRequested:
      self.alpha_long_announced = False
    elif not self.alpha_long_announced and not CS.standstill:
      events_sp.zp.add(EventNameZP.alphaLongNextStop)
      self.alpha_long_announced = True
