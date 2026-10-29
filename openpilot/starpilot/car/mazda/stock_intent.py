from opendbc.car import structs
from opendbc.car.mazda.stock_aol import qualified
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType

# develop (car_specific.py): the camera's fault holds lateral this long past its clear, so a fault flickering
# as it clears cannot toggle steering
CAMERA_FAULT_CLEAR_FRAMES = 100


class MazdaStockCardIntent(AolCardIntent):
  observe_stock_engagement = True

  def __init__(self, cp, settings):
    super().__init__(settings, explicit_latch=True)
    self.cp = cp
    self.tja_button = bool(cp.safetyConfigs[0].safetyParam & 8)
    self.observe_stock_engagement = not self.tja_button
    self.camera_fault_clear_frames = CAMERA_FAULT_CLEAR_FRAMES
    self.lkas_off = False
    self.pending_rejection_ns = 0
    self.independent_cruise_buttons = self.tja_button
    self.main_seen = False
    self.startup_main_pending = not self.tja_button
    self.startup_main_source_ns = 0
    self.startup_native_ready = False
    self.startup_binding = None
    self.main_available = False
    self.main_neutral = False
    self.main_pulse = False
    self.cancel_held = False
    self.cancel_ticks = 0

  @property
  def lkas_action(self):
    return AOL_TOGGLE if self.tja_button else super().lkas_action

  @property
  def main_action(self):
    return 0 if self.tja_button else super().main_action

  def observe_startup_readiness(self, sm, CS, *, native, panda_ready, now_ns, producer):
    from openpilot.starpilot.aol.intent import disarming_fault
    from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS
    from openpilot.starpilot.aol.transport_pause import _healthy_transport_native, availability_only

    self.startup_native_ready = False
    if not self.startup_main_pending or not qualified(self.cp, marked_only=True):
      return
    if CS.steerFaultPermanent or CS.accFaulted:
      self.startup_main_pending = False
      return
    try:
      event_ns = int(sm.logMonoTime['onroadEvents'])
      events_current = bool(sm.seen['onroadEvents'] and sm.valid['onroadEvents'] and sm.alive['onroadEvents'] and
        0 < event_ns <= now_ns and now_ns - event_ns <= 1_500_000_000)
      events = sm['onroadEvents']
      faults = [event for event in events if disarming_fault([event], CS)]
      if events_current and faults and not availability_only(faults):
        self.startup_main_pending = False
        return
      axis = sm['aolAxisState']
      native = (_healthy_transport_native(sm, self.cp, CS, str(axis.sessionId), now_ns, native)
                if native is not None else None)
      self.startup_native_ready = bool(events_current and panda_ready and native is not None and native.pandaInventory and
        not native.requestedLateral and not native.lateralAllowed and not native.requestedLongitudinal and not native.longitudinalAllowed and
        all(not slot.controlsAllowed for slot in native.pandaInventory) and
        sm.seen['aolAxisState'] and sm.valid['aolAxisState'] and sm.alive['aolAxisState'] and
        int(sm.logMonoTime['aolAxisState']) == axis.observedMonoTime and
        axis.validUntilMonoTime - axis.observedMonoTime <= AXIS_MAX_AGE_NS and
        axis.qualified and axis.nativeAcknowledged and str(axis.faultReason) == 'none' and
        str(axis.sessionId) == native.axisSessionId and producer and axis.intentProducerSessionId == producer and
        not axis.desiredLateral and not axis.desiredLongitudinal and not axis.lateralActive and not axis.longitudinalActive and
        0 < axis.sourceCarStateMonoTime <= axis.observedMonoTime <= now_ns <= axis.validUntilMonoTime and
        now_ns - axis.sourceCarStateMonoTime <= AXIS_MAX_AGE_NS)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self.startup_native_ready = False
    if self.startup_native_ready:
      binding = (str(axis.sessionId), producer, int(axis.intentIntegrityEpoch), native.pandaSerial,
                 tuple(slot.hardwareSerial for slot in native.pandaInventory))
      if binding != self.startup_binding:
        self.startup_main_source_ns = 0
        self.startup_binding = binding
    else:
      self.startup_binding = None
      self.startup_main_source_ns = 0

  def cancel_action(self, action):
    if action == AOL_TOGGLE and self.settings.enabled:
      self.allowed_latch = not self.allowed_latch
      if self.allowed_latch:
        self.pause_lateral = False
    else:
      self._perform(action)

  def update(self, CS, **kwargs):
    if self.tja_button:
      # The panda can withhold lateral for the camera's fault one sample before card parses it: hold a native
      # rejection one sample, and drop it when the fault's pause shows up instead.
      rejection = kwargs.get('native_rejection_ns', 0)
      if not rejection or self.lateral_held:
        self.pending_rejection_ns = 0
        kwargs['native_rejection_ns'] = 0 if self.lateral_held else rejection
      elif not self.pending_rejection_ns:
        self.pending_rejection_ns = rejection
        kwargs['native_rejection_ns'] = 0
      else:
        kwargs['native_rejection_ns'] = self.pending_rejection_ns
    state = structs.CarState(**CS.to_dict())
    if self.tja_button:
      # Only this lateral-intent view is independent of MRCC; cruiseState stays physical on the wire.
      state.cruiseState.available = True
    events = []
    for event in CS.buttonEvents:
      if event.type == ButtonType.cancel:
        if event.pressed:
          self.cancel_held = True
          self.cancel_ticks = 0
        else:
          if self.cancel_held and 0 < self.cancel_ticks < CRUISE_LONG_PRESS:
            self.cancel_action(self.settings.cancel_actions[0])
          self.cancel_held = False
          self.cancel_ticks = 0
      elif event.type in (ButtonType.mainCruise, ButtonType.lkas):
        action = self.main_action if event.type == ButtonType.mainCruise else self.lkas_action
        if self.tja_button and event.type == ButtonType.lkas and not self.allowed_latch and self.lateral_held:
          pass  # as develop: TJA cannot turn steering on through the camera's fault or with LKAS off (refused)
        elif action == AOL_TOGGLE:
          events.append(structs.CarState.ButtonEvent(**event.to_dict()))
        elif (not self.tja_button and event.type == ButtonType.mainCruise and action == 0 and
              (not self.allowed_latch or not event.pressed)):
          # A fresh unmapped ON may rearm standby, but never toggles an active latch off.
          events.append(structs.CarState.ButtonEvent(**event.to_dict()))
        elif event.pressed:
          self._perform(action)
      else:
        events.append(structs.CarState.ButtonEvent(**event.to_dict()))
    available = bool(state.cruiseState.available)
    healthy = state.canValid and not state.canTimeout
    if healthy:
      if self.main_pulse:
        events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=False))
        self.main_pulse = False
      if not available:
        self.main_neutral = True
      elif self.main_seen and self.main_neutral and not self.main_available:
        events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=True))
        self.main_pulse = True
        self.main_neutral = False
      self.main_seen = True
      self.main_available = available
    else:
      self.cancel_held = False
      self.cancel_ticks = 0
      self.main_neutral = False
    startup_main = False
    if self.startup_main_pending and healthy:
      eligible = bool(available and not state.cruiseState.enabled and self.settings.enabled and
        self.main_action != AOL_TOGGLE and self.lkas_action != AOL_TOGGLE and
        not state.steerFaultPermanent and not state.accFaulted and
        not (self.cancel_held and any(self.settings.cancel_actions)))
      source_ns = kwargs.get('now_ns', 0)
      if not eligible:
        self.startup_main_pending = False
      elif self.startup_native_ready and kwargs.get('fault_active'):
        self.startup_main_pending = False
      elif self.startup_native_ready and self.startup_main_source_ns and source_ns > self.startup_main_source_ns:
        # The first valid sample initializes the transport feedback pipeline.
        # Confirm MAIN on a subsequent fresh sample before admitting startup intent.
        self.startup_main_pending = False
        startup_main = True
      elif self.startup_native_ready and source_ns > 0:
        self.startup_main_source_ns = source_ns
    state.buttonEvents = events
    if self.tja_button:
      # a TJA press toggles the latch itself, also while a hold below pauses it
      self.pause_lateral = not self.allowed_latch
    super().update(state.as_reader(), **kwargs)
    if startup_main and not self._fault_inhibit:
      self.allowed_latch = True
      self._last_latch_edge_ns = kwargs.get('now_ns', 0)
    if not available:
      self.allowed_latch = False
    if self.cancel_held:
      self.cancel_ticks += 1
      if self.cancel_ticks == CRUISE_LONG_PRESS:
        self.cancel_action(self.settings.cancel_actions[1])
      elif self.cancel_ticks == CRUISE_LONG_PRESS * 5:
        self.cancel_action(self.settings.cancel_actions[2])
    if self.tja_button:
      # As develop's MADS with the declared TJA button: only TJA turns steering on or off; SET/RES, cancel and
      # main never do. Ordinary cruise engagement gets no lateral of its own while the latch is off. The camera's
      # fault and the car's lane keep switched off pause it, keeping the latch and cruise, and resume on their own.
      self.pause_lateral = not self.allowed_latch or self.lateral_held

  @property
  def lateral_held(self):
    return self.camera_fault_clear_frames < CAMERA_FAULT_CLEAR_FRAMES or self.lkas_off

  def observe_lateral_holds(self, CS):
    """Card's view for the AOL owners: with the TJA button the camera's fault (CAM_LKAS ERR_BIT_1, Mazda's only
    steerFaultPermanent) pauses lateral as develop's MADS did instead of disarming the driver's intent. The published
    CarState keeps the fault: selfdrived withholds lateral on it and the panda withholds torque."""
    if not self.tja_button:
      return CS
    self.camera_fault_clear_frames = 0 if CS.steerFaultPermanent else min(self.camera_fault_clear_frames + 1,
                                                                          CAMERA_FAULT_CLEAR_FRAMES)
    self.lkas_off = bool(CS.invalidLkasSetting)
    if self.lateral_held:
      self.pause_lateral = True  # this frame, before the AOL owners read it
    if not CS.steerFaultPermanent:
      return CS
    view = structs.CarState(**CS.to_dict())
    view.steerFaultPermanent = False
    return view.as_reader()

  def _arm_from_full_engagement(self, CS, *, source_ns):
    if not self.tja_button:
      super()._arm_from_full_engagement(CS, source_ns=source_ns)
