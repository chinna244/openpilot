"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The stock ECU hand-back: one correlated request/result pair between whoever ends the onroad
state and card, which owns the vehicle's diagnostic session.

Any software-initiated end of the onroad state (onroad cycle for a calibration reset or a
restart-needed toggle, forced offroad, reboot, shutdown, uninstall) kills TX within about
100 ms of `started` dropping. A brand that silences a stock ECU for the drive needs those
processes alive long enough to hand it back first, or the ECU recovers unattended through its
S3 timeout in a degraded state that blocks cruise until the next ignition.

Consumer (`StockEcuHandBackGate`, in hardwared and manager): writes StockEcuHandBackRequest
{id} and holds its action until StockEcuHandBackResult carries the same id. A second consumer
arriving while one request is open joins it, so exactly one hand-back is ever in flight.
Producer (`StockEcuHandBackServer`, in card): asserts the brand's hand-back off the control loop
while the request stands and writes the result with the vehicle's StockEcuState once it is one
of restored, failed or notNeeded. Ids are monotonic past every record on file, so a stale answer
never satisfies a new request, and a withdrawn request (the record removed while onroad) is a fresh
start for the vehicle. Both records outlive the offroad transition and clear on the next onroad one:
a stop's answered request keeps card asserting the hand-back until card exits, so a restored ECU
cannot be taken over again in the gap between manager's offroad clear and card's shutdown.

A stop that can be withdrawn (forced offroad, through its exit button) proceeds only on
restored or notNeeded and stays open on failed: the vehicle keeps neutral replacement traffic
going and a late recovery still completes it. A stop with no cancel (cycle, reboot, shutdown,
uninstall) proceeds on any answer, and on failed leaves FAILED_KEY for the alert. Either proceeds
after HANDBACK_WAIT_T with no answer at all, which means no card is alive to give one; it leaves
FAILED_KEY too when card's last status had the radar off. FAILED_KEY is persistent and clears only
when manager observes the ignition off (the radar's degraded state lasts until then), so a reboot
or shutdown with the ignition on keeps it. Ignition off and power loss cannot be held.
"""
import time
import capnp

from opendbc.car.mazda.stock_ecu import StockEcuState
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

REQUEST_KEY = "StockEcuHandBackRequest"
RESULT_KEY = "StockEcuHandBackResult"
# a stop went ahead on a failed hand-back; the session after an onroad cycle tells the driver
FAILED_KEY = "StockEcuHandBackFailed"

# The producer's own budget is one UDS attempt (10 s) plus the restore window; past this with no
# answer at all the processes are assumed dead, and the stop goes ahead regardless.
HANDBACK_WAIT_T = 15.0

SETTLED = (StockEcuState.RESTORED, StockEcuState.NOT_NEEDED)
ANSWERS = (*SETTLED, StockEcuState.FAILED)


def read_record(params: Params, key: str) -> dict | None:
  rec = params.get(key)
  return rec if isinstance(rec, dict) and isinstance(rec.get("id"), int) else None


class StockEcuHandBackGate:
  """Consumer side. Call ready() every loop while the stop is wanted, reset() if it is withdrawn."""

  def __init__(self, params: Params, now=time.monotonic) -> None:
    self.params = params
    self.now = now
    self.request_id: int | None = None
    self.requested_ts: float | None = None
    self.failed = False

  @property
  def pending(self) -> bool:
    return self.request_id is not None

  def _open(self) -> None:
    existing = read_record(self.params, REQUEST_KEY)
    result = read_record(self.params, RESULT_KEY)
    settled = result is not None and existing is not None and result["id"] == existing["id"] and result.get("outcome") in SETTLED
    if existing is not None and not settled:
      # one hand-back in flight: join it rather than queue another behind it
      self.request_id = existing["id"]
    else:
      self.request_id = max([rec["id"] for rec in (existing, result) if rec is not None] + [0]) + 1
      self.params.put(REQUEST_KEY, {"id": self.request_id}, block=True)
    self.requested_ts = self.now()
    cloudlog.warning(f"stock ECU hand-back {self.request_id} requested before ending onroad")

  def _close(self) -> None:
    self.request_id = None
    self.requested_ts = None
    self.failed = False

  def ready(self, started: bool, hold_on_failure: bool = False) -> bool:
    """True when the caller may end the onroad state now; the wait is cleared with it.
    hold_on_failure: the stop can be withdrawn, so a failed hand-back holds it open."""
    if not started:
      self._close()
      return True
    if self.request_id is None:
      self._open()
      return False
    result = read_record(self.params, RESULT_KEY)
    if result is not None and result["id"] == self.request_id:
      outcome = result.get("outcome")
      if outcome in SETTLED:
        self._close()
        return True
      if outcome == StockEcuState.FAILED:
        if not self.failed:
          cloudlog.error(f"stock ECU hand-back {self.request_id} failed")
        self.failed = True
        if not hold_on_failure:
          self.params.put_bool(FAILED_KEY, True, block=True)
          self._close()
          return True
        return False  # held open; a late recovery or a withdrawal ends it
    if self.now() - self.requested_ts > HANDBACK_WAIT_T:
      left_off = radar_left_off(self.params)
      cloudlog.error(f"stock ECU hand-back {self.request_id} unanswered, ending onroad anyway (radar left off: {left_off})")
      if left_off:
        self.params.put_bool(FAILED_KEY, True, block=True)
      self._close()
      return True
    return False

  def reset(self) -> None:
    if self.request_id is not None:
      # withdraw only a request nobody else is waiting on; card sees the record vanish
      existing = read_record(self.params, REQUEST_KEY)
      drive_request = self.params.get("DriveStateRequest")
      forced_offroad = isinstance(drive_request, dict) and drive_request.get("mode") == "offroad"
      if (not forced_offroad and existing is not None and existing["id"] == self.request_id and
          not any(self.params.get_bool(key) for key in ("DoReboot", "DoShutdown", "DoUninstall", "OnroadCycleRequested"))):
        self.params.remove(REQUEST_KEY)
    self._close()


class StockEcuHandBackServer:
  """Producer side, on card's control loop."""

  def __init__(self, params: Params) -> None:
    self.params = params
    self.request: dict | None = None
    self.started = False       # asserted once: held for as long as the request stands
    self.answered: tuple[int, StockEcuState] | None = None

  def update_params(self) -> None:
    # card's 10 Hz params_thread; the control loop reads only the attribute
    self.request = read_record(self.params, REQUEST_KEY)

  def _answer(self, request_id: int, outcome: StockEcuState) -> None:
    if self.answered == (request_id, outcome):
      return
    self.params.put(RESULT_KEY, {"id": request_id, "outcome": str(outcome)})
    self.answered = (request_id, outcome)
    (cloudlog.error if outcome == StockEcuState.FAILED else cloudlog.warning)(f"stock ECU hand-back {request_id} answered {outcome}")

  def update(self, enabled: bool, state: StockEcuState, CC_SP) -> None:
    """Runs at 100 Hz before CI.apply with the controller's stock ECU state. CC_SP is rebuilt
    every frame, so the assert is re-applied here; the session manager treats a dropped assert
    as a withdrawal."""
    if self.request is None:
      self.started = False
      return
    request_id = self.request["id"]
    if state == StockEcuState.NOT_NEEDED:
      self._answer(request_id, state)
      return
    # Never start on an engaged car: the hand-back revokes availability under the driver.
    # Once started it runs to its end.
    if not self.started and enabled:
      return
    self.started = True
    CC_SP.stockEcuHandBack = True
    if state in ANSWERS:
      self._answer(request_id, state)


def radar_left_off(params) -> bool:
  """Card's last word on this session's radar (any age: card may be gone). Off, or owned/mid-return,
  means the stop leaves it to recover unattended. No record at all means card never ran control,
  and only control can command a takeover, so the radar was never taken."""
  status = params.get("MazdaStockEcuStatus")
  if isinstance(status, dict) and type(status.get("radarOff")) is bool:
    return status["radarOff"] or status.get("state") in (StockEcuState.READY, StockEcuState.RESTORING, StockEcuState.FAILED)
  return params.get("MazdaStockEcuState") in (StockEcuState.READY, StockEcuState.RESTORING, StockEcuState.FAILED)


def clear_handback_failure_on_ignition_off(params, panda_states) -> None:
  """manager: a failed hand-back lasts until the car's ignition is seen off by a known panda."""
  from openpilot.cereal import log
  known = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if known and not any(ps.ignitionLine or ps.ignitionCan for ps in known) and params.get_bool(FAILED_KEY):
    params.remove(FAILED_KEY)


def mazda_handback_required(params) -> bool:
  """Shared stop owners wait only for a finalized Mazda alpha-long configuration."""
  from opendbc.car import structs
  raw = params.get("CarParams")
  if not isinstance(raw, bytes) or not raw:
    return params.get("MazdaStockEcuState") in set(StockEcuState) - {StockEcuState.NOT_NEEDED}
  try:
    with structs.CarParams.from_bytes(raw) as cp:
      return bool(cp.brand == "mazda" and cp.openpilotLongitudinalControl and not cp.passive and
                  len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
                  cp.safetyConfigs[0].safetyParam in (3, 11))
  except (ValueError, RuntimeError, capnp.KjException):
    return params.get("MazdaStockEcuState") in set(StockEcuState) - {StockEcuState.NOT_NEEDED}
