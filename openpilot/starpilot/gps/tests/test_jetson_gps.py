import json
import math
import os
import types

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.gps import jetson as jetson_mod
from openpilot.starpilot.gps.jetson import JetsonGps, JetsonGpsReader, record, write
from openpilot.starpilot.gps.source import observation
from openpilot.system.ubloxd.gps_arbiter import ExternalFix


@pytest.fixture(autouse=True)
def mazda_session(monkeypatch):
  """The consumers read records only during a Mazda session; these tests run as one unless they say otherwise."""
  monkeypatch.setattr(jetson_mod, 'mazda_session', lambda: True)


def test_reader_ignores_records_outside_a_mazda_session(tmp_path, monkeypatch):
  path = str(tmp_path / "g.json")
  write(record(GPS, 1.0), path)
  monkeypatch.setattr(jetson_mod, 'mazda_session', lambda: False)
  reader = JetsonGpsReader(path)
  assert reader.poll(1.) is None
  monkeypatch.setattr(jetson_mod, 'mazda_session', lambda: True)
  assert reader.poll(1.5) is None  # the gate is asked at most once a second
  assert reader.poll(2.1)['latitude'] == GPS['lat']


def test_mazda_session_reads_only_live_carparams(monkeypatch):
  import openpilot.starpilot.car.mazda.last_car as last_car
  seen = []
  monkeypatch.setattr(last_car, 'car_brand', lambda params, live_only=False: seen.append(live_only) or 'mazda')
  monkeypatch.undo()  # drop the autouse stub for this one
  monkeypatch.setattr(last_car, 'car_brand', lambda params, live_only=False: seen.append(live_only) or 'mazda')
  assert jetson_mod.mazda_session() and seen == [True]

# the bench Jetson's telemetry `gps`, as NmeaGps.encode writes it
GPS = {"lat": 42.971638, "lon": -87.893982, "alt_m": 209.7, "speed_mps": 0.005, "course_deg": 16.57, "hdop": 0.45, "sats": 36,
       "quality": 2, "fix_type": 3, "utc_ms": 1_792_876_061_000, "age_ms": 412}


class Client:
  def __init__(self):
    self.last_state = None


class Model:
  def __init__(self):
    self.client = Client()


class TestPoll:
  def test_each_telemetry_once(self):
    model, gps = Model(), JetsonGps()
    assert gps.poll(model) is None
    model.client.last_state = {"temp_c": 50., "gps": GPS}
    assert gps.poll(model) == GPS
    assert gps.poll(model) is None
    model.client.last_state = {"temp_c": 51., "gps": GPS}
    assert gps.poll(model) == GPS

  def test_telemetry_without_gps(self):
    model, gps = Model(), JetsonGps()
    for state in ({}, {"temp_c": 50.}, {"gps": None}, {"gps": [1, 2]}, "{}"):
      model.client.last_state = state
      assert gps.poll(model) is None

  def test_small_model_has_no_client(self):
    gps = JetsonGps()
    assert gps.poll(object()) is None
    model = Model()
    model.client = None
    assert gps.poll(model) is None


class TestRecord:
  def test_bench_fix(self):
    rec = record(GPS, 100.)
    assert (rec['latitude'], rec['longitude'], rec['altitude']) == (GPS["lat"], GPS["lon"], GPS["alt_m"])
    assert rec['speed'] == pytest.approx(0.005)
    assert rec['bearingDeg'] == pytest.approx(16.57)
    assert rec['hdop'] == pytest.approx(0.45)
    assert (rec['satelliteCount'], rec['fixQuality'], rec['fixType']) == (36, 2, 3)
    assert (rec['unixTimestampMillis'], rec['ageMs'], rec['mono']) == (GPS["utc_ms"], 412, 100.)
    fix = ExternalFix.from_record(json.loads(json.dumps(rec, allow_nan=False)))
    assert fix.received == pytest.approx(100. - 0.412)
    assert fix.valid()

  def test_optional_fields_missing(self):
    rec = record({k: GPS[k] for k in ("lat", "lon", "utc_ms", "age_ms")}, 5.)
    assert rec['bearingDeg'] is None and rec['hdop'] is None
    assert (rec['altitude'], rec['speed'], rec['satelliteCount'], rec['fixQuality'], rec['fixType']) == (0., 0., 0, 0, 0)
    fix = ExternalFix.from_record(json.loads(json.dumps(rec, allow_nan=False)))  # JSON never carries NaN
    assert math.isnan(fix.bearing_deg) and math.isnan(fix.hdop)
    assert not fix.valid()  # no HDOP: never trusted

  @pytest.mark.parametrize("change", [{"lat": None}, {"lon": "x"}, {"utc_ms": None}, {"age_ms": None}, {"lat": 91.}, {"lon": -181.},
                                      {"lat": math.nan}, {"lat": True}, {"age_ms": -1}, {"age_ms": 2**32}, {"utc_ms": 0}])
  def test_malformed_is_not_written(self, change):
    gps = {k: v for k, v in {**GPS, **change}.items() if v is not None}
    assert record(gps, 1.) is None

  def test_out_of_range_counts_are_clamped(self):
    rec = record({**GPS, "sats": 400, "quality": -1, "fix_type": math.inf, "speed_mps": -3.}, 1.)
    assert (rec['satelliteCount'], rec['fixQuality'], rec['fixType'], rec['speed']) == (255, 0, 0, 0.)


class TestAsk:
  def test_asks_once_a_second(self, monkeypatch):
    clock = [100.]
    monkeypatch.setattr(jetson_mod.time, "monotonic", lambda: clock[0])
    gps = JetsonGps()
    asked = []
    for _ in range(80):  # 5 s of frames, 1/16 s apart so the clock adds up exactly
      after_enqueue = gps.ask()
      if after_enqueue is not None:
        after_enqueue()  # what end() does with it: nothing
        asked.append(clock[0])
      clock[0] += 0.0625
    assert len(asked) == 5
    assert all(b - a == pytest.approx(1.0) for a, b in zip(asked, asked[1:], strict=False))


class TestFile:
  def test_each_record_once(self, tmp_path):
    path = str(tmp_path / "jetson_gps.json")
    reader = JetsonGpsReader(path, period=0.1)
    assert reader.poll(0.) is None  # no file yet
    write(record(GPS, 10.), path)
    assert reader.poll(1.)['mono'] == 10.
    assert reader.poll(2.) is None
    write(record(GPS, 11.), path)
    assert reader.poll(2.05) is None  # within the period
    assert reader.poll(3.)['mono'] == 11.
    assert [n for n in os.listdir(tmp_path) if n != "jetson_gps.json"] == []  # no temp files left behind

  @pytest.mark.parametrize("content", ["", "{", "[1, 2]", "null"])
  def test_garbage_is_ignored(self, tmp_path, content):
    path = tmp_path / "jetson_gps.json"
    path.write_text(content)
    assert JetsonGpsReader(str(path)).poll(1.) is None

  @pytest.mark.parametrize("rec", [{}, {"mono": 1.}, {**record(GPS, 1.), "latitude": "x"}, {**record(GPS, 1.), "fixType": None}])
  def test_bad_record_raises_what_ubloxd_catches(self, rec):
    with pytest.raises((KeyError, TypeError, ValueError)):
      ExternalFix.from_record(rec)


class Sm(dict):
  def __init__(self, event):
    super().__init__(gpsLocationExternal=event.gpsLocationExternal)
    self.valid = {'gpsLocationExternal': True}
    self.logMonoTime = {'gpsLocationExternal': event.logMonoTime}


@pytest.mark.parametrize("source, used", [("ublox", True), ("external", True), ("car", False)])
def test_navigation_takes_the_jetson_fix(source, used):
  now = 50_000_000_000
  event = messaging.new_message('gpsLocationExternal', valid=True, logMonoTime=now - 100_000_000)
  event.gpsLocationExternal = {'source': source, 'hasFix': True, 'horizontalAccuracy': 6., 'latitude': 30., 'longitude': -90.}
  assert (observation(Sm(event), 'gpsLocationExternal', now) is not None) == used


class TestHook:
  def model(self):
    """jetlink v0.9.0's own JetlinkModelState.run (develop's jetlink_repo, same commit), with only its link stubbed."""
    pytest.importorskip("jetlink.openpilot.model_state")
    from jetlink.openpilot.model_state import JetlinkModelState
    m = JetlinkModelState.__new__(JetlinkModelState)
    m.sent = []
    m.prepare = lambda *a: None
    m.end = lambda after_enqueue: after_enqueue() if after_enqueue is not None else None
    m.send = lambda want_telemetry=False: m.sent.append(want_telemetry)
    m.client = Client()
    m.held, m.behind = False, None
    return m

  def joined(self, big, small):
    """jetlink's own JoiningModelState (what attach returns on the car), driving `big` or `small`."""
    from jetlink.openpilot.joining import JoiningModelState
    j = JoiningModelState.__new__(JoiningModelState)
    j.__dict__.update(_ask_in_control=None, _lagging=False, _frame_drop_ratio=0., _joined=None, _small=small, _big=big,
                      _active=big, _small_frames=0, _loading=False, _output_at=0., _log=types.SimpleNamespace(warning=print, exception=print))
    return j

  @pytest.mark.parametrize("big_drives", [True, False])
  def test_joined_model_asks_once_a_second(self, monkeypatch, tmp_path, big_drives):
    clock = [100.]
    monkeypatch.setattr(jetson_mod.time, "monotonic", lambda: clock[0])
    big, small_calls = self.model(), []
    small = types.SimpleNamespace(run=lambda bufs, transforms, inputs, after_enqueue=None: small_calls.append(after_enqueue))
    j, hook = self.joined(big, small), jetson_mod.JetsonGpsHook(str(tmp_path / "g.json"))
    if not big_drives:
      j._active = small
    hook.bind(j)
    hook.bind(j)  # once only
    for _ in range(80):  # 4 s at 20 Hz, modeld's run(bufs, transforms, inputs, None): no chestnut
      j.run({}, {}, {}, None)
      clock[0] += 0.05
    if big_drives:
      assert sum(big.sent) == 4 and big.sent[0]
      big.sent.clear()
      j.run({}, {}, {}, lambda: None)  # modeld's own after_enqueue still asks
      assert big.sent == [True]
    else:
      assert sum(a is not None for a in small_calls) == 4  # handed down, and the small model just calls it

  def test_frame_writes_each_new_fix(self, tmp_path):
    path = str(tmp_path / "g.json")
    m, hook = self.model(), jetson_mod.JetsonGpsHook(path)
    assert not hook.frame(m)
    m.client.last_state = {"gps": GPS}
    assert hook.frame(m)
    assert not hook.frame(m)
    assert JetsonGpsReader(path).poll(0.)['latitude'] == GPS['lat']
    m.client.last_state = {"gps": {**GPS, "lat": None}}
    assert not hook.frame(m)  # malformed: not written

  def test_adapter_hooks_leave_other_cars_alone(self, monkeypatch, tmp_path):
    """The Jetson GPS fallback is the Mazda port's: on any other car nothing is bound and no fix is written."""
    from openpilot.starpilot.models import jetlink_adapter

    class Base:
      def __init__(self, *a, **k):
        pass

      def attach(self, model, small, w, h):
        return joined

      def handed_over(self, model):
        return 'base'

    monkeypatch.setattr(jetlink_adapter, '_mazda', lambda: False)
    hooks = jetlink_adapter._gps_hooks(Base)()
    hooks._jetson_gps.path = str(tmp_path / "g.json")
    small, model = object(), object()
    joined = self.joined(self.model(), small)
    assert hooks.attach(model, small, 1, 1) is joined
    assert not getattr(joined.run, '_jetson_gps', False)
    joined._big.client.last_state = {"gps": GPS}
    assert hooks.handed_over(joined) == 'base'
    assert not (tmp_path / "g.json").exists()

  def test_adapter_hooks_bind_and_never_raise(self, monkeypatch, tmp_path):
    from openpilot.starpilot.models import jetlink_adapter

    class Base:
      def __init__(self, *a, **k):
        pass

      def attach(self, model, small, w, h):
        return joined

      def handed_over(self, model):
        return 'base'

    monkeypatch.setattr(jetlink_adapter, '_mazda', lambda: True)
    hooks = jetlink_adapter._gps_hooks(Base)()
    hooks._jetson_gps.path = str(tmp_path / "g.json")
    small, model = object(), object()
    joined = self.joined(self.model(), small)
    assert hooks.attach(model, small, 1, 1) is joined
    assert getattr(joined.run, '_jetson_gps', False)
    joined._big.client.last_state = {"gps": GPS}
    assert hooks.handed_over(joined) == 'base'
    assert JetsonGpsReader(hooks._jetson_gps.path).poll(0.) is not None
    monkeypatch.setattr(hooks._jetson_gps, 'frame', lambda m: 1 / 0)
    assert hooks.handed_over(joined) == 'base'  # a GPS failure never reaches modeld
    joined = small
    assert hooks.attach(model, small, 1, 1) is small  # no link: nothing bound


def test_gate_open_follows_the_session(tmp_path, monkeypatch):
  path = str(tmp_path / "g.json")
  write(record(GPS, 1.0), path)
  reader = JetsonGpsReader(path)
  assert reader.poll(1.)['latitude'] == GPS['lat'] and reader.gate_open
  monkeypatch.setattr(jetson_mod, 'mazda_session', lambda: False)
  reader.poll(2.1)
  assert not reader.gate_open
