from types import SimpleNamespace

import pytest

from openpilot.system.ubloxd import gps_drive_audit, pigeond


def test_route_metrics_uses_real_gps_schema_and_trusted_provenance():
  metrics = gps_drive_audit.RouteMetrics("00000093--a1ef00c9c2")
  metrics.note_time(100.0)

  metrics.process_gps(
    100.1,
    SimpleNamespace(
      flags=0,
      hasFix=False,
      unixTimestampMillis=1_774_360_000_000,
      satelliteCount=0,
      horizontalAccuracy=41_000.0,
      latitude=0.0,
      longitude=0.0,
    ),
  )

  assert metrics.positive_timestamp_samples == 1
  assert metrics.first_receiver_utc is None

  provenance_message = b",".join((
    b'{"msg":"GPS receiver UTC provenance',
    b' cycle=1',
    b' classification=receiver_utc_unassisted_gnss',
    b' reason=fresh_gnss_time_evidence',
    b' independent=true"}',
  ))
  metrics.process_log_message(348.95, provenance_message)
  metrics.process_gps(
    414.95,
    SimpleNamespace(
      flags=1,
      hasFix=True,
      unixTimestampMillis=1_774_898_400_000,
      satelliteCount=5,
      horizontalAccuracy=41.6,
      latitude=32.8,
      longitude=-96.8,
    ),
  )
  metrics.process_gps(
    415.36,
    SimpleNamespace(
      flags=1,
      hasFix=True,
      unixTimestampMillis=1_774_898_400_410,
      satelliteCount=6,
      horizontalAccuracy=23.69,
      latitude=32.80001,
      longitude=-96.80001,
    ),
  )

  assert metrics.relative(metrics.first_receiver_utc) == pytest.approx(248.95)
  assert metrics.relative(metrics.first_fix) == pytest.approx(314.95)
  assert metrics.relative(metrics.first_25m) == pytest.approx(315.36)
  assert metrics.best_accuracy == 23.69
  assert metrics.max_satellites == 6


def test_rawx_week_and_leap_require_nonempty_measurements():
  metrics = gps_drive_audit.RouteMetrics("route")
  metrics.note_time(10.0)

  metrics.process_rawx(
    10.1,
    SimpleNamespace(measurements=(), gpsWeek=2411, leapSeconds=18),
  )

  assert metrics.first_rawx == 10.1
  assert metrics.first_nonempty_rawx is None
  assert metrics.first_valid_gps_week is None
  assert metrics.first_valid_leap_second is None

  measurements = (
    SimpleNamespace(gnssId=0),
    SimpleNamespace(gnssId=6),
  )
  metrics.process_rawx(
    258.98,
    SimpleNamespace(
      measurements=measurements,
      gpsWeek=2411,
      leapSeconds=18,
    ),
  )

  assert metrics.relative(metrics.first_nonempty_rawx) == pytest.approx(248.98)
  assert metrics.relative(metrics.first_gps_measurement) == pytest.approx(248.98)
  assert metrics.relative(metrics.first_glonass_measurement) == pytest.approx(248.98)
  assert metrics.relative(metrics.first_valid_gps_week) == pytest.approx(248.98)
  assert metrics.relative(metrics.first_valid_leap_second) == pytest.approx(248.98)


def test_route_selection_supports_exact_and_latest(tmp_path):
  older = tmp_path / "00000092--d36d5b033c--0"
  newer = tmp_path / "00000093--a1ef00c9c2--0"
  older.mkdir()
  newer.mkdir()

  selections = [
    gps_drive_audit.RouteSelection(
      "00000093--a1ef00c9c2",
      ((0, newer),),
      200.0,
    ),
    gps_drive_audit.RouteSelection(
      "00000092--d36d5b033c",
      ((0, older),),
      100.0,
    ),
  ]

  assert [selection.route for selection in gps_drive_audit.select_routes(selections, None, 1)] == [
    "00000093--a1ef00c9c2"
  ]
  assert [
    selection.route
    for selection in gps_drive_audit.select_routes(
      selections,
      "00000092--d36d5b033c",
      None,
    )
  ] == ["00000092--d36d5b033c"]


def test_checksums_are_relative_and_self_verifying(tmp_path):
  (tmp_path / "nested").mkdir()
  (tmp_path / "nested" / "evidence.txt").write_text("evidence\n", encoding="utf-8")
  (tmp_path / "summary.txt").write_text("summary\n", encoding="utf-8")

  gps_drive_audit.generate_checksums(tmp_path)
  checksum_text = (tmp_path / "SHA256SUMS.txt").read_text(encoding="utf-8")

  assert "/data/" not in checksum_text
  assert "  /" not in checksum_text
  assert "nested/evidence.txt" in checksum_text
  gps_drive_audit.verify_checksums(tmp_path)


def test_state_snapshot_labels_capture_identity(tmp_path):
  assistance_root = tmp_path / "assistance"
  destination = tmp_path / "state"
  assistance_root.mkdir()
  cache_payload = '{"version":1,"receiver_fingerprint":"test"}\n'
  (assistance_root / "navigation_cache.json").write_text(
    cache_payload,
    encoding="utf-8",
  )

  gps_drive_audit.copy_state_files(
    assistance_root,
    destination,
    "capture-boot",
  )

  scope = (destination / "STATE_SCOPE.txt").read_text(encoding="utf-8")
  assert "State files are current-device snapshots" in scope
  assert "capture_boot_id=capture-boot" in scope
  assert "navigation_cache.json: copied bytes=" in scope
  assert (
    destination / "navigation_cache.json"
  ).read_text(encoding="utf-8") == cache_payload


def test_decode_param_handles_bytes_without_encoding_argument():
  assert gps_drive_audit.decode_param(b"cd0e0fdc") == "cd0e0fdc"
  assert gps_drive_audit.decode_param(None) == "<missing>"


def test_yuma_commit_metadata_uses_supported_params_get(monkeypatch):
  captured = {}

  class ParamsStub:
    def get(self, key):
      assert key == "GitCommit"
      return b"cd0e0fdc4d4e18eed3ceeeb4bbed76d3a3ea9259"

  outcome = SimpleNamespace(
    receiver_cycle=2,
    completion_utc=None,
    trusted_now_utc=None,
  )

  def fake_save(path, saved_outcome, **kwargs):
    captured["path"] = path
    captured["outcome"] = saved_outcome
    captured.update(kwargs)

  monkeypatch.setattr(pigeond, "save_yuma_supplementation_outcome", fake_save)
  pigeond.persist_yuma_supplementation_outcome(outcome, ParamsStub())

  assert captured["outcome"] is outcome
  assert captured["commit"] == "cd0e0fdc4d4e18eed3ceeeb4bbed76d3a3ea9259"
  assert captured["receiver_cycle"] == 2

def test_yuma_commit_metadata_supports_legacy_params_get(monkeypatch):
  captured = {}

  class ParamsStub:
    def get(self, key, encoding):
      assert key == "GitCommit"
      assert encoding == "utf-8"
      return "legacy-commit"

  outcome = SimpleNamespace(
    receiver_cycle=3,
    completion_utc=None,
    trusted_now_utc=None,
  )

  def fake_save(path, saved_outcome, **kwargs):
    captured["path"] = path
    captured["outcome"] = saved_outcome
    captured.update(kwargs)

  monkeypatch.setattr(pigeond, "save_yuma_supplementation_outcome", fake_save)
  pigeond.persist_yuma_supplementation_outcome(outcome, ParamsStub())

  assert captured["outcome"] is outcome
  assert captured["commit"] == "legacy-commit"
  assert captured["receiver_cycle"] == 3
