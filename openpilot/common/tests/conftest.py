"""Shared pytest hooks for common tests."""

from __future__ import annotations

import pytest


_UBLOX_ONLY_SKIP = frozenset({
  "test_selected_fails_alternate_healthy_failover",
  "test_arbiter_restart_fresh_startup_race",
  "test_arbiter_reset_allows_startup_race_again",
})


def pytest_collection_modifyitems(config, items):
  skip_qcom = pytest.mark.skip(reason="qcomgpsd removed; u-blox-only GPS on mici")
  for item in items:
    if "test_gps_source_arbiter.py" not in str(item.fspath):
      continue
    if item.name == "test_qcomgpsd_not_managed":
      continue
    if "qcom" in item.name.lower() or item.name in _UBLOX_ONLY_SKIP:
      item.add_marker(skip_qcom)
