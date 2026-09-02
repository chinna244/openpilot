"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import base64
import gzip

from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.common.test import OpenpilotTestCase


def _b64(s: str, compressed: bool = False) -> str:
  raw = s.encode()
  if compressed:
    raw = gzip.compress(raw)
  return base64.b64encode(raw).decode()


class TestSunnylinkdMethods(OpenpilotTestCase):
  def setup_method(self):
    self.saved_params = []

    self.offroad_requests = []

    self.original_save = sunnylinkd.save_param_from_base64_encoded_string
    self.original_request_offroad = sunnylinkd.request_offroad_mode

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    def mock_request_offroad_mode(params, enable):
      self.offroad_requests.append(enable)

    sunnylinkd.save_param_from_base64_encoded_string = mock_save_param  # ty: ignore[invalid-assignment]
    sunnylinkd.request_offroad_mode = mock_request_offroad_mode  # ty: ignore[invalid-assignment]

  def teardown_method(self):
    sunnylinkd.save_param_from_base64_encoded_string = self.original_save  # ty: ignore[invalid-assignment]
    sunnylinkd.request_offroad_mode = self.original_request_offroad  # ty: ignore[invalid-assignment]

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
      "OnroadCycleRequested": "1",
      "AlphaLongitudinalEnabled": "1",
      "OffroadModeRequested": "1",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "SpeedLimitOffset": "5",
      "MyCustomParam": "123"
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "SpeedLimitOffset" in keys_saved
    assert "MyCustomParam" in keys_saved

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "SpeedLimitOffset": "10"
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "SpeedLimitOffset"
    assert self.saved_params[0][1] == "10"

  # OffroadMode is never stored from remote: hardwared and pandad act on it immediately,
  # so entering offroad goes through the local request path (hand-back, standstill gate)
  # and only leaving offroad clears the param, exactly like the local UI

  def test_remote_offroad_mode_enter_is_a_request(self):
    sunnylinkd.saveParams({"OffroadMode": _b64("1")})
    assert self.saved_params == []
    assert self.offroad_requests == [True]

  def test_remote_offroad_mode_leave_clears(self):
    sunnylinkd.saveParams({"OffroadMode": _b64("0")})
    assert self.saved_params == []
    assert self.offroad_requests == [False]

  def test_remote_offroad_mode_accepts_bool_spellings_and_compression(self):
    sunnylinkd.saveParams({"OffroadMode": _b64("true", compressed=True)}, compression=True)
    sunnylinkd.saveParams({"OffroadMode": _b64("false", compressed=True)}, compression=True)
    assert self.offroad_requests == [True, False]

  def test_remote_offroad_mode_redirect_does_not_swallow_neighbours(self):
    sunnylinkd.saveParams({"OffroadMode": _b64("1"), "SpeedLimitOffset": _b64("10")})
    assert [p[0] for p in self.saved_params] == ["SpeedLimitOffset"]
    assert self.offroad_requests == [True]

  def test_remote_offroad_mode_bad_payload_is_dropped(self):
    sunnylinkd.saveParams({"OffroadMode": "not base64!!"})
    assert self.saved_params == []
    assert self.offroad_requests == []
