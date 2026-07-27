from openpilot.system.ubloxd.yuma_almanac_config import (
  PUBLIC_YUMA_ALMANAC_ENABLED_PARAM,
  public_yuma_almanac_enabled,
)


class FakeParams:
  def __init__(self, enabled: bool) -> None:
    self.enabled = enabled
    self.requested_keys: list[str] = []

  def get_bool(self, key: str) -> bool:
    self.requested_keys.append(key)
    return self.enabled


def test_public_yuma_gate_reads_configured_param():
  params = FakeParams(True)

  assert public_yuma_almanac_enabled(params)
  assert params.requested_keys == [
    PUBLIC_YUMA_ALMANAC_ENABLED_PARAM
  ]


def test_public_yuma_gate_fails_closed_without_params_api():
  assert not public_yuma_almanac_enabled(object())

def test_public_yuma_gate_fails_closed_when_read_raises(
  monkeypatch,
):
  logs = []
  monkeypatch.setattr(
    "openpilot.system.ubloxd.yuma_almanac_config.cloudlog.exception",
    logs.append,
  )

  class RaisingParams:
    def __init__(self, error: Exception) -> None:
      self.error = error

    def get_bool(self, key: str) -> bool:
      raise self.error

  for error in (
    OSError("injected Params I/O failure"),
    RuntimeError("injected Params failure"),
  ):
    assert not public_yuma_almanac_enabled(
      RaisingParams(error)
    )

  assert logs == [
    "Failed to read public YUMA feature gate",
    "Failed to read public YUMA feature gate",
  ]
