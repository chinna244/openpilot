"""Test doubles (zoompilot develop's sunnypilot/selfdrive/car/tests/fakes.py FakeParams)."""
from openpilot.starpilot import schema_cache


class _Sink:
  def __init__(self):
    self.value = None

  def put(self, key, value, block=False):
    self.value = value


def envelope(key, raw):
  """StarPilot keeps torqued's caches and CarParamsPrevRoute in schema_cache envelopes; the tests
  store develop's raw bytes, so the fake wraps them on read as put_cache would have."""
  if key not in schema_cache.CACHE_KEYS or not isinstance(raw, bytes) or raw.startswith(schema_cache.MAGIC):
    return raw
  contract = schema_cache.CONTRACTS[key]
  sink = _Sink()
  with contract.root.from_bytes(raw) as msg:
    schema_cache.put_cache(sink, key, msg.as_builder())
  return sink.value


class FakeParams:
  """Dict-backed Params: bools and typed values."""

  def __init__(self, **values):
    self.values = dict(values)
    self.cleared: list = []

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, **kwargs):
    self.values[key] = value

  def get(self, key, **kwargs):
    return envelope(key, self.values.get(key))

  def put(self, key, value, **kwargs):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)

  def clear_all(self, flag):
    self.cleared.append(flag)
