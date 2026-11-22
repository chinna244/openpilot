class FakeParams:
  """develop's sunnypilot FakeParams: get_bool from the given values."""
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key, *args, **kwargs):
    return bool(self.values.get(key, False))
