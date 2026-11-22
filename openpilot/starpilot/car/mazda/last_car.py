"""The car this device runs (or last ran), for the port's Mazda-only processes that start before card fingerprints
(soundd's startup hold, Konik backup, the Jetson GPS producer). Unknown counts as not a Mazda, so every other car
keeps StarPilot's behaviour; callers that latch a decision latch only a known brand."""
from openpilot.common.swaglog import cloudlog


def _brand_from(raw) -> str | None:
  from openpilot.cereal import messaging
  from opendbc.car.structs import car
  brand = messaging.log_from_bytes(raw, car.CarParams).brand
  return brand or None


def car_brand(params, live_only: bool = False) -> str | None:
  """This session's CarParams (card's live write, plain capnp; cleared at manager start and each onroad transition)
  first, then, unless live_only, the last drive's CarParamsPersistent (a schema_cache envelope, possibly another car
  after a device move); None when neither says."""
  try:
    raw = params.get("CarParams")
    if isinstance(raw, bytes) and raw:
      return _brand_from(raw)
  except Exception:
    cloudlog.exception("mazda last_car: could not read CarParams")
  if live_only:
    return None
  try:
    from openpilot.starpilot.schema_cache import get_cache
    raw = get_cache(params, "CarParamsPersistent")
    if isinstance(raw, bytes) and raw:
      return _brand_from(raw)
  except Exception:
    cloudlog.exception("mazda last_car: could not read CarParamsPersistent")
  return None


def is_mazda(params, CP=None) -> bool:
  """A known current brand (this drive's CarParams handed in) wins over anything stored."""
  current = getattr(CP, 'brand', None) or None
  return (current or car_brand(params)) == "mazda"
