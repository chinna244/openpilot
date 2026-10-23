"""Upload-only Konik identity; comma's active API and DongleId are never changed."""
import re

from openpilot.common.api.base import BaseApi
from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params

API_HOST = "https://api.konik.ai"
PAIRING_HOST = "https://stable.konik.ai"


class KonikApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, API_HOST)


def valid_id(dongle_id) -> bool:
  return isinstance(dongle_id, str) and dongle_id != "UnregisteredDevice" and re.fullmatch(r"[\w-]{1,128}", dongle_id, re.ASCII) is not None


def register(params: Params) -> str:
  dongle_id = params.get("KonikDongleId")
  if valid_id(dongle_id):
    return dongle_id

  api = KonikApi(None)
  _, _, public_key = api.get_key_pair()
  if not public_key:
    raise RuntimeError("Device key unavailable for Konik registration")
  response = api.api_get("v2/pilotauth/", method="POST", timeout=15, serial=HARDWARE.get_serial(),
                         imei=HARDWARE.get_imei(), imei2="", public_key=public_key,
                         register_token=api.get_token({'register': True}))
  if response.status_code not in (200, 201):
    raise RuntimeError(f"Konik registration returned {response.status_code}")
  dongle_id = response.json().get("dongle_id")
  if not valid_id(dongle_id):
    raise RuntimeError("Konik registration returned an invalid device ID")
  params.put("KonikDongleId", dongle_id, block=True)
  return dongle_id


def pairing_url(params: Params) -> str | None:
  dongle_id = params.get("KonikDongleId")
  if not valid_id(dongle_id):
    return None
  return f"{PAIRING_HOST}/?pair={KonikApi(dongle_id).get_token({'pair': True}, expiry_hours=1)}"
