"""Upload-only Konik identity (develop's common/api/konik.py); comma's active API and DongleId are never changed.

It always signs with the device's comma key in /persist, independent of StarPilot's active cloud provider, so the
Konik backup keeps the identity it had on develop."""
import os
import re
import unicodedata
from datetime import datetime, timedelta, UTC

import jwt
import requests

from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.version import get_version

API_HOST = "https://api.konik.ai"
PAIRING_HOST = "https://stable.konik.ai"

# name: jwt signature algorithm
KEYS = {"id_rsa": "RS256",
        "id_ecdsa": "ES256"}


class BaseApi:
  def __init__(self, dongle_id, api_host, user_agent="openpilot-"):
    self.dongle_id = dongle_id
    self.api_host = api_host
    self.user_agent = user_agent
    self.jwt_algorithm, self.private_key, _ = self.get_key_pair()

  def get(self, *args, **kwargs):
    return self.request('GET', *args, **kwargs)

  def post(self, *args, **kwargs):
    return self.request('POST', *args, **kwargs)

  def request(self, method, endpoint, timeout=None, access_token=None, **params):
    return self.api_get(endpoint, method=method, timeout=timeout, access_token=access_token, **params)

  def _get_token(self, payload_extra=None, expiry_hours=1, **extra_payload):
    now = datetime.now(UTC).replace(tzinfo=None)
    payload = {
      'identity': self.dongle_id,
      'nbf': now,
      'iat': now,
      'exp': now + timedelta(hours=expiry_hours),
      **extra_payload
    }
    if payload_extra is not None:
      payload.update(payload_extra)
    assert self.private_key is not None
    token = jwt.encode(payload, self.private_key, algorithm=self.jwt_algorithm)
    if isinstance(token, bytes):
      token = token.decode('utf8')
    return token

  def get_token(self, payload_extra=None, expiry_hours=1):
    return self._get_token(payload_extra, expiry_hours)

  def remove_non_ascii_chars(self, text):
    normalized_text = unicodedata.normalize('NFD', text)
    ascii_encoded_text = normalized_text.encode('ascii', 'ignore')
    return ascii_encoded_text.decode()

  def api_get(self, endpoint, method='GET', timeout=None, access_token=None, session=None, json=None, **params):
    headers = {}
    if access_token is not None:
      headers['Authorization'] = "JWT " + access_token

    version = self.remove_non_ascii_chars(get_version())
    headers['User-Agent'] = self.user_agent + version

    req = requests if session is None else session
    return req.request(method, f"{self.api_host}/{endpoint}", timeout=timeout, headers=headers, json=json, params=params)

  @staticmethod
  def get_key_pair() -> tuple[str, str, str] | tuple[None, None, None]:
    for key in KEYS:
      if os.path.isfile(Paths.persist_root() + f'/comma/{key}') and os.path.isfile(Paths.persist_root() + f'/comma/{key}.pub'):
        with open(Paths.persist_root() + f'/comma/{key}') as private, open(Paths.persist_root() + f'/comma/{key}.pub') as public:
          return KEYS[key], private.read(), public.read()
    return None, None, None


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
