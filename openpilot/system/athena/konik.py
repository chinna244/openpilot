#!/usr/bin/env python3
"""Restricted Konik presence and parked live view, isolated from comma Athena."""
import threading
import time

import openpilot.cereal.messaging as messaging

from websocket import ABNF, WebSocketTimeoutException, create_connection

from openpilot.common.konik_api import KonikApi, valid_id
from openpilot.common.hardware.hw import Paths
from openpilot.system.loggerd import konik_queue
from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.version import get_version
from openpilot.system.athena import rpc
from openpilot.system.webrtc.helpers import StreamRequestBody, post_stream_request, wait_for_webrtcd

ATHENA_HOST = 'wss://athena.konik.ai'
MAX_SDP_BYTES = 256 * 1024
PING_TIMEOUT = 70


class KonikConnection:
  def __init__(self, params: Params, exit_event: threading.Event):
    self.params = params
    self.exit_event = exit_event

  def configured(self, dongle_id: str | None = None) -> bool:
    current_id = self.params.get('KonikDongleId')
    return (self.params.get_bool('KonikUploadsEnabled') and valid_id(current_id)
            and (dongle_id is None or current_id == dongle_id))

  def status(self, value: str):
    if self.params.get('KonikConnectionStatus') != value:
      self.params.put('KonikConnectionStatus', value, block=True)

  def _check_stream_allowed(self):
    if self.exit_event.is_set() or not self.configured() or not self.params.get_bool('IsOffroad'):
      raise RuntimeError('Konik live view requires the device to be parked and Konik enabled')

  def start_stream(self, sdp: str, enabled: bool) -> dict:
    if not isinstance(sdp, str) or not sdp or len(sdp.encode()) > MAX_SDP_BYTES or type(enabled) is not bool:
      raise TypeError('invalid live-view offer')
    self._check_stream_allowed()
    # The existing manager owns camera and WebRTC lifecycle for both providers.
    self.params.put_bool('IsLiveStreaming', True, block=True)
    wait_for_webrtcd()
    self._check_stream_allowed()
    return post_stream_request(StreamRequestBody(sdp, ['wideRoad'], enabled, [], ['carState', 'deviceState'], konik=True))

  def list_upload_queue(self) -> list[dict]:
    dongle_id = self.params.get('KonikDongleId')
    if not self.configured(dongle_id):
      return []
    return konik_queue.queue_items(self.params, Paths.log_root(), dongle_id)

  def cancel_upload(self, upload_id: str | list[str]):
    return konik_queue.cancel(self.params, Paths.log_root(), self.params.get('KonikDongleId'), upload_id)

  def upload_files(self, files_data: list[dict]):
    if not self.configured():
      raise RuntimeError('Konik is not enabled and registered')
    return konik_queue.enqueue(self.params, Paths.log_root(), files_data)

  def route_viewed(self, route: str):
    return konik_queue.view_route(self.params, self.params.get('KonikDongleId'), route)

  def get_message(self, service: str, timeout: int = 1000) -> dict:
    # Stable's voltage badge uses this RPC; no other messaging service is exposed.
    if service != 'peripheralState' or type(timeout) is not int or not 1 <= timeout <= 5000:
      raise ValueError('only peripheralState with a timeout of 1 to 5000 ms is supported')
    if not self.configured():
      raise RuntimeError('Konik is not enabled and registered')
    socket = messaging.sub_sock('peripheralState', timeout=timeout)
    try:
      message = messaging.recv_one(socket)
      if message is None:
        raise TimeoutError('peripheralState unavailable')
      return message.to_dict()
    finally:
      del socket

  def methods(self) -> dict:
    # Never use the global dispatcher, queues, proxy or upload handlers from athenad.
    return {
      'echo': lambda s: s,
      'getMessage': self.get_message,
      'getVersion': get_version,
      'getNetworkType': HARDWARE.get_network_type,
      'getNetworkMetered': lambda: HARDWARE.get_network_metered(HARDWARE.get_network_type()),
      'startStream': self.start_stream,
      'listUploadQueue': self.list_upload_queue,
      'cancelUpload': self.cancel_upload,
      'uploadFilesToUrls': self.upload_files,
      'setRouteViewed': self.route_viewed,
    }

  @staticmethod
  def send_response(ws, response: str):
    # A backlog can contain thousands of files; use the same fragment size as comma Athena.
    encoded = response.encode()
    if len(encoded) <= 4096:
      ws.send(response)
      return
    for offset in range(0, len(encoded), 4096):
      chunk = encoded[offset:offset + 4096]
      opcode = ABNF.OPCODE_TEXT if offset == 0 else ABNF.OPCODE_CONT
      ws.send_frame(ABNF.create_frame(chunk, opcode, offset + 4096 >= len(encoded)))

  def session(self):
    ws = None
    self.params.remove('KonikLastAthenaPingTime')
    try:
      dongle_id = self.params.get('KonikDongleId')
      if not self.configured(dongle_id) or self.exit_event.is_set():
        return
      self.status('connecting')
      token = KonikApi(dongle_id).get_token()
      if not self.configured(dongle_id) or self.exit_event.is_set():
        return
      ws = create_connection(f'{ATHENA_HOST}/ws/v2/{dongle_id}', cookie='jwt=' + token, enable_multithread=True, timeout=5)
      self.status('connected')
      last_ping = time.monotonic()
      methods = self.methods()
      while not self.exit_event.is_set() and self.configured(dongle_id):
        try:
          opcode, data = ws.recv_data(control_frame=True)
        except WebSocketTimeoutException:
          if time.monotonic() - last_ping > PING_TIMEOUT:
            break
          continue
        if not self.configured(dongle_id):
          break
        if opcode == ABNF.OPCODE_PING:
          # websocket-client automatically replies to the server's ping.
          last_ping = time.monotonic()
          self.params.put('KonikLastAthenaPingTime', time.monotonic_ns(), block=True)
        elif opcode == ABNF.OPCODE_CLOSE:
          break
        elif opcode in (ABNF.OPCODE_TEXT, ABNF.OPCODE_BINARY):
          try:
            message = rpc.loads(data)
          except (TypeError, ValueError, UnicodeDecodeError):
            self.send_response(ws, rpc.dumps_error(None, 'parse error', rpc.PARSE_ERROR))
            continue
          if rpc.is_call(message):
            response = rpc.handle(message, methods)
            if 'id' in message:
              self.send_response(ws, response)
    finally:
      try:
        if ws is not None:
          ws.close()
      finally:
        self.params.remove('KonikLastAthenaPingTime')
        self.status('disconnected')


def main(exit_event: threading.Event | None = None):
  exit_event = exit_event or threading.Event()
  connection = KonikConnection(Params(), exit_event)
  retries = 0
  while not exit_event.is_set():
    try:
      connection.session()
    except Exception as e:
      # Do not log authentication tokens or arbitrary remote command contents.
      cloudlog.warning('Konik online connection interrupted (%s)', type(e).__name__)
    if exit_event.is_set():
      break
    retries = min(retries + 1, 6)
    connection.status('reconnecting')
    exit_event.wait(min(2 ** retries, 60))


if __name__ == '__main__':
  main()
