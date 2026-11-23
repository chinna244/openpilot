"""Route 00000002--c9d55e7093 t=207.423: card's transport check failed on an ordinary brake cancel and the
log could not say which predicate. The check stays fail-closed; the first failure after a healthy run is
logged once (rate-limited) with its site and inputs."""
import inspect
from types import SimpleNamespace
from unittest import mock

from openpilot.starpilot.aol import transport_pause as tp


def _feedback():
  fb = tp.TransportPauseFeedback.__new__(tp.TransportPauseFeedback)
  fb.failed = False
  fb.fail_site = ''
  fb.fail_checks = None
  fb.fail_logged_ns = 0
  fb.session = 'axis'
  fb.signature = None
  return fb


def _fail(fb, site):
  def impl(sm, cp, cs, now_ns, **kw):
    fb.failed = True
    fb.fail_site = site
    fb.fail_checks = (False, True, True, False, True, True, True, 'none')
    return True
  return impl


def test_first_failure_logs_site_checks_and_args_once():
  fb = _feedback()
  fb._observe = _fail(fb, 'not_current_or_continuity')
  with mock.patch('openpilot.common.swaglog.cloudlog.event') as event:
    assert fb.observe(SimpleNamespace(), None, None, 5_000, latched=True, intent_paused=False) is True
  event.assert_called_once()
  name, rec = event.call_args.args[0], event.call_args.kwargs
  assert name == 'aol_transport_failed' and rec['site'] == 'not_current_or_continuity'
  assert rec['checks'][3] is False and rec['args'] == {'latched': True, 'intent_paused': False} and rec['now_ns'] == 5_000


def test_sticky_failure_and_rate_limit_do_not_spam():
  fb = _feedback()
  fb._observe = _fail(fb, 'not_current_or_continuity')
  with mock.patch('openpilot.common.swaglog.cloudlog.event') as event:
    fb.observe(SimpleNamespace(), None, None, 1_000)
    fb.observe(SimpleNamespace(), None, None, 2_000)  # already failed: sticky, not a new failure
    fb.failed = False
    fb.observe(SimpleNamespace(), None, None, 3_000)  # a new failure inside the 10 s window
    fb.failed = False
    fb.observe(SimpleNamespace(), None, None, 3_000 + tp.FAILURE_LOG_PERIOD_NS)
  assert event.call_count == 2


def test_result_is_unchanged_and_healthy_calls_do_not_log():
  fb = _feedback()
  fb._observe = lambda *a, **k: False
  with mock.patch('openpilot.common.swaglog.cloudlog.event') as event:
    assert fb.observe(SimpleNamespace(), None, None, 1) is False
  event.assert_not_called()


def test_record_survives_missing_inputs():
  rec = tp.failure_record(_feedback(), SimpleNamespace(), None, None, 7, {})
  assert rec['site'] == '' and rec['axis'] is None and rec['safety'] is None and rec['car'] is None


def test_every_hard_return_in_observe_is_tagged():
  lines = inspect.getsource(tp.TransportPauseFeedback._observe).split('\n')
  hard = [i for i, line in enumerate(lines) if line.strip() in ('return True', 'return hard')]
  assert len(hard) == 13
  for i in hard:
    assert "self.fail_site = '" in lines[i - 1], lines[i]
