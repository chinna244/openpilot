"""Card logs the Mazda TJA latch going armed -> cleared once, from that tick's already-computed AOL inputs
(route 00000001--f5fdd9a482: main off dropped the latch 3 of 5 times with no fault recorded)."""
import ast
from pathlib import Path

CARD = Path(__file__).resolve().parents[4] / 'selfdrive/car/card.py'
ALLOWED_CALLS = {'bool', 'int', 'str', 'getattr', 'get'}


def _card_ast():
  return ast.parse(CARD.read_text())


def _calls(tree, name):
  return [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == name]


def _log_call(tree):
  calls = [c for c in _calls(tree, 'event') if c.args and isinstance(c.args[0], ast.Constant) and
           c.args[0].value == 'mazda_tja_latch_cleared']
  assert len(calls) == 1
  return calls[0]


def test_snapshot_before_update_and_log_after_engagement():
  tree = _card_ast()
  snapshot = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Name) and t.id == 'mazda_tja_latched' for t in n.targets)]
  assert len(snapshot) == 1
  intent_update = [c for c in _calls(tree, 'update') if ast.unparse(c.func) == 'self.aol_card_intent.update']
  engagement = [c for c in _calls(tree, 'observe_full_engagement') if ast.unparse(c.func).startswith('self.aol_card_intent')]
  log = _log_call(tree)
  assert len(intent_update) == 1 and len(engagement) == 1
  assert snapshot[0].lineno < intent_update[0].lineno < engagement[0].lineno < log.lineno


def test_logged_once_on_the_armed_to_cleared_edge_only():
  tree = _card_ast()
  log = _log_call(tree)
  guard = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and log in ast.walk(n) and
               not any(log in ast.walk(c) for c in ast.iter_child_nodes(n) if isinstance(c, ast.If)))
  assert ast.unparse(guard.test) == 'mazda_tja_latched and (not self.aol_card_intent.allowed_latch)'
  snapshot = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                  any(isinstance(t, ast.Name) and t.id == 'mazda_tja_latched' for t in n.targets))
  text = ast.unparse(snapshot.value)
  assert "self.CP.brand == 'mazda'" in text and "'tja_button'" in text and 'self.aol_card_intent.allowed_latch' in text


def test_logged_values_are_read_not_recomputed():
  log = _log_call(_card_ast())
  keys = {kw.arg for kw in log.keywords}
  assert {'reason', 'native_missing', 'native_stale', 'native_lost', 'transport_reset', 'transport_failed', 'fault_active'} <= keys
  for kw in log.keywords:
    for node in ast.walk(kw.value):
      if isinstance(node, ast.Call):
        name = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, 'id', None)
        assert name in ALLOWED_CALLS, (kw.arg, ast.unparse(node))
