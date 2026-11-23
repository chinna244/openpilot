"""Card hands the Mazda controller Panda's long grant as it reads it (route 00000001--f5fdd9a482: ACC_ACTIVE went out
30-90 ms before the grant at 4 of 5 engagements). The controller side, with real Panda safety, is
opendbc/car/mazda/tests/test_mazda_aol_long_grant.py."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

CARD = Path(__file__).resolve().parents[4] / 'selfdrive/car/card.py'


def _assignment():
  tree = ast.parse(CARD.read_text())
  found = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
           any(ast.unparse(t) == 'self.CI.CC.aol_long_native' for t in n.targets)]
  assert len(found) == 1
  guard = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and found[0] in n.body)
  native = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
            any(isinstance(t, ast.Name) and t.id == 'native' for t in n.targets)]
  return found[0], guard, native


def test_set_only_for_the_mazda_native_owner_after_native_is_read():
  assign, guard, native = _assignment()
  assert ast.unparse(guard.test) == ('self.aol_card_intent is not None and mazda_aol and '
                                     'self.aol_card_intent.explicit_latch')
  assert native and all(n.lineno < assign.lineno for n in native)
  assert "current_native(" in ast.unparse(native[0].value)  # the existing freshness rules


@pytest.mark.parametrize("native,expected", [
  (None, None),  # no fresh native (stale, missing, wrong session): the controller keeps the engagement's grant
  (SimpleNamespace(requestedLongitudinal=True, longitudinalAllowed=True, observedMonoTime=7), (True, 7)),
  (SimpleNamespace(requestedLongitudinal=True, longitudinalAllowed=False, observedMonoTime=8), (False, 8)),
  (SimpleNamespace(requestedLongitudinal=False, longitudinalAllowed=True, observedMonoTime=9), (False, 9)),
])
def test_value_is_the_fresh_grant_and_its_observation(native, expected):
  assign, _, _ = _assignment()
  value = eval(compile(ast.Expression(assign.value), str(CARD), 'eval'), {'bool': bool, 'int': int}, {'native': native})
  assert value == expected
