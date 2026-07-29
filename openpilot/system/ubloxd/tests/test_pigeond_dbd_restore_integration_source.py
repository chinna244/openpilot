import ast
from pathlib import Path


PIGEOND = Path("openpilot/system/ubloxd/pigeond.py")
RUNTIME = Path("openpilot/system/ubloxd/navigation_database_restore_runtime.py")


def source_tree(path: Path) -> tuple[str, ast.Module]:
  source = path.read_text(encoding="utf-8")
  return source, ast.parse(source)


def named_node(tree: ast.Module, name: str) -> ast.AST:
  matches = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name == name]
  assert len(matches) == 1
  return matches[0]


def calls(node: ast.AST, name: str) -> list[ast.Call]:
  return [
    item
    for item in ast.walk(node)
    if isinstance(item, ast.Call)
    and (isinstance(item.func, ast.Name) and item.func.id == name or isinstance(item.func, ast.Attribute) and item.func.attr == name)
  ]


def test_initialize_receiver_cycle_has_no_legacy_restore_fallback() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "initialize_receiver_cycle")
  assert not calls(node, "restore_navigation_assistance")
  assert calls(node, "NavigationDatabaseRestoreRuntime")
  assert "navigation_database_runtime: NavigationDatabaseRestoreRuntime | None = None" in source


def test_live_loop_reuses_one_runtime_for_all_receiver_cycles() -> None:
  _source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  assert len(calls(node, "NavigationDatabaseRestoreRuntime")) == 1
  initialize_calls = calls(node, "initialize_receiver_cycle")
  assert len(initialize_calls) == 3
  for call in initialize_calls:
    keywords = {keyword.arg for keyword in call.keywords}
    assert "navigation_database_runtime" in keywords


def test_acquisition_latch_is_updated_before_receiver_processing() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert segment.index("receiver_frames_show_gnss_acquisition(frames)") < segment.index("process_receiver_frames(")
  assert "navigation_database_runtime.note_acquisition_started()" in segment


def test_database_decision_runs_before_yuma_transmission() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert segment.index("navigation_database_runtime.evaluate(") < segment.index("yuma_feature.evaluate_provisional(")
  assert segment.index("navigation_database_runtime.evaluate(") < segment.index("yuma_feature.evaluate(")
  assert segment.count("navigation_database_runtime.note_yuma_sent()") == 2


def test_skipped_database_never_populates_restored_quality_fields() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(
    tree,
    "navigation_assistance_result_from_database_execution",
  )
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert "evaluated_quality if disposition.database_available else None" in segment
  assert "restored_navigation_quality=restored_quality" in segment
  assert "captured_gps_almanac_available=" in segment
  assert "captured_gps_almanac_satellite_ids=" in segment


def test_yuma_database_state_uses_terminal_disposition() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "yuma_database_restore_state")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert "disposition = result.database_restore_disposition" in segment
  assert "if disposition.database_available" in segment


def test_new_runtime_does_not_power_cycle_or_reinitialize_receiver() -> None:
  _source, tree = source_tree(RUNTIME)
  assert not calls(tree, "set_power")
  assert not calls(tree, "initialize_receiver_cycle")


def test_runtime_persists_linux_boot_identity_and_terminal_state() -> None:
  source, tree = source_tree(RUNTIME)
  runtime = named_node(tree, "NavigationDatabaseRestoreRuntime")
  segment = ast.get_source_segment(source, runtime)
  assert segment is not None
  assert "boot_id_reader" in segment
  assert "_restore_persisted_state" in segment
  assert "recovered_interrupted_attempt" in segment
  assert "position_assistance_claimed" in source
  assert "acquisition_started" in source
  assert "yuma_sent" in source


def test_pigeond_change_does_not_add_direct_power_cycle_call() -> None:
  _source, tree = source_tree(PIGEOND)
  for name in (
    "initialize_receiver_cycle",
    "navigation_assistance_result_from_database_execution",
    "receiver_frames_show_gnss_acquisition",
    "run_receiving",
  ):
    assert not calls(named_node(tree, name), "set_power")
