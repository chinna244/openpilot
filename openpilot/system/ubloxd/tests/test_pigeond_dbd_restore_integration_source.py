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


def test_initialize_receiver_cycle_uses_receiver_cycle_runtime_adapter() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "initialize_receiver_cycle")
  restore_calls = calls(node, "restore_navigation_assistance")
  assert len(restore_calls) == 1
  keywords = {keyword.arg for keyword in restore_calls[0].keywords}
  assert "navigation_database_runtime" in keywords
  assert "authorized_time" in keywords
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert "allow_legacy_direct_restore" not in segment


def test_live_loop_creates_fresh_state_for_each_receiver_cycle() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert len(
    calls(node, "create_receiver_cycle_assistance_state")
  ) == 3
  assert len(
    calls(node, "prepare_receiver_cycle_response_state")
  ) == 2
  initialize_calls = calls(node, "initialize_receiver_cycle")
  assert len(initialize_calls) == 3
  for call in initialize_calls:
    keywords = {keyword.arg for keyword in call.keywords}
    assert "navigation_database_runtime" in keywords
    assert "response_state_prepared" not in keywords

  navigation_factory = named_node(
    tree,
    "create_receiver_cycle_navigation_state",
  )
  navigation_calls = calls(
    navigation_factory,
    "NavigationDatabaseRestoreRuntime",
  )
  assert len(navigation_calls) == 1
  assert any(
    keyword.arg == "new_receiver_cycle"
    and isinstance(keyword.value, ast.Constant)
    and keyword.value.value is True
    for keyword in navigation_calls[0].keywords
  )

  assistance_factories = [
    item
    for item in node.body
    if (
      isinstance(item, ast.FunctionDef)
      and item.name == "create_receiver_cycle_assistance_state"
    )
  ]
  assert len(assistance_factories) == 1
  retry_calls = calls(
    assistance_factories[0],
    "PositionAssistanceRetryRuntime",
  )
  assert len(retry_calls) == 1
  assert any(
    keyword.arg == "new_receiver_cycle"
    and isinstance(keyword.value, ast.Constant)
    and keyword.value.value is True
    for keyword in retry_calls[0].keywords
  )


def test_acquisition_latch_is_updated_before_receiver_processing() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert segment.index("receiver_frames_show_gnss_acquisition(frames)") < segment.index("process_receiver_frames(")
  assert "handle_receiver_acquisition_state(" in segment


def test_database_decision_runs_before_yuma_transmission() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None

  database_decision = segment.index(
    "navigation_database_runtime.evaluate("
  )
  assert database_decision < segment.index(
    "yuma_feature.evaluate_provisional("
  )
  assert database_decision < segment.index(
    "yuma_feature.evaluate("
  )

  assert "provisional_yuma_outcome = yuma_feature.evaluate_provisional(\n      send_yuma_message," in segment
  assert "yuma_outcome = yuma_feature.evaluate(\n      send_yuma_message," in segment
  assert "navigation_database_runtime.note_yuma_sent()" not in segment

  helper = named_node(tree, "send_yuma_with_durable_claim")
  helper_segment = ast.get_source_segment(source, helper)
  assert helper_segment is not None
  assert helper_segment.index(
    "navigation_database_runtime.claim_yuma_transmission()"
  ) < helper_segment.index("send_message(message)")
  assert "raise YumaAssistanceStateUnavailableError(" in helper_segment
  assert "return False" not in helper_segment


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


def test_yuma_database_state_preserves_pending_disposition() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "yuma_database_restore_state")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  assert "disposition = result.database_restore_disposition" in segment
  assert "if disposition is NavigationDatabaseRestoreDisposition.PENDING" in segment
  assert "return YumaDatabaseRestoreState.PENDING" in segment
  assert "if disposition.database_available" in segment


def test_initial_dbd_decision_precedes_time_and_normal_configuration() -> None:
  source, tree = source_tree(PIGEOND)
  init_node = named_node(tree, "init")
  init_segment = ast.get_source_segment(source, init_node)
  assert init_segment is not None
  assert init_segment.index("start_pigeon_transport(pigeon)") < init_segment.index("initialization.run()")
  assert init_segment.index("initialization.run()") < init_segment.index("finish_pigeon_initialization(pigeon)")

  node = named_node(tree, "initialize_receiver_cycle")
  segment = ast.get_source_segment(source, node)
  assert segment is not None
  restore = segment.index("restore_navigation_assistance(")
  time_assistance = segment.index("send_time_assistance(")
  observations = segment.index("provenance.enable_receiver_observations(")
  assert restore < time_assistance < observations
  assert "install_pre_acquisition_initialization(" in segment


def test_pre_database_setup_does_not_ignore_acquisition_frames() -> None:
  source, tree = source_tree(PIGEOND)
  initialize = named_node(tree, "initialize_receiver_cycle")
  segment = ast.get_source_segment(source, initialize)
  assert segment is not None
  assert "poll_mon_ver(pigeon)" in segment
  assert "configure_navx5_ack_aiding(pigeon, mon_ver_info)" in segment

  unrelated = named_node(tree, "_queue_unrelated_frames")
  unrelated_segment = ast.get_source_segment(source, unrelated)
  assert unrelated_segment is not None
  assert "pigeon.queue_pending_frames(" in unrelated_segment
  assert "pigeon.dispatch_pending_frames()" in unrelated_segment


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


def test_controlled_stop_start_brackets_pre_database_window() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "paused_gnss_acquisition")
  segment = ast.get_source_segment(source, node)
  assert segment is not None

  stop = segment.index("pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)")
  yielded = segment.index("yield")
  failure_path = segment.index("except BaseException:")
  success_path = segment.index("else:")
  start = segment.index("pigeon.send(CONTROLLED_GNSS_START_MESSAGE)")

  assert stop < yielded < failure_path < success_path < start
  assert "raise" in segment[failure_path:success_path]


def test_power_on_stop_precedes_baud_transition_and_transactions() -> None:
  source, tree = source_tree(PIGEOND)
  baud = named_node(tree, "init_baudrate")
  segment = ast.get_source_segment(source, baud)
  assert segment is not None
  power_on_baud = segment.index("pigeon.set_baud(9600)")
  stop = segment.index("pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)")
  baud_transition = segment.index(r'pigeon.send(b"\x24\x50\x55\x42\x58')
  assert power_on_baud < stop < baud_transition

  initialize = named_node(tree, "initialize_receiver_cycle")
  initialize_segment = ast.get_source_segment(source, initialize)
  assert initialize_segment is not None
  assert initialize_segment.index("restore_navigation_assistance(") < (initialize_segment.index("send_time_assistance("))

def test_structured_telemetry_keeps_assistance_command_order() -> None:
  source, tree = source_tree(PIGEOND)

  paused = named_node(tree, "paused_gnss_acquisition")
  paused_segment = ast.get_source_segment(source, paused)
  assert paused_segment is not None
  assert paused_segment.index(
    "pigeon.send(CONTROLLED_GNSS_STOP_MESSAGE)"
  ) < paused_segment.index("yield")
  assert paused_segment.index("yield") < paused_segment.index(
    "pigeon.send(CONTROLLED_GNSS_START_MESSAGE)"
  )

  initialize = named_node(tree, "initialize_receiver_cycle")
  initialize_segment = ast.get_source_segment(source, initialize)
  assert initialize_segment is not None
  restore = initialize_segment.index(
    "restore_navigation_assistance("
  )
  time_assistance = initialize_segment.index(
    "send_time_assistance("
  )
  acquisition_claim = initialize_segment.index(
    "claim_acquisition_start("
  )
  assert restore < time_assistance < acquisition_claim

  restore_helper = named_node(
    tree,
    "restore_navigation_assistance",
  )
  restore_segment = ast.get_source_segment(
    source,
    restore_helper,
  )
  assert restore_segment is not None
  database_restore = restore_segment.index(
    "navigation_database_runtime.evaluate("
  )
  position_restore = restore_segment.index(
    "navigation_database_runtime.send_position_once("
  )
  assert database_restore < position_restore

  run_receiving = named_node(tree, "run_receiving")
  run_segment = ast.get_source_segment(source, run_receiving)
  assert run_segment is not None
  receiver_start = run_segment.index(
    "initialize_receiver_cycle("
  )
  runtime_yuma = run_segment.index(
    "yuma_feature.evaluate("
  )
  assert receiver_start < runtime_yuma


# COMMIT9_DBD_RUNTIME_BEFORE_RECEIVER_TEST


def test_dbd_runtime_initialization_precedes_receiver_construction_and_io() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None

  assistance_state = segment.index(
    "create_receiver_cycle_assistance_state("
  )
  pigeon = segment.index("pigeon = TTYPigeon(")
  first_cycle = segment.index("initialize_receiver_cycle(")
  assert assistance_state < pigeon < first_cycle

  initialize = named_node(tree, "initialize_receiver_cycle")
  initialize_segment = ast.get_source_segment(source, initialize)
  assert initialize_segment is not None
  fallback_runtime = initialize_segment.index(
    "or NavigationDatabaseRestoreRuntime(receiver_fingerprint)"
  )
  receiver_start = initialize_segment.index("init(pigeon)")
  assert fallback_runtime < receiver_start


def test_recovery_flushes_old_frames_before_fresh_cycle_state() -> None:
  source, tree = source_tree(PIGEOND)
  node = named_node(tree, "run_receiving")
  segment = ast.get_source_segment(source, node)
  assert segment is not None

  for reason in ("no_data_watchdog", "all_zero_data"):
    cycle = segment.index(f'"{reason}"')
    cancel = segment.rfind(
      "position_assistance_retry.cancel_receiver_cycle(now)",
      0,
      cycle,
    )
    assert cancel >= 0
    response_reset = segment.index(
      "prepare_receiver_cycle_response_state(pigeon)",
      cancel,
      cycle,
    )
    fresh_state = segment.index(
      "create_receiver_cycle_assistance_state(",
      response_reset,
      cycle,
    )
    assert cancel < response_reset < fresh_state < cycle


def test_yuma_assistance_state_unavailable_is_terminal_without_retry() -> None:
  source, tree = source_tree(PIGEOND)
  helper = named_node(
    tree,
    "yuma_assistance_state_unavailable_outcome",
  )
  helper_segment = ast.get_source_segment(source, helper)
  assert helper_segment is not None
  assert 'getattr(result, "assistance_state_unavailable", False)' in helper_segment
  assert "is True" in helper_segment
  assert "YumaAlmanacTransmitStatus" not in helper_segment
  assert "attempted_satellite_ids" not in helper_segment
  assert "accepted_satellite_ids" not in helper_segment
  assert "unavailable_satellite_ids" not in helper_segment

  feature = named_node(tree, "YumaSupplementationFeature")
  feature_segment = ast.get_source_segment(source, feature)
  assert feature_segment is not None
  assert feature_segment.count(
    "yuma_assistance_state_unavailable_outcome("
  ) >= 2
  assert "self._cycle_injection_consumed = True" in feature_segment
  assert "self._runtime = None" in feature_segment
  assert "terminal=True" in feature_segment
  assert "retry_pending=False" in feature_segment
  assert "if outcome.receiver_write_attempted:" in feature_segment
  assert "self._provisional_reference_used = reference" in feature_segment
