# Mazda on StarPilot Dom

The Mazda source port is implemented on the local `mazda-on-dom` branch. StarPilot remains the base, including its planners, native permission owner, Galaxy service and companion protocol. This is a source integration with local regression and microcontroller build evidence; full AGNOS and physical vehicle qualification remain outstanding.

## Source revisions

| Component | Revision |
| --- | --- |
| StarPilotHQ/StarPilot Dom | af48674faee8f2dbe1c30e207a9f0c37cc562fcf |
| chinna244/openpilot develop | 8039965a033b3de1f3cca0f7fe6cfcae90b451e9 |
| Mazda opendbc | 7b96732d9cd9e8e4df33d8db6d43b8f49c34ed58 |
| Mazda Panda reference | daa5398504c4b33d82c597b7684f5bd5ea5a3e89 |

The Mazda stack requires API adaptation rather than a direct cherry-pick: the vehicle context is local to Mazda, preserving StarPilot's normal interface and controller signatures.

## Implemented behavior

- Imported Mazda fingerprints, firmware identification, DBC signals, measured steering envelope, radar parsing, acceleration shaping, standstill logic and radar session state machine.
- Retained learning observation and dashboard stop-sign evidence. Stock-long remains available; alpha-long keeps the source EPS/radar gates and StarPilot's non-release opt-in.
- Admission follows base StarPilot: only the CX-5 2022 and CX-9 2021 control; the source's other platforms (and its new CX-5 KE and CX-8 entries) are dashcam only. Alpha long is offered on the CX-5 2022 only. See [support-matrix.md](support-matrix.md).
- AOL (independent lateral) is admitted only with stock longitudinal, as in base StarPilot: its native longitudinal bit requires motion, so alpha long's standstill hold would be refused. Safety words in use: 2 (measured EPS), 3 (alpha long), 10 (measured EPS + TJA, AOL only). Word 0 keeps its original behavior; invalid words permit no transmission.
- Physical TJA is an immutable startup setting shared by the native UI and Galaxy, for AOL on stock longitudinal. Forwarding modifies only the camera-bound button copy.
- EPS delivery keeps the source's semantics: partial delivery during LKAS_BLOCK steers, the lane-keep re-arm block is exempt, only sustained road-speed zero delivery is a temporary fault. Only while AOL's native owner is active is a raw LKAS_BLOCK published as `steerFaultTemporary`, because that owner then withholds all lateral torque.
- Correlated radar hand-back delays software-initiated stops for finalized Mazda alpha-long sessions; an answered stop stays asserted until card exits, so a restored radar is not taken over again during the stop. Physical ignition/power loss cannot be delayed.
- Radar failures reach the driver through selfdrived events (`stockEcuFailed`, `stockEcuNotReady`, display-only) and the onroad notice. An unanswered hand-back leaves persistent failure evidence when the radar may be off; it clears when a panda reports the ignition off.

## Tests and build boundaries

Both pinned source fixtures are byte-for-byte unchanged and exercised. The controller fixture (`mazda_golden_tx.json`) is a source-algorithm unit test. The full-stack replay runs the production `CarInterface.update/apply` and native safety and is compared with the source's own `mazda_golden_stack.json`: 7,003 of 7,010 frames match after two documented native-API mappings, and the 7 differing frames are asserted individually (`STACK_SOURCE_EXCEPTIONS`).

Production-path tests also cover EPS delivery on a closed-loop simulated EPS, the real Card with and without AOL, the hand-back lifecycle through the real Card and manager's Params clears, and the shipped safety words through StarPilot's standard safety suite (`test_mazda_measured_profiles.py`).

This is a source build (no root `prebuilt` marker; `prebuilt.json` not regenerated). `openpilot/system/manager/build.py` records the last successful build and forces a full rebuild whenever tracked build outputs are not what that build left (updater `git reset --hard`, checkout, copy). Fast Update refuses non-prebuilt revisions. A matching AGNOS device build, firmware flash check and vehicle validation are still outstanding.

Exact counts, commands and artifact hashes are in the delivery verification report.

## Attribution

Mazda-specific imported files retain their original copyright notices, including Zeph Leggett's zoompilot notices. Upstream notices are preserved in [opendbc-LICENSE](opendbc-LICENSE) and [upstream-LICENSE.md](upstream-LICENSE.md).

This software is licensed under a custom license requiring permission for use.

This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.
