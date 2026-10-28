# StarPilot base with the Mazda port

Verified October 28, 2026. Used saved Mazda context, then checked the current source.

## Decision

Feasible as a source port. Use StarPilot `Dom` as the base, preserve its Galaxy service and phone apps, and adapt the Mazda vehicle/firmware/lifecycle behavior into StarPilot's current architecture. The existing Mazda commits are not a compatible cherry-pick stack.

## Pinned sources

| Component | Revision |
| --- | --- |
| StarPilotHQ/StarPilot `Dom` | `af48674faee8f2dbe1c30e207a9f0c37cc562fcf` |
| chinna244/openpilot `develop` | `8039965a033b3de1f3cca0f7fe6cfcae90b451e9` |
| That Mazda branch's opendbc dependency | `7b96732d9cd9e8e4df33d8db6d43b8f49c34ed58` |
| That Mazda branch's Panda dependency | `daa5398504c4b33d82c597b7684f5bd5ea5a3e89` |

The current Mazda source is newer than the October 16 `d81a06a` checkpoint in memory. Unpushed local experiments and the separate SLA freshness branch are not implicitly included.

## Why this requires adaptation

- StarPilot vendors opendbc and Panda as normal folders; Mazda develop tracks separate repositories as submodules. Replacing those folders wholesale would overwrite StarPilot's other vehicle and native permission changes.
- StarPilot constructs `CarInterface(CP)` and calls `CarController.update(CC, CS, now_nanos)`. The Mazda implementation requires `CP_SP` and `CC_SP`, including MADS state, cruise-button management, lead information, and the radar handback request.
- Mazda's `CarParamsSP`, `CarControlSP`, and `CarStateSP` Python structures are absent from the base. They need explicit StarPilot equivalents or a deliberately scoped adapter; dropping extra arguments does not preserve their behavior.
- StarPilot's existing Mazda AOL admission explicitly requires stock cruise, safety parameter zero, and no openpilot longitudinal control. Merely enabling alpha-long or importing Mazda flag values would revoke that admission.
- The Mazda safety implementation registers `fwd_modify` for the TJA frame behavior. StarPilot's safety hook interface has no such callback. The firmware path must be adapted and tested with its native lateral-permission owner; copying only `mazda.h` is insufficient.
- Radar return must precede software transitions that stop CAN transmission. Mazda's correlated handback gate is connected to its card, hardwared, and manager; StarPilot needs equivalent behavior for its own lifecycle and app requests.

## App compatibility

StarPilot includes Galaxy plus iOS and Android companion source. The companion documentation explicitly supports custom, unpushed comma branches: screens are served by the comma, and the apps cache them through `/api/companion/web-manifest` for Bluetooth use.

Keep Galaxy's pairing, manifest, Bluetooth bridge, APIs and phone protocol intact. Add Mazda settings through its typed settings gateway and shared native settings owners. Preserve the TJA setting as required by the prior Mazda work. Device-side Mazda settings do not inherently require rebuilding the phone app if its protocol and embedded assets stay unchanged.

App-store signing and native phone builds are separate from the comma port. Bluetooth requires an initial Wi-Fi/LAN screen sync; large transfers and several media operations require Wi-Fi.

## Port scope and verification order

1. **Vehicle identification, steering and CAN:** adapt Mazda platform/firmware identification, DBC signals, measured steering envelope, torque scaling and fault behavior in `opendbc_repo/opendbc/car/mazda/` and Mazda DBCs. Retain StarPilot's learning observation hooks. Run the corresponding Mazda interface, steering, parser and golden-message tests.
2. **Native lateral permission and TJA:** integrate the physical-button behavior with `openpilot/starpilot/car/mazda/`, its AOL policy and native Panda permissions. Preserve MAIN, Cancel, brake, Reverse and standstill semantics and test the setting both ways. Do not weaken admission for other configurations.
3. **Alpha longitudinal:** adapt Mazda's acceleration shaping, standstill/resume, radar session and synthetic messages. Integrate its explicit handback outcome into card, hardwared, manager, forced-offroad and reboot paths before exposing the opt-in. Radar takeover disables stock AEB while active, an existing source behavior the setting must explain.
4. **Firmware:** port the matching Mazda checks and required forwarding behavior into StarPilot's current safety/Panda implementation. Validate allowed and blocked frames, steering limits, cruise engagement, two-master avoidance and handback; build matching firmware and host artifacts.
5. **Galaxy settings and regression:** expose the supported Mazda configuration through the shared settings owner; verify parked-only changes, restart ordering, native/UI agreement and companion screen refresh. Run existing StarPilot Mazda/AOL, relevant Galaxy and firmware suites as regressions.

Keep StarPilot's planners as the base. Mazda-specific dependencies on planner output must be audited; bringing the entire Zoompilot/Sunnypilot planner stack would be a separate, much broader migration.

## Current state

Implemented locally on `mazda-on-dom`. See [the Mazda port notes](../../mazda-port/README.md) for the final scope, validation and remaining device qualification limits. No push or device deployment was performed.
