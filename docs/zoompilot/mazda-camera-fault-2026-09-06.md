# Mazda camera fault: the EPS's LKAS_FAULT bit, 2026-09-06

The cluster's "Front Camera Sensor System Malfunction" and openpilot's "LKAS Fault: Restart the
Car" are the camera's `CAM_LKAS.ERR_BIT_1` / `CAM_LANEINFO.ERR_BIT`. On the 2022 EPS they are
preceded, on every capture with a known onset, by a bit the EPS raises first: `STEER_RATE` byte 6
bit 5 (message bit 53), now decoded as `LKAS_FAULT`. This page records what the corpus says about
that bit, what triggers it, what was changed, and what is still open.

## The finding

Scanned: every rlog under `tools/mazda_long` (4,093 segments, 3,879 of them CX-5 2022 with EPS
frames, about 64 h) plus the new capture `000001bb--b4a16dc0f3` from `comma@192.168.1.144`.
Tool: `tools/mazda_long/lkas_fault_scan.py`.

| Fact | Count |
| --- | --- |
| Camera `ERR_BIT_1` rises with `LKAS_FAULT` already set | 15 of 16 |
| Rises with a known `LKAS_FAULT` onset in the same segment | 11, camera fault 5.25 to 5.55 s after the bit |
| `LKAS_FAULT` runs with a known onset that lasted 5.2 s or more without a camera fault | 0 of 10 |
| `LKAS_FAULT` runs that ever cleared (57 runs, 2,899 s) | 0 |
| EPS frames with the bit set outside a faulted drive | 0 |

The one rise without the bit (`00000043` seg 0, build `61919842ad`, July) has the EPS unblocked
and delivering right up to the fault; it is a different mechanism and is not explained here.

Byte 6 of `STEER_RATE` takes five values in the corpus: `00` (free), `04` (block: the camera is
reporting no lane lines, or nothing is arriving at all), `08` (hands-off), `14` (block with
`LKAS_TRACK_STATE`: standby from a stop, and the 3 s hands-on timer at speed), `24` (block with
`LKAS_FAULT`). `LKAS_FAULT` never appears with `LKAS_TRACK_STATE`.

The old reading, that the camera latches its fault from watching a request go undelivered, is
wrong in both directions: in `000001bb` the request had been zero for 5.25 s when the camera
faulted, and in the March lateral-only drive `test_data/drive_02` (build `5e64db2897`, alpha
long off) the EPS bit was set at 16 m/s with nothing but zero requests delivered for the 5.5 s
before the fault. The camera is reacting to the EPS bit, not to our request.

## What raises the bit

Of the 11 known onsets:

- **A break in the 0x243 stream of about 0.6 s, seven times.** The last delivered frame is
  0.613 to 0.623 s before the bit in every one (routes `00000116`, `00000117`, `00000139`,
  `00000148`, `00000102`, `drive_02`, `00000013`). The breaks were panda rejection bursts: the
  driver-torque staleness and the 25-versus-12 rate-down (`00000148`, fixed 2026-09-01), the
  MADS/panda arming order (`00000116`/`00000117`, fixed 2026-08-28), and on the March and July
  builds commands past the panda's 800-count limit. The break is what matters, not which check
  tripped: a rejection resets the panda's rate-limit reference to zero
  (`steer_torque_cmd_checks`, `desired_torque_last = 0`), so every later frame more than one
  step from zero is rejected too, and a controller that keeps ramping never gets a frame through
  again. Verified against the compiled safety model in
  `opendbc/safety/tests/test_mazda.py::test_without_the_echo_a_rejection_starves_the_eps`.
- **Three restarts with the panda not forwarding** (`00000024`, `00000034`, `00000013`): the
  bit rose within 0.5 s of the log starting or of the panda entering the Mazda safety mode, with
  nothing delivered. The same timeout, with the comma's boot as the break. An EPS that has never
  had a stream (`00000116` seg 0, 6.4 s of nothing at ignition) does not raise it.
- **Two at a crawl from a stop, unexplained** (`000001bb` at 0.49 m/s, `0000001a` at 0.42 m/s,
  the latter a July alpha-long drive). Both: EPS in standby (`14`), a ramping request opposing
  the driver's torque, `LKAS_EFFECTIVE` zero, the bit 240 to 280 ms after the first nonzero
  frame, no gap and no rejected frame. The corpus has 406 other stretches of a nonzero request
  delivered into a zero-delivery standby for 0.25 s or more (213 of them starting below 0.6
  m/s, 95 opposing the driver, requests up to 1,200 counts, up to 3 s long) and none raised
  the bit. No standby guard was added: nothing in request, delivery, speed, driver torque or
  wheel motion separates these two from the 406.

## What changed (opendbc, danger-unstable, uncommitted)

- `mazda_2017.dbc`: `STEER_RATE.LKAS_FAULT`, bit 53.
- `carstate.py`: decodes it (`lkas_fault`) for the log and tooling; the driver-facing fault
  stays the camera's `ERR_BIT_1`, unchanged. Exposes the EPS's echo of the last request it
  received (`lkas_request_echo`).
- `carcontroller.py` `recover_from_rejection`: when the echo matches none of the last four
  commands for five frames, the ramp restarts from zero, which is what the panda accepts after
  a rejection. Detection is 90 ms plus the echo lag; the closed-loop test bounds the outage at
  20 frames against the EPS's 60. This closes the gap path for any rejection cause, including
  ones not yet seen.
- Tests: `test_mazda_carstate.py::TestLkasFaultBit`, `test_mazda_steering.py::TestRejectionRecovery`,
  and two controller-through-panda runs in `opendbc/safety/tests/test_mazda.py`. Mazda suites:
  659 passed. The golden transmit fixture is unchanged.

No panda safety code changed. The camera's error bits are not masked; `ERR_BIT` forwarding in
the HUD frame (2026-08-26) stays, which is why the cluster shows the warning now and did not on
older builds that faulted just the same (drive_02 in March: camera `ERR_BIT` set in 88 of 114
frames, forwarded in 0 of 120).

## What is not proven

- That a stream never breaking prevents every fault. The two crawl onsets are not the gap
  mechanism and remain open; a lateral-only user capture of one is needed (the rlog around
  the fault, any build). `lkas_fault_scan.py` on it answers which case it is in seconds.
- On-car behaviour of the recovery. Replay establishes what goes on the wire, not the EPS's
  response to a 200 ms re-ramp. Drive danger-unstable through a hard driver override at low
  speed and check `LKAS_FAULT` stays clear in the log.
- Whether the bit clears on the EPS between ignition cycles or needs a longer power-down. Every
  run in the corpus lasts to the end of its segment; drives that follow one start faulted.

## Diagnostics still worth doing

Stored DTCs would name the EPS's condition. `tools/scripts/car/read_dtc_status.py` stops pandad,
puts the panda in ELM327 mode, opens an extended session on the FSC (`0x706`) and EPS (`0x730`)
and reads DTC status; it is active diagnostics, so run it parked, ignition on, engine off, and
read before clearing. The device was disconnected from the car during this investigation, so it
has not been run.

## Rejection-gap checker

`tools/mazda_long/lkas_starvation_check.py` previously closed a rejection burst only on two
delivered frames under 50 ms apart, which the 16 Hz forwarded camera stream never satisfies,
and reported minutes of false starvation. It now ends a gap on any delivered frame. It measures
rejection-associated gaps only; `lkas_fault_scan.py` is the check for the fault itself.
