# Mazda on StarPilot Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Port pinned Mazda behavior to StarPilot Dom while retaining Galaxy and companion protocols.
**Architecture:** Keep upstream Mazda algorithms and their regression cases. Adapt the control context locally under the Mazda package; integrate its handback and physical intent with StarPilot's existing host and firmware owners.
**Tech Stack:** Python 3.12, Cap'n Proto, C Panda safety, Galaxy settings.
**Spec:** docs/superpowers/specs/2026-10-28-mazda-port-design.md

## Global Constraints
- Preserve TJA toggle, radar handback, firmware limits and StarPilot companion transport.
- No push, device modification or deployment.
- Do not replace other vehicle ports, the global controller API or StarPilot planners.

## Review Focus
- Software stop racing with radar takeover must not terminate TX before the restore result.
- Physical TJA, MAIN, Cancel, brake and stale CAN must preserve independent permission semantics.
- No steering envelope or diagnostic permission can leak to other configurations.
- Existing Mazda stock behavior and app settings authority need regression coverage.
- A source/test port is not a qualified device release; report build and hardware limitations explicitly.

### Task 1: Mazda vehicle adapter
- [x] Pin regressions for standard CarInterface/CarController signatures, torque and alpha-long behavior; run RED.
- [x] Import pinned Mazda algorithms, CAN definitions and firmware identification; adapt only Mazda-specific context.
- [x] Run imported Mazda vehicle regressions and standard API tests GREEN.

### Task 2: Native safety and TJA
- [x] Add regression cases for port parameters, steering limits, physical intent, forward-only TJA rewrite and forbidden traffic; run RED.
- [x] Adapt the matching safety checks and forwarding hook, preserving the default profile and native axis owner.
- [x] Run Mazda safety and relevant shared regressions GREEN.

### Task 3: Host handback
- [x] Port handback regression cases and exercise stop ordering, stale results and cancellation; run RED.
- [x] Wire card, hardwared and manager to request/hold/result flow before dropping onroad.
- [x] Run handback and lifecycle regressions GREEN.

### Task 4: Settings and integration
- [x] Test Mazda TJA setting eligibility, parked authority and correct startup flags; run RED.
- [x] Connect the typed owner shared with Galaxy/native controls and pass live controller context.
- [x] Run Mazda/AOL and relevant Galaxy regression suites GREEN.

### Task 5: Qualification and delivery
- [x] Run relevant complete suites, lint and available firmware/host builds; record exact results.
- [x] Obtain independent whole-branch review, resolve actionable findings and repeat affected checks.
- [x] Save patch, source pins, implementation notes and limitations in outputs.
