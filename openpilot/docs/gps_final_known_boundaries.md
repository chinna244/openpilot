# Final GPS known issues / boundaries (PR82 / PR83)

## SOLVED SOFTWARE CORRECTNESS

- Startup/configuration ordering and assistance bounds (PR69–74)
- DBD safety, YUMA supplementation, trusted-time policy
- Parser/framer hardening and ephemeris assembly integrity (PR76)
- Receiver fix / lifecycle / fingerprint semantics (PR77)
- AssistNow Online / AssistNowToken retirement (PR78)
- locationd GPS finite/input safety (PR75)
- QCOM rehabilitation and coexistence (PR79)
- Dual-source arbitration, failover/recovery, gpsSourceState (PR80)
- measurementMonoNs producers, locationd measurement-time fusion, H/V→ECEF covariance (PR81)
- gpsard health aligned with locationd usability (PR81)
- Date/time authority no longer dies at calendar year 2035 (PR82)
- 10-bit GPS week era resolved with trustworthy nonempty RAWX evidence; ambiguity fails closed (PR82)
- Live SFRBX ephemeris weeks must match RAWX current week within ±1; far nearest-era results rejected (PR82)
- Default leap-second authority uses maintained known IERS offset data (no artificial GPS-rollover cutoff); explicit/receiver leap still allowed (PR82)
- gps_drive_audit consumes PR71/80/81 telemetry with historical-route degradation and conservative evidence-based classification (PR82)
- Startup NO_HEALTHY is not a failure classification; runtime authority loss is (PR82)
- locationd_llk restarts after unexpected crash while still required (PR83)
- gpsard↔locationd bearingDeg finite usability contract completed (PR83)
- timed exclusive host/GPS time bounds match shared `min < t < max` contract (PR83)
- Trusted-UTC GPS-week resolution and YUMA GPS→UTC conversion are leap-aware via maintained table (PR83)
- GPS-triggered Kalman reset preserves full ECEF position covariance (off-diagonals) (PR83)

## FIELD VALIDATION PENDING

- Complete C4 live validation after PR83 deterministic cleanup
  (see `gps_c4_field_validation_plan.md`)

## ARCHITECTURAL / EXTERNAL LIMITATIONS

- **Early offline YUMA after true power loss** — without trustworthy current UTC,
  time-dependent cached assistance cannot safely be selected early.
  Do not solve by trusting a bad RTC, weakening trusted-time provenance, or
  blocking GNSS START for network.
  Classification: `architectural_external_trusted_time`

- **Longer-lived offline ephemeris / predicted orbit** — ephemeris availability can
  dominate TTFF, but there is no proven safe compatible longer-lived predicted-orbit
  source to add today. Do not extend stale ephemeris lifetime or resurrect AssistNow Online.
  Classification: `future_feature_requires_proven_source`

## EVIDENCE-GATED FUTURE WORK

- **Stalled-acquisition reset** — do not implement time-based reset. Reconsider only if
  future evidence shows good RF + adequate nav/ephemeris + healthy parser/config and
  the receiver still abnormally refuses to fix.
  Classification: `evidence_gated_not_implemented`

## NON-BLOCKING MAINTAINABILITY

- Broad pigeond refactor — maintainability only, not a GPS correctness blocker.

## FREEZE RULE

Software deterministic cleanup through PR83 is **COMPLETE**.
Field validation on C4 remains pending.

Future GPS behavioral PRs require one of:

1. deterministic test proving a current bug
2. field route telemetry proving a current defect
3. authoritative protocol/hardware evidence proving the current implementation wrong
4. a clearly defined new feature with a real supported data source

“TTFF is sometimes slow” alone is **not** enough justification to reopen startup policy.
Leap-second tables are updated only after an actual standards/IERS change.
