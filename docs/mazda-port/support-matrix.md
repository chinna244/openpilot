# Mazda support matrix (StarPilot port, after remediation)

Qualification here means drives on this fork's code by the maintainer's own vehicle. Source eligibility
(chinna244/openpilot develop 8039965a03) is not qualification.

| Platform | Base StarPilot (af48674f) | Pinned source | Port: ordinary control | Port: AOL / TJA | Port: alpha long | Qualification |
|---|---|---|---|---|---|---|
| CX-5 2022-25 (MAZDA_CX5_2022) | Supported (dashcam off) | Supported | Yes | Yes, stock long only (words 2/10) | Yes: steer-to-zero EPS (or unread EPS), 2022 radar dialect or G46L; word 3 | Source development car (CX-5 2022). No drive on this port yet |
| CX-9 2021-23 (MAZDA_CX9_2021) | Supported | Supported | Yes | Yes (stock long only), words 2/10 | No (kept at base: never offered) | None on this port or the source maintainer's car |
| CX-5 2017-21 (MAZDA_CX5) | Dashcam only | Supported | No: dashcam only (base kept) | No | No | None |
| CX-9 2016-20 (MAZDA_CX9) | Dashcam only | Supported | No: dashcam only (base kept) | No | No | None |
| Mazda 3 2017-18 (MAZDA_3) | Dashcam only | Supported | No: dashcam only (base kept) | No | No | None |
| Mazda 6 2017-20 (MAZDA_6) | Dashcam only | Supported | No: dashcam only (base kept) | No | No | None |
| CX-5 2012-16 (MAZDA_CX5_KE) | Not present | Supported (incl. EPS swap + G46L) | No: dashcam only (new, unqualified) | No | No | None |
| CX-8 2023 (MAZDA_CX8_2023) | Not present | Supported | No: dashcam only (new, unqualified) | No | No | None |

## EPS and radar within admitted platforms

| Condition | Effect |
|---|---|
| Steer-to-zero EPS firmware (STEER_TO_ZERO_EPS_FW) or unread EPS on CX-5 2022 | No 45 kph floor; non-delivery latch; alpha long eligible |
| Legacy EPS firmware (any admitted platform) | 45 kph minimum steer speed; no alpha long |
| G46L radar on CX-5 2022 | Vision-only radarTracks; alpha long replays the G46L dialect |
| EPS swap into an older body | Older body stays dashcam only (no admission by EPS) |

## Intentional expansions over base StarPilot

1. Alpha long on CX-5 2022 (radar takeover/hand-back, factory AEB off while openpilot owns the radar).
2. Measured EPS envelope (1200 max, rate 10/10) on both admitted platforms, safety words 2/3/10.
3. Physical TJA button as AOL's lateral switch on stock-long CX-5 2022 and CX-9 2021 (word 10).
4. Legacy-EPS CX-9 2021 keeps steering above 45 kph with the new envelope.

## Not admitted (source-eligible only)

CX-5 2017-21, CX-9 2016-20, Mazda 3, Mazda 6, CX-5 KE, CX-8 2023, and alpha long on CX-9 2021 swaps and on
AOL-enabled cars (AOL is not admitted with alpha long: its native long bit needs vehicle_moving).
