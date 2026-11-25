# Loaded-tile road ambiguity

Before retaining a current road or taking a predicted road, mapd checks every way in the currently loaded tile against the current GPS fix. A candidate needs a positive OSM way ID, at least two nodes, and a successful existing `OnWay(fix, 2)` test. That test uses the projected distance, reported horizontal accuracy (with its existing 5 m floor), way width, and one-way/direction rule. Repeated tile records with the same way ID count once. Rank, road class, and posted limit do not resolve a tie.

No candidates produce `noMatch`. A new match still requires a sole candidate that passes the existing narrow selection threshold. A zero posted limit on a matched way remains `matchedNoLimit`.

At a junction, a previously confirmed road may retain its limit if the car is moving at least 3 m/s, the current fix has a positive bearing accuracy of at most 10°, and its heading is within 15° of the confirmed road in the existing travel direction. The retained way must pass the narrow match. A connected piece of the same named road can take over if it is also aligned, passes the narrow match, and has the same directional speed limit and conditional policy. Every other candidate must either be a connected piece of that road with the same limit tags or cross the heading by at least 50°.

## Resolved ambiguity

The envelope is wide (twice the accuracy plus the road width: about 25 m for a 2-lane road, 40 m for 4 lanes), so a ramp, a turn lane or a parallel piece of the same road beside the car made most freeway fixes ambiguous (routes 0e/0f: no map limit 57% of the time at 50+ mph with ~2 m GPS accuracy). When the junction rule does not apply, `resolveAmbiguity` decides only on a reliable heading (moving at least 3 m/s, bearing accuracy at most 10°). Roads at least 50° off the heading are crossing streets and are ignored; a road 30-50° off (a branch peeling away) keeps the fix unknown. Among the roads within 30° of the heading, one of three rules must decide:

1. Every one posts the same directional limit and conditional rule: the nearest is published (which one the car is on does not change the limit).
2. The nearest passes its narrow match and every other one is at least `max(10 m, 3 x accuracy)` farther.
3. The road matched on the previous fix passes its narrow match, runs within 15° of the heading in the same direction, and is no farther than the nearest by more than `max(3 m, accuracy)`.

Offline on the C4's tiles with the logged fixes of routes 0c-0f, the share of moving fixes with a map limit rose from 42-70% to 77-79%, the published limit agreed with the car's or the camera's reading 98-100% of the time, and no fix that matched before changed its limit.

Otherwise two or more distinct IDs produce a neutral invalid `unknown` output. An uncertain heading, a slow car, a branch 30-50° off, or a stale fix leaves the ambiguity unresolved. On failure the state clears its current way, upcoming evidence, curve output, accepted/override limit, and numeric map limit before serialization. A way retained from another tile cannot be published when the loaded tile has no plausible way.

The packed synthetic-tile tests cover parallel and crossing competitors, prior sticky and predicted ways, empty tile and new-fix recovery, duplicate IDs, a sole road admitted only by the broad envelope, genuinely unique direction-sensitive one-way roads, and no posted limit. A 64/256/1024-way benchmark checks enumeration cost; it does not establish worst-case device performance or justify a spatial index.
