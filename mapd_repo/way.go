package main

import (
	"errors"
	"math"
	"time"

	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/cereal/log"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
	"pfeifer.dev/mapd/utils"
)

type CurrentWay struct {
	Way               maps.Way
	Distance          maps.DistanceResult
	OnWay             maps.OnWayResult
	StartPosition     m.Position
	EndPosition       m.Position
	ConfidenceCounter int
	LastChangeTime    time.Time
	StableDistance    float32
	SelectionType     custom.WaySelectionType
	maxSpeed          utils.Curry[float64]
}

func (w *CurrentWay) _maxSpeed() float64 {
	maxSpeed := w.Way.MaxSpeed()
	if w.OnWay.IsForward && w.Way.MaxSpeedForward() > 0 {
		maxSpeed = w.Way.MaxSpeedForward()
	} else if !w.OnWay.IsForward && w.Way.MaxSpeedBackward() > 0 {
		maxSpeed = w.Way.MaxSpeedBackward()
	}
	return maxSpeed
}

func (w *CurrentWay) MaxSpeed() float64 {
	return w.maxSpeed.Value(w._maxSpeed)
}

// the raw maxspeed:conditional tag for the direction of travel
func (w *CurrentWay) ConditionalMaxSpeedRaw() string {
	return w.Way.ConditionalMaxSpeedRaw(w.OnWay.IsForward)
}

// MaxSpeed with an applying conditional speed limit folded in when
// conditional speed limit control is enabled. Not memoized because the
// applying rule changes with the time of day.
func (w *CurrentWay) EffectiveMaxSpeed() float64 {
	if ms.Settings.ConditionalSpeedLimitControlEnabled {
		rules := w.Way.ConditionalSpeedRules(w.OnWay.IsForward)
		if conditional := maps.ConditionalSpeedAt(rules, time.Now()); conditional > 0 {
			return conditional
		}
	}
	return w.MaxSpeed()
}

// ErrAmbiguousWay means more than one distinct loaded-tile road fits the same
// GPS uncertainty envelope. The rank scorer must not turn that into a limit.
var ErrAmbiguousWay = errors.New("multiple plausible road IDs")

// plausibleLoadedWays considers every way in the current tile before sticky or
// predicted selection. A conservative existing OnWay(..., 2) envelope admits
// candidates; direction/one-way constraints are applied inside OnWay. Duplicate
// records for the same OSM way ID do not create artificial ambiguity.
func plausibleLoadedWays(offlineMaps *maps.Offline, location log.GpsLocationData) []maps.Way {
	if offlineMaps == nil || !offlineMaps.Loaded {
		return nil
	}
	ways := make([]maps.Way, 0, 2)
	seen := make(map[int64]struct{})
	// The grid lists, in tile order, every way that could pass OnWay here.
	candidates, indexed := offlineMaps.WaysNear(m.NewPosition(location.Latitude(), location.Longitude()))
	count := len(candidates)
	if !indexed {
		count = offlineMaps.Ways.Len()
	}
	for c := range count {
		i := c
		if indexed {
			i = int(candidates[c])
		}
		if !offlineMaps.WayMayMatch(i, location, 2) {
			continue
		}
		way := offlineMaps.Ways.At(i)
		if way.Id() <= 0 || way.Nodes.Len() < 2 {
			continue
		}
		if _, exists := seen[way.Id()]; exists {
			continue
		}
		onWay, err := way.OnWay(location, 2)
		if err != nil || !onWay.OnWay {
			continue
		}
		seen[way.Id()] = struct{}{}
		ways = append(ways, way)
	}
	return ways
}

// Junction exception to the ambiguity rule (MATCHING.md): the road matched on the previous fix is kept while the car
// drives straight along it and every competitor is either a connected piece of the same road with the same limit or a
// clearly crossing street. Anything else, including a turn, a slow or unreliable heading, or a parallel road, stays
// ambiguous.
const (
	junctionMinSpeed      = 3.0   // m/s: below it the GPS heading is not trusted
	junctionMaxBearingAcc = 10.0  // deg
	junctionAlignedSin    = 0.259 // |sin| of heading vs the kept road: within 15 deg
	junctionCrossingSin   = 0.766 // |sin| of heading vs a competitor: at least 50 deg off
)

func sameRoadLimit(a, b *maps.Way) bool {
	return a.Name() != "" && a.Name() == b.Name() && a.MaxSpeed() == b.MaxSpeed() &&
		a.MaxSpeedForward() == b.MaxSpeedForward() && a.MaxSpeedBackward() == b.MaxSpeedBackward() &&
		a.MaxSpeedConditional() == b.MaxSpeedConditional() && a.MaxSpeedForwardConditional() == b.MaxSpeedForwardConditional() &&
		a.MaxSpeedBackwardConditional() == b.MaxSpeedBackwardConditional()
}

func waysConnected(a, b *maps.Way) bool {
	a0, a1 := a.Nodes.At(0), a.Nodes.At(a.Nodes.Len()-1)
	b0, b1 := b.Nodes.At(0), b.Nodes.At(b.Nodes.Len()-1)
	return a0.Equals(b0) || a0.Equals(b1) || a1.Equals(b0) || a1.Equals(b1)
}

func junctionKeep(currentWay CurrentWay, candidates []maps.Way, location log.GpsLocationData) (maps.Way, bool) {
	speed, bearing := float64(location.Speed()), location.BearingDeg()
	if currentWay.Way.Nodes.Len() < 2 || !(speed >= junctionMinSpeed) || math.IsInf(speed, 0) || !(bearing >= 0 && bearing <= 360) {
		return maps.Way{}, false
	}
	if acc := location.BearingAccuracyDeg(); !(acc > 0 && acc <= junctionMaxBearingAcc) {
		return maps.Way{}, false
	}
	var current *maps.Way
	for i := range candidates {
		if candidates[i].Id() == currentWay.Way.Id() {
			current = &candidates[i]
		}
	}
	if current == nil {
		return maps.Way{}, false
	}
	direction, err := current.OnWay(location, 2)
	currentMatch, narrowErr := current.OnWay(location, current.DistanceMultiplier())
	alignment, alignErr := current.BearingAlignment(location)
	if err != nil || narrowErr != nil || alignErr != nil || alignment > junctionAlignedSin || direction.IsForward != currentWay.OnWay.IsForward {
		return maps.Way{}, false
	}
	// Preserve the current way while narrow, or transfer only to a connected, aligned piece.
	var kept *maps.Way
	if currentMatch.OnWay {
		kept = current
	}
	for i := range candidates {
		w := &candidates[i]
		if w == current {
			continue
		}
		if sameRoadLimit(current, w) && waysConnected(current, w) {
			broad, err := w.OnWay(location, 2)
			currentLimit := CurrentWay{Way: *current, OnWay: direction}
			candidateLimit := CurrentWay{Way: *w, OnWay: broad}
			if err == nil && broad.OnWay && currentLimit.EffectiveMaxSpeed() == candidateLimit.EffectiveMaxSpeed() &&
				currentLimit.ConditionalMaxSpeedRaw() == candidateLimit.ConditionalMaxSpeedRaw() {
				if kept == nil {
					narrow, err := w.OnWay(location, w.DistanceMultiplier())
					aligned, alignErr := w.BearingAlignment(location)
					if err == nil && narrow.OnWay && alignErr == nil && aligned <= junctionAlignedSin {
						kept = w
					}
				}
				continue
			}
		}
		if s, err := w.BearingAlignment(location); err != nil || s < junctionCrossingSin {
			return maps.Way{}, false // parallel, or not clearly crossing
		}
	}
	if kept == nil {
		return maps.Way{}, false
	}
	return *kept, true
}

// Resolving the rest of the ambiguity (MATCHING.md "Resolved ambiguity"). The envelope above is deliberately wide (twice
// accuracy plus road width: 25 m for a 2-lane road, 40 m for 4 lanes), so a ramp, a turn lane or a parallel piece of the
// same road beside the car's road made every such fix unknown: on routes 0e/0f the freeway had no map limit 57% of the
// time with GPS accuracy ~2 m (develop's matcher: 97%). Only roads running the car's way count, and only on a reliable
// heading; one of three rules must then decide, or the fix stays unknown:
//   - every aligned road posts the same limit: which one the car is on does not change the limit;
//   - one aligned road is clearly nearest: ambiguityMargin closer than every other aligned one, and narrow-matched;
//   - the road matched on the previous fix is still narrow-matched and aligned (continuity through a ramp's gore).
//
// A road 30-50 deg off the heading (a branch peeling away) keeps the fix unknown; one at 50 deg or more is a crossing.
const (
	ambiguityAlignedSin = 0.5  // |sin| of heading vs a road: within 30 deg runs the car's way
	ambiguityMarginMin  = 10.0 // m: the nearest aligned road must be this much nearer than the next ...
	ambiguityMarginAcc  = 3.0  // ... and at least this many times the fix's reported accuracy
	// continuity keeps the previous road only while it is no farther than the nearest aligned road by this (or accuracy)
	continuityToleranceMin = 3.0 // m
	ambiguityLeaveFactor   = 2.0 // leaving the road held for one clearly nearer takes this many margins
)

type alignedWay struct {
	way      maps.Way
	distance float32
	sin      float32
	narrow   bool
	limit    CurrentWay // the way with its direction of travel, for the directional limit
}

func resolveAmbiguity(currentWay CurrentWay, candidates []maps.Way, location log.GpsLocationData) (maps.Way, bool) {
	speed, bearing := float64(location.Speed()), location.BearingDeg()
	if !(speed >= junctionMinSpeed) || math.IsInf(speed, 0) || !(bearing >= 0 && bearing <= 360) {
		return maps.Way{}, false
	}
	if acc := location.BearingAccuracyDeg(); !(acc > 0 && acc <= junctionMaxBearingAcc) {
		return maps.Way{}, false
	}
	aligned := make([]alignedWay, 0, len(candidates))
	for i := range candidates {
		w := candidates[i]
		s, err := w.BearingAlignment(location)
		if err != nil {
			return maps.Way{}, false
		}
		if s >= junctionCrossingSin {
			continue // a crossing street
		}
		if s > ambiguityAlignedSin {
			return maps.Way{}, false // neither running along nor crossing
		}
		broad, err := w.OnWay(location, 2)
		if err != nil || !broad.OnWay {
			return maps.Way{}, false
		}
		narrow, err := w.OnWay(location, w.DistanceMultiplier())
		aligned = append(aligned, alignedWay{w, broad.Distance.Distance, s, err == nil && narrow.OnWay, CurrentWay{Way: w, OnWay: broad}})
	}
	if len(aligned) == 0 {
		return maps.Way{}, false
	}
	nearest := 0
	for i := range aligned {
		if aligned[i].distance < aligned[nearest].distance {
			nearest = i
		}
	}
	// The previous fix's road, if it is still under the car (narrow), along the heading and driven the same direction.
	held := -1
	if currentWay.Way.Nodes.Len() >= 2 {
		for i := range aligned {
			a := &aligned[i]
			if a.way.Id() == currentWay.Way.Id() && a.narrow && a.sin <= junctionAlignedSin && a.limit.OnWay.IsForward == currentWay.OnWay.IsForward {
				held = i
			}
		}
	}
	// Same posted limit on every aligned road (and the same conditional rule): the road held is kept, else the nearest is
	// published (no flipping between equal-limit pieces, which would move the planner's camera fence each time).
	first := aligned[0].limit.EffectiveMaxSpeed()
	same := first > 0
	for i := range aligned {
		if aligned[i].limit.EffectiveMaxSpeed() != first || aligned[i].limit.ConditionalMaxSpeedRaw() != aligned[0].limit.ConditionalMaxSpeedRaw() {
			same = false
		}
	}
	if same && held >= 0 {
		return aligned[held].way, true
	}
	if same && aligned[nearest].narrow {
		return aligned[nearest].way, true
	}
	// Clearly nearest. Leaving a road still held takes twice the margin: one fix pulled toward a road beside it (multipath,
	// with an understated accuracy) must not swap the limit (review 2026-11-24).
	margin := float32(max(ambiguityMarginMin, ambiguityMarginAcc*float64(location.HorizontalAccuracy())))
	if held >= 0 && held != nearest {
		margin *= ambiguityLeaveFactor
	}
	if aligned[nearest].narrow {
		clear := true
		for i := range aligned {
			if i != nearest && aligned[i].distance-aligned[nearest].distance < margin {
				clear = false
			}
		}
		if clear {
			return aligned[nearest].way, true
		}
	}
	// Continuity: the previous fix's road, still under the car along its heading, driven the same direction, and still
	// the nearest within the fix's accuracy. A freeway's narrow match is ~30 m wide, so narrow alone would keep the old
	// road after the car moved onto a parallel one beside it.
	tolerance := float32(max(continuityToleranceMin, float64(location.HorizontalAccuracy())))
	if held >= 0 {
		for i := range aligned {
			a := &aligned[i]
			if i == held && a.distance <= aligned[nearest].distance+tolerance {
				return a.way, true
			}
		}
	}
	return maps.Way{}, false
}

func GetCurrentWay(currentWay CurrentWay, nextWays []maps.NextWayResult, offlineMaps *maps.Offline, location log.GpsLocationData) (CurrentWay, error) {
	candidates := plausibleLoadedWays(offlineMaps, location)
	if len(candidates) == 0 {
		return CurrentWay{SelectionType: custom.WaySelectionType_fail}, errors.New("no plausible loaded-tile way")
	}
	if len(candidates) > 1 {
		kept, ok := junctionKeep(currentWay, candidates, location)
		if !ok {
			kept, ok = resolveAmbiguity(currentWay, candidates, location)
		}
		if !ok {
			return CurrentWay{SelectionType: custom.WaySelectionType_fail}, ErrAmbiguousWay
		}
		candidates = []maps.Way{kept}
	}
	// Use the loaded tile's representation, even when a prior sticky way has the
	// same ID. This prevents an old tile's geometry/limit from being republished.
	way := candidates[0]
	onWay, err := way.OnWay(location, 2)
	if err != nil || !onWay.OnWay {
		return CurrentWay{SelectionType: custom.WaySelectionType_fail}, errors.New("sole way lost geographic match")
	}
	selection := custom.WaySelectionType_fail
	confidence := 1
	lastChange := time.Now()
	stableDistance := onWay.Distance.Distance
	narrow, narrowErr := way.OnWay(location, way.DistanceMultiplier())
	sameCurrent := currentWay.Way.Nodes.Len() > 1 && currentWay.Way.Id() == way.Id()
	if sameCurrent {
		lastChange = currentWay.LastChangeTime
		if narrowErr == nil && narrow.OnWay && narrow.Distance.LinePosition.T != 0 && narrow.Distance.LinePosition.T != 1 {
			selection = custom.WaySelectionType_current
			confidence = currentWay.ConfidenceCounter + 1
			onWay = narrow
		}
	}
	if selection == custom.WaySelectionType_fail && narrowErr == nil && narrow.OnWay {
		for _, next := range nextWays {
			if next.Way.Nodes.Len() > 1 && next.Way.Id() == way.Id() {
				selection = custom.WaySelectionType_predicted
				onWay = narrow
				break
			}
		}
	}
	if selection == custom.WaySelectionType_fail && narrowErr == nil && narrow.OnWay {
		selection = custom.WaySelectionType_possible
		onWay = narrow
	}
	if selection == custom.WaySelectionType_fail && sameCurrent {
		selection = custom.WaySelectionType_extended
		confidence = currentWay.ConfidenceCounter
		stableDistance = currentWay.StableDistance
	}
	if selection == custom.WaySelectionType_fail {
		return CurrentWay{SelectionType: custom.WaySelectionType_fail}, errors.New("sole broad-envelope way failed selection threshold")
	}
	start, end := way.GetStartEnd(onWay.IsForward)
	return CurrentWay{Way: way, Distance: onWay.Distance, OnWay: onWay,
		StartPosition: start, EndPosition: end, ConfidenceCounter: confidence,
		LastChangeTime: lastChange, StableDistance: stableDistance,
		SelectionType: selection}, nil
}

func NextWays(location log.GpsLocationData, currentWay CurrentWay, offlineMaps *maps.Offline, isForward bool) ([]maps.NextWayResult, error) {
	nextWays := []maps.NextWayResult{}
	dist := float32(0.0)
	wayIdx := currentWay.Way
	forward := isForward
	startPos := m.NewPosition(location.Latitude(), location.Longitude())
	for dist < ms.MIN_WAY_DIST {
		d, err := wayIdx.DistanceToEnd(startPos, forward)
		if err != nil || d <= 0 {
			break
		}
		dist += d
		nw, err := wayIdx.NextWay(offlineMaps, forward)
		if err != nil {
			break
		}
		nextWays = append(nextWays, nw)
		wayIdx = nw.Way

		startPos = nw.StartPosition
		forward = nw.IsForward
	}

	if len(nextWays) == 0 {
		nextWay, err := currentWay.Way.NextWay(offlineMaps, isForward)
		if err != nil {
			return []maps.NextWayResult{}, err
		}
		nextWays = append(nextWays, nextWay)
	}

	return nextWays, nil
}
