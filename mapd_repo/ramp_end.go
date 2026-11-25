package main

import (
	"math"

	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/cereal/log"
	"pfeifer.dev/mapd/cereal/offline"
	"pfeifer.dev/mapd/maps"
)

// A freeway ramp has no limit of its own in OSM, so a shadow mapd gave the planner nothing while on it: the old
// limit held until the car read the next sign, often after the driver had braked (exit: route 87 t=965-1001, 70
// held to Moorland's 40; on-ramp: route 86, 30 held to the merge). Shadow mode publishes no NextWays-based limit
// ahead. On a motorway/trunk link only, it now publishes the limit of the road the ramp joins as the limit ahead.
// The ramp's end is found along mapd's own next-way chain; the road it joins is matched at that node by its end
// points and also by its interior nodes (OSM ramps often join a street mid-way: Moorland, Touhy). Only one limit
// for every road there is published: a junction offering two different limits (a split to two roads) gives none.
// Replay of 161 routes 2026-11-25: 17 exits found, 0 on plain freeway, the joined limit right 12/12 where tagged.

const rampEndMaxM = 1500 // a ramp this long or longer is not followed to its end

type RampEnd struct {
	Speed    float32
	Distance float32
}

type rampJunction struct {
	wayID   int64
	forward bool
	speed   float32
}

func isRampClass(c offline.HighwayClass) bool {
	return c == offline.HighwayClass_motorwayLink || c == offline.HighwayClass_trunkLink
}

// The ramp's next ways are looked up here only: the matcher keeps its shadow-mode input (no NextWays), so no road
// selection changes.
func (s *State) rampEnd(location log.GpsLocationData) RampEnd {
	if !s.ShadowOnly || s.CurrentWay.SelectionType != custom.WaySelectionType_current || !isRampClass(s.CurrentWay.Way.HighwayClass()) {
		s.rampJunction = rampJunction{}
		return RampEnd{}
	}
	last, forward := s.CurrentWay.Way, s.CurrentWay.OnWay.IsForward
	_, end := last.GetStartEnd(forward)
	distance, found, err := last.DistanceToNode(s.Position, forward, end)
	if err != nil || !found {
		return RampEnd{}
	}
	nextWays, err := NextWays(location, s.CurrentWay, &s.Data, forward)
	if err != nil {
		nextWays = nil
	}
	for _, next := range nextWays {
		if !isRampClass(next.Way.HighwayClass()) {
			break
		}
		last, forward = next.Way, next.IsForward
		distance += last.Distance()
	}
	if distance <= 0 || distance > rampEndMaxM || math.IsNaN(float64(distance)) {
		return RampEnd{}
	}
	if s.rampJunction.wayID != last.Id() || s.rampJunction.forward != forward {
		s.rampJunction = rampJunction{last.Id(), forward, joinedLimit(&s.Data, last, forward)}
	}
	speed := s.rampJunction.speed
	if speed <= 0 || math.Abs(float64(speed-s.MatchedLimitMps)) <= .02 {
		return RampEnd{}
	}
	return RampEnd{speed, distance}
}

// joinedLimit: the one limit of the roads (not ramps) a car can drive onto where this ramp ends, 0 for none or two.
func joinedLimit(data *maps.Offline, ramp maps.Way, forward bool) float32 {
	_, end := ramp.GetStartEnd(forward)
	limits := []float64{}
	add := func(way maps.Way, wayForward bool) {
		road := CurrentWay{Way: way, OnWay: maps.OnWayResult{IsForward: wayForward}}
		if limit := road.EffectiveMaxSpeed(); limit > 0 && !math.IsNaN(limit) && !math.IsInf(limit, 0) {
			limits = append(limits, limit)
		}
	}
	for i := 0; i < data.Ways.Len(); i++ {
		way := data.Ways.At(i)
		n := way.Nodes.Len()
		if way.Id() == ramp.Id() || n < 2 || isRampClass(way.HighwayClass()) {
			continue
		}
		if box := way.Box(); !box.PosInside(end) {
			continue
		}
		for k := range n {
			if node := way.Nodes.At(k); !node.Equals(end) {
				continue
			}
			// leaving the node along the way's direction needs k before its last node, against it k after its first
			if k < n-1 {
				add(way, true)
			}
			if k > 0 && !way.OneWay() {
				add(way, false)
			}
			break
		}
	}
	if len(limits) == 0 {
		return 0
	}
	for _, limit := range limits[1:] {
		if math.Abs(limit-limits[0]) > .5 {
			return 0
		}
	}
	return float32(limits[0])
}
