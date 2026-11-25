package main

import (
	"math"
	"testing"

	"capnproto.org/go/capnp/v3"
	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/cereal/log"
	"pfeifer.dev/mapd/cereal/offline"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
)

type rampTestWay struct {
	id     int64
	speed  float64
	class  offline.HighwayClass
	nodes  [][2]float64
	oneWay bool
}

const (
	fwy    = offline.HighwayClass_motorway
	ramp   = offline.HighwayClass_motorwayLink
	street = offline.HighwayClass_secondary
)

// freeway north to A, ramp A->B (556 m, no limit), street west-east through B as an interior node
func exitRoads() []rampTestWay {
	return []rampTestWay{
		{1, 31, fwy, [][2]float64{{35.09, -97.9}, {35.1, -97.9}}, true},
		{2, 0, ramp, [][2]float64{{35.1, -97.9}, {35.105, -97.9}}, true},
		{3, 20, street, [][2]float64{{35.105, -97.91}, {35.105, -97.9}, {35.105, -97.89}}, false},
	}
}

func rampTile(tb testing.TB, specs []rampTestWay) maps.Offline {
	tb.Helper()
	msg, seg, err := capnp.NewMessage(capnp.SingleSegment(nil))
	if err != nil {
		tb.Fatal(err)
	}
	tile, err := offline.NewRootOffline(seg)
	if err != nil {
		tb.Fatal(err)
	}
	tile.SetMinLat(35)
	tile.SetMaxLat(35.25)
	tile.SetMinLon(-98)
	tile.SetMaxLon(-97.75)
	ways, err := tile.NewWays(int32(len(specs)))
	if err != nil {
		tb.Fatal(err)
	}
	for i, spec := range specs {
		way := ways.At(i)
		way.SetId(spec.id)
		way.SetMaxSpeed(spec.speed)
		way.SetOneWay(spec.oneWay)
		way.SetLanes(2)
		way.SetHighwayClass(spec.class)
		minLat, maxLat, minLon, maxLon := 90., -90., 180., -180.
		nodes, err := way.NewNodes(int32(len(spec.nodes)))
		if err != nil {
			tb.Fatal(err)
		}
		for k, n := range spec.nodes {
			nodes.At(k).SetLatitude(n[0])
			nodes.At(k).SetLongitude(n[1])
			minLat, maxLat, minLon, maxLon = min(minLat, n[0]), max(maxLat, n[0]), min(minLon, n[1]), max(maxLon, n[1])
		}
		way.SetMinLat(minLat - 0.001)
		way.SetMaxLat(maxLat + 0.001)
		way.SetMinLon(minLon - 0.001)
		way.SetMaxLon(maxLon + 0.001)
	}
	data, err := msg.MarshalPacked()
	if err != nil {
		tb.Fatal(err)
	}
	parsed := maps.ReadOffline(data)
	if !parsed.Loaded {
		tb.Fatal("packed tile rejected")
	}
	return parsed
}

func rampLocation(tb testing.TB, lat, lon float64) log.GpsLocationData {
	tb.Helper()
	_, seg, err := capnp.NewMessage(capnp.SingleSegment(nil))
	if err != nil {
		tb.Fatal(err)
	}
	loc, err := log.NewRootGpsLocationData(seg)
	if err != nil {
		tb.Fatal(err)
	}
	loc.SetHasFix(true)
	loc.SetLatitude(lat)
	loc.SetLongitude(lon)
	return loc
}

// on way index `on` (forward), at lat/lon
func rampState(tb testing.TB, specs []rampTestWay, on int, lat, lon float64) State {
	tb.Helper()
	tile := rampTile(tb, specs)
	return State{ShadowOnly: true, SnapshotID: "verified-snapshot", Data: tile, Position: m.NewPosition(lat, lon),
		MatchedLimitMps: float32(specs[on].speed), RoadStatus: custom.MapdOut_SampleStatus_matchedNoLimit,
		CurrentWay: CurrentWay{Way: tile.Ways.At(on), OnWay: maps.OnWayResult{IsForward: true}, SelectionType: custom.WaySelectionType_current}}
}

func rampEndAt(tb testing.TB, specs []rampTestWay, on int, lat, lon float64) RampEnd {
	tb.Helper()
	s := rampState(tb, specs, on, lat, lon)
	return s.rampEnd(rampLocation(tb, lat, lon))
}

func TestRampEndStreetJoinedMidWay(t *testing.T) {
	got := rampEndAt(t, exitRoads(), 1, 35.101, -97.9)
	if got.Speed != 20 || math.Abs(float64(got.Distance)-445) > 5 {
		t.Fatalf("exit ramp end: %+v", got)
	}
}

func TestRampEndPublishedOnlyInShadowOutput(t *testing.T) {
	s := rampState(t, exitRoads(), 1, 35.101, -97.9)
	s.RampEnd = s.rampEnd(rampLocation(t, 35.101, -97.9))
	msg, err := s.buildMessageAt(0)
	if err != nil {
		t.Fatal(err)
	}
	event, err := log.ReadRootEvent(msg)
	if err != nil {
		t.Fatal(err)
	}
	out, err := event.MapdOut()
	if err != nil {
		t.Fatal(err)
	}
	if out.NextSpeedLimit() != 20 || out.NextSpeedLimitDistance() < 400 || out.SpeedLimit() != 0 || out.HighwayClass() != custom.HighwayClass(ramp) {
		t.Fatalf("output next=%v d=%v limit=%v", out.NextSpeedLimit(), out.NextSpeedLimitDistance(), out.SpeedLimit())
	}
	s.clearRoadDerived()
	if s.RampEnd != (RampEnd{}) {
		t.Fatal("ramp end survived a cleared road")
	}
}

func TestRampEndStreetSplitAtTheJunction(t *testing.T) {
	roads := exitRoads()[:2]
	roads = append(roads,
		rampTestWay{3, 20, street, [][2]float64{{35.105, -97.91}, {35.105, -97.9}}, false},
		rampTestWay{4, 20, street, [][2]float64{{35.105, -97.9}, {35.105, -97.89}}, false})
	if got := rampEndAt(t, roads, 1, 35.101, -97.9); got.Speed != 20 {
		t.Fatalf("split street: %+v", got)
	}
}

func TestRampEndChainOfRamps(t *testing.T) {
	roads := []rampTestWay{
		{1, 0, ramp, [][2]float64{{35.1, -97.9}, {35.103, -97.9}}, true},
		{2, 0, ramp, [][2]float64{{35.103, -97.9}, {35.105, -97.9}}, true},
		{3, 20, street, [][2]float64{{35.105, -97.91}, {35.105, -97.9}, {35.105, -97.89}}, false},
	}
	if got := rampEndAt(t, roads, 0, 35.101, -97.9); got.Speed != 20 || math.Abs(float64(got.Distance)-445) > 5 {
		t.Fatalf("ramp chain: %+v", got)
	}
}

func TestRampEndOnRampToFreeway(t *testing.T) {
	roads := []rampTestWay{
		{1, 31, fwy, [][2]float64{{35.1, -97.91}, {35.105, -97.9}}, true}, // upstream freeway ends at the merge: not drivable onto
		{2, 0, ramp, [][2]float64{{35.1, -97.9}, {35.105, -97.9}}, true},
		{3, 29, fwy, [][2]float64{{35.105, -97.9}, {35.11, -97.9}}, true},
	}
	if got := rampEndAt(t, roads, 1, 35.101, -97.9); got.Speed != 29 {
		t.Fatalf("on-ramp end: %+v", got)
	}
}

func TestRampEndGivesNothingWhenUnsure(t *testing.T) {
	for _, tc := range []struct {
		name string
		edit func([]rampTestWay) ([]rampTestWay, int)
		prep func(*State)
	}{
		{"untagged street", func(r []rampTestWay) ([]rampTestWay, int) { r[2].speed = 0; return r, 1 }, nil},
		{"two limits at the junction", func(r []rampTestWay) ([]rampTestWay, int) {
			return append(r, rampTestWay{4, 15, street, [][2]float64{{35.105, -97.9}, {35.11, -97.89}}, false}), 1
		}, nil},
		{"one-way street only into the junction", func(r []rampTestWay) ([]rampTestWay, int) {
			r[2] = rampTestWay{3, 20, street, [][2]float64{{35.105, -97.91}, {35.105, -97.9}}, true}
			return r, 1
		}, nil},
		{"not on a ramp", func(r []rampTestWay) ([]rampTestWay, int) { r[1].class = offline.HighwayClass_primary; return r, 1 }, nil},
		{"primary link is not a freeway ramp", func(r []rampTestWay) ([]rampTestWay, int) {
			r[1].class = offline.HighwayClass_primaryLink
			return r, 1
		}, nil},
		{"ramp's own limit already the joined one", func(r []rampTestWay) ([]rampTestWay, int) { r[1].speed = 20; return r, 1 }, nil},
		{"ramp too long", func(r []rampTestWay) ([]rampTestWay, int) {
			r[1].nodes[0] = [2]float64{35.08, -97.9}
			return r, 1
		}, nil},
		{"non-shadow mapd", func(r []rampTestWay) ([]rampTestWay, int) { return r, 1 }, func(s *State) { s.ShadowOnly = false }},
		{"predicted road", func(r []rampTestWay) ([]rampTestWay, int) { return r, 1 }, func(s *State) {
			s.CurrentWay.SelectionType = custom.WaySelectionType_predicted
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			roads, on := tc.edit(exitRoads())
			lat := 35.101
			if tc.name == "ramp too long" {
				lat = 35.081
			}
			s := rampState(t, roads, on, lat, -97.9)
			if tc.prep != nil {
				tc.prep(&s)
			}
			if got := s.rampEnd(rampLocation(t, lat, -97.9)); got != (RampEnd{}) {
				t.Fatalf("unsure ramp end published: %+v", got)
			}
		})
	}
}
