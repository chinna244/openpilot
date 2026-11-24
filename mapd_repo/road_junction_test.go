package main

import (
	"math"
	"testing"
	"time"

	"pfeifer.dev/mapd/cereal"
	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
)

func TestJunctionContinuity(t *testing.T) {
	settings := ms.Settings
	if err := initShadowDefaults(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { ms.Settings = settings })

	road := syntheticWay{1, 11.176, 35.149, -97.9, 35.1505, -97.9, false, 2}
	crossing := syntheticWay{2, 13.4112, 35.1505, -97.901, 35.1505, -97.899, false, 2}
	parallel := syntheticWay{3, 13.4112, 35.15038, -97.89988, 35.151, -97.89988, false, 2}
	continuation := syntheticWay{4, road.speed, road.lat2, road.lon2, 35.152, -97.9, false, 2}

	reversed := syntheticWay{4, road.speed, continuation.lat2, continuation.lon2, continuation.lat1, continuation.lon1, false, 2}

	for _, tc := range []struct {
		name                     string
		ways                     []syntheticWay
		first                    bool
		lat, lon                 float64
		speed, bearing, accuracy float32
		age                      uint64
		status                   custom.MapdOut_SampleStatus
		way                      int64
	}{
		{"straight through crossing", []syntheticWay{road, crossing}, true, 35.15046, -97.9, 8, 0, 5, 0, custom.MapdOut_SampleStatus_matchedLimit, 1},
		{"turn toward crossing", []syntheticWay{road, crossing}, true, 35.1505, -97.89996, 5, 90, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"reverse heading", []syntheticWay{road, crossing}, true, 35.15046, -97.9, 8, 180, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"unreliable heading", []syntheticWay{road, crossing}, true, 35.15046, -97.9, 8, 0, 20, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"low speed", []syntheticWay{road, crossing}, true, 35.15046, -97.9, 2, 0, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"nonfinite speed", []syntheticWay{road, crossing}, true, 35.15046, -97.9, float32(math.Inf(1)), 0, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"parallel rival", []syntheticWay{road, crossing, parallel}, true, 35.15046, -97.9, 8, 0, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"cannot acquire using heading alone", []syntheticWay{road, crossing}, false, 35.15046, -97.9, 8, 0, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"expired fix", []syntheticWay{road, crossing}, true, 35.15046, -97.9, 8, 0, 5, 2_000_000_000, custom.MapdOut_SampleStatus_noGps, 0},
		{"connected different directional limit", []syntheticWay{road, crossing, reversed}, true, 35.15046, -97.9, 8, 0, 5, 0, custom.MapdOut_SampleStatus_unknown, 0},
		{"connected same limit", []syntheticWay{road, crossing, continuation}, true, 35.15067, -97.9, 8, 0, 5, 0, custom.MapdOut_SampleStatus_matchedLimit, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			tile := packedRoadTile(t, tc.ways)
			for i := 0; i < tile.Ways.Len(); i++ {
				way := tile.Ways.At(i)
				if way.Id() == 1 || way.Id() == 4 {
					if tc.name == "connected different directional limit" {
						way.Way.SetMaxSpeedForward(11.176)
						way.Way.SetMaxSpeedBackward(13.4112)
					}
					if err := way.Way.SetName("Through Road"); err != nil {
						t.Fatal(err)
					}
				}
			}
			s := State{ShadowOnly: true}
			s.Init()
			boot := uint64(10_000_000_000)
			s.bootNow = func() uint64 { return boot }
			load := func(m.Position) (maps.Offline, error) { return tile, nil }
			if tc.first {
				initial := syntheticSample(t, cereal.GpsSourceExternal, boot, 1, true, true)
				initial.Location.SetLatitude(35.1499)
				initial.Location.SetSpeed(8)
				initial.Location.SetBearingAccuracyDeg(5)
				s.ProcessGps(initial, true, load, time.Unix(0, int64(boot)))
				if !s.RoadMatched() || s.CurrentWay.Way.Id() != 1 {
					t.Fatal("initial unambiguous road did not match")
				}
				boot += 100_000_000
			}
			fix := syntheticSample(t, cereal.GpsSourceExternal, boot-tc.age, 1, false, true)
			fix.Location.SetLatitude(tc.lat)
			fix.Location.SetLongitude(tc.lon)
			fix.Location.SetSpeed(tc.speed)
			fix.Location.SetBearingDeg(tc.bearing)
			fix.Location.SetBearingAccuracyDeg(tc.accuracy)
			s.ProcessGps(fix, true, load, time.Unix(0, int64(boot)))
			evt, out := decodedOutput(t, &s)
			if out.RoadStatus() != tc.status || out.WayId() != tc.way || evt.Valid() != (tc.way != 0) {
				t.Fatalf("status=%s way=%d valid=%t; want %s way=%d", out.RoadStatus(), out.WayId(), evt.Valid(), tc.status, tc.way)
			}
			if tc.name == "turn toward crossing" {
				// Geometric acquisition, after clearing the prior road, must give the cross street's own limit.
				boot += 100_000_000
				fix.FixMonoTime = boot
				fix.Location.SetLongitude(-97.8994)
				s.ProcessGps(fix, true, load, time.Unix(0, int64(boot)))
				evt, out = decodedOutput(t, &s)
				if !evt.Valid() || out.WayId() != 2 || out.SpeedLimit() != float32(crossing.speed) {
					t.Fatalf("turn did not acquire the cross street: way=%d limit=%f", out.WayId(), out.SpeedLimit())
				}
			}
		})
	}
}
