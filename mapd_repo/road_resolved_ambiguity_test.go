package main

import (
	"testing"
	"time"

	"pfeifer.dev/mapd/cereal"
	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
)

// way.go resolveAmbiguity: which roads inside the broad envelope still decide a limit, and which stay unknown.
// At 35.15 N, 0.0001 deg of longitude is ~9.1 m.
func TestResolvedAmbiguity(t *testing.T) {
	settings := ms.Settings
	if err := initShadowDefaults(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { ms.Settings = settings })

	const lat0, lat1 = 35.149, 35.152
	road := syntheticWay{1, 11.176, lat0, -97.9, lat1, -97.9, false, 2}               // 25 mph, the car's road
	near8 := syntheticWay{2, 22.352, lat0, -97.89991, lat1, -97.89991, false, 6}      // ~8 m east, 50 mph
	near8Same := syntheticWay{2, 11.176, lat0, -97.89991, lat1, -97.89991, false, 6}  // ~8 m east, 25 mph
	far27 := syntheticWay{2, 22.352, lat0, -97.8997, lat1, -97.8997, false, 6}        // ~27 m east, 50 mph (broad: 54 m)
	branch40 := syntheticWay{2, 22.352, 35.1503, -97.9, 35.1513, -97.89897, false, 6} // peels off at ~40 deg
	crossing := syntheticWay{3, 13.4112, 35.1505, -97.901, 35.1505, -97.899, false, 2}
	wide := syntheticWay{1, 29.0576, lat0, -97.9, lat1, -97.9, false, 6}               // 65 mph, 6 lanes: narrow ~41 m
	frontage12 := syntheticWay{2, 20.1168, lat0, -97.89987, lat1, -97.89987, false, 2} // ~12 m east, 45 mph

	type fix struct {
		lon                     float64
		speed, bearingAcc, hAcc float32
	}
	along := fix{-97.9, 8, 5, 2}
	for _, tc := range []struct {
		name   string
		ways   []syntheticWay
		prior  bool // matched alone on the previous fix (only `road`, or ways[0] when priorFirst, in the tile)
		fix    fix
		status custom.MapdOut_SampleStatus
		way    int64
	}{
		// must stay unknown
		{"parallel 8 m, different limit, no prior road", []syntheticWay{road, near8}, false, along, custom.MapdOut_SampleStatus_unknown, 0},
		{"branch peeling off at 40 deg", []syntheticWay{road, branch40}, false, fix{-97.9, 8, 5, 2}, custom.MapdOut_SampleStatus_unknown, 0},
		{"unreliable heading", []syntheticWay{road, far27}, false, fix{-97.9, 8, 20, 2}, custom.MapdOut_SampleStatus_unknown, 0},
		{"too slow for a heading", []syntheticWay{road, far27}, false, fix{-97.9, 2, 5, 2}, custom.MapdOut_SampleStatus_unknown, 0},
		{"poor accuracy widens the margin", []syntheticWay{road, far27}, false, fix{-97.9, 8, 5, 10}, custom.MapdOut_SampleStatus_unknown, 0},
		// moved onto the parallel road: the old road is 8 m away, no longer the nearest within accuracy
		{"continuity gives way to a nearer parallel road", []syntheticWay{road, near8}, true, fix{-97.89991, 8, 5, 2}, custom.MapdOut_SampleStatus_unknown, 0},
		// decided
		{"parallel with the same limit", []syntheticWay{road, near8Same}, false, along, custom.MapdOut_SampleStatus_matchedLimit, 1},
		{"clearly nearest", []syntheticWay{road, far27}, false, along, custom.MapdOut_SampleStatus_matchedLimit, 1},
		{"clearly nearest past a crossing street", []syntheticWay{road, far27, crossing}, false, fix{-97.9, 8, 5, 2}, custom.MapdOut_SampleStatus_matchedLimit, 1},
		{"continuity beside a parallel road", []syntheticWay{road, near8}, true, along, custom.MapdOut_SampleStatus_matchedLimit, 1},
		// held road: one fix pulled 13 m toward a frontage road 12 m beside it (multipath) does not swap the limit; without a
		// held road the same fix takes the clearly nearer frontage road
		{"held road is not left on one pulled fix", []syntheticWay{wide, frontage12}, true, fix{-97.89986, 8, 5, 2}, custom.MapdOut_SampleStatus_unknown, 0},
		{"without a held road the nearer one is taken", []syntheticWay{wide, frontage12}, false, fix{-97.89986, 8, 5, 2}, custom.MapdOut_SampleStatus_matchedLimit, 2},
		// equal-limit pieces: the held one is kept even when the other is nearer
		{"equal limit keeps the held road", []syntheticWay{road, near8Same}, true, fix{-97.89993, 8, 5, 2}, custom.MapdOut_SampleStatus_matchedLimit, 1},
		{"equal limit without a held road takes the nearest", []syntheticWay{road, near8Same}, false, fix{-97.89993, 8, 5, 2}, custom.MapdOut_SampleStatus_matchedLimit, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s := State{ShadowOnly: true}
			s.Init()
			boot := uint64(10_000_000_000)
			s.bootNow = func() uint64 { return boot }
			if tc.prior {
				alone := packedRoadTile(t, []syntheticWay{tc.ways[0]})
				first := syntheticSample(t, cereal.GpsSourceExternal, boot, 1, true, true)
				first.Location.SetLatitude(35.1495)
				first.Location.SetSpeed(8)
				first.Location.SetBearingAccuracyDeg(5)
				first.Location.SetHorizontalAccuracy(2)
				s.ProcessGps(first, true, func(m.Position) (maps.Offline, error) { return alone, nil }, time.Unix(0, int64(boot)))
				if !s.RoadMatched() || s.CurrentWay.Way.Id() != 1 {
					t.Fatal("prior road did not match")
				}
				boot += 100_000_000
			}
			tile := packedRoadTile(t, tc.ways)
			if tc.prior {
				s.Data = tile // the same fix area, now with the rival loaded
			}
			sample := syntheticSample(t, cereal.GpsSourceExternal, boot, 1, !tc.prior, true)
			sample.Location.SetLatitude(35.1500)
			sample.Location.SetLongitude(tc.fix.lon)
			sample.Location.SetSpeed(tc.fix.speed)
			sample.Location.SetBearingDeg(0)
			sample.Location.SetBearingAccuracyDeg(tc.fix.bearingAcc)
			sample.Location.SetHorizontalAccuracy(tc.fix.hAcc)
			s.ProcessGps(sample, true, func(m.Position) (maps.Offline, error) { return tile, nil }, time.Unix(0, int64(boot)))
			evt, out := decodedOutput(t, &s)
			if out.RoadStatus() != tc.status || out.WayId() != tc.way || evt.Valid() != (tc.way != 0) {
				t.Fatalf("status=%s way=%d valid=%t; want %s way=%d", out.RoadStatus(), out.WayId(), evt.Valid(), tc.status, tc.way)
			}
			if tc.way == 0 && out.SpeedLimit() != 0 {
				t.Fatalf("unknown published a limit %f", out.SpeedLimit())
			}
		})
	}
}
