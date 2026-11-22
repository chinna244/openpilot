# Highway ramp speed advisories

Ramp speed advisories use the existing camera reader's numeric classifier and a
regional OpenStreetMap graph. SLA can offer the advisory after confirming the
car took the branch. The offer uses the advisory speed with zero offset and
requires a cruise decrement press. The posted speed limit continues through
the existing SLA resolver.

For the camera path, a yellow EXIT or RAMP label and two matching numeric
reads are required. If GPS drops after the sign is assigned to a particular
branch, measured travel and the camera's road-edge split can confirm that
branch. The map can also supply an explicit advisory for a confirmed ramp.
A ramp with no read or mapped advisory leaves SLA waiting for a posted limit.

The camera path uses the US MPH reader. The prompt is available when SLA is
enabled and the card cruise arbiter owns it, including Mazda with ICBM active.
The ordinary assist path handles unsupported modes.

## Regional map

Build a graph from one or more overlapping OpenStreetMap XML extracts that
include complete highway ways and their nodes. Include approaches and
continuations around the exits you want covered.

```sh
python -m openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_map \
  ramp_graph.json region.osm
```

The reader and planner look for
`/data/media/0/osm/offline/ramp_graph.json` at startup. A missing graph leaves
ordinary SLA in use. A graph covers the geometry in its source extracts.
Update it when road geometry or advisory tags change. OpenStreetMap data is
available under the ODbL. See [OpenStreetMap attribution](https://www.openstreetmap.org/copyright).
