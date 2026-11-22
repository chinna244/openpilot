from collections import Counter, defaultdict
from itertools import pairwise
import re
import xml.etree.ElementTree as ET

import numpy as np

from pathlib import Path
import json
import math

RAMP_GRAPH_PATH = Path("/data/media/0/osm/offline/ramp_graph.json")


def projection(latitude):
  return np.array([111120.0, 111120.0 * math.cos(math.radians(latitude))])


def angle_dist(a, b):
  return np.abs((a - b + 180.0) % 360.0 - 180.0)


MAINLINE = {"motorway", "trunk"}
LINKS = {"motorway_link", "trunk_link", "primary_link", "secondary_link"}


def speed_mps(text):
  match = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*(mph|km/h|kmh|kph)?\s*", text, re.IGNORECASE)
  if not match:
    return 0.0
  return float(match[1]) * (0.44704 if match[2] and match[2].lower() == "mph" else 1.0 / 3.6)


def direction_geometry(polyline, scale):
  points = np.asarray(polyline) * scale
  delta = np.diff(points, axis=0)
  good = np.linalg.norm(delta, axis=1) > 0.05
  delta = delta[good]
  return (points[:-1][good], delta, np.sum(delta**2, axis=1), np.degrees(np.arctan2(delta[:, 1], delta[:, 0])))


def load_tagged_roads(paths):
  nodes, osm_ways = {}, {}
  for path in paths:
    for _, element in ET.iterparse(path, events=("end",)):
      if element.tag == "node":
        nodes[element.get("id")] = (float(element.get("lat")), float(element.get("lon")))
        element.clear()
      elif element.tag == "way":
        tags = {tag.get("k"): tag.get("v") for tag in element.findall("tag")}
        if tags.get("highway") in MAINLINE | LINKS:
          osm_ways[element.get("id")] = ([nd.get("ref") for nd in element.findall("nd")], tags)
        element.clear()
      elif element.tag == "relation":
        element.clear()
  uses = Counter(n for path, _ in osm_ways.values() for n in set(path))
  roads = []
  for way_id, (node_ids, tags) in osm_ways.items():
    if len(node_ids) < 2 or any(n not in nodes for n in node_ids):
      continue
    splits = [0, *[i for i, n in enumerate(node_ids[1:-1], 1) if uses[n] > 1], len(node_ids) - 1]
    highway = tags["highway"]
    lanes_raw = tags.get("lanes", "")
    lanes = int(lanes_raw) if lanes_raw.isdigit() else (3 if highway in MAINLINE else 1)
    maximum = speed_mps(tags.get("maxspeed", ""))
    oneway = tags.get("oneway", "yes" if highway in {"motorway", "motorway_link"} else "no")
    for start, end in pairwise(splits):
      piece = node_ids[start : end + 1]
      directions = [piece[::-1]] if oneway == "-1" else [piece]
      if oneway in ("no", "0", "false"):
        directions.append(piece[::-1])
      for direction in directions:
        polyline = [nodes[n] for n in direction]
        advisory_tag = "maxspeed:advisory:forward" if direction == piece else "maxspeed:advisory:backward"
        roads.append(
          {
            "id": way_id,
            "highway": highway,
            "name": tags.get("name", ""),
            "ref": tags.get("ref", ""),
            "lanes": lanes,
            "max": maximum,
            "advisory": speed_mps(tags.get(advisory_tag, tags.get("maxspeed:advisory", ""))),
            "start": direction[0],
            "end": direction[-1],
            "points": polyline,
          }
        )
  return roads


def build_forks(roads, scale):
  outgoing = defaultdict(list)
  incoming = defaultdict(list)
  for i, road in enumerate(roads):
    outgoing[road["start"]].append(i)
    incoming[road["end"]].append(i)

  def follows(i, category, indexes, reverse=False):
    """Extend on a unique continuation with the same road role."""
    points = list(reversed(roads[i]["points"])) if reverse else list(roads[i]["points"])
    used = {i}
    current = i
    while np.linalg.norm((np.array(points[-1]) - points[0]) * scale) < 320.0:
      road = roads[current]
      end_key, opposite_key = ("start", "end") if reverse else ("end", "start")
      candidates = [j for j in indexes[road[end_key]] if j not in used and roads[j]["highway"] in category and roads[j][end_key] != road[opposite_key]]
      if len(candidates) != 1:
        break
      current = candidates[0]
      used.add(current)
      more = roads[current]["points"]
      points.extend((list(reversed(more)) if reverse else more)[1:])
    return list(reversed(points)) if reverse else points

  forks = []
  for parent, current in enumerate(roads):
    if current["highway"] not in MAINLINE:
      continue
    children = [j for j in outgoing[current["end"]] if roads[j]["end"] != current["start"]]

    def is_candidate(j, parent_lanes=current["lanes"]):
      road = roads[j]
      return road["highway"] in LINKS or (road["highway"] in MAINLINE and road["advisory"] > 0.0 and road["lanes"] < parent_lanes)

    for ramp in filter(is_candidate, children):
      alternatives = [j for j in children if j != ramp]
      if not alternatives:
        continue
      branch_types = LINKS if roads[ramp]["highway"] in LINKS else MAINLINE
      geometries = [direction_geometry(follows(j, LINKS if roads[j]["highway"] in LINKS else MAINLINE, outgoing), scale) for j in alternatives]
      competing_geometry = tuple(np.concatenate(parts) for parts in zip(*geometries, strict=True))
      forks.append(
        (
          parent,
          ramp,
          alternatives[0],
          current["points"][-1],
          direction_geometry(follows(ramp, branch_types, outgoing), scale),
          competing_geometry,
          direction_geometry(follows(parent, MAINLINE, incoming, reverse=True), scale),
        )
      )
  return forks


def load_graph(path=RAMP_GRAPH_PATH):
  graph = json.loads(path.read_text())
  if graph["version"] != 1 or not -85 <= graph["latitude"] <= 85:
    raise ValueError("Unsupported ramp graph")
  roads = graph["roads"]
  scale = projection(graph["latitude"])
  return roads, build_forks(roads, scale), scale


if __name__ == "__main__":
  import argparse

  parser = argparse.ArgumentParser(description="Build a ramp graph from local OpenStreetMap XML extracts")
  parser.add_argument("output", type=Path)
  parser.add_argument("extracts", nargs="+")
  args = parser.parse_args()
  roads = load_tagged_roads(args.extracts)
  if not roads:
    parser.error("the extracts contain no complete highway ways")
  latitude = float(np.median([road["points"][0][0] for road in roads]))
  args.output.write_text(
    json.dumps(
      {"version": 1, "latitude": latitude, "roads": roads, "attribution": "© OpenStreetMap contributors, ODbL, https://www.openstreetmap.org/copyright"},
      separators=(",", ":"),
    )
  )
