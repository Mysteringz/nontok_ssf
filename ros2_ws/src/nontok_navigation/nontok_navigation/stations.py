"""Stations: where each one is and how to approach it. Pure Python.

A station's position is not written down twice: it comes from its two tags in tags.yaml.
stations.yaml only adds how far away to stop.

Station frame: origin midway between the two tags, x pointing out of the wall toward the
robot. A robot "at the station" stands on the station's x axis, facing the tags - or, at a
back_in station, with its back to them: the tray tips out over the rear of the robot.
"""
import math
from dataclasses import dataclass
from typing import Dict, Tuple

import yaml

Pose2D = Tuple[float, float, float]


@dataclass(frozen=True)
class Station:
    name: str
    left_tag: int
    right_tag: int
    standoff_m: float          # base_link to the tag wall when aligned
    approach_m: float          # base_link to the tag wall at the Nav2 goal
    lateral_offset_m: float = 0.0   # + = stop to the right of centre, as seen facing the tags
    is_dock: bool = False
    back_in: bool = False      # the robot reverses up to the tags (rear camera) instead of facing them
    contact_range_m: float = -1.0   # front ultrasonic reading that counts as arrived (-1: docking.yaml's)


def load_stations(path: str) -> Dict[str, Station]:
    with open(path) as handle:
        data = yaml.safe_load(handle)
    stations = {}
    for name, entry in data['stations'].items():
        stations[name] = Station(
            name, int(entry['tag_ids'][0]), int(entry['tag_ids'][1]),
            float(entry['standoff_m']), float(entry['approach_m']),
            float(entry.get('lateral_offset_m', 0.0)), bool(entry.get('is_dock', False)),
            bool(entry.get('back_in', False)), float(entry.get('contact_range_m', -1.0)))
    return stations


def station_frame(station: Station, tag_map) -> Pose2D:
    """Pose of the station frame in the map: (x, y, direction of its x axis)."""
    left, right = tag_map.get(station.left_tag), tag_map.get(station.right_tag)
    if left is None or right is None:
        raise KeyError('Station %s: tags %d and %d must both be in tags.yaml' % (
            station.name, station.left_tag, station.right_tag))
    yaw = math.atan2(math.sin(left.yaw) + math.sin(right.yaw), math.cos(left.yaw) + math.cos(right.yaw))
    return ((left.x + right.x) / 2.0, (left.y + right.y) / 2.0, yaw)


def tag_spacing(station: Station, tag_map) -> float:
    left, right = tag_map.get(station.left_tag), tag_map.get(station.right_tag)
    return math.hypot(left.x - right.x, left.y - right.y)


def pose_in_front(frame: Pose2D, distance: float, lateral: float = 0.0, back_in: bool = False) -> Pose2D:
    """Map pose of a robot `distance` in front of the station, facing it (back_in: facing away)."""
    x, y, yaw = frame
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * distance - s * lateral, y + s * distance + c * lateral,
            math.atan2(s, c) if back_in else math.atan2(-s, -c))


def approach_pose(station: Station, tag_map) -> Pose2D:
    """The Nav2 goal for this station."""
    return pose_in_front(station_frame(station, tag_map), station.approach_m, station.lateral_offset_m,
                         station.back_in)


def aligned_pose(station: Station, tag_map) -> Pose2D:
    """Where the robot stands when docked or aligned at this station."""
    return pose_in_front(station_frame(station, tag_map), station.standoff_m, station.lateral_offset_m,
                         station.back_in)
