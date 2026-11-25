"""Tests for the map generator, the stations and the shipped placeholder configuration."""
import dataclasses
import math
import pathlib

import numpy as np
import pytest
import yaml

from nontok_localization.tag_geometry import TagMap
from nontok_navigation import draw_map
from nontok_navigation.draw_map import FREE, OCCUPIED, cell_value, rasterise
from nontok_navigation.stations import aligned_pose, approach_pose, load_stations, station_frame, tag_spacing

SRC = pathlib.Path(__file__).parents[2]
NAV_CONFIG = SRC / 'nontok_navigation' / 'config'
TAGS = TagMap.load(str(SRC / 'nontok_localization' / 'config' / 'tags.yaml'))
STATIONS = load_stations(str(NAV_CONFIG / 'stations.yaml'))


def test_rasterise_cells_and_orientation():
    bounds = [0.0, 0.0, 1.0, 0.5]
    image = rasterise(bounds, [[0.0, 0.0, 0.1, 0.1], [0.9, 0.4, 1.0, 0.5]], 0.1)
    assert image.shape == (5, 10)
    assert image[4, 0] == OCCUPIED          # bottom-left of the map is the last image row
    assert image[0, 9] == OCCUPIED          # top-right
    assert int((image == OCCUPIED).sum()) == 2
    assert cell_value(image, bounds, 0.1, 0.05, 0.05) == OCCUPIED
    assert cell_value(image, bounds, 0.1, 0.25, 0.25) == FREE


def test_partial_overlap_marks_the_cell_and_outside_is_clipped():
    image = rasterise([0.0, 0.0, 1.0, 1.0], [[0.26, 0.26, 0.31, 0.31], [-5.0, 0.9, 5.0, 9.0]], 0.1)
    assert cell_value(image, [0, 0, 1, 1], 0.1, 0.25, 0.25) == OCCUPIED
    assert cell_value(image, [0, 0, 1, 1], 0.1, 0.35, 0.35) == OCCUPIED   # rectangle reaches 0.31
    assert cell_value(image, [0, 0, 1, 1], 0.1, 0.45, 0.25) == FREE
    assert (image[0] == OCCUPIED).all()                                    # clipped to the map


def test_write_map_files(tmp_path):
    bounds, rectangles, resolution = draw_map.load_layout(str(NAV_CONFIG / 'lab_layout.yaml'))
    image = rasterise(bounds, rectangles, resolution)
    draw_map.write_map(str(tmp_path / 'map'), image, bounds, resolution)
    meta = yaml.safe_load((tmp_path / 'map.yaml').read_text())
    assert meta['image'] == 'map.pgm' and meta['origin'][:2] == [bounds[0], bounds[1]]
    data = (tmp_path / 'map.pgm').read_bytes()
    header = b'P5\n%d %d\n255\n' % (image.shape[1], image.shape[0])
    assert data.startswith(header) and len(data) == len(header) + image.size


def test_shipped_map_is_up_to_date():
    """config/map.pgm must be what draw_map produces from config/lab_layout.yaml."""
    bounds, rectangles, resolution = draw_map.load_layout(str(NAV_CONFIG / 'lab_layout.yaml'))
    image = rasterise(bounds, rectangles, resolution)
    data = (NAV_CONFIG / 'map.pgm').read_bytes()
    assert data.endswith(image.tobytes())


def test_station_frames_and_poses():
    dock = STATIONS['dock']
    frame = station_frame(dock, TAGS)
    assert frame[:2] == pytest.approx((0.35, 0.0))   # real room, map package 24 Nov
    assert abs(math.cos(frame[2]) + 1.0) < 1e-9                 # faces -x, back toward the robot
    assert aligned_pose(dock, TAGS) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)   # docked = map origin
    assert approach_pose(dock, TAGS) == pytest.approx((-0.75, 0.0, 0.0), abs=1e-9)
    electrical = approach_pose(STATIONS['electrical'], TAGS)
    assert electrical == pytest.approx((-1.7, -0.3, -math.pi / 2), abs=1e-9)   # out from the front wall, facing it
    assert not any(station.back_in for station in STATIONS.values())
    backed = approach_pose(dataclasses.replace(STATIONS['electrical'], back_in=True), TAGS)
    assert backed == pytest.approx((-1.7, -0.3, math.pi / 2), abs=1e-9)   # same spot, back to the wall


def test_every_station_is_consistent_with_tags_and_map():
    bounds, rectangles, resolution = draw_map.load_layout(str(NAV_CONFIG / 'lab_layout.yaml'))
    image = rasterise(bounds, rectangles, resolution)
    assert set(STATIONS) == {'dock', 'pickup', 'electrical', 'mechanical', 'tools'}   # no unknown (24 Nov); tools 25 Nov
    for station in STATIONS.values():
        assert tag_spacing(station, TAGS) == pytest.approx(0.30, abs=0.01)
        frame = station_frame(station, TAGS)
        left, right = TAGS.get(station.left_tag), TAGS.get(station.right_tag)
        # "left" must really be on the left for someone facing the tags.
        to_left = (left.x - frame[0], left.y - frame[1])
        viewer_left = (math.sin(frame[2]), -math.cos(frame[2]))
        assert to_left[0] * viewer_left[0] + to_left[1] * viewer_left[1] > 0, station.name
        assert station.approach_m > station.standoff_m
        for pose in (approach_pose(station, TAGS), aligned_pose(station, TAGS)):
            assert cell_value(image, bounds, resolution, pose[0], pose[1]) == FREE, station.name
