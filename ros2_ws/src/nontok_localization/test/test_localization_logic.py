"""Tests for tag localisation. Synthetic views with known answers; no ROS, no camera."""
import math

import numpy as np
import pytest

from nontok_localization import fix_gate
from nontok_localization.detector import TagDetector
from nontok_localization.fix_gate import FixGate, HealthTracker
from nontok_localization.geometry import between, camera_mount, compose, inverse, wrap
from nontok_localization.pose_solver import Detection, SolverConfig, solve
from nontok_localization.synthetic import detections_from_pose, nominal_camera, project_tag, render
from nontok_localization.tag_geometry import Tag, TagMap, pair_map

CAMERA_HEIGHT = 0.20
FRONT = nominal_camera('front_left', camera_mount(0.12, 0.06, CAMERA_HEIGHT, 0.0))
REAR = nominal_camera('rear', camera_mount(-0.12, 0.0, CAMERA_HEIGHT, math.pi))

# A wall along x = 3 m with tags facing back toward the origin (normal = -x, yaw 180 deg).
WALL_PAIR = TagMap([Tag(10, 3.0, 0.15, 0.20, math.pi), Tag(11, 3.0, -0.15, 0.20, math.pi)])
SINGLE = TagMap([Tag(30, 3.0, 0.0, 0.20, math.pi)])


def close(fix, pose, xy=0.02, yaw_deg=1.5):
    assert math.hypot(fix.x - pose[0], fix.y - pose[1]) < xy, (fix, pose)
    assert abs(wrap(fix.yaw - pose[2])) < math.radians(yaw_deg), (fix, pose)


# ---------------- geometry ----------------
def test_se2_helpers():
    a, b = (1.0, 2.0, 0.5), (0.3, -0.2, -1.0)
    assert between(a, compose(a, b)) == pytest.approx(b)
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_tag_corner_layout():
    tag = Tag(1, 3.0, 0.0, 0.2, math.pi, 0.16)   # on the x = 3 wall, facing -x
    tl, tr, br, bl = tag.corners_map()
    assert tl[2] == pytest.approx(0.28) and bl[2] == pytest.approx(0.12)   # top is up
    # Seen from the origin looking along +x, the viewer's left is +y.
    assert tl[1] == pytest.approx(0.08) and tr[1] == pytest.approx(-0.08)
    assert np.allclose([tl[0], tr[0], br[0], bl[0]], 3.0)


def test_projection_matches_intuition():
    corners = project_tag(SINGLE.get(30), FRONT, (1.0, 0.0, 0.0))
    tl, tr, br, bl = corners
    assert tl[0] < tr[0] and tl[1] < bl[1]            # left of right, above bottom
    assert project_tag(SINGLE.get(30), FRONT, (1.0, 0.0, math.pi)) is None   # facing away
    assert project_tag(SINGLE.get(30), REAR, (1.0, 0.0, math.pi)) is not None


# ---------------- solver ----------------
@pytest.mark.parametrize('pose', [
    (1.5, 0.0, 0.0), (2.2, 0.3, -0.2), (1.2, -0.5, 0.35), (0.9, 0.2, 0.1), (2.0, -0.4, 0.15),
])
def test_pair_gives_exact_pose_without_noise(pose):
    fix = solve(detections_from_pose(WALL_PAIR, FRONT, pose), FRONT, WALL_PAIR)
    assert fix is not None and fix.n_tags == 2 and fix.yaw_trusted
    close(fix, pose, xy=0.002, yaw_deg=0.1)


def test_pair_accuracy_with_pixel_noise():
    rng = np.random.default_rng(1)
    errors_xy, errors_yaw = [], []
    for _ in range(200):
        pose = (rng.uniform(1.0, 2.4), rng.uniform(-0.4, 0.4), rng.uniform(-0.25, 0.25))
        detections = detections_from_pose(WALL_PAIR, FRONT, pose, noise_px=0.3, rng=rng)
        if len(detections) < 2:
            continue
        fix = solve(detections, FRONT, WALL_PAIR)
        assert fix is not None
        errors_xy.append(math.hypot(fix.x - pose[0], fix.y - pose[1]))
        errors_yaw.append(abs(wrap(fix.yaw - pose[2])))
    assert len(errors_xy) > 100
    assert np.percentile(errors_xy, 95) < 0.05
    assert np.percentile(errors_yaw, 95) < math.radians(3.0)


def test_single_tag_close_is_trusted_far_is_position_only():
    near = (1.8, 0.1, 0.05)   # 1.1 m from the tag
    fix = solve(detections_from_pose(SINGLE, FRONT, near), FRONT, SINGLE)
    close(fix, near, xy=0.005, yaw_deg=0.5)
    assert fix.yaw_trusted
    far = (0.8, 0.1, 0.05)    # 2.1 m: beyond the heading range
    fix = solve(detections_from_pose(SINGLE, FRONT, far), FRONT, SINGLE)
    close(fix, far, xy=0.01, yaw_deg=1.0)
    assert not fix.yaw_trusted
    _, var_yaw = fix.variances(SolverConfig())
    assert var_yaw >= 1000.0


def test_single_tag_noise_never_gives_a_wild_pose():
    """With noise the two-solution ambiguity appears; the answer must stay near the truth."""
    rng = np.random.default_rng(2)
    bad = 0
    total = 0
    for _ in range(300):
        pose = (rng.uniform(1.0, 2.2), rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2))
        detections = detections_from_pose(SINGLE, FRONT, pose, noise_px=0.3, rng=rng)
        if not detections:
            continue
        fix = solve(detections, FRONT, SINGLE, prior_yaw=pose[2] + rng.normal(0, 0.02))
        if fix is None:
            continue
        total += 1
        if math.hypot(fix.x - pose[0], fix.y - pose[1]) > 0.15:
            bad += 1
    assert total > 200
    assert bad <= total * 0.02


def test_range_gate():
    pose = (0.2, 0.0, 0.0)    # 2.7 m from the tags
    assert solve(detections_from_pose(WALL_PAIR, FRONT, pose), FRONT, WALL_PAIR) is None


def test_view_angle_gate():
    # Standing almost alongside the wall: very oblique view.
    pose = (2.7, 1.2, -1.25)
    detections = detections_from_pose(SINGLE, FRONT, pose)
    if detections:
        assert solve(detections, FRONT, SINGLE) is None


def test_wrong_corner_order_is_rejected():
    detections = detections_from_pose(SINGLE, FRONT, (1.8, 0.0, 0.0))
    rotated = [Detection(d.id, np.roll(d.corners, 1, axis=0)) for d in detections]
    assert solve(rotated, FRONT, SINGLE) is None


def test_unknown_tags_are_ignored():
    detections = detections_from_pose(WALL_PAIR, FRONT, (1.5, 0.0, 0.0))
    assert solve(detections, FRONT, SINGLE) is None


def test_rear_camera():
    pose = (1.6, 0.2, math.pi + 0.1)   # facing away from the wall: the rear camera sees it
    fix = solve(detections_from_pose(WALL_PAIR, REAR, pose), REAR, WALL_PAIR)
    close(fix, (pose[0], pose[1], wrap(pose[2])), xy=0.002, yaw_deg=0.1)


def test_station_frame_pair_for_docking():
    station = pair_map(10, 11, z=0.20)
    # Robot 0.8 m in front of the station, facing it (toward -x in the station frame).
    pose = (0.8, 0.05, math.pi - 0.1)
    fix = solve(detections_from_pose(station, FRONT, pose), FRONT, station)
    close(fix, pose, xy=0.002, yaw_deg=0.1)


# ---------------- detector on rendered images ----------------
def test_detector_finds_rendered_tags_and_pose_is_right():
    detector = TagDetector()
    pose = (1.7, 0.1, -0.05)
    image = render(WALL_PAIR, FRONT, pose)
    detections = detector.detect(image)
    assert sorted(d.id for d in detections) == [10, 11]
    truth = {d.id: d.corners for d in detections_from_pose(WALL_PAIR, FRONT, pose)}
    for d in detections:
        assert np.abs(d.corners - truth[d.id]).max() < 1.5   # corner order and position
    fix = solve(detections, FRONT, WALL_PAIR)
    close(fix, pose, xy=0.03, yaw_deg=2.0)


def test_detector_at_working_range():
    detector = TagDetector()
    pose = (0.6, 0.0, 0.0)    # 2.4 m: near the limit in PLAN.md 5.2
    detections = detector.detect(render(WALL_PAIR, FRONT, pose))
    assert sorted(d.id for d in detections) == [10, 11]
    fix = solve(detections, FRONT, WALL_PAIR)
    close(fix, pose, xy=0.10, yaw_deg=4.0)


# ---------------- gate and health ----------------
def test_gate_accepts_rejects_and_relocalises():
    gate = FixGate()
    estimate = (1.0, 1.0, 0.0)
    assert gate.check((1.1, 1.0, 0.05), estimate, True) == fix_gate.ACCEPT
    assert gate.check((3.0, 1.0, 0.0), estimate, True) == fix_gate.REJECT
    assert gate.check((3.02, 1.0, 0.0), estimate, True) == fix_gate.REJECT
    assert gate.check((3.01, 1.01, 0.0), estimate, True) == fix_gate.RELOCALISE
    # Scattered outliers never trigger a reset.
    gate = FixGate()
    for spot in [(3.0, 1.0), (5.0, 2.0), (0.0, 4.0), (3.0, 1.0), (6.0, 6.0)]:
        assert gate.check((spot[0], spot[1], 0.0), estimate, True) == fix_gate.REJECT
    # An accepted fix in between clears the count.
    gate = FixGate()
    gate.check((3.0, 1.0, 0.0), estimate, True)
    gate.check((3.0, 1.0, 0.0), estimate, True)
    gate.check((1.0, 1.0, 0.0), estimate, True)
    assert gate.check((3.0, 1.0, 0.0), estimate, True) == fix_gate.REJECT


def test_gate_position_only_fixes():
    gate = FixGate()
    estimate = (1.0, 1.0, 0.0)
    assert gate.check((1.1, 1.0, 2.0), estimate, False) == fix_gate.ACCEPT   # heading ignored
    for _ in range(5):
        assert gate.check((3.0, 1.0, 0.0), estimate, False) == fix_gate.REJECT
    assert gate.check((1.0, 1.0, 0.0), None, False) == fix_gate.REJECT
    assert gate.check((1.0, 1.0, 0.0), None, True) == fix_gate.RELOCALISE


def test_health_tracker():
    health = HealthTracker(max_age_s=10.0, max_travel_m=3.0)
    assert not health.healthy(0.0)
    health.on_odometry(0.0, 0.0)
    health.on_fix(0.0)
    assert health.healthy(5.0)
    assert health.healthy(60.0)              # long ago, but the robot has not moved
    for i in range(1, 40):
        health.on_odometry(0.1 * i, 0.0)     # 3.9 m without a fix
    assert not health.healthy(60.0)
    assert health.healthy(5.0)               # recent fix is enough on its own
    health.on_fix(61.0)
    assert health.healthy(62.0) and health.metres_since_fix() == 0.0
