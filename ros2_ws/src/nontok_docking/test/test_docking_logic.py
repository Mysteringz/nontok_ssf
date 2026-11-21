"""Docking tests on a small simulation: a robot that follows velocity commands, odometry that
has drifted away from the truth, and a camera that reports late and with noise."""
import math

import numpy as np
import pytest

from nontok_docking import dock_controller as dc
from nontok_docking.dock_controller import DockConfig, DockController, PoseBuffer
from nontok_localization.geometry import between, compose, wrap

DT = 0.05
STATION = (2.0, 1.0, math.pi)      # station frame in the world: tags on a wall, facing -x


class Sim:
    def __init__(self, start_in_station, seed=0, camera_noise=0.004, yaw_noise=0.01, latency=0.10,
                 blind_below=0.30, standoff=0.35):
        self.rng = np.random.default_rng(seed)
        self.true = compose(STATION, start_in_station)
        self.odom_origin = (5.0, -3.0, 0.7)       # odometry frame differs from the world: only motion matters
        self.t = 0.0
        self.history = []
        self.camera_noise, self.yaw_noise, self.latency = camera_noise, yaw_noise, latency
        self.blind_below = blind_below
        self.standoff = standoff
        self.charge_v = 0.0
        self.tags_visible = True
        self.buffer = PoseBuffer()

    def odom(self):
        return compose(self.odom_origin, self.true)

    def robot_in_station(self):
        return between(STATION, self.true)

    def run(self, controller, seconds=40.0, hook=None):
        steps = int(seconds / DT)
        for i in range(steps):
            self.t += DT
            self.buffer.add(self.t, self.odom())
            self.history.append((self.t, self.robot_in_station(), self.odom()))
            # Camera at 10 Hz, delivered `latency` late; no view when very close to the wall.
            if i % 2 == 0 and self.tags_visible:
                past = [h for h in self.history if h[0] <= self.t - self.latency]
                if past:
                    stamp, seen, _ = past[-1]
                    if seen[0] > self.blind_below:      # closer than this the tags leave the image
                        noisy = (seen[0] + self.rng.normal(0, self.camera_noise),
                                 seen[1] + self.rng.normal(0, self.camera_noise),
                                 seen[2] + self.rng.normal(0, self.yaw_noise))
                        controller.on_measurement(noisy, self.buffer.at(stamp), self.t)
            if hook:
                hook(self)
            vx, vy, wz = controller.step(self.odom(), self.t, front_range=self.robot_in_station()[0] - 0.17,
                                         charge_v=self.charge_v)
            mid = self.true[2] + 0.5 * wz * DT
            self.true = (self.true[0] + (vx * math.cos(mid) - vy * math.sin(mid)) * DT,
                         self.true[1] + (vx * math.sin(mid) + vy * math.cos(mid)) * DT,
                         wrap(self.true[2] + wz * DT))
            if controller.done:
                break


def final_error(sim, cfg):
    x, y, yaw = sim.robot_in_station()
    return x - cfg.standoff_m, y - cfg.lateral_offset_m, wrap(yaw - math.pi)


@pytest.mark.parametrize('offset', [
    (0.90, 0.0, math.pi), (0.90, 0.15, math.pi), (0.90, -0.15, math.pi),
    (0.95, 0.10, math.pi + math.radians(10)), (0.85, -0.12, math.pi - math.radians(10)),
    (1.00, 0.15, math.pi - math.radians(8)),
])
def test_aligns_within_tolerance_from_offset_starts(offset):
    cfg = DockConfig()
    for seed in range(3):
        sim = Sim(offset, seed=seed)
        controller = DockController(cfg)
        sim.run(controller)
        assert controller.success, (controller.phase, controller.message)
        dx, dy, dyaw = final_error(sim, cfg)
        # True error: the CP6 limits, with a little allowance for camera noise.
        assert abs(dy) <= 0.020 and abs(dx) <= 0.025 and abs(dyaw) <= math.radians(3.5), (dx, dy, dyaw)


def test_speed_limits_are_respected():
    cfg = DockConfig()
    sim = Sim((0.9, 0.15, math.pi + 0.15))
    controller = DockController(cfg)
    speeds = []
    original = controller.step

    def recording(*args, **kwargs):
        command = original(*args, **kwargs)
        speeds.append((math.hypot(command[0], command[1]), abs(command[2]), controller.phase))
        return command
    controller.step = recording
    sim.run(controller)
    assert max(s[0] for s in speeds) <= cfg.approach_speed + 1e-9
    assert max(s[1] for s in speeds) <= cfg.max_yaw_rate + 1e-9
    assert max(s[0] for s in speeds if s[2] == dc.FINAL) <= cfg.final_speed + 1e-9


def test_lateral_offset_and_standoff_are_used():
    cfg = DockConfig(standoff_m=0.50, lateral_offset_m=0.06)
    sim = Sim((1.0, 0.0, math.pi), standoff=0.50)
    controller = DockController(cfg)
    sim.run(controller)
    assert controller.success
    x, y, _ = sim.robot_in_station()
    assert x == pytest.approx(0.50, abs=0.025) and y == pytest.approx(0.06, abs=0.02)


def test_fails_when_tags_are_never_seen():
    sim = Sim((0.9, 0.0, math.pi))
    sim.tags_visible = False
    controller = DockController(DockConfig())
    sim.run(controller)
    assert controller.phase == dc.FAILED and 'not seen' in controller.message
    assert sim.robot_in_station() == pytest.approx((0.9, 0.0, math.pi))    # never moved


def test_stops_then_fails_when_tags_are_lost_far_out():
    sim = Sim((1.2, 0.1, math.pi))
    controller = DockController(DockConfig())

    def hide(s):
        if s.t > 2.0:
            s.tags_visible = False
    sim.run(controller, hook=hide)
    assert controller.phase == dc.FAILED and 'lost' in controller.message
    assert sim.robot_in_station()[0] > 0.6      # stopped well short of the station


def test_finishes_blind_when_tags_disappear_close_in():
    cfg = DockConfig()
    sim = Sim((0.9, 0.05, math.pi))
    controller = DockController(cfg)

    def hide(s):
        if controller.phase == dc.FINAL and s.robot_in_station()[0] < cfg.standoff_m + 0.10:
            s.tags_visible = False
    sim.run(controller, hook=hide)
    assert controller.success
    dx, dy, _ = final_error(sim, cfg)
    assert abs(dx) <= 0.03 and abs(dy) <= 0.02


def test_obstacle_in_front_aborts():
    sim = Sim((0.9, 0.0, math.pi))
    controller = DockController(DockConfig())
    sim.run(controller, seconds=1.0)
    assert not controller.done
    controller.step(sim.odom(), sim.t + DT, front_range=0.03)
    assert controller.phase == dc.FAILED


def test_dock_waits_for_charging():
    cfg = DockConfig(standoff_m=0.60)
    sim = Sim((1.1, 0.05, math.pi), standoff=0.60)
    controller = DockController(cfg, require_charge=True)

    def charger(s):
        s.charge_v = 5.0 if abs(s.robot_in_station()[0] - 0.60) < 0.03 and abs(s.robot_in_station()[1]) < 0.03 else 0.0
    sim.run(controller, hook=charger)
    assert controller.success and controller.message == 'charging'


def test_dock_retries_then_gives_up_without_charge():
    cfg = DockConfig(standoff_m=0.60, total_timeout_s=120.0)
    sim = Sim((1.1, 0.0, math.pi), standoff=0.60)
    controller = DockController(cfg, require_charge=True)
    phases = []
    sim.run(controller, seconds=120.0, hook=lambda s: phases.append(controller.phase))
    assert controller.phase == dc.FAILED and 'not charging' in controller.message
    entries = sum(1 for a, b in zip(phases, phases[1:]) if b == dc.CHARGE and a != dc.CHARGE)
    assert entries == cfg.charge_attempts


def test_pose_buffer_interpolates():
    buffer = PoseBuffer()
    buffer.add(1.0, (0.0, 0.0, 3.1))
    buffer.add(2.0, (1.0, 2.0, -3.1))
    x, y, yaw = buffer.at(1.5)
    assert (x, y) == pytest.approx((0.5, 1.0))
    assert abs(abs(yaw) - math.pi) < 0.05          # interpolates across the +/-pi seam
    assert buffer.at(0.5) is None and buffer.at(3.0) is None


def test_backs_in_to_a_station_for_a_rear_tipping_tray():
    cfg = DockConfig(back_in=True)
    for start in ((0.90, 0.0, 0.0), (0.95, 0.12, math.radians(10)), (0.85, -0.12, -math.radians(8))):
        sim = Sim(start)
        controller = DockController(cfg)
        reversed_only = []
        original = controller.step

        def recording(*args, **kwargs):
            command = original(*args, **kwargs)
            reversed_only.append(command[0] <= 1e-9)
            return command
        controller.step = recording
        sim.run(controller)
        assert controller.success, (controller.phase, controller.message)
        x, y, yaw = sim.robot_in_station()
        assert abs(x - cfg.standoff_m) <= 0.025 and abs(y) <= 0.020 and abs(wrap(yaw)) <= math.radians(3.5)
        assert sum(reversed_only) > 0.95 * len(reversed_only)      # it came in backwards
