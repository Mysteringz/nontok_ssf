"""Precision docking logic (PLAN.md sections 5.7 and 13.9). Pure Python.

The camera says, about ten times a second and a little late, where the robot is relative
to the station's tags. Each such measurement is turned into "where the station is in the
odometry frame". The control loop then steers on odometry toward a target fixed to that
station frame. The robot therefore moves smoothly between measurements and can finish the
last few centimetres on odometry when the tags leave the image.

Station frame: origin midway between the two tags, x pointing out of the wall. The robot's
target is on that x axis (plus an optional sideways offset), facing the tags - or with its
back to them (back_in), for a tray that tips out over the rear.
"""
import bisect
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from nontok_localization.geometry import Pose2D, between, compose, inverse, wrap

ACQUIRE, APPROACH, FINAL, SETTLE, CHARGE, DONE, FAILED = (
    'ACQUIRE', 'APPROACH', 'FINAL', 'SETTLE', 'CHARGE', 'DONE', 'FAILED')


@dataclass
class DockConfig:
    standoff_m: float = 0.35
    lateral_offset_m: float = 0.0
    back_in: bool = False                 # reverse in and stop with the back to the tags
    pre_distance_m: float = 0.15          # APPROACH stops this far short of the standoff
    # speed limits
    approach_speed: float = 0.08
    final_speed: float = 0.04
    max_yaw_rate: float = 0.3
    # gains (1/s)
    gain_forward: float = 0.8
    gain_sideways: float = 1.0
    gain_yaw: float = 1.5
    # when APPROACH hands over to FINAL
    approach_tolerance_m: float = 0.03
    approach_tolerance_rad: float = math.radians(3.0)
    # final tolerances
    lateral_tolerance_m: float = 0.015
    distance_tolerance_m: float = 0.020
    yaw_tolerance_rad: float = math.radians(3.0)
    settle_s: float = 0.5
    stop_fraction: float = 0.5            # FINAL stops inside this fraction of the tolerances
    # tags
    acquire_count: int = 5
    acquire_timeout_s: float = 3.0
    filter_factor: float = 0.5            # weight of a new measurement
    tag_pause_s: float = 0.3              # no tags for this long: stand still
    tag_timeout_s: float = 2.0            # no tags for this long: give up
    blind_distance_m: float = 0.30        # this close to the target the tags may leave the image
    # safety
    min_front_range_m: float = 0.05
    total_timeout_s: float = 30.0
    # charger (dock only)
    charge_threshold_v: float = 4.0
    charge_confirm_s: float = 1.0
    charge_wait_s: float = 3.0
    charge_attempts: int = 3


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class PoseBuffer:
    """Recent odometry poses, to look up where the robot was when an image was taken."""

    def __init__(self, keep_s: float = 2.0):
        self.keep_s = keep_s
        self._times: List[float] = []
        self._poses: List[Pose2D] = []

    def add(self, stamp: float, pose: Pose2D) -> None:
        if self._times and stamp <= self._times[-1]:
            return
        self._times.append(stamp)
        self._poses.append(pose)
        while self._times and stamp - self._times[0] > self.keep_s:
            self._times.pop(0)
            self._poses.pop(0)

    def latest(self) -> Optional[Pose2D]:
        return self._poses[-1] if self._poses else None

    def at(self, stamp: float) -> Optional[Pose2D]:
        """Pose at `stamp`, interpolated; None if the buffer does not reach that time."""
        if not self._times or stamp < self._times[0] - 0.05 or stamp > self._times[-1] + 0.2:
            return None
        i = bisect.bisect_left(self._times, stamp)
        if i == 0:
            return self._poses[0]
        if i >= len(self._times):
            return self._poses[-1]
        t0, t1 = self._times[i - 1], self._times[i]
        a, b = self._poses[i - 1], self._poses[i]
        f = (stamp - t0) / (t1 - t0)
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), wrap(a[2] + f * wrap(b[2] - a[2])))


class DockController:
    def __init__(self, cfg: DockConfig, require_charge: bool = False):
        self.cfg = cfg
        self.require_charge = require_charge
        self.phase = ACQUIRE
        self.message = ''
        self.errors: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # forward, sideways, yaw
        self._station: Optional[Pose2D] = None    # station frame in the odometry frame
        self._measurements = 0
        self._last_measurement = None
        self._start = None
        self._phase_start = None
        self._charge_since = None
        self._attempts = 0

    # ---- inputs ----
    def start(self, now: float) -> None:
        self._start = self._phase_start = now

    def on_measurement(self, robot_in_station: Pose2D, odom_at_image: Pose2D, now: float) -> None:
        """A camera measurement: the robot's pose in the station frame, and the odometry pose
        the robot had when that image was taken."""
        station = compose(odom_at_image, inverse(robot_in_station))
        if self._station is None:
            self._station = station
        else:
            f = self.cfg.filter_factor
            old = self._station
            self._station = (old[0] + f * (station[0] - old[0]), old[1] + f * (station[1] - old[1]),
                             wrap(old[2] + f * wrap(station[2] - old[2])))
        self._measurements += 1
        self._last_measurement = now

    # ---- state ----
    @property
    def measurements(self) -> int:
        return self._measurements

    def tag_age(self, now: float) -> float:
        """Seconds since the last camera measurement (infinite if there was none)."""
        return float('inf') if self._last_measurement is None else now - self._last_measurement

    @property
    def done(self) -> bool:
        return self.phase in (DONE, FAILED)

    @property
    def success(self) -> bool:
        return self.phase == DONE

    def _enter(self, phase: str, now: float, message: str = '') -> None:
        self.phase = phase
        self._phase_start = now
        if message:
            self.message = message

    def _target(self, extra: float) -> Pose2D:
        cfg = self.cfg
        return compose(self._station, (cfg.standoff_m + extra, cfg.lateral_offset_m,
                                       0.0 if cfg.back_in else math.pi))

    def _within_final_tolerance(self, fraction: float = 1.0) -> bool:
        cfg = self.cfg
        return (abs(self.errors[0]) <= fraction * cfg.distance_tolerance_m
                and abs(self.errors[1]) <= fraction * cfg.lateral_tolerance_m
                and abs(self.errors[2]) <= fraction * cfg.yaw_tolerance_rad)

    # ---- one control step ----
    def step(self, odom_pose: Pose2D, now: float, front_range: Optional[float] = None,
             charge_v: float = 0.0) -> Tuple[float, float, float]:
        """Returns the velocity command (vx, vy, wz) for this instant."""
        cfg = self.cfg
        if self._start is None:
            self.start(now)
        if self.done:
            return (0.0, 0.0, 0.0)
        if now - self._start > cfg.total_timeout_s:
            self._enter(FAILED, now, 'timed out in %s' % self.phase)
            return (0.0, 0.0, 0.0)
        # Backing in, the front ultrasonic points away from the station: nothing senses behind.
        if (not cfg.back_in and front_range is not None and front_range < cfg.min_front_range_m
                and self.phase in (APPROACH, FINAL)):
            self._enter(FAILED, now, 'something %.0f mm ahead' % (front_range * 1000))
            return (0.0, 0.0, 0.0)

        if self.phase == ACQUIRE:
            if self._measurements >= cfg.acquire_count:
                self._enter(APPROACH, now)
            elif now - self._phase_start > cfg.acquire_timeout_s:
                self._enter(FAILED, now, 'station tags not seen')
            return (0.0, 0.0, 0.0)

        extra = cfg.pre_distance_m if self.phase == APPROACH else 0.0
        error = between(odom_pose, self._target(extra))    # the target seen from the robot
        self.errors = (error[0], error[1], error[2])
        remaining = math.hypot(error[0], error[1])

        tag_age = now - self._last_measurement
        blind_ok = self.phase in (FINAL, SETTLE, CHARGE) and remaining <= cfg.blind_distance_m
        if tag_age > cfg.tag_timeout_s and not blind_ok:
            self._enter(FAILED, now, 'lost the station tags')
            return (0.0, 0.0, 0.0)
        if tag_age > cfg.tag_pause_s and not blind_ok:
            return (0.0, 0.0, 0.0)     # wait for the tags to come back

        if self.phase == APPROACH:
            if remaining <= cfg.approach_tolerance_m and abs(error[2]) <= cfg.approach_tolerance_rad:
                self._enter(FINAL, now)
                return (0.0, 0.0, 0.0)
            return self._command(error, cfg.approach_speed)

        if self.phase == FINAL:
            # Drive on to well inside the tolerance; stopping at its edge leaves no margin
            # for the roll-out, and the result is then checked against the full tolerance.
            if self._within_final_tolerance(cfg.stop_fraction):
                self._enter(SETTLE, now)
                return (0.0, 0.0, 0.0)
            return self._command(error, cfg.final_speed)

        if self.phase == SETTLE:
            if now - self._phase_start >= cfg.settle_s:
                if not self._within_final_tolerance():
                    self._enter(FINAL, now)          # rolled out of tolerance while stopping
                elif self.require_charge:
                    self._charge_since = None
                    self._enter(CHARGE, now)
                else:
                    self._enter(DONE, now, 'aligned')
            return (0.0, 0.0, 0.0)

        if self.phase == CHARGE:
            if charge_v >= cfg.charge_threshold_v:
                if self._charge_since is None:
                    self._charge_since = now
                if now - self._charge_since >= cfg.charge_confirm_s:
                    self._enter(DONE, now, 'charging')
            else:
                self._charge_since = None
                if now - self._phase_start > cfg.charge_wait_s:
                    self._attempts += 1
                    if self._attempts >= cfg.charge_attempts:
                        self._enter(FAILED, now, 'aligned but not charging')
                    else:
                        self._enter(APPROACH, now)   # back off and come in again
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def _command(self, error: Pose2D, speed_limit: float) -> Tuple[float, float, float]:
        cfg = self.cfg
        vx = cfg.gain_forward * error[0]
        vy = cfg.gain_sideways * error[1]
        speed = math.hypot(vx, vy)
        if speed > speed_limit:      # scale both together: keep the direction
            vx, vy = vx * speed_limit / speed, vy * speed_limit / speed
        return (vx, vy, _clip(cfg.gain_yaw * error[2], cfg.max_yaw_rate))
