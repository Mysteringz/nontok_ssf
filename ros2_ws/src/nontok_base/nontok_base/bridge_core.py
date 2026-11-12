"""The decisions of the base bridge, without ROS or a serial port. Pure Python.

The node (base_bridge_node.py) feeds received lines and commands in and publishes what
comes out.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import protocol
from .clock_sync import ClockSync
from .kinematics import MecanumParams, count_delta, counts_to_body
from .protocol import Odo, Ready, Reply, Rng, Sta


@dataclass
class OdomSample:
    stamp: float                 # host time of the measurement
    vx: float                    # m/s, body frame
    vy: float
    yaw_rate: float              # rad/s from the gyro, scaled
    x: float                     # dead-reckoned pose (wheel distance + gyro heading)
    y: float
    yaw: float


@dataclass
class RangeSample:
    stamp: float
    name: str
    range_m: float


class RangeFilter:
    """Turns raw ultrasonic readings into what the costmap should see (PLAN.md 13.5).

    - a valid echo under max_range is passed on;
    - anything further, and "no echo", becomes exactly max_range ("clear to the maximum");
    - except a sudden "no echo" right after a close reading, which is more likely a slanted
      surface than free space: nothing is reported then.
    """

    def __init__(self, max_range: float = 1.5, near_m: float = 0.5, hold_s: float = 0.5):
        self.max_range = max_range
        self.near_m = near_m
        self.hold_s = hold_s
        self._last_near_time: Optional[float] = None

    def update(self, reading_m: Optional[float], now: float) -> Optional[float]:
        if reading_m is not None:
            if reading_m < self.near_m:
                self._last_near_time = now
            return min(reading_m, self.max_range)
        if self._last_near_time is not None and now - self._last_near_time <= self.hold_s:
            return None
        return self.max_range


class BridgeCore:
    def __init__(self, params: MecanumParams, gyro_scale: float = 1.0, cmd_timeout_s: float = 0.25,
                 odo_timeout_s: float = 0.5, max_range: float = 1.5):
        self.params = params
        self.gyro_scale = gyro_scale
        self.cmd_timeout_s = cmd_timeout_s
        self.odo_timeout_s = odo_timeout_s
        self.clock = ClockSync()
        self.range_filters = {name: RangeFilter(max_range) for name in protocol.RANGE_NAMES}

        self.status: Optional[Sta] = None
        self.armed = False       # follows the Mega's replies, so it is known at once after ARM
        # Re-arming after a watchdog trip (see rearm_line).
        self.want_armed = False
        self.max_rearms = 3
        self.rearm_window_s = 60.0
        self.rearm_delay_s = 0.3
        self._tripped_at: Optional[float] = None
        self._rearm_times: List[float] = []
        self.ready_seen = False
        self.x = self.y = self.yaw = 0.0
        self._last_odo: Optional[Odo] = None
        self._last_odo_stamp: Optional[float] = None
        self._last_odo_receive: Optional[float] = None
        self._cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._cmd_time: Optional[float] = None
        self.replies: List[Reply] = []   # OK / DONE / ERR / EVT, for the node to act on

    # ---- commands toward the Mega ----
    def set_command(self, vx: float, vy: float, wz: float, now: float) -> None:
        self._cmd = (vx, vy, wz)
        self._cmd_time = now

    def velocity_line(self, now: float) -> Optional[str]:
        """The VEL line to send now, or None when the Mega is not armed.

        Sent at a steady rate while armed, zeros included: the firmware disarms itself if
        it hears nothing for 300 ms.
        """
        if not self.armed:
            return None
        fresh = self._cmd_time is not None and now - self._cmd_time <= self.cmd_timeout_s
        vx, vy, wz = self._cmd if fresh else (0.0, 0.0, 0.0)
        return protocol.encode_vel(vx, vy, wz)

    def rearm_line(self, now: float) -> Optional[str]:
        """ARM, once, a moment after the Mega's watchdog disarmed a base that was meant to be armed.

        The watchdog trips when this computer stalls for 300 ms; the motors stop, which is
        the point. Once commands flow again the mission should carry on rather than sit
        disarmed until it times out. A stop or disarm request, a Bluetooth stop or a Mega
        restart cancels this, and after max_rearms trips in rearm_window_s it gives up:
        repeated stalls are a fault for a person to look at.
        """
        if self.armed or not self.want_armed or self._tripped_at is None:
            return None
        if now - self._tripped_at < self.rearm_delay_s:
            return None
        self._tripped_at = None
        self._rearm_times = [t for t in self._rearm_times if now - t < self.rearm_window_s]
        if len(self._rearm_times) >= self.max_rearms:
            self.want_armed = False
            return None
        self._rearm_times.append(now)
        self._cmd_time = None   # start again from zero velocity
        return protocol.ARM

    # ---- health ----
    def link_fault(self, now: float) -> str:
        if self._last_odo_receive is None:
            return 'NO_DATA'
        if now - self._last_odo_receive > self.odo_timeout_s:
            return 'LINK_LOST'
        return ''

    # ---- lines from the Mega ----
    def handle_line(self, line: str, receive_time: float):
        """Returns an OdomSample, a list of RangeSample, a Sta, a Ready, a Reply or None."""
        message = protocol.parse_line(line)
        if isinstance(message, Odo):
            return self._handle_odo(message, receive_time)
        if isinstance(message, Rng):
            stamp = self.clock.stamp(message.t_ms, receive_time)
            samples = []
            for name, reading in zip(protocol.RANGE_NAMES, message.ranges_m):
                value = self.range_filters[name].update(reading, stamp)
                if value is not None:
                    samples.append(RangeSample(stamp, name, value))
            return samples
        if isinstance(message, Sta):
            self.status = message
            self.armed = message.armed
            return message
        if isinstance(message, Ready):
            # The Mega restarted: its clock and encoder counts began again.
            self.ready_seen = True
            self.clock.reset()
            self._last_odo = None
            self._last_odo_stamp = None
            self.status = None
            self.armed = False
            self.want_armed = False
            return message
        if isinstance(message, Reply):
            if message.kind == 'OK' and message.arg == 'ARM':
                self.armed = True
                self.want_armed = True
            elif (message.kind, message.arg) == ('ERR', 'WATCHDOG'):
                self.armed = False
                self._tripped_at = receive_time
            elif (message.kind, message.arg) in (('OK', 'STOP'), ('OK', 'DISARM'), ('EVT', 'BTSTOP')):
                self.armed = False
                self.want_armed = False
            self.replies.append(message)
            return message
        return None

    def _handle_odo(self, odo: Odo, receive_time: float) -> Optional[OdomSample]:
        stamp = self.clock.stamp(odo.t_ms, receive_time)
        self._last_odo_receive = receive_time
        previous, previous_stamp = self._last_odo, self._last_odo_stamp
        self._last_odo, self._last_odo_stamp = odo, stamp
        if previous is None or previous_stamp is None:
            return None
        dt = (odo.t_ms - previous.t_ms) / 1000.0
        if dt <= 0 or dt > 0.5:
            return None  # duplicate, or a gap too long to turn into a velocity
        deltas = [count_delta(new, old) for new, old in zip(odo.counts, previous.counts)]
        dx, dy, _ = counts_to_body(deltas, self.params)
        yaw_rate = odo.yaw_rate * self.gyro_scale

        # Dead-reckoned pose, for calibration tools and as a fallback. The EKF does the real fusion.
        mid_yaw = self.yaw + 0.5 * yaw_rate * dt
        self.x += dx * math.cos(mid_yaw) - dy * math.sin(mid_yaw)
        self.y += dx * math.sin(mid_yaw) + dy * math.cos(mid_yaw)
        self.yaw += yaw_rate * dt
        return OdomSample(stamp, dx / dt, dy / dt, yaw_rate, self.x, self.y, self.yaw)
