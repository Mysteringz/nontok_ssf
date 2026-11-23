"""Serial protocol between the Jetson and the Mega (PLAN.md section 6.3).

Pure Python: no ROS, no serial port. The serial line uses integer mm/s, mrad/s and mm;
everything returned from here is in SI units.
"""
from dataclasses import dataclass
from typing import Optional, Tuple, Union

VEL_LIMIT_MM_S = 500
VEL_LIMIT_MRAD_S = 3000
RANGE_NAMES = ('left45', 'right45', 'front', 'left', 'right')  # order of the RNG fields


@dataclass(frozen=True)
class Odo:
    t_ms: int
    counts: Tuple[int, int, int, int]  # cumulative, FL FR RL RR
    yaw_rate: float                    # rad/s, counter-clockwise positive


@dataclass(frozen=True)
class Rng:
    t_ms: int
    ranges_m: Tuple[Optional[float], ...]  # None = no valid echo; order RANGE_NAMES


@dataclass(frozen=True)
class Sta:
    t_ms: int
    armed: bool
    tray_deg: float
    charge_v: float
    battery_v: float
    fault: str  # '' when healthy


@dataclass(frozen=True)
class Ready:
    name: str
    version: int


@dataclass(frozen=True)
class Reply:
    kind: str  # 'OK', 'DONE', 'PONG', 'ERR' or 'EVT'
    arg: str = ''


Message = Union[Odo, Rng, Sta, Ready, Reply]


def _clamp(value: float, limit: int) -> int:
    return int(max(-limit, min(limit, round(value))))


def encode_vel(vx: float, vy: float, wz: float) -> str:
    """Body velocity in m/s, m/s, rad/s -> a VEL line (clamped to the firmware's limits)."""
    return 'VEL %d %d %d' % (
        _clamp(vx * 1000.0, VEL_LIMIT_MM_S),
        _clamp(vy * 1000.0, VEL_LIMIT_MM_S),
        _clamp(wz * 1000.0, VEL_LIMIT_MRAD_S),
    )


TRAY_MAX_DEG = 130   # stepper travel the firmware allows (0 = flat at power-up, 130 = dump)


def encode_tray(angle_deg: float) -> str:
    """Tray stepper angle in degrees from the power-up position -> a TRAY line (clamped)."""
    return 'TRAY %d' % int(max(0, min(TRAY_MAX_DEG, round(angle_deg))))


def encode_oled(text: str) -> str:
    clean = ''.join(ch for ch in text if 32 <= ord(ch) < 127 and ch != '!')
    return 'OLED ' + clean[:16]


PING, ARM, DISARM, STOP, CAL = 'PING', 'ARM', 'DISARM', 'STOP', 'CAL'


def parse_line(line: str) -> Optional[Message]:
    """One received line -> a message, or None if it is empty, damaged or unknown."""
    parts = line.strip().split()
    if not parts:
        return None
    word, args = parts[0], parts[1:]
    try:
        if word == 'ODO' and len(args) == 6:
            numbers = [int(a) for a in args]
            if numbers[0] < 0 or abs(numbers[5]) > 10000:
                return None
            return Odo(numbers[0], tuple(numbers[1:5]), numbers[5] / 1000.0)
        if word == 'RNG' and len(args) == 6:
            numbers = [int(a) for a in args]
            if numbers[0] < 0:
                return None
            ranges = tuple(None if mm < 0 else mm / 1000.0 for mm in numbers[1:])
            if any(r is not None and r > 6.0 for r in ranges):
                return None
            return Rng(numbers[0], ranges)
        if word == 'STA' and len(args) == 6:
            t_ms, armed, tray, charge, battery = (int(a) for a in args[:5])
            if t_ms < 0 or armed not in (0, 1):
                return None
            fault = '' if args[5] == 'NONE' else args[5]
            return Sta(t_ms, bool(armed), float(tray), charge / 1000.0, battery / 1000.0, fault)
        if word == 'READY' and len(args) == 2:
            return Ready(args[0], int(args[1]))
    except ValueError:
        return None
    if word == 'PONG' and not args:
        return Reply('PONG')
    if word in ('OK', 'DONE', 'ERR', 'EVT') and len(args) == 1:
        return Reply(word, args[0])
    return None
