"""Everything the dashboard shows, kept as plain values. Pure Python: no ROS, no web server.

The node writes the newest value of each thing with set(); the web page asks for snapshot(),
which adds how old each value is so that a part that has gone quiet is seen as stale.
"""
import math
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional


class Telemetry:
    def __init__(self, log_length: int = 60, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._values: Dict[str, Any] = {}
        self._times: Dict[str, float] = {}
        self._counts: Dict[str, int] = {}
        self._rates: Dict[str, float] = {}
        self._rate_marks: Dict[str, tuple] = {}
        self._log = deque(maxlen=log_length)
        self._events = deque(maxlen=30)

    def set(self, key: str, value: Any) -> None:
        now = self._clock()
        with self._lock:
            self._values[key] = value
            self._times[key] = now
            self._counts[key] = self._counts.get(key, 0) + 1
            mark_time, mark_count = self._rate_marks.get(key, (now, 0))
            if now - mark_time >= 2.0:      # messages per second, over the last two seconds or so
                self._rates[key] = (self._counts[key] - mark_count) / (now - mark_time)
                self._rate_marks[key] = (now, self._counts[key])
            elif key not in self._rate_marks:
                self._rate_marks[key] = (now, 0)

    def log(self, level: str, source: str, text: str) -> None:
        with self._lock:
            self._log.append({'t': time.time(), 'level': level, 'source': source, 'text': text[:300]})

    def event(self, text: str) -> None:
        """A moment worth a line of its own: a state change, the wake word, something said."""
        with self._lock:
            self._events.append({'t': time.time(), 'text': text})

    def snapshot(self) -> Dict[str, Any]:
        now = self._clock()
        with self._lock:
            return {
                'time': time.time(),
                'values': dict(self._values),
                'age': {key: round(now - t, 2) for key, t in self._times.items()},
                'rate': {key: round(rate, 1) for key, rate in self._rates.items()},
                'log': list(self._log),
                'events': list(self._events),
            }


def yaw_of(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


def thin_path(points: List[tuple], max_points: int = 80) -> List[List[float]]:
    """A long planned path, cut down to at most max_points for drawing."""
    if len(points) <= max_points:
        return [[round(x, 3), round(y, 3)] for x, y in points]
    step = (len(points) - 1) / (max_points - 1)
    return [[round(points[int(round(i * step))][0], 3), round(points[int(round(i * step))][1], 3)]
            for i in range(max_points)]


def system_stats(read=None) -> Dict[str, Optional[float]]:
    """Load, memory and temperature of the computer, from /proc and /sys (None where unreadable)."""
    def default_read(path):
        with open(path) as handle:
            return handle.read()
    read = read or default_read
    out: Dict[str, Optional[float]] = {'load1': None, 'mem_used_mb': None, 'mem_total_mb': None, 'temp_c': None}
    try:
        out['load1'] = float(read('/proc/loadavg').split()[0])
    except (OSError, ValueError, IndexError):
        pass
    try:
        fields = {line.split(':')[0]: float(line.split()[1]) for line in read('/proc/meminfo').splitlines() if ':' in line}
        out['mem_total_mb'] = round(fields['MemTotal'] / 1024)
        out['mem_used_mb'] = round((fields['MemTotal'] - fields['MemAvailable']) / 1024)
    except (OSError, ValueError, KeyError, IndexError):
        pass
    temps = []
    for zone in range(6):
        try:
            temps.append(float(read('/sys/devices/virtual/thermal/thermal_zone%d/temp' % zone)) / 1000.0)
        except (OSError, ValueError):
            continue
    if temps:
        out['temp_c'] = round(max(t for t in temps if t < 150.0), 1)
    return out
