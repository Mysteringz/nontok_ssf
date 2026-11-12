"""Turns the Mega's millisecond clock into host time. Pure Python.

A line is received some variable time after it was measured. The smallest (receive time -
Mega time) seen recently is the best estimate of the fixed offset, because delays only add.
"""
from collections import deque
from typing import Deque, Optional, Tuple


class ClockSync:
    def __init__(self, window_s: float = 10.0, reset_gap_ms: int = 1000):
        self.window_s = window_s
        self.reset_gap_ms = reset_gap_ms
        self._samples: Deque[Tuple[float, float]] = deque()  # (receive time, offset)
        self._last_raw: Optional[int] = None
        self._wraps = 0

    def reset(self) -> None:
        self._samples.clear()
        self._last_raw = None
        self._wraps = 0

    def stamp(self, t_ms: int, receive_time: float) -> float:
        """Host time (seconds) at which the Mega took the measurement stamped t_ms."""
        if self._last_raw is not None and t_ms < self._last_raw:
            if self._last_raw > 2 ** 32 - 60000 and t_ms < 60000:
                self._wraps += 1          # the 32-bit millisecond counter wrapped (49.7 days)
            elif self._last_raw - t_ms > self.reset_gap_ms:
                self.reset()              # the Mega restarted: its clock began again from zero
        self._last_raw = t_ms
        mega_s = (t_ms + self._wraps * 2 ** 32) / 1000.0

        self._samples.append((receive_time, receive_time - mega_s))
        while self._samples and receive_time - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        offset = min(sample[1] for sample in self._samples)
        return mega_s + offset
