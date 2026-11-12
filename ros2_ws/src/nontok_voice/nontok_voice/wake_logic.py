"""When does a stream of wake-word scores count as "the robot was called"? Pure Python."""
from collections import deque


class WakeTrigger:
    """Fires when the score reaches `threshold` in `needed` of the last `window` audio blocks,
    then ignores everything for `refractory_s` so one call gives one trigger."""

    def __init__(self, threshold: float = 0.5, window: int = 3, needed: int = 2, refractory_s: float = 3.0):
        self.threshold = threshold
        self.needed = needed
        self.refractory_s = refractory_s
        self._recent = deque(maxlen=window)
        self._quiet_until = float('-inf')

    def reset(self) -> None:
        self._recent.clear()

    def update(self, score: float, now: float) -> bool:
        if now < self._quiet_until:
            return False
        self._recent.append(score >= self.threshold)
        if sum(self._recent) >= self.needed:
            self._recent.clear()
            self._quiet_until = now + self.refractory_s
            return True
        return False
