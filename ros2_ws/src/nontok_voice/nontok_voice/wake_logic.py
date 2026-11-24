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


def find_microphone(devices, wanted: str):
    """Index of the first input device whose name contains `wanted` (None = system default).

    The microphones here are the USB cameras' own, and two cameras of one model have the
    same name; sounddevice refuses an ambiguous name, so the first match is taken.
    """
    if not wanted:
        return None
    if str(wanted).isdigit():
        return int(wanted)
    for index, info in enumerate(devices):
        if info['max_input_channels'] > 0 and wanted.lower() in info['name'].lower():
            return index
    raise RuntimeError('No microphone with "%s" in its name' % wanted)


def load_model(model_path: str):
    """The openWakeWord model for `model_path`, using the two feature models stored next to
    it (melspectrogram.onnx, embedding_model.onnx) so nothing is downloaded on the robot."""
    import os
    from openwakeword.model import Model
    folder = os.path.dirname(os.path.abspath(model_path))
    extra = {}
    for key, name in (('melspec_model_path', 'melspectrogram.onnx'), ('embedding_model_path', 'embedding_model.onnx')):
        if os.path.exists(os.path.join(folder, name)):
            extra[key] = os.path.join(folder, name)
    return Model(wakeword_models=[model_path], inference_framework='onnx', **extra)

