"""Item classifier (PLAN.md sections 7.4 and 13.10).

The model is a YOLO classification network exported to ONNX and run on the CPU with ONNX
Runtime. preprocess() and vote() are pure functions and are tested without a model.
"""
import ast
from collections import Counter
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml


def preprocess(crop_bgr: np.ndarray, size: int = 224) -> np.ndarray:
    """A square BGR crop -> the network input: 1 x 3 x size x size, RGB, values 0-1."""
    if crop_bgr.shape[0] != size or crop_bgr.shape[1] != size:
        crop_bgr = cv2.resize(crop_bgr, (size, size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])


def probabilities(output: np.ndarray) -> np.ndarray:
    """Model output -> probabilities. Applies softmax only if the export did not."""
    scores = np.asarray(output, dtype=np.float64).reshape(-1)
    if np.all(scores >= 0) and abs(scores.sum() - 1.0) < 1e-3:
        return scores
    scores = np.exp(scores - scores.max())
    return scores / scores.sum()


def vote(results: Sequence[Tuple[str, float]], min_confidence: float = 0.80) -> Tuple[bool, str, float]:
    """Several per-frame answers -> one decision: (accepted, label, confidence).

    The label must win more than half of the frames, and its mean confidence must reach
    min_confidence. Otherwise the item is treated as unknown.
    """
    if not results:
        return False, '', 0.0
    label, count = Counter(r[0] for r in results).most_common(1)[0]
    confidence = float(np.mean([r[1] for r in results if r[0] == label]))
    accepted = count * 2 > len(results) and confidence >= min_confidence
    return accepted, label, confidence


def load_items(path: str) -> Dict[str, str]:
    """items.yaml: item name -> station name."""
    with open(path) as handle:
        return {str(k): str(v) for k, v in (yaml.safe_load(handle).get('items') or {}).items()}


class OnnxClassifier:
    def __init__(self, model_path: str, labels: Optional[List[str]] = None):
        import onnxruntime
        self.session = onnxruntime.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.size = int(self.session.get_inputs()[0].shape[-1] or 224)
        if labels is None:
            # Ultralytics stores the class names in the model file as text like "{0: 'bolt', ...}".
            names = self.session.get_modelmeta().custom_metadata_map.get('names')
            if names is None:
                raise ValueError('The model has no class names; pass labels explicitly')
            parsed = ast.literal_eval(names)
            labels = [parsed[i] for i in sorted(parsed)]
        self.labels = labels

    def classify(self, crop_bgr: np.ndarray) -> Tuple[str, float]:
        output = self.session.run(None, {self.input_name: preprocess(crop_bgr, self.size)})[0]
        scores = probabilities(output)
        best = int(np.argmax(scores))
        return self.labels[best], float(scores[best])
