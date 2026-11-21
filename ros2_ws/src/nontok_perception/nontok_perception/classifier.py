"""Item classifier (PLAN.md sections 7.4 and 13.10).

Two interchangeable models, both run on the CPU with ONNX Runtime:

- OnnxClassifier: a YOLO classification network trained on photos of the team's own items
  (training/train_cls.py). The accurate one, once a dataset exists.
- ClipZeroShot: a general image-text model (CLIP) that picks the item whose written
  description fits the picture best. Needs no training photos, only the descriptions in
  items.yaml, so it works from day one and for items added later. Slower and less exact.

preprocess(), clip_preprocess(), zero_shot_scores() and vote() are pure functions and are
tested without a model.
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


def load_descriptions(path: str) -> Dict[str, List[str]]:
    """items.yaml: label -> the phrases that describe it, for the zero-shot model.

    Every item gets its own name as a description unless `descriptions` lists others.
    Labels under `not_items` (returned with a leading "_") describe what is not an item.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    listed = data.get('descriptions') or {}
    out = {}
    for item in (data.get('items') or {}):
        phrases = listed.get(item) or [str(item).replace('_', ' ')]
        out[str(item)] = [phrases] if isinstance(phrases, str) else [str(p) for p in phrases]
    for name, phrases in (data.get('not_items') or {}).items():
        out['_' + str(name)] = [phrases] if isinstance(phrases, str) else [str(p) for p in phrases]
    return out


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


CLIP_MEAN = np.array([0.48145466, 0.4578275, 0.40821073], dtype=np.float32)
CLIP_STD = np.array([0.26862954, 0.26130258, 0.27577711], dtype=np.float32)


def clip_preprocess(crop_bgr: np.ndarray, size: int = 224) -> np.ndarray:
    """A BGR crop -> CLIP's input: 1 x 3 x size x size, RGB, normalised. The shorter side is
    scaled to `size` and the centre is kept."""
    height, width = crop_bgr.shape[:2]
    scale = size / min(height, width)
    resized = cv2.resize(crop_bgr, (max(size, round(width * scale)), max(size, round(height * scale))),
                         interpolation=cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA)
    top, left = (resized.shape[0] - size) // 2, (resized.shape[1] - size) // 2
    rgb = cv2.cvtColor(resized[top:top + size, left:left + size], cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(((rgb - CLIP_MEAN) / CLIP_STD).transpose(2, 0, 1)[None])


def zero_shot_scores(image_embedding: np.ndarray, text_embeddings: np.ndarray, scale: float = 100.0) -> np.ndarray:
    """Probability of each description given the picture: softmax of scaled cosine similarity."""
    image = np.asarray(image_embedding, dtype=np.float64).reshape(-1)
    image = image / (np.linalg.norm(image) + 1e-12)
    text = np.asarray(text_embeddings, dtype=np.float64)
    text = text / (np.linalg.norm(text, axis=1, keepdims=True) + 1e-12)
    logits = scale * (text @ image)
    logits = np.exp(logits - logits.max())
    return logits / logits.sum()


class ClipZeroShot:
    """classify() like OnnxClassifier, from descriptions instead of training photos.

    prompts_path is the .npz written by training/build_item_prompts.py: `labels` (item
    names, plus names starting with "_" for things that are not items, such as an empty
    tray or a hand) and `embeddings` (one row per label).
    """

    def __init__(self, vision_model_path: str, prompts_path: str, stations: Optional[Dict[str, str]] = None,
                 threads: int = 0):
        import onnxruntime
        options = onnxruntime.SessionOptions()
        if threads:
            options.intra_op_num_threads = threads
        self.session = onnxruntime.InferenceSession(vision_model_path, options, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        outputs = [o.name for o in self.session.get_outputs()]
        self.output_name = 'image_embeds' if 'image_embeds' in outputs else outputs[0]
        data = np.load(prompts_path, allow_pickle=False)
        self.labels = [str(label) for label in data['labels']]
        self.embeddings = data['embeddings'].astype(np.float32)
        self.scale = float(data['scale']) if 'scale' in data else 100.0
        self.stations = stations or {}

    def scores(self, crop_bgr: np.ndarray) -> np.ndarray:
        embedding = self.session.run([self.output_name], {self.input_name: clip_preprocess(crop_bgr)})[0]
        return zero_shot_scores(embedding, self.embeddings, self.scale)

    def classify(self, crop_bgr: np.ndarray) -> Tuple[str, float]:
        return pick_by_station(self.labels, self.scores(crop_bgr), self.stations)


def pick_by_station(labels: Sequence[str], scores: Sequence[float], stations: Dict[str, str]) -> Tuple[str, float]:
    """(item, confidence), judged by station.

    The robot only has to choose a station, and look-alikes usually share one (bolt and nut,
    pliers and wire stripper). So the scores of all items of a station are added, the best
    station wins, and the answer is that station's most likely item with the station's
    total as its confidence. A label starting with "_" (empty tray, a hand) is not an item:
    if it wins, the answer is no item ('').
    """
    totals: Dict[str, float] = {}
    for label, score in zip(labels, scores):
        group = label if label.startswith('_') else stations.get(label, label)
        totals[group] = totals.get(group, 0.0) + float(score)
    group = max(totals, key=totals.get)
    if group.startswith('_'):
        return '', totals[group]
    members = [(float(score), label) for label, score in zip(labels, scores)
               if not label.startswith('_') and stations.get(label, label) == group]
    return max(members)[1], totals[group]
