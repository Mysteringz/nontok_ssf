#!/usr/bin/env python3
"""Runs the trained classifier through a video and reports what lay on the surface, when.

    python detect_video.py videos/test/test.mkv [--model items.onnx] [--min-conf 0.80]

Same steps as the robot (PLAN.md 7.3/7.4), except the empty reference is a moving median of the video (Background in video_to_dataset.py);
item present = changed pixels, item still (hand gone), crop around it, ONNX classifier
with the robot's preprocessing. Like the robot, an answer below --min-conf is "unknown".
Writes <video>_detections.jpg: one crop per detected segment with its label.
"""
import argparse
import ast
import collections
import os

import cv2
import numpy as np

from video_to_dataset import Background, Still, changed_mask, grey, item_crop
from train_quick import preprocess   # = nontok_perception.classifier.preprocess

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('video')
    ap.add_argument('--model', default=os.path.join(HERE, 'items.onnx'))
    ap.add_argument('--min-conf', type=float, default=0.80)
    ap.add_argument('--still-s', type=float, default=0.4)
    ap.add_argument('--min-frac', type=float, default=0.0008)
    ap.add_argument('--max-frac', type=float, default=0.25)
    args = ap.parse_args()

    import onnxruntime
    sess = onnxruntime.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    names = ast.literal_eval(sess.get_modelmeta().custom_metadata_map['names'])
    labels = [names[i] for i in sorted(names)]
    inp = sess.get_inputs()[0].name
    print('Model classes: %s   (anything else can only be reported as unknown)' % ', '.join(labels))

    background = Background(args.video)
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    prev_g, roi, n = None, None, -1
    still = Still(args.still_s * fps)
    hits = []                                   # (t, label, conf, crop)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        t = n / fps
        if frame.mean() < 12:
            prev_g = None
            continue
        g = grey(frame)
        if roi is None:
            H, W = g.shape
            roi = np.zeros((H, W), np.uint8)
            b = int(0.03 * min(H, W))
            roi[b:H - b, b:W - b] = 1
        motion = 1.0 if prev_g is None else float(((cv2.absdiff(g, prev_g) > 25) & (roi > 0)).mean())
        prev_g = g
        if not still.update(motion) or n % 3:        # classify ~10 times a second
            continue
        mask = changed_mask(g, background.at(t), roi)
        frac = float(mask.mean())
        if not args.min_frac <= frac <= args.max_frac:
            continue
        crop, _ = item_crop(frame, mask)
        out = sess.run(None, {inp: preprocess(crop)})[0].reshape(-1).astype(np.float64)
        if not (np.all(out >= 0) and abs(out.sum() - 1) < 1e-3):
            out = np.exp(out - out.max()); out /= out.sum()
        k = int(np.argmax(out))
        hits.append((t, labels[k], float(out[k]), crop))
    cap.release()

    # group consecutive classifications (gap < 1 s) into segments
    segments = []
    for h in hits:
        if segments and h[0] - segments[-1][-1][0] < 1.0:
            segments[-1].append(h)
        else:
            segments.append([h])
    print('\n%-15s %-10s %-6s %-8s %s' % ('time', 'answer', 'conf', 'frames', 'votes'))
    tiles = []
    for seg in segments:
        if len(seg) < 3:                         # too short to be an item at rest
            continue
        votes = collections.Counter(h[1] for h in seg)
        label, count = votes.most_common(1)[0]
        conf = float(np.mean([h[2] for h in seg if h[1] == label]))
        answer = label if count * 2 > len(seg) and conf >= args.min_conf else 'unknown'
        print('%5.1f - %5.1f s   %-10s %.2f   %-8d %s' % (seg[0][0], seg[-1][0], answer, conf, len(seg),
                                                     ', '.join('%s %d' % kv for kv in votes.most_common())))
        tile = seg[len(seg) // 2][3].copy()
        cv2.rectangle(tile, (0, 0), (223, 40), (0, 0, 0), -1)
        cv2.putText(tile, '%.1fs %s %.2f' % (seg[0][0], answer, conf), (4, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        tiles.append(tile)
    if tiles:
        sheet = os.path.splitext(args.video)[0] + '_detections.jpg'
        cv2.imwrite(sheet, np.hstack(tiles))
        print('\nCrops: %s' % sheet)


if __name__ == '__main__':
    main()
