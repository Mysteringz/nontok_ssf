#!/usr/bin/env python3
"""Quick end-to-end check of the classifier pipeline on whatever images exist.

    python train_quick.py --items led resistor           (default: all items in dataset/)

1. Split dataset/<session>/<item>/ per item by time: the last --val-frac of each item's
   images go to validation. (Optimistic: with several sessions, use the robot's
   training/split_by_session.py instead, which holds out whole sessions.)
2. Train YOLO11n-cls, 224 px, on the Mac GPU (MPS), with the robot plan's augmentation
   (rotation up to 180 deg, both flips, colour jitter).
3. Export to ONNX and score the ONNX model with the robot's own preprocessing
   (nontok_perception/classifier.py: plain resize to 224, RGB, 0-1) and ONNX Runtime CPU.

For the robot's real model use ../train_cls.py on a session split (../split_by_session.py);
this script is the quick check while there are only a few videos.
"""
import argparse
import ast
import glob
import os
import shutil
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'nontok_perception'))
from nontok_perception.classifier import preprocess   # the robot's own input path


def split(dataset, out, items, val_frac, sessions=None):
    if os.path.isdir(out):
        shutil.rmtree(out)
    counts = {}
    for item in items:
        files = sorted(f for f in glob.glob(os.path.join(dataset, '[!_]*', item, '*.jpg'))
                       if not sessions or f.split(os.sep)[-3] in sessions)
        n_val = max(1, int(round(len(files) * val_frac)))
        for part, chunk in (('train', files[:-n_val]), ('val', files[-n_val:])):
            os.makedirs(os.path.join(out, part, item), exist_ok=True)
            for f in chunk:
                shutil.copy2(f, os.path.join(out, part, item, os.path.basename(f)))
            counts[(part, item)] = len(chunk)
    for item in items:
        print('  %-12s train %4d  val %4d' % (item, counts[('train', item)], counts[('val', item)]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--dataset', default=os.path.join(HERE, 'dataset'))
    ap.add_argument('--items', nargs='+')
    ap.add_argument('--sessions', nargs='+', help='only these sessions (default: all)')
    ap.add_argument('--test-sessions', nargs='+', default=[],
                    help='also score the exported model on all images of these sessions (unseen data)')
    ap.add_argument('--val-frac', type=float, default=0.25)
    ap.add_argument('--epochs', type=int, default=30)
    ap.add_argument('--device', default='mps')
    ap.add_argument('--name', default='quick')
    args = ap.parse_args()

    items = args.items or sorted({os.path.basename(p) for p in glob.glob(os.path.join(args.dataset, '[!_]*', '*'))
                                  if os.path.isdir(p)})
    split_dir = os.path.join(HERE, 'dataset_split')
    print('Split (by time within each video):')
    split(args.dataset, split_dir, items, args.val_frac, args.sessions)

    from ultralytics import YOLO
    model = YOLO('yolo11n-cls.pt')
    model.train(data=split_dir, imgsz=224, epochs=args.epochs, batch=32, device=args.device,
                degrees=180, flipud=0.5, fliplr=0.5, hsv_v=0.4, hsv_s=0.5,
                project=os.path.join(HERE, 'runs'), name=args.name, exist_ok=True, verbose=False, plots=False)
    onnx_path = model.export(format='onnx', imgsz=224)
    target = os.path.join(HERE, 'items.onnx')
    shutil.copy2(onnx_path, target)
    print('Exported %s' % target)

    import onnxruntime
    sess = onnxruntime.InferenceSession(target, providers=['CPUExecutionProvider'])
    names = ast.literal_eval(sess.get_modelmeta().custom_metadata_map['names'])
    labels = [names[i] for i in sorted(names)]
    inp = sess.get_inputs()[0].name
    score(sess, labels, items, lambda item: sorted(glob.glob(os.path.join(split_dir, 'val', item, '*.jpg'))),
          'held-out images (same sessions)')
    for test in args.test_sessions:
        score(sess, labels, items, lambda item: sorted(glob.glob(os.path.join(args.dataset, test, item, '*.jpg'))),
              'images of unseen session %s' % test)


def score(sess, labels, items, files_of, what):
    inp = sess.get_inputs()[0].name
    total = right = 0
    times = []
    confusion = {}
    for item in items:
        for f in files_of(item):
            x = preprocess(cv2.imread(f))
            t0 = time.perf_counter()
            out = sess.run(None, {inp: x})[0].reshape(-1)
            times.append(time.perf_counter() - t0)
            pred = labels[int(np.argmax(out))]
            total += 1
            right += pred == item
            confusion[(item, pred)] = confusion.get((item, pred), 0) + 1
    if not total:
        return
    print('ONNX (robot preprocessing) on %d %s: %.1f %% correct' % (total, what, 100.0 * right / max(total, 1)))
    for (truth, pred), n in sorted(confusion.items()):
        print('  %-12s -> %-12s %d' % (truth, pred, n))
    print('Mac CPU time per image: %.1f ms (median)' % (1000 * float(np.median(times))))


if __name__ == '__main__':
    main()
