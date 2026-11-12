#!/usr/bin/env python3
"""Scores the exported classifier on the held-out session, by item and by station (CP7).

    python3 training/eval_station_accuracy.py --model training/items.onnx

Uses the same code path as the robot: preprocessing, ONNX Runtime and items.yaml.
The CP7 target is 95 % at station level.
"""
import argparse
import collections
import glob
import os
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
PERCEPTION = os.path.join(HERE, '..', 'ros2_ws', 'src', 'nontok_perception')
sys.path.insert(0, PERCEPTION)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--model', default=os.path.join(HERE, 'items.onnx'))
    parser.add_argument('--val', default=os.path.join(HERE, 'dataset_split', 'val'))
    parser.add_argument('--items', default=os.path.join(PERCEPTION, 'config', 'items.yaml'))
    parser.add_argument('--min-confidence', type=float, default=0.80)
    args = parser.parse_args()
    from nontok_perception.classifier import OnnxClassifier, load_items

    stations = load_items(args.items)
    model = OnnxClassifier(args.model)
    total = right_item = right_station = confident_wrong = unknown = 0
    confusion = collections.Counter()
    for folder in sorted(glob.glob(os.path.join(args.val, '*'))):
        truth = os.path.basename(folder)
        for path in sorted(glob.glob(os.path.join(folder, '*.jpg'))):
            label, confidence = model.classify(cv2.imread(path))
            total += 1
            right_item += label == truth
            same_station = stations.get(label) == stations.get(truth) and truth in stations
            right_station += same_station
            if confidence < args.min_confidence:
                unknown += 1
            elif not same_station:
                confident_wrong += 1        # the dangerous case: delivered to the wrong box
            if label != truth:
                confusion[(truth, label)] += 1
    if not total:
        raise SystemExit('No validation images under %s' % args.val)
    print('Images:                      %d' % total)
    print('Item accuracy:               %.1f %%' % (100.0 * right_item / total))
    print('Station accuracy:            %.1f %%   (target 95 %%)' % (100.0 * right_station / total))
    print('Below the confidence limit:  %.1f %%   (robot would ask to reposition)' % (100.0 * unknown / total))
    print('Confident and wrong station: %.1f %%   (must be near zero)' % (100.0 * confident_wrong / total))
    for (truth, label), count in confusion.most_common(10):
        print('  %s mistaken for %s: %d' % (truth, label, count))


if __name__ == '__main__':
    main()
