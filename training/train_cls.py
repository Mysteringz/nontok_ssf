#!/usr/bin/env python3
"""Trains the item classifier, exports it to ONNX and checks the export (PLAN.md 13.10).

Run on a PC with a GPU or on Colab (pip install ultralytics onnxruntime):

    python3 training/train_cls.py                      train, export, check
    python3 training/train_cls.py --weights runs/.../best.pt --skip-train     export and check only

Copy the resulting items.onnx to ros2_ws/src/nontok_perception/models/ on the robot.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'nontok_perception'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--data', default=os.path.join(HERE, 'dataset_split'))
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--weights', default='yolo11n-cls.pt')
    parser.add_argument('--skip-train', action='store_true')
    parser.add_argument('--out', default=os.path.join(HERE, 'items.onnx'))
    args = parser.parse_args()
    from ultralytics import YOLO

    model = YOLO(args.weights)
    if not args.skip_train:
        # Items lie in the tray at any angle, so rotate and flip freely.
        model.train(data=args.data, imgsz=224, epochs=args.epochs, degrees=180, flipud=0.5, fliplr=0.5,
                    hsv_v=0.4, hsv_s=0.5, project=os.path.join(HERE, 'runs'), name='items')
    exported = model.export(format='onnx', imgsz=224)
    os.replace(exported, args.out)
    print('Exported %s' % args.out)

    # Parity check: the robot's own preprocessing + ONNX Runtime must agree with Ultralytics.
    from nontok_perception.classifier import OnnxClassifier
    onnx_model = OnnxClassifier(args.out)
    images = sorted(glob.glob(os.path.join(args.data, 'val', '*', '*.jpg')))[::max(1, len(glob.glob(
        os.path.join(args.data, 'val', '*', '*.jpg'))) // 20)][:20]
    disagreements = 0
    for path in images:
        image = cv2.imread(path)
        reference = model.predict(image, imgsz=224, verbose=False)[0]
        expected = reference.names[int(reference.probs.top1)]
        label, _ = onnx_model.classify(image)
        disagreements += label != expected
    print('Parity check: %d of %d images agree' % (len(images) - disagreements, len(images)))
    if disagreements:
        raise SystemExit('The ONNX path disagrees with Ultralytics: check the preprocessing before using this model.')


if __name__ == '__main__':
    main()
