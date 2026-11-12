#!/usr/bin/env python3
"""Collects classifier training images from the tray camera (PLAN.md section 13.10).

Run on the robot, from ros2_ws/ (after `source install/setup.bash`, or with PYTHONPATH set to
src/nontok_perception:src/nontok_localization):

    python3 ../training/capture_tray_dataset.py --item bolt --session mon_am

1. With the tray EMPTY, press Enter: the empty reference is taken.
2. Put the item in, take your hand away, press Enter: one image is saved.
3. Move or turn the item and repeat. Aim for 150+ images per item over several sessions
   (different days, lights, places). Type q then Enter to finish.

Images are saved exactly as the robot will see them when classifying: cropped around the
item by the same code. The full frame is kept as well, in case the crop method changes.
"""
import argparse
import os
import time

import cv2

from nontok_perception.tray_logic import TrayConfig, TrayLogic


def grab(capture, count=5):
    frames = []
    for _ in range(count + 5):          # the first few may be stale
        ok, frame = capture.read()
        if ok:
            frames.append(frame)
    return frames[-count:]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--item', required=True, help='class name, e.g. bolt (must match items.yaml)')
    parser.add_argument('--session', default=time.strftime('%Y%m%d_%H%M'))
    parser.add_argument('--device', default='/dev/cam_tray')
    parser.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dataset'))
    parser.add_argument('--roi', type=float, nargs=8, default=[0.10, 0.10, 0.90, 0.10, 0.90, 0.90, 0.10, 0.90])
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not capture.isOpened():
        raise SystemExit('Cannot open %s' % args.device)
    logic = TrayLogic(TrayConfig(roi=[(args.roi[i], args.roi[i + 1]) for i in range(0, 8, 2)]))
    crops = os.path.join(args.out, args.session, args.item)
    fulls = os.path.join(args.out, '_full', args.session, args.item)
    os.makedirs(crops, exist_ok=True)
    os.makedirs(fulls, exist_ok=True)

    input('Empty the tray, then press Enter...')
    logic.set_reference(grab(capture))
    count = 0
    while True:
        if input('[%d saved] Place/move the item, hand away, Enter (q to quit): ' % count).strip().lower() == 'q':
            break
        frame = grab(capture, 1)[-1]
        crop = logic.item_crop(frame)
        if crop is None:
            print('  Nothing differs from the empty tray: not saved.')
            continue
        stamp = '%s_%04d' % (args.session, count)
        cv2.imwrite(os.path.join(crops, stamp + '.jpg'), crop)
        cv2.imwrite(os.path.join(fulls, stamp + '.jpg'), frame)
        count += 1
    print('Saved %d images to %s' % (count, crops))


if __name__ == '__main__':
    main()
