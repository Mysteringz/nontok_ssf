#!/usr/bin/env python3
"""Builds the train/val folders Ultralytics expects, keeping whole sessions together.

    python3 training/split_by_session.py --val-sessions fri_pm

Input:  training/dataset/<session>/<item>/*.jpg     (from capture_tray_dataset.py)
Output: training/dataset_split/{train,val}/<item>/*.jpg

Images from one session look alike, so splitting at random would make the validation score
too optimistic. Hold out one or more complete sessions instead.
"""
import argparse
import os
import shutil


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--dataset', default=os.path.join(here, 'dataset'))
    parser.add_argument('--out', default=os.path.join(here, 'dataset_split'))
    parser.add_argument('--val-sessions', nargs='+', required=True)
    args = parser.parse_args()

    if os.path.isdir(args.out):
        shutil.rmtree(args.out)
    counts = {}
    sessions = [s for s in sorted(os.listdir(args.dataset))
                if not s.startswith('_') and os.path.isdir(os.path.join(args.dataset, s))]
    missing = set(args.val_sessions) - set(sessions)
    if missing:
        raise SystemExit('No such session(s): %s. Found: %s' % (sorted(missing), sessions))
    for session in sessions:
        part = 'val' if session in args.val_sessions else 'train'
        for item in sorted(os.listdir(os.path.join(args.dataset, session))):
            source = os.path.join(args.dataset, session, item)
            if not os.path.isdir(source):
                continue
            target = os.path.join(args.out, part, item)
            os.makedirs(target, exist_ok=True)
            for name in os.listdir(source):
                shutil.copy2(os.path.join(source, name), os.path.join(target, name))
                counts[(part, item)] = counts.get((part, item), 0) + 1
    items = sorted({item for _, item in counts})
    print('%-20s %6s %6s' % ('item', 'train', 'val'))
    for item in items:
        train, val = counts.get(('train', item), 0), counts.get(('val', item), 0)
        note = '  <- needs 150+ training images' if train < 150 else ''
        note += '  <- no validation images' if val == 0 else ''
        print('%-20s %6d %6d%s' % (item, train, val, note))


if __name__ == '__main__':
    main()
