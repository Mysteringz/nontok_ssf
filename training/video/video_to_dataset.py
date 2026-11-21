#!/usr/bin/env python3
"""Turns tray-camera videos into classifier training images.

    python video_to_dataset.py videos/s1/*.mkv            -> dataset/s1/<item>/*.jpg
    python video_to_dataset.py videos/s1/wire.mkv --show  (also writes a review sheet)

Video name = item (wire.mkv, wire_2.mkv -> "wire"); folder = session.
Output layout is the one training/capture_tray_dataset.py on the robot produces:
dataset/<session>/<item>/*.jpg (the crops) and dataset/_full/<session>/<item>/*.jpg.

The crop follows PLAN.md 13.10 (the robot's tray_logic):
  grey, 5x5 blur, |frame - empty reference| > 25, bounding box of the changed pixels,
  grown 20 %, padded to a square, at least 96 px, resized to 224x224.
Checked 2026-11-21 against nontok_perception/tray_logic.py (TrayLogic.item_crop and
_changed_mask): same 1.4x square, same brightness-offset removal.

Frames are kept only when the item lies still (no hand, no camera motion), and
near-duplicates of the last kept image are dropped.
"""
import argparse
import os
import re

import cv2
import numpy as np

DIFF = 25            # grey levels: changed vs the empty reference
GROW = 0.20          # margin on EACH side: square side = 1.4 x the box (tray_logic crop_margin)
MIN_SIDE = 96
SIZE = 224


def grey(frame):
    return cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)


def changed_mask(g, ref_g, roi):
    # like tray_logic: remove an overall brightness change (auto-exposure) before differencing
    inside = roi > 0
    offset = float(np.median(g[inside])) - float(np.median(ref_g[inside]))
    diff = np.abs(g.astype(np.float32) - offset - ref_g.astype(np.float32))
    mask = ((diff > DIFF) & inside).astype(np.uint8)
    # drop single-pixel noise; keep thin items (wire legs) by using a small kernel
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Recording aid, not in the robot's tray_logic: a camera that shifts a little after
    # the reference was taken shows up as changed edges touching the image border.
    # Items lie inside the frame, so drop regions that touch the border of the ROI.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    ys, xs = np.nonzero(roi)
    x_lo, x_hi, y_lo, y_hi = xs.min() + 2, xs.max() - 2, ys.min() + 2, ys.max() - 2
    keep = np.zeros(count, np.uint8)
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        touches = x <= x_lo or y <= y_lo or x + w - 1 >= x_hi or y + h - 1 >= y_hi
        keep[i] = 0 if touches or area < 12 else 1
    return keep[labels]


class Still(object):
    """'Nothing is moving' test that tolerates camera flicker: over the last `window`
    frames the median moving-pixel fraction is below `quiet` and no frame exceeds
    `burst` (a hand or the item moving)."""

    def __init__(self, window, quiet=0.003, burst=0.03):
        self.window, self.quiet, self.burst = max(3, int(window)), quiet, burst
        self.values = []

    def update(self, motion):
        self.values = (self.values + [motion])[-self.window:]
        return (len(self.values) == self.window and float(np.median(self.values)) < self.quiet
                and max(self.values) < self.burst)


class Background(object):
    """Empty-surface reference for a recorded video: per-pixel median of the frames in a
    window of +-`window_s` seconds (sampled at 2 fps). An item that rests in one spot for
    less than half the window, or a hand passing through, does not survive the median,
    and a camera that shifts gets a fresh reference within a few seconds.
    (The robot instead captures an explicit empty-tray reference; see PLAN.md 7.3.)"""

    def __init__(self, path, window_s=10.0, sample_fps=2.0):
        cap = cv2.VideoCapture(path)
        self.fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        step = max(1, int(round(self.fps / sample_fps)))
        self.times, self.frames = [], []
        n = -1
        while True:
            ok = cap.grab()
            if not ok:
                break
            n += 1
            if n % step:
                continue
            ok, frame = cap.retrieve()
            if ok and frame.mean() >= 12:            # skip black warm-up frames
                self.times.append(n / self.fps)
                self.frames.append(grey(frame))
        cap.release()
        self.times = np.array(self.times)
        self.window_s = window_s
        self._cache = {}

    def at(self, t):
        key = int(t)                                 # recompute once per second
        if key not in self._cache:
            idx = np.nonzero(np.abs(self.times - (key + 0.5)) <= self.window_s)[0]
            if len(idx) == 0:
                idx = np.array([int(np.argmin(np.abs(self.times - t)))])
            self._cache = {key: np.median(np.stack([self.frames[i] for i in idx]), axis=0).astype(np.uint8)}
        return self._cache[key]


def item_crop(frame, mask):
    """Square crop around the changed pixels, as tray_logic.TrayLogic.item_crop."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None, None
    x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
    w, h = x1 - x0, y1 - y0
    side = max(int(max(w, h) * (1 + 2 * GROW)), MIN_SIDE)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    left, top = int(round(cx - side / 2.0)), int(round(cy - side / 2.0))
    H, W = frame.shape[:2]
    padded = cv2.copyMakeBorder(frame, side, side, side, side, cv2.BORDER_REPLICATE)
    crop = padded[top + side:top + 2 * side, left + side:left + 2 * side]
    return cv2.resize(crop, (SIZE, SIZE), interpolation=cv2.INTER_AREA), (x0, y0, x1, y1)


def process(path, out_root, args):
    session = os.path.basename(os.path.dirname(os.path.abspath(path)))
    item = re.sub(r'_\d+$', '', os.path.splitext(os.path.basename(path))[0])
    tag = os.path.splitext(os.path.basename(path))[0]
    crops_dir = os.path.join(out_root, session, item)
    full_dir = os.path.join(out_root, '_full', session, item)
    os.makedirs(crops_dir, exist_ok=True)
    os.makedirs(full_dir, exist_ok=True)

    background = Background(path)
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    prev_g = None
    still = Still(args.still_s * fps, args.motion)
    last_kept = None
    last_kept_t = -1e9
    stats = dict(frames=0, black=0, moving=0, empty=0, too_big=0, dup=0, kept=0)
    review = []
    roi = None
    n = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        t = n / fps
        stats['frames'] += 1
        if frame.mean() < 12:                      # camera warm-up: black frames
            stats['black'] += 1
            prev_g = None
            continue
        g = grey(frame)
        if roi is None:
            H, W = g.shape
            roi = np.zeros((H, W), np.uint8)
            b = int(0.03 * min(H, W))
            roi[b:H - b, b:W - b] = 1
        motion = 1.0 if prev_g is None else float(((cv2.absdiff(g, prev_g) > DIFF) & (roi > 0)).mean())
        prev_g = g
        if not still.update(motion):
            stats['moving'] += 1
            continue
        mask = changed_mask(g, background.at(t), roi)
        frac = float(mask.mean())
        if frac < args.min_frac:
            stats['empty'] += 1
            continue
        if frac > args.max_frac:                   # hand at rest, camera moved, or lighting jump
            stats['too_big'] += 1
            continue
        crop, box = item_crop(frame, mask)
        small = cv2.resize(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (32, 32)).astype(np.float32)
        if last_kept is not None and (np.abs(small - last_kept).mean() < args.dup or t - last_kept_t < args.gap_s):
            stats['dup'] += 1
            continue
        last_kept, last_kept_t = small, t
        name = '%s_%s_%05d' % (session, tag, n)
        cv2.imwrite(os.path.join(crops_dir, name + '.jpg'), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
        cv2.imwrite(os.path.join(full_dir, name + '.jpg'), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        stats['kept'] += 1
        if args.show:
            review.append(crop)
    cap.release()
    print('%-28s item=%-10s %s' % (path, item, ' '.join('%s=%d' % kv for kv in stats.items())))
    if args.show and review:
        cols = 10
        tiles = [cv2.resize(c, (112, 112)) for c in review]
        while len(tiles) % cols:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
        cv2.imwrite(os.path.join(out_root, 'review_%s_%s.jpg' % (session, tag)), sheet)
    return stats['kept']


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('videos', nargs='+')
    ap.add_argument('--out', default=os.path.join(here, 'dataset'))
    ap.add_argument('--still-s', type=float, default=0.4, help='item must be still this long')
    ap.add_argument('--motion', type=float, default=0.003, help='max median moving-pixel fraction over the still window')
    ap.add_argument('--min-frac', type=float, default=0.0008, help='min changed fraction = an item is there')
    ap.add_argument('--max-frac', type=float, default=0.25, help='more changed than this = hand/camera moved')
    ap.add_argument('--dup', type=float, default=3.0, help='mean grey difference below which a crop is a duplicate')
    ap.add_argument('--gap-s', type=float, default=0.25, help='min time between kept frames')
    ap.add_argument('--show', action='store_true', help='write review_<session>_<video>.jpg sheets')
    args = ap.parse_args()
    total = sum(process(v, args.out, args) for v in args.videos)
    print('kept %d images in %s' % (total, args.out))


if __name__ == '__main__':
    main()
