"""Camera calibration with a printed checkerboard (PLAN.md section 13.6).

    ros2 run nontok_localization calibrate_camera --device /dev/cam_front_left \\
        --out src/nontok_localization/config/cameras/front_left.yaml
    python3 -m nontok_localization.calibrate_camera --images 'shots/*.png' --out cam.yaml

Board: 9 x 6 inner corners, 25 mm squares, glued flat on something rigid. With --device the
tool grabs a view every 2 s whenever the board is found; move the board to cover the whole
image, near and far, tilted each way, until 30 views are collected. Aim for an error below
0.5 px. No screen is needed: progress is printed.
"""
import argparse
import glob
import time

import cv2
import numpy as np
import yaml


def find_corners(gray, pattern):
    found, corners = cv2.findChessboardCorners(
        gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def calibrate(views, pattern, square_m, size):
    """views: list of detected corner arrays. Returns (rms error, camera matrix, distortion)."""
    grid = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m
    result = cv2.calibrateCamera([grid] * len(views), views, size, None, None)
    return result[0], result[1], result[2]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--device', help='camera device, e.g. /dev/cam_front_left')
    parser.add_argument('--images', help='glob of saved images to use instead of a camera')
    parser.add_argument('--out', required=True)
    parser.add_argument('--cols', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--square-mm', type=float, default=25.0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--views', type=int, default=30)
    args, _ = parser.parse_known_args(argv)
    pattern = (args.cols, args.rows)
    views, size = [], None

    if args.images:
        for path in sorted(glob.glob(args.images)):
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            corners = find_corners(gray, pattern) if gray is not None else None
            print('%s: %s' % (path, 'ok' if corners is not None else 'board not found'))
            if corners is not None:
                views.append(corners)
                size = gray.shape[::-1]
    elif args.device:
        capture = cv2.VideoCapture(args.device)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        last = 0.0
        while len(views) < args.views:
            ok, frame = capture.read()
            if not ok:
                raise SystemExit('Cannot read from %s' % args.device)
            if time.monotonic() - last < 2.0:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners = find_corners(gray, pattern)
            if corners is not None:
                views.append(corners)
                size = gray.shape[::-1]
                last = time.monotonic()
                print('view %d / %d: move the board' % (len(views), args.views))
        capture.release()
    else:
        raise SystemExit('Give --device or --images')

    if len(views) < 10:
        raise SystemExit('Only %d usable views; at least 10 are needed' % len(views))
    error, matrix, distortion = calibrate(views, pattern, args.square_mm / 1000.0, size)
    with open(args.out, 'w') as handle:
        yaml.safe_dump({
            'calibrated': True,
            'width': int(size[0]),
            'height': int(size[1]),
            'camera_matrix': [float(v) for v in matrix.ravel()],
            'distortion': [float(v) for v in distortion.ravel()],
            'reprojection_error_px': float(error),
        }, handle, sort_keys=False)
    print('Reprojection error %.3f px (%s). Saved %s' % (
        error, 'good' if error < 0.5 else 'above the 0.5 px target: collect better views', args.out))


if __name__ == '__main__':
    main()
