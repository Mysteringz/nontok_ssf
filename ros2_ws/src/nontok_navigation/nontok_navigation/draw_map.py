"""Draws the Nav2 map from a list of measured rectangles (PLAN.md section 13.8).

    ros2 run nontok_navigation draw_map --layout config/lab_layout.yaml --out config/map
    python3 -m nontok_navigation.draw_map --layout ... --out ...        (no ROS needed)

Writes <out>.pgm and <out>.yaml. The map is regenerated from the measurements every time,
never painted by hand, so a correction is a one-line edit of the layout file.
"""
import argparse
import math
import os
from typing import List, Sequence

import numpy as np
import yaml

FREE, OCCUPIED = 254, 0


def rasterise(bounds: Sequence[float], rectangles: List[Sequence[float]], resolution: float) -> np.ndarray:
    """Occupancy image, row 0 at the top (largest y). A cell touched by a rectangle is occupied."""
    x_min, y_min, x_max, y_max = bounds
    width = int(math.ceil((x_max - x_min) / resolution - 1e-9))
    height = int(math.ceil((y_max - y_min) / resolution - 1e-9))
    grid = np.full((height, width), FREE, dtype=np.uint8)
    for x1, y1, x2, y2 in rectangles:
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        col1 = max(0, int(math.floor((x1 - x_min) / resolution + 1e-9)))
        col2 = min(width, int(math.ceil((x2 - x_min) / resolution - 1e-9)))
        row1 = max(0, int(math.floor((y1 - y_min) / resolution + 1e-9)))
        row2 = min(height, int(math.ceil((y2 - y_min) / resolution - 1e-9)))
        if col2 > col1 and row2 > row1:
            grid[row1:row2, col1:col2] = OCCUPIED
    return grid[::-1]   # image rows run downward, map y runs upward


def cell_value(image: np.ndarray, bounds: Sequence[float], resolution: float, x: float, y: float) -> int:
    """Value of the cell containing map point (x, y)."""
    col = int((x - bounds[0]) / resolution)
    row = image.shape[0] - 1 - int((y - bounds[1]) / resolution)
    return int(image[row, col])


def load_layout(path: str):
    with open(path) as handle:
        layout = yaml.safe_load(handle)
    rectangles = []
    for group in ('walls', 'furniture', 'keep_out'):
        for item in layout.get(group) or []:
            rectangles.append(item['rect'] if isinstance(item, dict) else item)
    return layout['bounds'], rectangles, float(layout.get('resolution', 0.05))


def write_map(out_base: str, image: np.ndarray, bounds: Sequence[float], resolution: float) -> None:
    with open(out_base + '.pgm', 'wb') as handle:
        handle.write(b'P5\n%d %d\n255\n' % (image.shape[1], image.shape[0]))
        handle.write(image.tobytes())
    with open(out_base + '.yaml', 'w') as handle:
        yaml.safe_dump({
            'image': os.path.basename(out_base) + '.pgm',
            'mode': 'trinary',
            'resolution': resolution,
            'origin': [float(bounds[0]), float(bounds[1]), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, handle, sort_keys=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--layout', required=True)
    parser.add_argument('--out', required=True, help='output path without extension')
    args, _ = parser.parse_known_args(argv)
    bounds, rectangles, resolution = load_layout(args.layout)
    image = rasterise(bounds, rectangles, resolution)
    write_map(args.out, image, bounds, resolution)
    print('Wrote %s.pgm (%d x %d cells) and %s.yaml' % (args.out, image.shape[1], image.shape[0], args.out))


if __name__ == '__main__':
    main()
