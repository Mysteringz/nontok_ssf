"""Makes printable A4 pages of AprilTag 36h11 tags (PLAN.md section 5.2).

    ros2 run nontok_localization make_tags --ids 0 1 2 3 10-17 30-35 --out tags.pdf
    python3 -m nontok_localization.make_tags --ids 0-3 --out tags.pdf      (no ROS needed)

Print at 100 % scale ("actual size", no "fit to page"), then MEASURE the 100 mm bar on the
paper. If it is not 100 mm, the tags are not 160 mm either and every distance will be wrong.
Use matte paper and glue each sheet flat onto foam board.
"""
import argparse
from typing import List

import cv2
import numpy as np
from PIL import Image

from .detector import tag_image

DPI = 300
A4_MM = (210.0, 297.0)


def mm(value: float) -> int:
    return int(round(value / 25.4 * DPI))


def parse_ids(items: List[str]) -> List[int]:
    ids = []
    for item in items:
        if '-' in item:
            first, last = item.split('-')
            ids.extend(range(int(first), int(last) + 1))
        else:
            ids.append(int(item))
    return ids


def page(tag_id: int, size_mm: float, label: str = '') -> np.ndarray:
    width, height = mm(A4_MM[0]), mm(A4_MM[1])
    sheet = np.full((height, width), 255, dtype=np.uint8)
    side = mm(size_mm)
    cells = 8                           # 6 x 6 data cells plus the one-cell black border
    side -= side % cells                # whole pixels per cell keeps the edges sharp
    left, top = (width - side) // 2, mm(45.0)
    sheet[top:top + side, left:left + side] = cv2.resize(
        tag_image(tag_id, cells * 10), (side, side), interpolation=cv2.INTER_NEAREST)

    text = 'ID %d   %s' % (tag_id, label)
    cv2.putText(sheet, text, (left, top + side + mm(22.0)), cv2.FONT_HERSHEY_SIMPLEX, 3.0, 0, 8)
    cv2.putText(sheet, 'TOP', (left, top - mm(8.0)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 120, 4)
    # Scale check bar.
    bar_left, bar_y = left, top + side + mm(40.0)
    cv2.line(sheet, (bar_left, bar_y), (bar_left + mm(100.0), bar_y), 0, 6)
    for x in (bar_left, bar_left + mm(100.0)):
        cv2.line(sheet, (x, bar_y - mm(4.0)), (x, bar_y + mm(4.0)), 0, 6)
    cv2.putText(sheet, 'This bar must measure 100 mm. Tag side: %.0f mm.' % (side / DPI * 25.4),
                (bar_left, bar_y + mm(12.0)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    return sheet


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--ids', nargs='+', required=True, help='tag ids, e.g. 0 1 10-17')
    parser.add_argument('--size-mm', type=float, default=160.0)
    parser.add_argument('--out', default='tags.pdf')
    parser.add_argument('--labels', nargs='*', default=[], help='optional id=text, e.g. 0=dock-left')
    args, _ = parser.parse_known_args(argv)
    labels = dict(item.split('=', 1) for item in args.labels)
    pages = [Image.fromarray(page(i, args.size_mm, labels.get(str(i), ''))) for i in parse_ids(args.ids)]
    pages[0].save(args.out, save_all=True, append_images=pages[1:], resolution=DPI)
    print('Wrote %d page(s) to %s' % (len(pages), args.out))


if __name__ == '__main__':
    main()
