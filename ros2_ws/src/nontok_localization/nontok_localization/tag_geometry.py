"""Where the tags are (PLAN.md section 5.2). NumPy only.

A tag is described by the position of its centre and the direction it faces:
    id: {x, y, z, yaw, size}
x, y, z in metres in the map frame; yaw = map direction of the tag's outward normal (the
direction a person looking at the tag is looking *from*); size = side of the black square.
Tags are assumed to hang vertically.

Corner order everywhere: top-left, top-right, bottom-right, bottom-left, as seen by someone
looking at the upright tag.
"""
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import numpy as np
import yaml

from .geometry import transform


def local_corners(size: float) -> np.ndarray:
    """Corners in the tag's own frame: x right, y up, z out of the tag toward the viewer."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass(frozen=True)
class Tag:
    id: int
    x: float
    y: float
    z: float
    yaw: float
    size: float = 0.160

    def matrix(self) -> np.ndarray:
        """Tag frame -> map."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rotation = np.array([[-s, 0.0, c], [c, 0.0, s], [0.0, 1.0, 0.0]])  # columns: x, y, z axes
        return transform(rotation, (self.x, self.y, self.z))

    def corners_map(self) -> np.ndarray:
        m = self.matrix()
        return local_corners(self.size) @ m[:3, :3].T + m[:3, 3]

    def normal(self) -> np.ndarray:
        return np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])

    def centre(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class TagMap:
    def __init__(self, tags: Iterable[Tag]):
        self.tags: Dict[int, Tag] = {tag.id: tag for tag in tags}

    def get(self, tag_id: int) -> Optional[Tag]:
        return self.tags.get(tag_id)

    def __contains__(self, tag_id: int) -> bool:
        return tag_id in self.tags

    def __len__(self) -> int:
        return len(self.tags)

    @classmethod
    def from_dict(cls, data: dict, default_size: float = 0.160) -> 'TagMap':
        tags = []
        for tag_id, entry in (data or {}).items():
            tags.append(Tag(int(tag_id), float(entry['x']), float(entry['y']), float(entry['z']),
                            math.radians(float(entry['yaw_deg'])),
                            float(entry.get('size', default_size))))
        return cls(tags)

    @classmethod
    def load(cls, path: str) -> 'TagMap':
        with open(path) as handle:
            data = yaml.safe_load(handle) or {}
        return cls.from_dict(data.get('tags', {}), float(data.get('default_size', 0.160)))


def pair_map(left_id: int, right_id: int, spacing: float = 0.300, size: float = 0.160,
             z: float = 0.0) -> TagMap:
    """A station's two tags in the station's own frame, used for docking.

    Station frame: origin midway between the tags, x pointing out of the wall toward the
    robot, z up; +y is then to the right of someone facing the tags.
    `left_id` is the tag on that person's left.
    """
    half = spacing / 2.0
    return TagMap([Tag(left_id, 0.0, -half, z, 0.0, size), Tag(right_id, 0.0, half, z, 0.0, size)])
