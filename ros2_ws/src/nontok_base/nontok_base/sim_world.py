"""A simulated room for the fake Mega: walls and tables (rectangles) and walking people
(circles), seen through the five ultrasonic sensors. Pure Python.

The simulated ultrasonic is simple: a few rays across the beam, the nearest hit wins, no
echoes lost on slanted or soft surfaces. Real sensors are worse than this.
"""
import json
import math
import os
from typing import List, Optional, Sequence, Tuple

Rect = Tuple[float, float, float, float]      # x1, y1, x2, y2 with x1 < x2 and y1 < y2


def ray_rect(ox: float, oy: float, dx: float, dy: float, rect: Rect) -> Optional[float]:
    """Distance along the ray (origin, unit direction) to the rectangle; None = missed."""
    t_near, t_far = 0.0, math.inf
    for o, d, low, high in ((ox, dx, rect[0], rect[2]), (oy, dy, rect[1], rect[3])):
        if abs(d) < 1e-12:
            if o < low or o > high:
                return None
            continue
        t1, t2 = (low - o) / d, (high - o) / d
        t_near, t_far = max(t_near, min(t1, t2)), min(t_far, max(t1, t2))
        if t_near > t_far:
            return None
    return t_near


def ray_circle(ox: float, oy: float, dx: float, dy: float, cx: float, cy: float, r: float) -> Optional[float]:
    fx, fy = cx - ox, cy - oy
    along = fx * dx + fy * dy
    off2 = fx * fx + fy * fy - along * along
    if off2 > r * r:
        return None
    t = along - math.sqrt(r * r - off2)
    if t < 0.0:
        return 0.0 if fx * fx + fy * fy <= r * r else None
    return t


def point_rect_distance(x: float, y: float, rect: Rect) -> float:
    return math.hypot(max(rect[0] - x, 0.0, x - rect[2]), max(rect[1] - y, 0.0, y - rect[3]))


class Person:
    """Walks back and forth along a list of points. Does not walk into the robot: stops short
    of it, and turns back if the robot is still in the way after `patience_s`."""

    def __init__(self, name: str, path: Sequence[Sequence[float]], speed: float = 0.5, radius: float = 0.20,
                 patience_s: float = 3.0, margin_m: float = 0.25):
        self.name, self.path, self.speed, self.radius = name, [tuple(p) for p in path], speed, radius
        self.patience_s, self.margin = patience_s, margin_m
        self.x, self.y = self.path[0]
        self.target, self.direction, self.waited = 1, 1, 0.0

    def _turn_back(self) -> None:
        self.direction = -self.direction
        self.target = max(0, min(len(self.path) - 1, self.target + self.direction))

    def step(self, dt: float, robot_x: float, robot_y: float, robot_radius: float) -> None:
        tx, ty = self.path[self.target]
        distance = math.hypot(tx - self.x, ty - self.y)
        if distance <= self.speed * dt:
            self.x, self.y = tx, ty
            if not 0 <= self.target + self.direction < len(self.path):
                self.direction = -self.direction
            self.target += self.direction
            return
        nx = self.x + (tx - self.x) / distance * self.speed * dt
        ny = self.y + (ty - self.y) / distance * self.speed * dt
        keep = self.radius + robot_radius + self.margin
        now, then = math.hypot(self.x - robot_x, self.y - robot_y), math.hypot(nx - robot_x, ny - robot_y)
        if then < keep and then < now:
            self.waited += dt
            if self.waited > self.patience_s:
                self.waited = 0.0
                self._turn_back()
            return
        self.waited = 0.0
        self.x, self.y = nx, ny


class World:
    def __init__(self, walls: List[Rect], tables: List[Tuple[str, Rect]], people: List[Person],
                 sensors: List[Tuple[float, float, float]], fov_rad: float = 0.26, robot_radius: float = 0.16,
                 reach_m: float = 4.0):
        """sensors: (x, y, yaw) of each ultrasonic on the robot, in the order of the RNG line."""
        self.walls, self.tables, self.people, self.sensors = walls, tables, people, sensors
        self.fov, self.robot_radius, self.reach = fov_rad, robot_radius, reach_m
        self.contacts = 0
        self.touching: Optional[str] = None

    def step(self, dt: float, x: float, y: float) -> None:
        for person in self.people:
            person.step(dt, x, y, self.robot_radius)
        touching = None
        for name, rect in self.tables:
            if point_rect_distance(x, y, rect) < self.robot_radius:
                touching = name
        for person in self.people:
            if math.hypot(person.x - x, person.y - y) < self.robot_radius + person.radius:
                touching = person.name
        if touching and touching != self.touching:
            self.contacts += 1
        self.touching = touching

    def ranges_mm(self, x: float, y: float, yaw: float) -> List[int]:
        out = []
        rects = self.walls + [rect for _, rect in self.tables]
        for sx, sy, syaw in self.sensors:
            ox = x + sx * math.cos(yaw) - sy * math.sin(yaw)
            oy = y + sx * math.sin(yaw) + sy * math.cos(yaw)
            best = math.inf
            for k in range(-2, 3):
                angle = yaw + syaw + k * self.fov / 4.0
                dx, dy = math.cos(angle), math.sin(angle)
                for rect in rects:
                    hit = ray_rect(ox, oy, dx, dy, rect)
                    if hit is not None:
                        best = min(best, hit)
                for person in self.people:
                    hit = ray_circle(ox, oy, dx, dy, person.x, person.y, person.radius)
                    if hit is not None:
                        best = min(best, hit)
            out.append(int(round(max(best, 0.03) * 1000)) if best <= self.reach else -1)
        return out

    def snapshot(self, now: float) -> dict:
        return {'t': now, 'contacts': self.contacts, 'touching': self.touching,
                'people': [[round(p.x, 3), round(p.y, 3), p.radius] for p in self.people]}


def _rect(values: Sequence[float]) -> Rect:
    x1, x2 = sorted((float(values[0]), float(values[2])))
    y1, y2 = sorted((float(values[1]), float(values[3])))
    return (x1, y1, x2, y2)


def load_world(scene_path: str, layout_path: str, robot_path: str) -> World:
    import yaml
    with open(scene_path) as handle:
        scene = yaml.safe_load(handle) or {}
    with open(layout_path) as handle:
        layout = yaml.safe_load(handle) or {}
    with open(robot_path) as handle:
        robot = yaml.safe_load(handle) or {}
    from .protocol import RANGE_NAMES
    walls = [_rect(item['rect']) for group in ('walls', 'furniture') for item in layout.get(group) or []]
    tables = [(item.get('name', 'table'), _rect(item['rect'])) for item in scene.get('tables') or []]
    people = [Person(item.get('name', 'person'), item['path'], float(item.get('speed', 0.5)),
                     float(item.get('radius', 0.20))) for item in scene.get('people') or []]
    sensors = [(robot['ranges'][name]['xyz'][0], robot['ranges'][name]['xyz'][1],
                math.radians(robot['ranges'][name]['yaw_deg'])) for name in RANGE_NAMES]
    return World(walls, tables, people, sensors)


def write_snapshot(world: World, path: str, now: float) -> None:
    """For the dashboard. Written whole, then moved into place, so a reader never sees half."""
    with open(path + '.tmp', 'w') as handle:
        json.dump(world.snapshot(now), handle)
    os.replace(path + '.tmp', path)
