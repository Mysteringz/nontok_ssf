"""Builds the robot's URDF from config/robot.yaml. Pure Python.

Only fixed frames are described (cameras, ultrasonics, gyro), which is all the other nodes
need. Frame names: imu_link, cam_<name> and cam_<name>_optical, range_<name>.
"""
import math

import yaml

# Body-style camera frame (x forward) -> optical frame (z forward, x right, y down).
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def _joint(parent: str, child: str, xyz, rpy) -> str:
    return (
        '  <link name="%s"/>\n'
        '  <joint name="%s_joint" type="fixed">\n'
        '    <parent link="%s"/>\n'
        '    <child link="%s"/>\n'
        '    <origin xyz="%.4f %.4f %.4f" rpy="%.6f %.6f %.6f"/>\n'
        '  </joint>\n' % (child, child, parent, child, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))


def build_urdf(config: dict) -> str:
    parts = ['<?xml version="1.0"?>\n<robot name="nontok">\n  <link name="base_link"/>\n']
    parts.append(_joint('base_link', 'imu_link', config.get('imu', {}).get('xyz', [0, 0, 0]), (0, 0, 0)))
    for name, cam in config.get('cameras', {}).items():
        rpy = (0.0, math.radians(cam.get('pitch_deg', 0.0)), math.radians(cam.get('yaw_deg', 0.0)))
        parts.append(_joint('base_link', 'cam_' + name, cam['xyz'], rpy))
        parts.append(_joint('cam_' + name, 'cam_%s_optical' % name, (0, 0, 0), OPTICAL_RPY))
    for name, sensor in config.get('ranges', {}).items():
        parts.append(_joint('base_link', 'range_' + name, sensor['xyz'],
                            (0.0, 0.0, math.radians(sensor.get('yaw_deg', 0.0)))))
    parts.append('</robot>\n')
    return ''.join(parts)


def load_config(path: str) -> dict:
    with open(path) as handle:
        return yaml.safe_load(handle)


def footprint_string(config: dict) -> str:
    """The footprint in the text form Nav2 expects."""
    return '[' + ', '.join('[%.3f, %.3f]' % (p[0], p[1]) for p in config['footprint']) + ']'
