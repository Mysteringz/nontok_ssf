"""The URDF must describe the same camera mounting that the localisation maths assumes."""
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

from nontok_description.urdf_builder import build_urdf, footprint_string, load_config
from nontok_localization.geometry import camera_mount, transform

CONFIG = load_config(str(pathlib.Path(__file__).parents[1] / 'config' / 'robot.yaml'))


def rpy_matrix(roll, pitch, yaw):
    """URDF convention: fixed-axis roll, then pitch, then yaw."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def chain(root, frame):
    """base_link -> frame as a 4x4 matrix, following the URDF joints."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    matrix = np.eye(4)
    while frame != 'base_link':
        joint = joints[frame]
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz').split()]
        rpy = [float(v) for v in origin.get('rpy').split()]
        matrix = transform(rpy_matrix(*rpy), xyz) @ matrix
        frame = joint.find('parent').get('link')
    return matrix


def test_urdf_has_all_frames():
    root = ET.fromstring(build_urdf(CONFIG))
    links = {link.get('name') for link in root.findall('link')}
    expected = {'base_link', 'imu_link'}
    expected |= {'cam_%s_optical' % name for name in CONFIG['cameras']}
    expected |= {'range_' + name for name in ('left45', 'right45', 'front', 'left', 'right')}
    assert expected <= links


def test_camera_frames_match_localisation_maths():
    root = ET.fromstring(build_urdf(CONFIG))
    for name, cam in CONFIG['cameras'].items():
        expected = camera_mount(*cam['xyz'], math.radians(cam['yaw_deg']), math.radians(cam['pitch_deg']),
                                math.radians(cam.get('roll_deg', 0.0)))
        assert np.allclose(chain(root, 'cam_%s_optical' % name), expected, atol=1e-5), name


def test_camera_directions():
    root = ET.fromstring(build_urdf(CONFIG))
    for name in ('front_left', 'front_right'):   # measured mounts (24 Nov): a few degrees off straight ahead
        forward = chain(root, 'cam_%s_optical' % name)[:3, 2]   # optical z = viewing direction
        assert forward[0] > math.cos(math.radians(10)), name
    assert np.allclose(chain(root, 'cam_rear_optical')[:3, 2], [-1, 0, 0], atol=1e-5)
    assert np.allclose(chain(root, 'cam_tray_optical')[:3, 2], [0, 0, -1], atol=1e-5)   # looks down
    assert np.allclose(chain(root, 'range_left45')[:3, 0], [math.sqrt(0.5), math.sqrt(0.5), 0], atol=1e-5)


def test_footprint_string():
    assert footprint_string(CONFIG).startswith('[[0.172, 0.123], ')   # measured 345 x 247 mm (24 Nov)
