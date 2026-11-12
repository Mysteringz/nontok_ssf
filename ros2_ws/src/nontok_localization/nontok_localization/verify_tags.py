"""Checks tags.yaml against what the cameras see (PLAN.md section 13.7).

Put the robot at an exactly known pose (the dock is the map origin), run the localisation
launch, then:

    ros2 run nontok_localization verify_tags --x 0 --y 0 --yaw-deg 0

For every visible tag it prints where the robot would be according to that tag alone, and
how far that is from the pose you gave. A tag that disagrees by more than a few centimetres
is mis-measured in tags.yaml (or the camera mounting in robot.yaml is wrong, if all tags of
one camera disagree the same way).
"""
import argparse
import math
import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nontok_interfaces.msg import TagDetectionArray

from .camera import make_camera
from .geometry import wrap
from .pose_solver import SolverConfig, solve
from .ros_util import detections_of, matrix_of
from .tag_geometry import TagMap


class Verifier(Node):
    def __init__(self, truth, tags_file, cameras_dir, cameras):
        super().__init__('verify_tags')
        self.truth = truth
        self.tag_map = TagMap.load(tags_file)
        self.cameras_dir = cameras_dir
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.results = {}   # (camera, id) -> list of (dx, dy, dyaw, rms)
        # Loose gates: this tool is meant to show bad tags, not hide them.
        self.cfg = SolverConfig(max_range_m=6.0, max_view_angle_rad=math.radians(80), max_reprojection_px=50.0)
        for name in cameras:
            self.create_subscription(TagDetectionArray, 'tags/%s/detections' % name, self.on_detections, 10)

    def on_detections(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time(), Duration(seconds=0.0))
        except TransformException:
            return
        camera = make_camera(msg.camera, os.path.join(self.cameras_dir, msg.camera + '.yaml'),
                             matrix_of(tf.transform))
        for detection in detections_of(msg):
            if detection.id not in self.tag_map:
                self.results.setdefault((msg.camera, detection.id), None)
                continue
            single = TagMap([self.tag_map.get(detection.id)])
            fix = solve([detection], camera, single, self.cfg)
            if fix is None:
                continue
            self.results.setdefault((msg.camera, detection.id), []).append(
                (fix.x - self.truth[0], fix.y - self.truth[1], wrap(fix.yaw - self.truth[2]), fix.reprojection_px))


def main(argv=None):
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('nontok_localization')
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--tags-file', default=os.path.join(share, 'config', 'tags.yaml'))
    parser.add_argument('--cameras-dir', default=os.path.join(share, 'config', 'cameras'))
    parser.add_argument('--cameras', nargs='+', default=['front_left', 'front_right', 'rear'])
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Verifier((args.x, args.y, math.radians(args.yaw_deg)), args.tags_file, args.cameras_dir, args.cameras)
    end = node.get_clock().now() + Duration(seconds=args.seconds)
    while rclpy.ok() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.results:
        print('No tags seen. Are the tag detectors running, and is a tag in view?')
    print('%-12s %4s %8s %8s %8s %7s  %s' % ('camera', 'id', 'dx mm', 'dy mm', 'dyaw deg', 'rms px', 'verdict'))
    for (camera, tag_id), samples in sorted(node.results.items()):
        if samples is None:
            print('%-12s %4d %s' % (camera, tag_id, 'not in tags.yaml'))
            continue
        if not samples:
            continue
        n = len(samples)
        dx, dy, dyaw, rms = (sum(s[i] for s in samples) / n for i in range(4))
        bad = math.hypot(dx, dy) > 0.05 or abs(dyaw) > math.radians(4) or rms > 2.0
        print('%-12s %4d %8.0f %8.0f %8.1f %7.2f  %s' % (
            camera, tag_id, dx * 1000, dy * 1000, math.degrees(dyaw), rms, 'CHECK' if bad else 'ok'))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
