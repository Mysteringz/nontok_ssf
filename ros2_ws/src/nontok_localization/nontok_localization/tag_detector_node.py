"""ROS 2 node: detects AprilTags in one camera and publishes their corners (PLAN.md 13.6).

One process per camera. Images never travel through ROS; only the detections do.
"""
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage

from nontok_interfaces.msg import TagDetection, TagDetectionArray

from .camera import LatestFrameCamera
from .detector import TagDetector


class TagDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_detector')
        p = self.declare_parameter
        self.camera_name = p('camera_name', 'front_left').value
        device = p('device', '/dev/cam_front_left').value
        exposure = p('exposure', -1.0).value
        self.rate_hz = p('rate_hz', 10.0).value
        self.debug = p('debug', False).value
        self.frame_id = 'cam_%s_optical' % self.camera_name
        cv2.setNumThreads(2)
        self.detector = TagDetector(
            backend=p('backend', 'auto').value,
            quad_decimate=p('quad_decimate', 2.0).value,
            threads=p('threads', 2).value,
            min_margin=p('min_decision_margin', 50.0).value,
        )
        self.camera = LatestFrameCamera(device, p('width', 1280).value, p('height', 720).value,
                                        p('fps', 30).value, exposure if exposure >= 0 else None)
        if not self.camera.is_open():
            raise RuntimeError('Cannot open camera %s' % device)
        self.pub = self.create_publisher(TagDetectionArray, 'tags/%s/detections' % self.camera_name, 5)
        self.debug_pub = self.create_publisher(
            CompressedImage, 'tags/%s/debug/compressed' % self.camera_name, 1) if self.debug else None
        self._last_debug = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, name='detect', daemon=True)
        self._thread.start()
        self.get_logger().info('Tag detector %s on %s using the %s backend' % (
            self.camera_name, device, self.detector.backend))

    def _loop(self) -> None:
        next_time = time.monotonic()
        while self._running and rclpy.ok():
            rate = self.get_parameter('rate_hz').value   # can be changed while running; 0 pauses
            if rate <= 0:
                time.sleep(0.2)
                continue
            now = time.monotonic()
            if now < next_time:
                time.sleep(min(0.01, next_time - now))
                continue
            item = self.camera.read()
            if item is None:
                time.sleep(0.002)
                continue
            next_time = max(next_time + 1.0 / rate, now)
            frame, captured = item
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            detections = self.detector.detect(gray)

            msg = TagDetectionArray()
            age = time.monotonic() - captured
            msg.header.stamp = Time(nanoseconds=self.get_clock().now().nanoseconds - int(age * 1e9)).to_msg()
            msg.header.frame_id = self.frame_id
            msg.camera = self.camera_name
            for d in detections:
                out = TagDetection()
                out.id = int(d.id)
                out.hamming = int(d.hamming)
                out.decision_margin = float(d.decision_margin)
                out.corners = [float(v) for v in d.corners.reshape(8)]
                msg.detections.append(out)
            self.pub.publish(msg)   # also when empty: shows the camera is alive

            if self.debug_pub is not None and now - self._last_debug > 0.5:
                self._last_debug = now
                self._publish_debug(frame, detections, msg.header)

    def _publish_debug(self, frame, detections, header) -> None:
        view = frame.copy()
        for d in detections:
            points = d.corners.astype(int)
            cv2.polylines(view, [points.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
            cv2.putText(view, str(d.id), tuple(points[0]), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        scale = 640.0 / view.shape[1]
        view = cv2.resize(view, None, fx=scale, fy=scale)
        ok, encoded = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
