"""ROS 2 node: tray monitor and item classifier (PLAN.md section 13.10).

The camera is opened only while enabled (at the pickup point and after a dump), so it costs
nothing while the robot drives.

For work without a camera or model, set simulate:=true. The tray state then follows the
topic /tray/sim_occupied (std_msgs/Bool) and classify returns the parameter fake_item.
"""
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from nontok_interfaces.msg import TrayState
from nontok_interfaces.srv import ClassifyItem

from . import tray_logic
from .classifier import ClipZeroShot, OnnxClassifier, load_items, vote
from .tray_logic import TrayConfig, TrayLogic


class TrayNode(Node):
    def __init__(self):
        super().__init__('tray_node')
        share = get_package_share_directory('nontok_perception')
        p = self.declare_parameter
        self.device = p('device', '/dev/cam_tray').value
        self.width, self.height = p('width', 640).value, p('height', 480).value
        self.simulate = p('simulate', False).value
        self.fake_item = p('fake_item', '').value
        self.votes = p('votes', 5).value
        self.min_confidence = p('min_confidence', 0.80).value
        roi = p('roi', [0.10, 0.10, 0.90, 0.10, 0.90, 0.90, 0.10, 0.90]).value
        cfg = TrayConfig(
            roi=[(roi[i], roi[i + 1]) for i in range(0, len(roi), 2)],
            diff_threshold=p('diff_threshold', 25).value,
            occupied_on=p('occupied_on', 0.02).value,
            occupied_off=p('occupied_off', 0.01).value,
            motion_threshold=p('motion_threshold', 0.005).value,
            stable_s=p('stable_s', 2.0).value,
        )
        self.logic = TrayLogic(cfg)
        self.items = load_items(p('items_file', os.path.join(share, 'config', 'items.yaml')).value)
        model_path = p('model', os.path.join(share, 'models', 'items.onnx')).value
        clip_path = p('clip_model', os.path.join(share, 'models', 'clip_vision.onnx')).value
        prompts_path = p('clip_prompts', os.path.join(share, 'models', 'items_clip.npz')).value
        self.classifier = None
        if self.simulate or self.fake_item:
            pass   # no camera, so no model: the vision model alone takes 600 MB of memory
        elif os.path.exists(model_path):
            self.classifier = OnnxClassifier(model_path)
            self.get_logger().info('Item model: trained classifier, %d classes' % len(self.classifier.labels))
        elif os.path.exists(clip_path) and os.path.exists(prompts_path):
            # No trained model yet: recognise items from their descriptions instead.
            self.classifier = ClipZeroShot(clip_path, prompts_path, self.items)
            self.votes = min(self.votes, p('clip_votes', 3).value)   # it is slow on the Jetson Nano
            self.min_confidence = p('clip_min_confidence', 0.60).value
            self.get_logger().info('Item model: zero-shot (descriptions), %d labels' % len(self.classifier.labels))
        else:
            self.get_logger().warn('No model at %s or %s: classify will report "unknown"' % (model_path, clip_path))
        if self.classifier is not None:
            unknown = [label for label in self.classifier.labels
                       if label not in self.items and label != 'empty' and not label.startswith('_')]
            if unknown:
                self.get_logger().warn('Model classes with no station in items.yaml: %s' % unknown)

        self.camera = None
        self.lock = threading.Lock()
        # The detector whose camera cannot stream together with the tray camera ('' = none).
        self.pause_node = p('pause_detector', 'tag_detector_rear').value
        self.sim_occupied = False
        self.sim_since = time.monotonic()
        group = ReentrantCallbackGroup()
        self.pause_client = self.create_client(SetParameters, '/%s/set_parameters' % self.pause_node,
                                               callback_group=group) if self.pause_node else None
        self.state_pub = self.create_publisher(TrayState, 'tray/state', 5)
        self.create_service(SetBool, 'tray/enable', self.on_enable, callback_group=group)
        self.create_service(Trigger, 'tray/capture_reference', self.on_reference, callback_group=group)
        self.create_service(ClassifyItem, 'tray/classify', self.on_classify, callback_group=group)
        self.create_subscription(Bool, 'tray/sim_occupied', self.on_sim, 5, callback_group=group)
        # A small picture of the tray for the dashboard, only while someone is watching it.
        self.debug_pub = self.create_publisher(CompressedImage, 'tray/debug/compressed', 1)
        self.watched = self.last_debug = -1e9
        self.create_subscription(Bool, 'viz/watching', self.on_watching, 2, callback_group=group)
        self.create_timer(0.2, self.on_timer, callback_group=group)

    # ---- camera ----
    def frames(self, count: int, timeout_s: float = 3.0):
        """`count` fresh frames from the tray camera (fewer if it is slow or closed)."""
        out = []
        deadline = time.monotonic() + timeout_s
        while len(out) < count and time.monotonic() < deadline:
            with self.lock:
                item = self.camera.read() if self.camera is not None else None
            if item is None:
                time.sleep(0.02)
                continue
            out.append(item[0])
        return out

    def on_enable(self, request, response):
        if self.simulate:
            response.success = True
            return response
        if request.data:
            self.pause_other(True)
        with self.lock:
            if request.data and self.camera is None:
                from nontok_localization.camera import LatestFrameCamera
                camera = None
                for _ in range(6):      # the other camera may need a moment to let go of the bus
                    camera = LatestFrameCamera(self.device, self.width, self.height, 15)
                    if camera.is_open() and self.first_frame(camera):
                        break
                    camera.close()
                    camera = None
                    time.sleep(0.4)
                if camera is None:
                    response.success, response.message = False, 'cannot open %s' % self.device
                else:
                    self.camera = camera
                    response.success = True
            elif not request.data and self.camera is not None:
                self.camera.close()
                self.camera = None
                response.success = True
            else:
                response.success = True
        if not request.data or not response.success:
            self.pause_other(False)
        return response

    @staticmethod
    def first_frame(camera, timeout_s: float = 1.5) -> bool:
        """True once the camera has delivered a frame (it opens even when the bus is full)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if camera.read() is not None:
                return True
            time.sleep(0.05)
        return False

    def pause_other(self, paused: bool) -> None:
        """Ask the detector that shares the USB bus with the tray camera to let go, or carry on."""
        if self.pause_client is None or self.simulate:
            return
        if not self.pause_client.service_is_ready():
            return   # that detector is not running (bench work): nothing to pause
        request = SetParameters.Request()
        request.parameters = [Parameter('paused', Parameter.Type.BOOL, paused).to_parameter_msg()]
        done = threading.Event()
        self.pause_client.call_async(request).add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            self.get_logger().warn('No answer from %s when %s it' % (self.pause_node, 'pausing' if paused else 'resuming'))
        elif paused:
            time.sleep(0.3)   # until its camera has stopped streaming

    def on_reference(self, request, response):
        if self.simulate:
            response.success = True
            return response
        frames = self.frames(5)
        if len(frames) < 3:
            response.success, response.message = False, 'no frames from the tray camera (is it enabled?)'
            return response
        with self.lock:
            self.logic.set_reference(frames)
        response.success = True
        return response

    # ---- state ----
    def on_sim(self, msg):
        if msg.data != self.sim_occupied:
            self.sim_occupied, self.sim_since = msg.data, time.monotonic()

    def on_timer(self):
        msg = TrayState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.simulate:
            msg.state = TrayState.OCCUPIED if self.sim_occupied else TrayState.EMPTY
            msg.occupied_fraction = 0.1 if self.sim_occupied else 0.0
            msg.stable = time.monotonic() - self.sim_since >= self.logic.cfg.stable_s
            msg.has_reference = True
            self.state_pub.publish(msg)
            return
        with self.lock:
            item = self.camera.read() if self.camera is not None else None
            if item is None:
                return
            reading = self.logic.update(item[0], item[1])
        self.publish_picture(item[0], reading)
        msg.state = {tray_logic.EMPTY: TrayState.EMPTY, tray_logic.OCCUPIED: TrayState.OCCUPIED,
                     tray_logic.MOVING: TrayState.MOVING}[reading.state]
        msg.occupied_fraction = reading.occupied_fraction
        msg.stable = reading.stable
        msg.has_reference = reading.has_reference
        self.state_pub.publish(msg)

    def on_watching(self, msg):
        if msg.data:
            self.watched = time.monotonic()

    def publish_picture(self, frame, reading) -> None:
        now = time.monotonic()
        if now - self.watched > 3.0 or now - self.last_debug < 0.5:
            return
        self.last_debug = now
        import cv2
        view = cv2.resize(frame, (480, int(480 * frame.shape[0] / frame.shape[1])))
        name = {tray_logic.EMPTY: 'EMPTY', tray_logic.OCCUPIED: 'OCCUPIED', tray_logic.MOVING: 'MOVING'}[reading.state]
        cv2.putText(view, '%s %.0f%%' % (name, 100 * reading.occupied_fraction), (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        ok, encoded = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)

    # ---- classification ----
    def on_classify(self, request, response):
        if self.fake_item:
            response.success = self.fake_item in self.items
            response.item, response.station = self.fake_item, self.items.get(self.fake_item, '')
            response.confidence = 1.0
            return response
        if self.classifier is None:
            return response
        results = []
        for frame in self.frames(self.votes):
            with self.lock:
                crop = self.logic.item_crop(frame)
            if crop is not None:
                results.append(self.classifier.classify(crop))
        # Frames vote on the station, since that is the decision; look-alike items of one
        # station (bolt, nut) must not split the vote. The item reported is the commonest one.
        accepted, station, confidence = vote([(self.items.get(i, ''), c) for i, c in results], self.min_confidence)
        names = [i for i, _ in results if i and self.items.get(i, '') == station]
        item = max(set(names), key=names.count) if names else ''
        response.item, response.confidence = item, float(confidence)
        response.station = station
        response.success = accepted and bool(station)
        self.get_logger().info('Classified %s: %s (%.2f) -> %s' % (
            'ok' if response.success else 'UNKNOWN', item or '-', confidence, response.station or '-'))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrayNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
