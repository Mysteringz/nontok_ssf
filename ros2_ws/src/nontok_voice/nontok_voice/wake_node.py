"""ROS 2 node: listens for "Nontok" and publishes /wake (PLAN.md sections 7.2 and 13.11).

Listening is switched on by the mission only while the robot is docked.
"""
import os
import queue
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import SetBool

from .wake_logic import WakeTrigger

BLOCK = 1280      # 80 ms at 16 kHz: the block size openWakeWord works in
SAMPLE_RATE = 16000


class WakeNode(Node):
    def __init__(self):
        super().__init__('wake_node')
        share = get_package_share_directory('nontok_voice')
        p = self.declare_parameter
        model_path = p('model', os.path.join(share, 'models', 'nontok.onnx')).value
        device = p('device', '').value
        self.trigger = WakeTrigger(p('threshold', 0.5).value, p('window', 3).value, p('needed', 2).value,
                                   p('refractory_s', 3.0).value)
        self.enabled = p('start_enabled', False).value
        self.pub = self.create_publisher(Empty, 'wake', 5)
        self.create_service(SetBool, 'voice/enable', self.on_enable)
        self.audio = queue.Queue(maxsize=50)
        self.model = None
        self.stream = None
        if not os.path.exists(model_path):
            self.get_logger().warn('No wake-word model at %s: voice start is unavailable. '
                                   'Use "ros2 run nontok_voice trigger" or Bluetooth W instead.' % model_path)
            return
        import sounddevice
        from openwakeword.model import Model
        self.model = Model(wakeword_models=[model_path], inference_framework='onnx')
        self.stream = sounddevice.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype='int16', blocksize=BLOCK,
            device=device or None, callback=self.on_audio)
        self.stream.start()
        self.create_timer(0.02, self.on_timer)
        self.get_logger().info('Wake word ready (%s)' % os.path.basename(model_path))

    def on_audio(self, data, frames, time_info, status):
        try:
            self.audio.put_nowait(np.array(data[:, 0], dtype=np.int16))
        except queue.Full:
            pass   # the node fell behind; dropping audio is harmless

    def on_enable(self, request, response):
        self.enabled = request.data
        self.trigger.reset()
        response.success = True
        return response

    def on_timer(self):
        while True:
            try:
                block = self.audio.get_nowait()
            except queue.Empty:
                return
            scores = self.model.predict(block)     # run always, so the model's audio history stays current
            if self.enabled and self.trigger.update(float(max(scores.values())), time.monotonic()):
                self.get_logger().info('Wake word heard')
                self.pub.publish(Empty())

    def destroy_node(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
