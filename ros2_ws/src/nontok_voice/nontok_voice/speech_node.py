"""ROS 2 node: the robot's voice. Plays a recorded phrase when the mission reaches certain
moments: it answers the call, asks for the item, says what it is carrying and where to, and
says when it is back or needs help. The phrases are WAV files made by training/make_speech.py.
"""
import os
import queue
import subprocess
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from nontok_interfaces.msg import MissionState

from .speech_logic import sound_for_display, sound_for_transition


def usb_speaker() -> str:
    """ALSA name of the first USB playback device, or '' if there is none."""
    try:
        listing = subprocess.run(['aplay', '-l'], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    for line in listing.splitlines():
        if line.startswith('card ') and 'USB' in line:
            return 'plughw:%s,0' % line.split(':')[0].split()[1]
    return ''


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        share = get_package_share_directory('nontok_voice')
        p = self.declare_parameter
        self.sounds = p('sounds_dir', os.path.join(share, 'sounds')).value
        self.device = p('device', '').value or usb_speaker()
        self.enabled = p('enabled', True).value
        self.previous = None
        self.last_display = ''
        self.queue = queue.Queue(maxsize=4)
        self.create_subscription(MissionState, 'mission/state', self.on_state, 10)
        self.create_subscription(String, 'base/oled', self.on_display, 10)
        self.create_subscription(String, 'say', lambda msg: self.say(msg.data), 5)   # for trying it by hand
        threading.Thread(target=self._play_loop, name='speech', daemon=True).start()
        self.get_logger().info('Speech on %s' % (self.device or 'the default sound device'))

    def on_state(self, msg: MissionState) -> None:
        sound = sound_for_transition(self.previous, msg.state, msg.detail)
        self.previous = msg.state
        if sound:
            self.say(sound)

    def on_display(self, msg: String) -> None:
        if msg.data != self.last_display:
            self.last_display = msg.data
            sound = sound_for_display(msg.data)
            if sound:
                self.say(sound)

    def say(self, sound: str) -> None:
        if not self.enabled:
            return
        path = os.path.join(self.sounds, sound + '.wav')
        if not os.path.exists(path) and sound.startswith('fault_'):
            path = os.path.join(self.sounds, 'fault.wav')
        if not os.path.exists(path):
            self.get_logger().warn('No recording for "%s"' % sound)
            return
        self.get_logger().info('Saying "%s"' % os.path.splitext(os.path.basename(path))[0])
        try:
            self.queue.put_nowait(path)
        except queue.Full:
            pass   # far behind: better silent than talking about the past

    def _play_loop(self) -> None:
        while rclpy.ok():
            try:
                path = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            command = ['aplay', '-q'] + (['-D', self.device] if self.device else []) + [path]
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=20)
                if result.returncode != 0:
                    self.get_logger().warn('Could not play %s: %s' % (os.path.basename(path), result.stderr.strip()[:120]),
                                           throttle_duration_sec=30.0)
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().warn('Could not play sound: %s' % error, throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
