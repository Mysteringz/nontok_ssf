#!/usr/bin/env python3
"""Voice for the Mac demo (tools/demo.sh voice): this Mac's microphone and speakers stand in
for the robot's.

Say "Nontok" - or press Enter - while the robot is on its dock. The same wake-word model and
the same recorded phrases as on the robot are used; only the sound devices are the Mac's.
Needs: numpy onnxruntime openwakeword sounddevice (tools/demo.sh installs them in .local/demo-venv).
"""
import os
import queue
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VOICE = os.path.join(ROOT, 'ros2_ws', 'src', 'nontok_voice')
sys.path.insert(0, VOICE)

from nontok_voice.speech_logic import sound_for_display, sound_for_transition   # noqa: E402
from nontok_voice.wake_logic import WakeTrigger, load_model                     # noqa: E402

BLOCK, SAMPLE_RATE = 1280, 16000
THRESHOLD = float(os.environ.get('NONTOK_WAKE_THRESHOLD', '0.6'))
IN_CONTAINER = ('source /opt/ros/humble/setup.bash && source install/setup.bash && '
                'exec python3 -u /nontok/tools/demo_human.py')


class Demo:
    def __init__(self):
        self.state = None
        self.speaking = threading.Event()
        self.sounds = queue.Queue()
        self.robot = subprocess.Popen(['docker', 'exec', '-i', 'nontok-sim', 'bash', '-c', IN_CONTAINER],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def call(self, how: str) -> None:
        if self.state != 'S0_DOCKED':
            print('  (%s, but the robot is busy: it only listens on its dock)' % how)
            return
        print('  >>> "Nontok!" (%s)' % how)
        self.robot.stdin.write('wake\n')
        self.robot.stdin.flush()

    def follow_robot(self) -> None:
        shown = None
        for line in self.robot.stdout:
            kind, _, rest = line.strip().partition(' ')
            if kind == 'STATE':
                state, _, detail = rest.partition(' ')
                sound = sound_for_transition(self.state, state, detail)
                self.state = state
                print('%-18s %s' % (state, detail))
                if state == 'S0_DOCKED':
                    print('  say "Nontok" or press Enter')
            elif kind == 'SHOW' and rest != shown:
                shown, sound = rest, sound_for_display(rest)
            else:
                continue
            if sound:
                self.sounds.put(sound)
        print('The simulation has stopped.')
        os._exit(0)

    def speak(self) -> None:
        while True:
            sound = self.sounds.get()
            path = os.path.join(VOICE, 'sounds', sound + '.wav')
            if not os.path.exists(path) and sound.startswith('fault_'):
                path = os.path.join(VOICE, 'sounds', 'fault.wav')
            if not os.path.exists(path):
                continue
            print('  robot says: %s' % sound)
            self.speaking.set()
            subprocess.run(['afplay', path])
            time.sleep(0.3)
            self.speaking.clear()

    def listen(self) -> None:
        import numpy as np
        import sounddevice
        model = load_model(os.path.join(VOICE, 'models', 'nontok.onnx'))
        trigger = WakeTrigger(THRESHOLD, 3, 2, 3.0)
        audio = queue.Queue(maxsize=50)

        def on_audio(data, frames, time_info, status):
            try:
                audio.put_nowait(np.array(data[:, 0], dtype=np.int16))
            except queue.Full:
                pass

        heard_sound, started = False, time.monotonic()
        with sounddevice.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16', blocksize=BLOCK,
                                     callback=on_audio):
            while True:
                block = audio.get()
                if not heard_sound:
                    heard_sound = bool(np.abs(block).max() > 0)
                    if not heard_sound and time.monotonic() - started > 5.0:
                        print('  The microphone gives only silence: allow this terminal in System Settings > '
                              'Privacy & Security > Microphone. Enter still works.')
                        started = float('inf')
                score = float(max(model.predict(block).values()))
                if self.state != 'S0_DOCKED' or self.speaking.is_set():
                    trigger.reset()     # like the robot: deaf away from the dock and while talking
                    continue
                if trigger.update(score, time.monotonic()):
                    self.call('heard, score %.2f' % score)


def main() -> None:
    demo = Demo()
    threading.Thread(target=demo.follow_robot, daemon=True).start()
    threading.Thread(target=demo.speak, daemon=True).start()
    try:
        threading.Thread(target=demo.listen, daemon=True).start()
    except Exception as error:
        print('No microphone (%s): press Enter to call the robot.' % error)
    try:
        for _ in sys.stdin:
            demo.call('Enter')
    except KeyboardInterrupt:
        pass
    demo.robot.terminate()


if __name__ == '__main__':
    main()
