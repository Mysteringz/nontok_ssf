"""First power-on check of the motors and encoders (checkpoint CP1). No ROS, nothing else running.

    ros2 run nontok_base wheel_check                 (or: python3 -m nontok_base.wheel_check)
    ... --port /dev/nontok_mega --speed 0.10

PUT THE ROBOT ON BLOCKS FIRST: all four wheels off the ground, hand on the cut-off switch.
It drives the wheels slowly for 2.5 s in each of three patterns (forward, sideways to the
left, turning left) and prints, per wheel, what the encoder counted against what it should
have counted. It stops at once on Ctrl-C, and the Mega stops by itself 0.3 s after this
program dies.

Reading the result:
- wrong sign on every pattern for one wheel: flip that wheel's ENC_SIGN in the firmware
  config.h (the wheel will also have run fast, because the speed loop was fighting itself);
- no counts: the encoder is not being read, or that motor did not turn;
- right sign but the wheel visibly turned the wrong way: that motor's wires are swapped.
"""
import argparse
import sys
import time

from . import protocol
from .kinematics import MecanumParams, body_to_wheels, count_delta

WHEELS = ('front-left', 'front-right', 'rear-left', 'rear-right')
PATTERNS = (('forward', (1.0, 0.0, 0.0)), ('sideways to the left', (0.0, 1.0, 0.0)), ('turning left', (0.0, 0.0, 1.0)))


def judge(expected_counts, counted):
    """One verdict per wheel: 'ok', 'wrong sign', 'no counts', 'too few' or 'too many'."""
    out = []
    for want, got in zip(expected_counts, counted):
        if abs(got) < 0.1 * abs(want):
            out.append('no counts')
        elif (got > 0) != (want > 0):
            out.append('wrong sign')
        elif abs(got) < 0.5 * abs(want):
            out.append('too few')
        elif abs(got) > 1.6 * abs(want):
            out.append('too many')
        else:
            out.append('ok')
    return out


class Mega:
    def __init__(self, port: str, baud: int):
        import serial
        self.serial = serial.Serial(port, baud, timeout=0.05)
        self.counts = None
        self.status = None
        self.replies = []

    def send(self, line: str) -> None:
        self.serial.write((line + '\n').encode('ascii'))

    def poll(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            message = protocol.parse_line(self.serial.readline().decode('ascii', 'replace'))
            if isinstance(message, protocol.Odo):
                self.counts = message.counts
            elif isinstance(message, protocol.Sta):
                self.status = message
            elif isinstance(message, protocol.Reply):
                self.replies.append((message.kind, message.arg))

    def run(self, vx: float, vy: float, wz: float, seconds: float):
        """Drive for `seconds`; returns the count change of each wheel, or None if it would not arm."""
        self.replies.clear()
        self.send(protocol.ARM)
        # From here a VEL line must go out at least every 0.3 s or the Mega disarms itself.
        for _ in range(10):
            self.send(protocol.encode_vel(0.0, 0.0, 0.0))
            self.poll(0.05)
            if ('OK', 'ARM') in self.replies:
                break
        else:
            return None
        start = self.counts
        end_time = time.monotonic() + seconds
        try:
            while time.monotonic() < end_time:
                self.send(protocol.encode_vel(vx, vy, wz))
                self.poll(0.05)
        finally:
            for _ in range(10):                      # let the wheels stop before reading the counts
                self.send(protocol.encode_vel(0.0, 0.0, 0.0))
                self.poll(0.05)
            self.send(protocol.DISARM)
            self.poll(0.3)
        return [count_delta(new, old) for new, old in zip(self.counts, start)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/nontok_mega')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--speed', type=float, default=0.10, help='wheel rim speed in m/s (default 0.10)')
    parser.add_argument('--seconds', type=float, default=2.5)
    parser.add_argument('--yes', action='store_true', help='do not ask whether the wheels are off the ground')
    args = parser.parse_args()
    params = MecanumParams()
    mega = Mega(args.port, args.baud)
    mega.poll(3.0)   # opening the port restarts the Mega
    if mega.counts is None:
        print('No odometry from the Mega on %s. Is it the Nontok firmware?' % args.port)
        return 1
    print('Mega is talking. Encoder counts now: %s' % (mega.counts,))
    if not args.yes and input('Are ALL FOUR WHEELS OFF THE GROUND? Type yes to spin them: ').strip().lower() != 'yes':
        print('Nothing was moved.')
        return 1
    problems = 0
    try:
        for name, (ux, uy, uw) in PATTERNS:
            body = (ux * args.speed, uy * args.speed, uw * args.speed / params.rotation_arm_m)
            expected = [v * args.seconds / params.metres_per_count
                        for v in body_to_wheels(*body, params.rotation_arm_m)]
            counted = mega.run(*body, args.seconds)
            if counted is None:
                print('The Mega refused to arm (replies: %s). Check the cut-off switch and the battery.' % mega.replies)
                return 1
            verdicts = judge(expected, counted)
            print('\n%s:' % name.capitalize())
            for wheel, want, got, verdict in zip(WHEELS, expected, counted, verdicts):
                print('  %-12s should turn %-9s  expected %+6.0f counts, counted %+6d   %s' % (
                    wheel, 'forward' if want > 0 else 'backward', want, got, verdict.upper() if verdict != 'ok' else 'ok'))
            problems += sum(v != 'ok' for v in verdicts)
            if ('ERR', 'WATCHDOG') in mega.replies:
                print('  The Mega\'s watchdog tripped during this run.')
            time.sleep(1.0)
    except KeyboardInterrupt:
        mega.send(protocol.STOP)
        print('\nStopped.')
        return 1
    print('\n%s' % ('All wheels count as expected. Also confirm by eye that each turned the way the table says.'
                    if problems == 0 else '%d wheel result(s) need attention; see the notes at the top of this file.' % problems))
    return 0 if problems == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
