"""A stand-in for the Mega, speaking the same protocol, for work without the robot.

FakeMegaCore is pure Python and is used directly by the tests. Run as a program
(`ros2 run nontok_base fake_mega`, or `python3 -m nontok_base.fake_mega`) it opens a
pseudo-terminal that the bridge can use as its serial port.

The simulated robot follows its velocity commands exactly (with the firmware's
acceleration limit); it does not model slip or motor lag.
"""
import math
from typing import List, Optional

from .kinematics import body_to_wheels

PERIOD_S = 0.040
WATCHDOG_S = 0.300
ACCEL = 0.3          # m/s^2
ACCEL_YAW = 1.5      # rad/s^2
RNG_EVERY, STA_EVERY = 5, 12


def _step_toward(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


class FakeMegaCore:
    def __init__(self, wheel_diameter_m: float = 0.080, counts_per_rev: float = 660.0,
                 rotation_arm_m: float = 0.160):
        self.metres_per_count = math.pi * wheel_diameter_m / counts_per_rev
        self.arm = rotation_arm_m
        self.t = 0.0
        self.armed = False
        self.fault = 'NONE'
        self.wanted = [0.0, 0.0, 0.0]
        self.ramped = [0.0, 0.0, 0.0]
        self.last_vel_t = 0.0
        self.wheel_m = [0.0, 0.0, 0.0, 0.0]   # distance rolled by each wheel
        self.tray_deg = 0
        self.tray_target = 0
        self.tray_moving = False
        self.cycle = 0
        # Scene, settable by tests: ranges in mm (-1 = no echo), charger voltage in mV.
        self.ranges_mm = [-1, -1, -1, -1, -1]
        self.charge_mv = 0
        self.battery_mv = 7800
        # True pose of the simulated robot (for tests and for simulated sensors).
        self.x = self.y = self.yaw = 0.0
        self.oled = ''
        # Reading / true turn rate. The real gyro reads 0.9 % low and the bridge corrects for
        # that (gyro_scale in base.yaml); main() makes the simulated one read low to match.
        self.gyro_gain = 1.0
        self._gyro_carry = 0.0

    def boot(self) -> List[str]:
        return ['READY NONTOK-SSF 1']

    # ---- commands ----
    def feed(self, line: str) -> List[str]:
        if '!' in line:
            self._disarm()
            return ['OK STOP']
        parts = line.strip().split()
        if not parts:
            return []
        word, args = parts[0], parts[1:]
        if word == 'OLED':
            self.oled = line.strip()[5:]
            return []
        try:
            numbers = [int(a) for a in args]
        except ValueError:
            return ['ERR UNKNOWN']
        if word == 'PING' and not args:
            return ['PONG']
        if word == 'ARM' and not args:
            self.armed, self.fault = True, 'NONE'
            self.last_vel_t = self.t
            self.wanted = [0.0, 0.0, 0.0]
            return ['OK ARM']
        if word in ('DISARM', 'STOP') and not args:
            self._disarm()
            return ['OK ' + word]
        if word == 'CAL' and not args:
            return ['OK CAL']
        if word == 'VEL' and len(numbers) == 3:
            if abs(numbers[0]) > 500 or abs(numbers[1]) > 500 or abs(numbers[2]) > 3000:
                return ['ERR UNKNOWN']
            if self.armed:
                self.wanted = [numbers[0] / 1000.0, numbers[1] / 1000.0, numbers[2] / 1000.0]
                self.last_vel_t = self.t
            return []
        if word == 'TRAY' and len(numbers) == 1:
            self.tray_target = max(0, min(numbers[0], 130))   # the firmware clips to 0..130
            self.tray_moving = True
            return ['OK TRAY']
        return ['ERR UNKNOWN']

    def _disarm(self) -> None:
        self.armed = False
        self.wanted = [0.0, 0.0, 0.0]
        self.ramped = [0.0, 0.0, 0.0]

    # ---- one 40 ms control cycle ----
    def step(self) -> List[str]:
        out: List[str] = []
        dt = PERIOD_S
        self.t += dt
        if self.armed and self.t - self.last_vel_t > WATCHDOG_S:
            self._disarm()
            self.fault = 'WATCHDOG'
            out.append('ERR WATCHDOG')
        if self.armed:
            self.ramped[0] = _step_toward(self.ramped[0], self.wanted[0], ACCEL * dt)
            self.ramped[1] = _step_toward(self.ramped[1], self.wanted[1], ACCEL * dt)
            self.ramped[2] = _step_toward(self.ramped[2], self.wanted[2], ACCEL_YAW * dt)
        vx, vy, wz = self.ramped
        for i, speed in enumerate(body_to_wheels(vx, vy, wz, self.arm)):
            self.wheel_m[i] += speed * dt
        mid = self.yaw + 0.5 * wz * dt
        self.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        self.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        self.yaw += wz * dt

        t_ms = int(round(self.t * 1000))
        counts = [int(round(m / self.metres_per_count)) for m in self.wheel_m]
        # The gyro reading in whole mrad/s; the rounding error is carried to the next cycle so
        # that the readings add up to the true turn (otherwise the pose the bridge works out
        # slowly rotates away from the true one, and from the simulated room).
        exact = wz * 1000.0 * self.gyro_gain + self._gyro_carry
        gyro = int(round(exact))
        self._gyro_carry = exact - gyro
        out.append('ODO %d %d %d %d %d %d' % (t_ms, *counts, gyro))
        if self.cycle % RNG_EVERY == 0:
            out.append('RNG %d %d %d %d %d %d' % (t_ms, *self.ranges_mm))
        if self.cycle % STA_EVERY == 0:
            out.append('STA %d %d %d %d %d %s' % (
                t_ms, int(self.armed), self.tray_deg, self.charge_mv, self.battery_mv, self.fault))
        self.cycle = (self.cycle + 1) % (RNG_EVERY * STA_EVERY)

        if self.tray_moving:
            if self.tray_deg < self.tray_target:
                self.tray_deg = min(self.tray_target, self.tray_deg + 5)   # stepper: ~1 s for 0..130
            elif self.tray_deg > self.tray_target:
                self.tray_deg = max(self.tray_target, self.tray_deg - 5)
            if self.tray_deg == self.tray_target:
                self.tray_moving = False
                out.append('DONE TRAY')
        return out


def main(argv: Optional[List[str]] = None) -> None:
    """Serve the fake Mega on a pseudo-terminal. Prints the device path to use as the port."""
    import argparse
    import os
    import select
    import time
    import tty

    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--link', default='/tmp/nontok_fake_mega',
                        help='symlink to create for the pseudo-terminal')
    parser.add_argument('--charge-mv', type=int, default=0, help='simulated charger voltage')
    parser.add_argument('--gyro-scale', type=float, default=1.0,
                        help="the bridge's gyro_scale: the simulated gyro reads low by this factor")
    parser.add_argument('--scene', default='', help='tables and walking people (a scene_*.yaml); needs the next two')
    parser.add_argument('--layout', default='', help='lab_layout.yaml: the walls')
    parser.add_argument('--robot', default='', help='robot.yaml: where the ultrasonics are')
    parser.add_argument('--world-file', default='/tmp/nontok_sim_world.json',
                        help='where the people are, for the dashboard')
    args, _ = parser.parse_known_args(argv)

    master, slave = os.openpty()
    tty.setraw(slave)  # no echo and no line editing: behave like a plain serial cable
    path = os.ttyname(slave)
    if os.path.islink(args.link):
        os.unlink(args.link)
    os.symlink(path, args.link)
    print('Fake Mega on %s (also %s)' % (path, args.link), flush=True)

    core = FakeMegaCore()
    core.charge_mv = args.charge_mv
    core.gyro_gain = 1.0 / args.gyro_scale
    world = None
    if args.scene:
        from .sim_world import load_world, write_snapshot
        world = load_world(args.scene, args.layout, args.robot)
        print('Simulated room: %d tables, %d people' % (len(world.tables), len(world.people)), flush=True)
    elif os.path.exists(args.world_file):
        os.unlink(args.world_file)
    cycles = 0

    def send(lines: List[str]) -> None:
        for line in lines:
            os.write(master, (line + '\n').encode('ascii'))

    send(core.boot())
    pending = b''
    next_cycle = time.monotonic() + PERIOD_S
    try:
        while True:
            timeout = max(0.0, next_cycle - time.monotonic())
            readable, _, _ = select.select([master], [], [], timeout)
            if readable:
                pending += os.read(master, 256)
                while b'\n' in pending:
                    raw, pending = pending.split(b'\n', 1)
                    send(core.feed(raw.decode('ascii', errors='replace')))
            if time.monotonic() >= next_cycle:
                next_cycle += PERIOD_S
                if world is not None:
                    was = world.contacts
                    world.step(PERIOD_S, core.x, core.y)
                    core.ranges_mm = world.ranges_mm(core.x, core.y, core.yaw)
                    if world.contacts > was:
                        print('SIMULATION: the robot touched %s' % world.touching, flush=True)
                    cycles += 1
                    if cycles % 5 == 0:
                        write_snapshot(world, args.world_file, time.time())
                send(core.step())
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link):
            os.unlink(args.link)


if __name__ == '__main__':
    main()
