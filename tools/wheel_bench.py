#!/usr/bin/env python3
"""Wheel bench tests for checkpoint CP1, run ON THE JETSON (not the Mac: VEL must arrive every
300 ms and the Wi-Fi link to the Mac stalls for longer). Works with the Jetson's Python 3.6.

  python3 wheel_bench.py watch   # hand test: live encoder counts and turns, never arms
  python3 wheel_bench.py ff      # FFTEST: open-loop PWM 60/90/120 both ways, fits FF_A/FF_B
  python3 wheel_bench.py hold    # speed hold 100/200/300 mm/s both ways, then strafe and turn

ff and hold drive the motors: wheels raised, 12 V on, a hand on the cut-off switch. They ask you
to type LIFTED first. Every exit (end, Ctrl-C, error, lost SSH) sends '!' to stop and disarm.
The raw serial lines are saved under ~/Documents/David/wheel_bench/.
Opening the port resets the Mega, so the tray stepper position restarts at 0: tray flat first.
"""
import argparse
import math
import os
import select
import signal
import statistics
import sys
import time

import serial

WHEELS = ('FL', 'FR', 'RL', 'RR')
# Must match firmware/nontok_ssf_mega/include/config.h.
WHEEL_DIAMETER_MM = 80.0
ENC_COUNTS_PER_REV = 660.0
ROTATION_ARM_MM = 192.5   # 165/2 + 220/2, measured 23 Nov
MAX_WHEEL_MM_S = 400.0
ACCEL_MM_S2 = 300.0
ACCEL_RAD_S2 = 1.5
MAX_PWM = 120


class Bench:
    def __init__(self, port, log_dir, mode, mm_per_count):
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, '%s_%s.log' % (mode, time.strftime('%Y%m%d_%H%M%S')))
        self.log = open(self.log_path, 'w')
        self.mm_per_count = mm_per_count
        self.ser = serial.Serial(port, 115200, timeout=0.02)  # opening resets the Mega
        self.buf = b''

    def note(self, text):
        self.log.write('# %.3f %s\n' % (time.time(), text))
        self.log.flush()

    def send(self, line):
        self.ser.write((line + '\n').encode())
        self.note('> ' + line)

    def stop(self):
        try:
            self.ser.write(b'!')
            self.ser.write(b'DISARM\n')
            self.ser.flush()
            self.note('> ! DISARM (exit)')
        except Exception:
            pass

    def lines(self):
        """Complete lines received so far (non-blocking apart from the 20 ms read timeout)."""
        self.buf += self.ser.read(512)
        out = []
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            text = raw.decode('ascii', 'replace').strip()
            if text:
                self.log.write('%.3f %s\n' % (time.time(), text))
                out.append(text)
        if out:
            self.log.flush()
        return out

    def wait_for(self, prefix, timeout):
        end = time.time() + timeout
        while time.time() < end:
            for text in self.lines():
                if text.startswith(prefix):
                    return text
                if text.startswith('ERR'):
                    raise RuntimeError('Mega answered %s while waiting for %s' % (text, prefix))
        raise RuntimeError('no %s within %.0f s' % (prefix, timeout))

    def wait_ready(self):
        try:
            line = self.wait_for('READY', 6.0)
        except RuntimeError:
            # No reset on open (DTR not toggled): pulse DTR, which resets the Mega, and wait again.
            try:
                self.ser.dtr = False
                time.sleep(0.1)
                self.ser.dtr = True
            except OSError:
                raise RuntimeError('no READY within 6 s and DTR cannot be pulsed: is the nontok_ssf '
                                   'firmware flashed, and is this the Mega port?')
            line = self.wait_for('READY', 6.0)
        if 'NONTOK-SSF' not in line:
            raise RuntimeError('unexpected firmware: %s' % line)
        print(line)


def parse_odo(text):
    """'ODO t c1 c2 c3 c4 gz' -> (t_ms, [counts x4], gz) or None."""
    parts = text.split()
    if len(parts) != 7 or parts[0] != 'ODO':
        return None
    try:
        nums = [int(p) for p in parts[1:]]
    except ValueError:
        return None
    return nums[0], nums[1:5], nums[5]


def confirm_lifted():
    print('This drives the motors. Wheels raised and clear, 12 V on, hand on the cut-off switch,')
    print('tray flat. Type LIFTED to go on:', end=' ', flush=True)
    if sys.stdin.readline().strip() != 'LIFTED':
        print('Not confirmed; nothing was sent.')
        sys.exit(1)


# ---------------- watch ----------------
def run_watch(bench, args):
    print('Never arms. Turn one wheel at a time by hand: 10 turns forward, then 10 back.')
    print('Expected: about +%d counts, then back near 0. Enter = zero all.' % (10 * ENC_COUNTS_PER_REV))
    print('A count that stays near 0 while the wheel turns: channel B is not connected.')
    print('RR (M4) has no channel B, so it counts up whichever way it turns.\n')
    zero = None
    last_print = 0.0
    last_sta = None
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            zero = None
        for text in bench.lines():
            if text.startswith('ERR'):
                print('\n' + text)
            elif text.startswith('STA'):
                fault = text.split()[-1]
                if fault != last_sta:
                    last_sta = fault
                    print('\nfault: %s (%s)' % (fault, text))
            odo = parse_odo(text)
            if not odo:
                continue
            counts = odo[1]
            if zero is None:
                zero = list(counts)
            if time.time() - last_print >= 0.2:
                last_print = time.time()
                rel = [c - z for c, z in zip(counts, zero)]
                cells = ['%s %+7d (%+6.2f turns)' % (n, r, r / ENC_COUNTS_PER_REV)
                         for n, r in zip(WHEELS, rel)]
                print('\r' + '   '.join(cells) + '   gz %+5d' % odo[2], end='', flush=True)


# ---------------- ff ----------------
def fit_line(points):
    """Least squares pwm = a + b * v over (v, pwm) points."""
    n = len(points)
    mv = sum(p[0] for p in points) / n
    mp = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mv) ** 2 for p in points)
    if sxx <= 0:
        return None
    b = sum((p[0] - mv) * (p[1] - mp) for p in points) / sxx
    return mp - b * mv, b


def run_ff(bench, args):
    confirm_lifted()
    bench.send('ARM')
    bench.wait_for('OK ARM', 2.0)
    bench.send('FFTEST')
    print('FFTEST running (about 13 s). Press Ctrl-C to abort.')
    rows = []
    end = time.time() + 30
    while time.time() < end:
        for text in bench.lines():
            if text.startswith('FF '):
                nums = [int(p) for p in text.split()[1:]]
                rows.append((nums[0], nums[1:5]))
                print('  PWM %+4d   ' % nums[0] +
                      '   '.join('%s %+5d mm/s' % (n, v) for n, v in zip(WHEELS, nums[1:5])))
            elif text.startswith('DONE FFTEST'):
                end = 0
                break
            elif text.startswith('ERR'):
                raise RuntimeError('Mega: ' + text)
    if len(rows) != 6:
        raise RuntimeError('expected 6 FF lines, got %d' % len(rows))

    print('\nPer wheel (speeds as wheel-forward; reverse levels flipped):')
    problems, fits, top = [], [], []
    for i, name in enumerate(WHEELS):
        pts = []  # (|v| along the command, |pwm|)
        for pwm, speeds in rows:
            along = speeds[i] * (1 if pwm > 0 else -1)
            if along < -20:
                problems.append('%s turns the WRONG WAY at PWM %+d (%+d mm/s): MOTOR_SIGN or ENC_SIGN'
                                % (name, pwm, speeds[i]))
            elif abs(pwm) >= 90 and along < 20:
                problems.append('%s barely moves at PWM %+d (%+d mm/s): not turning or encoder not '
                                'counting' % (name, pwm, speeds[i]))
            if along > 20:
                pts.append((float(along), float(abs(pwm))))
        fwd120 = [s[i] for p, s in rows if p == MAX_PWM][0]
        rev120 = -[s[i] for p, s in rows if p == -MAX_PWM][0]
        top.append(min(fwd120, rev120))
        diff = abs(fwd120 - rev120) / max(1.0, (fwd120 + rev120) / 2) * 100
        fit = fit_line(pts) if len(pts) >= 2 else None
        fits.append(fit)
        print('  %s: at PWM 120 fwd %4d / rev %4d mm/s (differ %3.0f %%)' % (name, fwd120, rev120, diff) +
              ('   fit PWM = %.1f + %.3f x mm/s' % fit if fit else '   no fit'))
        if diff > 15:
            problems.append('%s forward and reverse differ by %.0f %%' % (name, diff))
        if fit and (fit[1] <= 0 or fit[0] < 0):
            problems.append('%s fit looks wrong (a %.1f, b %.3f); check the raw lines' % (name, fit[0], fit[1]))
    med = statistics.median(top)
    for name, t in zip(WHEELS, top):
        if med > 0 and abs(t - med) / med > 0.15:
            problems.append('%s top speed %d mm/s is >15 %% from the median %d' % (name, t, med))
    print('  RR (M4): reverse sign follows the command (no channel B), so WATCH it turn the right way.')

    reach = min(top)
    print('\nSlowest wheel at PWM 120 (the cap): %d mm/s.' % reach)
    if reach < 330:
        print('  -> 300 mm/s cannot be held within 10 %% at the cap; hold test top speed: %d mm/s'
              % (int(reach * 0.85) // 10 * 10))
    if any('WRONG WAY' in x or 'barely moves' in x for x in problems):
        print('\nNo config.h lines: fix the direction / encoder problems below and run ff again.')
    elif all(fits):
        print('\nconfig.h lines (wheels raised; floor values are refined at CP2):')
        print('const float FF_A[4] = {%s};' % ', '.join('%.1ff' % f[0] for f in fits))
        print('const float FF_B[4] = {%s};' % ', '.join('%.3ff' % f[1] for f in fits))
    print('\n' + ('PROBLEMS:\n  ' + '\n  '.join(problems) if problems else 'No problems found.'))


# ---------------- hold ----------------
def mecanum(vx, vy, wz):
    w = wz * ROTATION_ARM_MM
    wheel = [vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w]
    peak = max(abs(x) for x in wheel)
    if peak > MAX_WHEEL_MM_S:
        wheel = [x * MAX_WHEEL_MM_S / peak for x in wheel]
    return wheel


def step_toward(cur, target, step):
    return target if abs(target - cur) <= step else cur + math.copysign(step, target - cur)


def run_hold(bench, args):
    confirm_lifted()
    steps = []
    for s in args.speeds:
        steps.append(('forward %d' % s, s, 0, 0))
    for s in args.speeds:
        steps.append(('reverse %d' % s, -s, 0, 0))
    if not args.no_patterns:
        steps.append(('strafe left 150', 0, 150, 0))
        steps.append(('turn left', 0, 0, round(150 / ROTATION_ARM_MM * 1000)))  # 150 mm/s per wheel
    bench.send('ARM')
    bench.wait_for('OK ARM', 2.0)

    results = []
    last_vel = 0.0
    ramped = [0.0, 0.0, 0.0]
    history = []  # (t_ms, counts)
    wrong_since = [None] * 4
    fast_since = [None] * 4
    dead_since = [None] * 4
    for name, vx, vy, wz in steps + [('stop', 0, 0, 0)]:
        print('%-16s' % name, end=' ', flush=True)
        hold_s = 1.5 if name == 'stop' else args.seconds
        t_end = time.time() + hold_s
        window = []
        while time.time() < t_end:
            now = time.time()
            if now - last_vel >= 0.1:
                bench.send('VEL %d %d %d' % (vx, vy, wz))
                last_vel = now
            for text in bench.lines():
                if text.startswith('ERR') or text.startswith('READY'):
                    raise RuntimeError('Mega: ' + text)
                odo = parse_odo(text)
                if not odo:
                    continue
                t_ms, counts, _ = odo
                if history:
                    dt = (t_ms - history[-1][0]) * 0.001
                    if dt > 0:  # same ramp as the firmware (40 ms cycles)
                        ramped[0] = step_toward(ramped[0], vx, ACCEL_MM_S2 * dt)
                        ramped[1] = step_toward(ramped[1], vy, ACCEL_MM_S2 * dt)
                        ramped[2] = step_toward(ramped[2], wz * 0.001, ACCEL_RAD_S2 * dt)
                history.append((t_ms, counts))
                history = history[-10:]
                if now > t_end - args.measure:
                    window.append((t_ms, counts))
                # Safety: speed over the last ~200 ms against the ramped target.
                old = [h for h in history if t_ms - h[0] >= 180]
                if not old:
                    continue
                t0, c0 = old[-1]
                speed = [(c - b) * bench.mm_per_count / ((t_ms - t0) * 0.001) for c, b in zip(counts, c0)]
                expect = mecanum(ramped[0], ramped[1], ramped[2])
                for i in range(4):
                    e, v = expect[i], speed[i]
                    wrong = abs(e) > 50 and v * math.copysign(1, e) < -40
                    fast = abs(v) > 1.5 * abs(e) + 60
                    # No counts while it should turn: the speed loop is then driving that wheel
                    # at the PWM cap, and nothing above can see it.
                    dead = abs(e) > 80 and abs(v) < 15
                    wrong_since[i] = (wrong_since[i] or now) if wrong else None
                    fast_since[i] = (fast_since[i] or now) if fast else None
                    dead_since[i] = (dead_since[i] or now) if dead else None
                    if dead_since[i] and now - dead_since[i] > 0.6:
                        raise RuntimeError('%s gives NO COUNTS (wanted %+.0f mm/s): encoder not read, '
                                           'or the wheel is not turning; stopped' % (WHEELS[i], e))
                    if wrong_since[i] and now - wrong_since[i] > 0.3:
                        raise RuntimeError('%s turns the WRONG WAY (%+.0f mm/s, wanted %+.0f): stopped'
                                           % (WHEELS[i], v, e))
                    if fast_since[i] and now - fast_since[i] > 0.5:
                        raise RuntimeError('%s runs away (%+.0f mm/s, wanted %+.0f): feed-forward too '
                                           'high or counts per turn wrong; stopped' % (WHEELS[i], v, e))
        if name == 'stop':
            print()
            break
        target = mecanum(vx, vy, wz * 0.001)
        if len(window) < 2:
            raise RuntimeError('no ODO lines during "%s"' % name)
        (ta, ca), (tb, cb) = window[0], window[-1]
        meas = [(b - a) * bench.mm_per_count / ((tb - ta) * 0.001) for a, b in zip(ca, cb)]
        cells, ok = [], True
        for i in range(4):
            err = (meas[i] - target[i]) / abs(target[i]) * 100 if target[i] else 0
            good = abs(err) <= 10
            ok = ok and good
            cells.append('%s %+5.0f/%+4.0f %s' % (WHEELS[i], meas[i], target[i], 'ok ' if good else 'BAD'))
        results.append((name, ok))
        print('  '.join(cells) + ('   PASS' if ok else '   FAIL'))
        bench.send('VEL 0 0 0')  # brief stop between steps
        pause_end = time.time() + 1.0
        while time.time() < pause_end:
            if time.time() - last_vel >= 0.1:
                bench.send('VEL 0 0 0')
                last_vel = time.time()
            for text in bench.lines():
                if text.startswith('ERR') or text.startswith('READY'):
                    raise RuntimeError('Mega: ' + text)
                odo = parse_odo(text)
                if odo:
                    history.append(odo[:2])
                    history = history[-10:]
        ramped = [0.0, 0.0, 0.0]  # the firmware ramps back to 0 during the pause; close enough
        wrong_since = [None] * 4
        fast_since = [None] * 4
        dead_since = [None] * 4
    bench.send('DISARM')
    passed = sum(1 for _, ok in results if ok)
    print('\n%d of %d steps pass (each wheel within 10 %% of its target, measured over the last %.1f s).'
          % (passed, len(results), args.measure))
    print('Columns: measured/target mm/s. RR (M4) direction is assumed from the command: watch it.')


def _interrupt(signum, frame):
    raise KeyboardInterrupt  # lost SSH (SIGHUP) or kill: leave through the stop in finally


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=('watch', 'ff', 'hold'))
    p.add_argument('--port', default='/dev/nontok_mega')
    p.add_argument('--log-dir', default=os.path.expanduser('~/Documents/David/wheel_bench'))
    p.add_argument('--wheel-mm', type=float, default=WHEEL_DIAMETER_MM,
                   help='must match the flashed firmware (config.h)')
    p.add_argument('--speeds', type=int, nargs='+', default=[100, 200, 300], help='hold: mm/s')
    p.add_argument('--seconds', type=float, default=3.0, help='hold: time per step')
    p.add_argument('--measure', type=float, default=1.5, help='hold: judged over the last N s')
    p.add_argument('--no-patterns', action='store_true', help='hold: skip strafe and turn')
    args = p.parse_args()
    if any(s <= 0 or s > MAX_WHEEL_MM_S for s in args.speeds):
        p.error('--speeds must be 1..%d' % MAX_WHEEL_MM_S)
    if args.measure >= args.seconds:
        p.error('--measure must be shorter than --seconds')

    bench = Bench(args.port, args.log_dir, args.mode, math.pi * args.wheel_mm / ENC_COUNTS_PER_REV)
    signal.signal(signal.SIGHUP, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    code = 0
    try:
        bench.wait_ready()
        {'watch': run_watch, 'ff': run_ff, 'hold': run_hold}[args.mode](bench, args)
    except KeyboardInterrupt:
        print('\nStopped.')
    except RuntimeError as e:
        print('\nSTOPPED: %s' % e)
        code = 2
    finally:
        bench.stop()
        print('Raw log: %s' % bench.log_path)
    sys.exit(code)


if __name__ == '__main__':
    main()
