#!/usr/bin/env python3
"""Wheel bench tests for checkpoint CP1, run ON THE JETSON (not the Mac: VEL must arrive every
300 ms and the Wi-Fi link to the Mac stalls for longer). Works with the Jetson's Python 3.6.

  python3 wheel_bench.py watch   # hand test: live encoder counts and turns, never arms
  python3 wheel_bench.py ff      # FFTEST: open-loop PWM 30/40/50/60/90/120 both ways (--levels), fits FF
  python3 wheel_bench.py hold    # speed hold 100/200/300 mm/s both ways, then strafe and turn
  python3 wheel_bench.py drive   # ON THE FLOOR: straight --distance mm (default 1000) at --speed 150
  python3 wheel_bench.py route --route "F1500 R90 U200 D"   # ON THE FLOOR: legs, approach, dump

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
        # Plain ASCII over a bare SSH session: keep any odd byte (the Mega's reset noise) as \xNN.
        self.log = open(self.log_path, 'w', encoding='ascii', errors='backslashreplace')
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
            text = raw.decode('ascii', 'backslashreplace').strip()
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
    levels = sorted(set(args.levels))
    chunks = [levels[i:i + 3] for i in range(0, len(levels), 3)]  # FFTEST takes up to 3 levels
    rows = []
    for chunk in chunks:
        bench.send('ARM')   # FFTEST disarms when it ends
        bench.wait_for('OK ARM', 2.0)
        bench.send('FFTEST ' + ' '.join(str(v) for v in chunk))
        print('FFTEST %s forward, then reverse (about %d s). Ctrl-C aborts.'
              % ('/'.join(str(v) for v in chunk), 4 * len(chunk) + 1))
        got, end = 0, time.time() + 8 * len(chunk) + 10
        while time.time() < end:
            for text in bench.lines():
                if text.startswith('FF '):
                    nums = [int(p) for p in text.split()[1:]]
                    rows.append((nums[0], nums[1:5]))
                    got += 1
                    print('  PWM %+4d   ' % nums[0] +
                          '   '.join('%s %+5d mm/s' % (n, v) for n, v in zip(WHEELS, nums[1:5])))
                elif text.startswith('DONE FFTEST'):
                    end = 0
                    break
                elif text.startswith('ERR'):
                    raise RuntimeError('Mega: ' + text)
        if got != 2 * len(chunk):
            raise RuntimeError('expected %d FF lines, got %d' % (2 * len(chunk), got))
        time.sleep(0.5)

    print('\nLowest PWM at which each wheel turns (> 20 mm/s), forward / reverse:')
    for i, name in enumerate(WHEELS):
        first = []
        for sign in (1, -1):
            moving = [abs(p) for p, s in rows if p * sign > 0 and s[i] * sign > 20]
            first.append(str(min(moving)) if moving else 'none')
        print('  %s: %s / %s' % (name, first[0], first[1]))

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
        hi = max(levels)  # forward/reverse balance and top speed at the highest level tested
        fwd_hi = [s[i] for p, s in rows if p == hi][0]
        rev_hi = -[s[i] for p, s in rows if p == -hi][0]
        top.append(min(fwd_hi, rev_hi))
        diff = abs(fwd_hi - rev_hi) / max(1.0, (fwd_hi + rev_hi) / 2) * 100
        fit = fit_line(pts) if len(pts) >= 2 else None
        fits.append(fit)
        print('  %s: at PWM %d fwd %4d / rev %4d mm/s (differ %3.0f %%)' % (name, hi, fwd_hi, rev_hi, diff) +
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
    print('\nSlowest wheel at PWM %d: %d mm/s.' % (max(levels), reach))
    if max(levels) == MAX_PWM and reach < 330:
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


# ---------------- drive (on the floor) ----------------
def run_drive(bench, args):
    """Straight line forward (or back with a negative distance) by encoder distance, then stop.
    The firmware ramps at ACCEL_MM_S2, so VEL 0 is sent one stopping distance early."""
    dist, speed = args.distance, args.speed
    sign = 1 if dist > 0 else -1
    print('ON THE FLOOR. Drives %+d mm at %d mm/s, then stops. Clear path, someone ready to grab it '
          'or cut 12 V.' % (dist, speed))
    print('Type FLOOR to go on:', end=' ', flush=True)
    if sys.stdin.readline().strip() != 'FLOOR':
        print('Not confirmed; nothing was sent.')
        sys.exit(1)
    bench.send('ARM')
    bench.wait_for('OK ARM', 2.0)
    brake_mm = speed ** 2 / (2 * ACCEL_MM_S2)
    start = None
    heading = 0.0
    last_t = None
    last_vel = 0.0
    braking = False
    t_start = time.time()
    timeout = abs(dist) / speed + 6
    still_since = None
    last_counts = None
    travelled = 0.0
    near_before = [False, False, False]
    while True:
        now = time.time()
        if now - t_start > timeout:
            raise RuntimeError('timeout after %.0f s at %.0f mm: stopped' % (timeout, travelled))
        if now - last_vel >= 0.1:
            bench.send('VEL %d 0 0' % (0 if braking else sign * speed))
            last_vel = now
        for text in bench.lines():
            if text.startswith('ERR') or text.startswith('READY'):
                raise RuntimeError('Mega: ' + text)
            if text.startswith('RNG') and not braking:
                r = text.split()
                if sign > 0 and len(r) == 7:
                    front = [int(r[2]), int(r[3]), int(r[4])]  # FL45, FR45, front
                    near = [0 <= front[0] < 150, 0 <= front[1] < 150, 0 <= front[2] < 250]
                    # Two readings in a row (200 ms apart): one stray echo must not stop the run.
                    if any(a and b for a, b in zip(near, near_before)):
                        raise RuntimeError('obstacle ahead (FL45/FR45/front %s mm, twice): stopped' % front)
                    near_before = near
            odo = parse_odo(text)
            if not odo:
                continue
            t_ms, counts, gz = odo
            if start is None:
                start = list(counts)
            if last_t is not None:
                heading += gz * 0.001 * (t_ms - last_t) * 0.001
            last_t = t_ms
            per = [(c - s) * bench.mm_per_count for c, s in zip(counts, start)]
            travelled = sum(per) / 4.0
            if not braking and abs(travelled) >= abs(dist) - brake_mm:
                braking = True
                bench.send('VEL 0 0 0')
                last_vel = now
            if not braking and now - t_start > 2.0 and abs(travelled) < 30:
                raise RuntimeError('hardly moving after 2 s (%.0f mm): wheels slip or stall; stopped' % travelled)
            if braking:
                if counts == last_counts:
                    still_since = still_since or now
                else:
                    still_since = None
                if still_since and now - still_since > 0.4:
                    bench.send('DISARM')
                    print('Stopped. Encoders: %+.0f mm (FL %+.0f  FR %+.0f  RL %+.0f  RR %+.0f), '
                          'gyro heading change %+.1f deg.'
                          % (travelled, per[0], per[1], per[2], per[3], math.degrees(heading)))
                    print('Now MEASURE the real distance with a tape and tell Claude: wheel diameter '
                          '= %.1f mm x measured / %.0f.' % (WHEEL_DIAMETER_MM, abs(travelled)))
                    return
            last_counts = counts


# ---------------- route (on the floor): measured legs, approach, dump ----------------
ROUTE_HELP = """Route steps, separated by spaces:
  F<mm> / B<mm>  straight forward / back by encoder distance   (F1500)
  S<mm>          strafe, + = left, - = right                   (S-300)
  L<deg> / R<deg> turn left (CCW) / right (CW) by gyro          (R90)
  U<mm>          creep forward until the FRONT ultrasonic reads <mm> (U200), at most 1.5 m
  D              dump: tray to 130 deg, hold 1.5 s, back to 0 (robot stopped and disarmed)
  W<s>           wait"""


class Mover:
    """Streams VEL at 10 Hz and tracks encoder distance, gyro heading and ranges."""

    def __init__(self, bench):
        self.b = bench
        self.counts = None
        self.heading = 0.0  # rad, CCW +
        self.t_ms = None
        self.rng = None
        self.near_before = [False, False, False]

    def poll(self, check_front=True):
        for text in self.b.lines():
            if text.startswith('ERR') or text.startswith('READY'):
                raise RuntimeError('Mega: ' + text)
            if text.startswith('RNG'):
                r = text.split()
                if len(r) == 7:
                    self.rng = [int(x) for x in r[2:7]]
                    front = self.rng[:3]  # FL45, FR45, front
                    near = [0 <= front[0] < 150, 0 <= front[1] < 150, 0 <= front[2] < 250]
                    if check_front and any(a and b for a, b in zip(near, self.near_before)):
                        raise RuntimeError('obstacle ahead (FL45/FR45/front %s mm, twice): stopped' % front)
                    self.near_before = near
            odo = parse_odo(text)
            if odo:
                t_ms, counts, gz = odo
                if self.t_ms is not None:
                    self.heading += gz * 0.001 * (t_ms - self.t_ms) * 0.001
                self.t_ms, self.counts = t_ms, counts

    def wait_counts(self):
        end = time.time() + 2
        while self.counts is None:
            self.poll(False)
            if time.time() > end:
                raise RuntimeError('no ODO lines')

    def leg(self, name, vel, done, check_front, timeout):
        """Drive VEL vel until done(dx, dy, dyaw) says brake; then ramp to 0 and wait until still."""
        self.wait_counts()
        c0, h0 = list(self.counts), self.heading
        t0, last = time.time(), 0.0
        braking, still_since, prev = False, None, None
        while True:
            now = time.time()
            if now - t0 > timeout:
                raise RuntimeError('%s: timeout after %.0f s: stopped' % (name, timeout))
            if now - last >= 0.1:
                self.b.send('VEL %d %d %d' % ((0, 0, 0) if braking else vel))
                last = now
            self.poll(check_front and not braking)
            d = [(c - s) * self.b.mm_per_count for c, s in zip(self.counts, c0)]
            dx = sum(d) / 4.0
            dy = (-d[0] + d[1] + d[2] - d[3]) / 4.0
            dyaw = self.heading - h0
            if not braking and done(dx, dy, dyaw):
                braking = True
            if not braking and now - t0 > 2.5 and max(abs(dx), abs(dy)) < 30 and abs(dyaw) < 0.05:
                raise RuntimeError('%s: hardly moving after 2.5 s: wheels slip or stall; stopped' % name)
            if braking:
                still_since = (still_since or now) if self.counts == prev else None
                if still_since and now - still_since > 0.4:
                    print('  %-7s done: forward %+5.0f mm, left %+5.0f mm, turned %+5.1f deg'
                          % (name, dx, dy, math.degrees(dyaw)))
                    return
            prev = list(self.counts)


def run_route(bench, args):
    steps = args.route.split()
    for s in steps:
        if not (s == 'D' or (len(s) > 1 and s[0] in 'FBSLRUW' and s[1:].lstrip('-').replace('.', '', 1).isdigit())):
            raise RuntimeError('bad route step %r\n%s' % (s, ROUTE_HELP))
    print('ON THE FLOOR. Route: %s' % ' '.join(steps))
    print('Clear path, someone ready to grab it or cut 12 V. Type FLOOR to go on:', end=' ', flush=True)
    if sys.stdin.readline().strip() != 'FLOOR':
        print('Not confirmed; nothing was sent.')
        sys.exit(1)
    m = Mover(bench)
    v, wz = args.speed, 500  # mm/s, mrad/s (about 29 deg/s)
    brake_mm = v ** 2 / (2 * ACCEL_MM_S2)
    brake_rad = (wz * 0.001) ** 2 / (2 * ACCEL_RAD_S2)
    for s in steps:
        kind, val = s[0], float(s[1:]) if len(s) > 1 else 0.0
        if kind in 'FBSLRU':
            bench.send('ARM')
            bench.wait_for('OK ARM', 2.0)
        if kind in 'FB':
            sign = 1 if kind == 'F' else -1
            m.leg(s, (sign * v, 0, 0), lambda dx, dy, a: abs(dx) >= val - brake_mm,
                  sign > 0, val / v + 6)
        elif kind == 'S':
            sign = 1 if val > 0 else -1
            m.leg(s, (0, sign * v, 0), lambda dx, dy, a: abs(dy) >= abs(val) - brake_mm,
                  False, abs(val) / v + 6)
        elif kind in 'LR':
            sign = 1 if kind == 'L' else -1
            target = math.radians(val)
            m.leg(s, (0, 0, sign * wz), lambda dx, dy, a: abs(a) >= target - brake_rad,
                  False, target / (wz * 0.001) + 6)
        elif kind == 'U':
            slow = 100
            def close(dx, dy, a):
                f = m.rng[2] if m.rng else -1
                return (0 <= f <= val + slow ** 2 / (2 * ACCEL_MM_S2)) or dx >= 1500
            m.rng = None
            m.leg(s, (slow, 0, 0), close, False, 1500 / slow + 6)
            print('  front ultrasonic now %s mm' % (m.rng[2] if m.rng else '?'))
        elif kind == 'W':
            end = time.time() + val
            while time.time() < end:
                m.poll(False)
        elif kind == 'D':
            bench.send('DISARM')
            for target in (130, 0):
                bench.send('TRAY %d' % target)
                bench.wait_for('OK TRAY', 2.0)
                bench.wait_for('DONE TRAY', 8.0)
                print('  tray at %d deg' % target)
                if target:
                    end = time.time() + 1.5
                    while time.time() < end:
                        m.poll(False)
        bench.send('DISARM')
    print('Route done. Heading change %+.1f deg (gyro).' % math.degrees(m.heading))


def _interrupt(signum, frame):
    raise KeyboardInterrupt  # lost SSH (SIGHUP) or kill: leave through the stop in finally


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=('watch', 'ff', 'hold', 'drive', 'route'))
    p.add_argument('--port', default='/dev/nontok_mega')
    p.add_argument('--log-dir', default=os.path.expanduser('~/Documents/David/wheel_bench'))
    p.add_argument('--wheel-mm', type=float, default=WHEEL_DIAMETER_MM,
                   help='must match the flashed firmware (config.h)')
    p.add_argument('--levels', type=int, nargs='+', default=[30, 40, 50, 60, 90, 120],
                   help='ff: open-loop PWM levels, 1..%d, at most 6' % MAX_PWM)
    p.add_argument('--speeds', type=int, nargs='+', default=[100, 200, 300], help='hold: mm/s')
    p.add_argument('--distance', type=int, default=1000, help='drive: mm, negative = backward')
    p.add_argument('--speed', type=int, default=150, help='drive / route: mm/s')
    p.add_argument('--route', default='', help='route: steps such as "F1500 R90 U200 D" (see route help)')
    p.add_argument('--seconds', type=float, default=3.0, help='hold: time per step')
    p.add_argument('--measure', type=float, default=1.5, help='hold: judged over the last N s')
    p.add_argument('--no-patterns', action='store_true', help='hold: skip strafe and turn')
    args = p.parse_args()
    if any(s <= 0 or s > MAX_WHEEL_MM_S for s in args.speeds):
        p.error('--speeds must be 1..%d' % MAX_WHEEL_MM_S)
    if any(v < 1 or v > MAX_PWM for v in args.levels) or not 2 <= len(set(args.levels)) <= 6:
        p.error('--levels: 2 to 6 different values, each 1..%d' % MAX_PWM)
    if not 50 <= abs(args.distance) <= 3000 or not 50 <= args.speed <= 250:
        p.error('drive: |--distance| 50..3000 mm, --speed 50..250 mm/s')
    if args.measure >= args.seconds:
        p.error('--measure must be shorter than --seconds')

    bench = Bench(args.port, args.log_dir, args.mode, math.pi * args.wheel_mm / ENC_COUNTS_PER_REV)
    signal.signal(signal.SIGHUP, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    code = 0
    try:
        bench.wait_ready()
        {'watch': run_watch, 'ff': run_ff, 'hold': run_hold, 'drive': run_drive, 'route': run_route}[args.mode](bench, args)
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
