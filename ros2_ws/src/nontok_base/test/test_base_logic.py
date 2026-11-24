"""Tests for the base bridge logic. No ROS and no robot needed."""
import math
import os
import threading
import time

import pytest

from nontok_base import protocol
from nontok_base.bridge_core import BridgeCore, OdomSample, RangeFilter
from nontok_base.clock_sync import ClockSync
from nontok_base.fake_mega import PERIOD_S, FakeMegaCore
from nontok_base.kinematics import (MecanumParams, body_to_wheels, count_delta, counts_to_body,
                                    wheels_to_body)
from nontok_base.protocol import Odo, Ready, Reply, Rng, Sta


# ---------------- protocol ----------------
def test_encode_vel_converts_and_clamps():
    assert protocol.encode_vel(0.25, -0.08, 0.8) == 'VEL 250 -80 800'
    assert protocol.encode_vel(9.0, 0.0, -9.0) == 'VEL 500 0 -3000'


def test_parse_valid_lines():
    assert protocol.parse_line('ODO 1040 5 -6 7 -8 -120\n') == Odo(1040, (5, -6, 7, -8), -0.12)
    assert protocol.parse_line('RNG 40 310 -1 1500 20 999') == Rng(40, (0.31, None, 1.5, 0.02, 0.999))
    assert protocol.parse_line('STA 80 1 45 4980 7700 NONE') == Sta(80, True, 45.0, 4.98, 7.7, '')
    assert protocol.parse_line('STA 80 0 0 0 0 WATCHDOG').fault == 'WATCHDOG'
    assert protocol.parse_line('READY NONTOK-SSF 1') == Ready('NONTOK-SSF', 1)
    assert protocol.parse_line('PONG') == Reply('PONG')
    assert protocol.parse_line('DONE TRAY') == Reply('DONE', 'TRAY')
    assert protocol.parse_line('EVT WAKE') == Reply('EVT', 'WAKE')


@pytest.mark.parametrize('line', [
    '', '   ', 'ODO 1 2 3', 'ODO a b c d e f', 'ODO 10 1 2 3 4 5 6', 'ODO -5 1 2 3 4 0',
    'RNG 40 310 -1 1500 20', 'RNG 40 310 -1 1500 20 99999', 'STA 80 2 0 0 0 NONE',
    'OD0 1 2 3 4 5 6', 'garbage', 'OK', 'ODO 10 1 2 3 4 99999',
])
def test_parse_rejects_damaged_lines(line):
    assert protocol.parse_line(line) is None


def test_oled_text_is_cleaned():
    assert protocol.encode_oled('go!\n now') == 'OLED go now'
    assert len(protocol.encode_oled('x' * 40)) == 5 + 16


# ---------------- kinematics ----------------
@pytest.mark.parametrize('body', [(0.2, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.7), (0.1, -0.05, 0.3)])
def test_forward_inverts_inverse(body):
    wheels = body_to_wheels(*body, 0.16)
    assert wheels_to_body(wheels, 0.16) == pytest.approx(body)


def test_counts_to_distance():
    params = MecanumParams()
    one_rev = [660, 660, 660, 660]
    dx, dy, dyaw = counts_to_body(one_rev, params)
    assert dx == pytest.approx(math.pi * 0.080)
    assert dy == pytest.approx(0.0) and dyaw == pytest.approx(0.0)
    # Strafing left: FL and RR roll backwards.
    dx, dy, _ = counts_to_body([-660, 660, 660, -660], MecanumParams(scale_y=0.9))
    assert dx == pytest.approx(0.0)
    assert dy == pytest.approx(0.9 * math.pi * 0.080)


def test_count_delta_handles_wrap():
    assert count_delta(5, 3) == 2
    assert count_delta(-2 ** 31, 2 ** 31 - 1) == 1
    assert count_delta(2 ** 31 - 1, -2 ** 31) == -1


# ---------------- clock ----------------
def test_clock_sync_uses_smallest_delay():
    sync = ClockSync()
    # Mega time 1.000 s was measured at host time 100.000; lines arrive 5-30 ms late.
    assert sync.stamp(1000, 100.030) == pytest.approx(100.030)
    assert sync.stamp(1040, 100.045) == pytest.approx(100.045)   # 5 ms delay: best so far
    assert sync.stamp(1080, 100.110) == pytest.approx(100.085)   # late line, still stamped right


def test_clock_sync_restarts_after_mega_reset():
    sync = ClockSync()
    sync.stamp(50000, 200.0)
    assert sync.stamp(40, 201.0) == pytest.approx(201.0)


# ---------------- range filter ----------------
def test_range_filter_rules():
    f = RangeFilter(max_range=1.5)
    assert f.update(0.8, 0.0) == 0.8
    assert f.update(2.4, 0.2) == 1.5          # far reading: clear to the maximum
    assert f.update(None, 0.4) == 1.5         # no echo, nothing close recently: clear
    assert f.update(0.3, 1.0) == 0.3
    assert f.update(None, 1.2) is None        # lost right after a close reading: say nothing
    assert f.update(None, 1.8) == 1.5         # still nothing 0.8 s later: clear


# ---------------- bridge + fake Mega ----------------
def run(core: BridgeCore, mega: FakeMegaCore, seconds: float, command=None, t0: float = 0.0):
    """Runs both for a while; the bridge sends VEL every 50 ms like the node's timer."""
    samples = []
    steps = int(round(seconds / PERIOD_S))
    next_send = 0.0
    for i in range(steps):
        now = t0 + (i + 1) * PERIOD_S
        if command is not None:
            core.set_command(*command, now)
        if now >= next_send:
            next_send = now + 0.05
            line = core.velocity_line(now)
            if line:
                for reply in mega.feed(line):
                    core.handle_line(reply, now)
        for line in mega.step():
            result = core.handle_line(line, now)
            if isinstance(result, OdomSample):
                samples.append(result)
    return samples


def start():
    core, mega = BridgeCore(MecanumParams()), FakeMegaCore()
    for line in mega.boot():
        core.handle_line(line, 0.0)
    return core, mega


def arm(core, mega, now=0.0):
    for reply in mega.feed('ARM'):
        core.handle_line(reply, now)


def test_not_armed_sends_nothing_and_does_not_move():
    core, mega = start()
    run(core, mega, 1.0, command=(0.2, 0.0, 0.0))
    assert core.velocity_line(1.0) is None
    assert mega.x == pytest.approx(0.0)


def test_drives_forward_and_odometry_matches_truth():
    core, mega = start()
    arm(core, mega)
    samples = run(core, mega, 4.0, command=(0.2, 0.0, 0.0))
    assert mega.armed
    assert samples[-1].vx == pytest.approx(0.2, abs=0.01)
    assert samples[-1].vy == pytest.approx(0.0, abs=0.005)
    assert core.x == pytest.approx(mega.x, abs=0.005)
    assert mega.x > 0.6


def test_strafe_and_turn_odometry():
    core, mega = start()
    arm(core, mega)
    run(core, mega, 2.0, command=(0.0, 0.08, 0.0))
    assert core.y == pytest.approx(mega.y, abs=0.005) and mega.y > 0.1
    run(core, mega, 2.0, command=(0.0, 0.0, 0.5), t0=2.0)
    assert core.yaw == pytest.approx(mega.yaw, abs=0.01) and mega.yaw > 0.5


def test_stale_command_becomes_zero_but_keeps_the_link_alive():
    core, mega = start()
    arm(core, mega)
    run(core, mega, 2.0, command=(0.2, 0.0, 0.0))
    samples = run(core, mega, 2.0, command=None, t0=2.0)   # /cmd_vel stops arriving
    assert mega.armed                                       # zero VELs keep the watchdog fed
    assert abs(samples[-1].vx) < 0.005


def test_watchdog_disarms_when_the_bridge_goes_silent():
    core, mega = start()
    arm(core, mega)
    run(core, mega, 1.0, command=(0.2, 0.0, 0.0))
    lines = []
    for _ in range(12):  # 0.48 s with no VEL at all
        lines += mega.step()
    assert 'ERR WATCHDOG' in lines
    assert not mega.armed and mega.ramped == [0.0, 0.0, 0.0]
    for line in lines:
        core.handle_line(line, 2.0)
    assert not core.armed and core.velocity_line(2.0) is None


def test_mega_restart_does_not_create_a_jump():
    core, mega = start()
    arm(core, mega)
    run(core, mega, 2.0, command=(0.2, 0.0, 0.0))
    x_before = core.x
    mega2 = FakeMegaCore()                                  # counts and clock start from zero
    for line in mega2.boot():
        core.handle_line(line, 2.0)
    samples = run(core, mega2, 0.5, t0=2.0)
    assert not core.armed
    assert core.x == pytest.approx(x_before, abs=0.001)
    assert all(abs(s.vx) < 0.01 for s in samples)


def test_link_fault_reporting():
    core, mega = start()
    assert core.link_fault(0.0) == 'NO_DATA'
    run(core, mega, 0.2)
    assert core.link_fault(0.2) == ''
    assert core.link_fault(1.0) == 'LINK_LOST'


def test_tray_command_round_trip():
    core, mega = start()
    assert mega.feed(protocol.encode_tray(50)) == ['OK TRAY']
    lines = []
    for _ in range(40):
        lines += mega.step()
    assert 'DONE TRAY' in lines and mega.tray_deg == 50


# ---------------- over a real (pseudo) serial port ----------------
@pytest.mark.skipif(os.name != 'posix', reason='needs a pseudo-terminal')
def test_fake_mega_over_pty():
    serial = pytest.importorskip('serial')
    from nontok_base import fake_mega

    link = '/tmp/nontok_fake_mega_test_%d' % os.getpid()
    thread = threading.Thread(target=fake_mega.main, args=(['--link', link],), daemon=True)
    thread.start()
    deadline = time.time() + 3.0
    while not os.path.exists(link) and time.time() < deadline:
        time.sleep(0.02)
    port = serial.Serial(link, 115200, timeout=1.0)
    port.write(b'PING\n')
    seen = []
    deadline = time.time() + 3.0
    while time.time() < deadline and not ('PONG' in seen and any(l.startswith('ODO') for l in seen)):
        seen.append(port.readline().decode().strip())
    port.close()
    assert 'PONG' in seen
    assert any(protocol.parse_line(l).__class__ is Odo for l in seen if l)


def test_rearms_after_a_watchdog_trip_but_not_after_a_stop_and_not_for_ever():
    core = BridgeCore(MecanumParams())
    core.handle_line('OK ARM', 0.0)
    assert core.rearm_line(0.1) is None                 # armed: nothing to do
    for k in range(core.max_rearms):
        t = 1.0 + k
        core.handle_line('ERR WATCHDOG', t)
        assert not core.armed and core.velocity_line(t) is None
        assert core.rearm_line(t + 0.1) is None         # too soon
        assert core.rearm_line(t + 0.4) == protocol.ARM
        assert core.rearm_line(t + 0.5) is None         # only once per trip
        core.handle_line('OK ARM', t + 0.5)
    core.handle_line('ERR WATCHDOG', 10.0)              # one trip too many within the window
    assert core.rearm_line(10.5) is None and not core.want_armed

    core = BridgeCore(MecanumParams())
    core.handle_line('OK ARM', 0.0)
    core.handle_line('OK STOP', 1.0)
    core.handle_line('ERR WATCHDOG', 1.1)
    assert core.rearm_line(2.0) is None                 # a person asked for the stop


def test_no_movement_without_fresh_ultrasonic_readings():
    core = BridgeCore(MecanumParams())
    core.handle_line('OK ARM', 0.0)
    core.set_command(0.2, 0.0, 0.0, 0.0)
    zero, moving = protocol.encode_vel(0.0, 0.0, 0.0), protocol.encode_vel(0.2, 0.0, 0.0)
    assert core.velocity_line(0.1) == zero              # no ultrasonics heard yet
    core.handle_line('RNG 100 500 500 500 500 500', 0.1)
    core.set_command(0.2, 0.0, 0.0, 0.2)
    assert core.velocity_line(0.2) == moving
    core.set_command(0.2, 0.0, 0.0, 1.5)
    assert core.velocity_line(1.5) == zero              # readings stopped more than 1 s ago
    assert not core.ranges_fresh(1.5)
    assert BridgeCore(MecanumParams(), range_timeout_s=0.0).ranges_fresh(5.0)   # check switched off


def test_wheel_check_verdicts_and_a_run_against_the_simulated_mega():
    from nontok_base.wheel_check import judge
    assert judge([500, 500, -500, -500], [480, -470, -20, -900]) == ['ok', 'wrong sign', 'no counts', 'too many']
    assert judge([500, -500], [200, -510]) == ['too few', 'ok']


# ---- simulated room ----
def _world(people=()):
    from nontok_base.sim_world import World
    sensors = [(0.11, 0.07, math.radians(45)), (0.11, -0.07, math.radians(-45)), (0.13, 0.0, 0.0),
               (0.0, 0.10, math.radians(90)), (0.0, -0.10, math.radians(-90))]
    return World([(2.0, -5.0, 2.1, 5.0)], [('table', (-1.0, 0.5, 1.0, 1.0))], list(people), sensors)


def test_simulated_ultrasonics_see_walls_tables_and_people():
    from nontok_base.sim_world import Person
    left45, right45, front, left, right = _world().ranges_mm(0.0, 0.0, 0.0)
    assert abs(front - 1870) <= 2          # wall 2.0 m ahead, sensor 0.13 m forward
    assert abs(left - 400) <= 2            # table edge at y = 0.5, sensor at y = 0.10
    assert right == -1                     # nothing on the right
    person = Person('p', [(1.0, 0.0), (1.0, 0.1)], radius=0.2)
    assert abs(_world([person]).ranges_mm(0.0, 0.0, 0.0)[2] - 670) <= 2


def test_simulated_person_walks_but_not_into_the_robot():
    from nontok_base.sim_world import Person
    person = Person('p', [(0.0, 0.0), (2.0, 0.0)], speed=0.5)
    for _ in range(100):                   # 4 s, robot far away
        person.step(0.04, 10.0, 10.0, 0.16)
    assert abs(person.x - 2.0) < 0.05
    person = Person('p', [(0.0, 0.0), (2.0, 0.0)], speed=0.5)
    world = _world([person])
    for _ in range(50):                    # robot parked on the path at x = 1
        world.step(0.04, 1.0, 0.0)
    assert person.x < 1.0 - 0.16 - 0.20 and world.contacts == 0
    stopped_at = person.x
    for _ in range(60):                    # ... after 3 s the person gives up and walks back
        world.step(0.04, 1.0, 0.0)
    assert person.x < stopped_at - 0.2 and world.contacts == 0


def test_simulated_contact_is_counted_once():
    world = _world()
    for _ in range(10):
        world.step(0.04, 0.0, 0.45)        # 5 cm from the table edge
    assert world.contacts == 1 and world.touching == 'table'


def test_bridge_pose_follows_the_simulated_robot_through_many_turns():
    """The simulated room is placed by the fake Mega's true pose, the simulated tags by the
    bridge's; they must not drift apart (they did, by the gyro correction, before)."""
    mega = FakeMegaCore()
    mega.gyro_gain = 1.0 / 1.009
    bridge = BridgeCore(MecanumParams(0.080, 660.0, 0.160), gyro_scale=1.009)
    bridge.yaw_from_encoders = False   # this test is about the gyro path (yaw_source: gyro)
    mega.feed('ARM')
    for i in range(5000):                  # 200 s of driving in circles and arcs
        if i % 5 == 0:
            mega.feed('VEL 150 %d %d' % (40 if i % 400 < 200 else -40, 700 if i % 1000 < 600 else -500))
        for line in mega.step():
            bridge.handle_line(line, mega.t)
    assert abs(bridge.yaw - mega.yaw) < 0.005      # it was 0.4 rad without the matching gyro
    assert math.hypot(bridge.x - mega.x, bridge.y - mega.y) < 0.02
