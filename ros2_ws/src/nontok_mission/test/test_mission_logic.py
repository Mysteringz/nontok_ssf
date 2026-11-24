"""The mission run against a scripted fake robot: the normal path and every failure branch."""
import pytest

from nontok_mission import state_machine as sm
from nontok_mission.state_machine import FAILED, OK, RUNNING, Mission, MissionConfig, RobotApi

DT = 0.1


class FakeRobot(RobotApi):
    """Every action takes a few ticks, then gives the next scripted result (default: OK)."""

    def __init__(self):
        self.calls = []
        self.nav_script, self.dock_script, self.retreat_script = [], [], []
        self.classify_script = [(True, 'bolt', 'mechanical', 0.93)]
        self.tray_script = {}          # tray angle history
        self.item_on_tray = False
        self.item_stable = True
        self.item_leaves_on_dump = True
        self.dumps_needed = 1
        self.healthy = True
        self.armed = False
        self.arm_works = True
        self.tray_works = True
        self.tray_camera_works = True
        self.charging = False
        self.wake = False
        self.stop_flag = False
        self.tray_camera = False
        self.wake_enabled = False
        self.tray_deg = 0.0
        self.shown = []
        self._nav = self._dock = self._retreat = None      # [ticks left, result]
        self._dumps = 0
        self.action_ticks = 3

    def _start(self, script):
        return [self.action_ticks, script.pop(0) if script else OK]

    @staticmethod
    def _poll(action):
        if action is None:
            return FAILED
        if action[0] > 0:
            action[0] -= 1
            return RUNNING
        return action[1]

    def start_navigation(self, station):
        self.calls.append(('nav', station))
        self._nav = self._start(self.nav_script)

    def navigation_result(self): return self._poll(self._nav)
    def cancel_navigation(self): self.calls.append(('cancel_nav',))

    def start_dock(self, station, require_charge):
        self.calls.append(('dock', station, require_charge))
        self._dock = self._start(self.dock_script)

    def dock_result(self): return self._poll(self._dock)
    def cancel_dock(self): self.calls.append(('cancel_dock',))

    def start_retreat(self):
        self.calls.append(('retreat',))
        self._retreat = self._start(self.retreat_script)

    def retreat_result(self): return self._poll(self._retreat)

    def arm(self, on):
        self.calls.append(('arm', on))
        if on and not self.arm_works:
            return False
        self.armed = on
        return True

    def stop(self): self.calls.append(('stop',))
    def on_charger(self): return self.charging
    def reset_pose_to_dock(self): self.calls.append(('reset_pose',))
    def localization_healthy(self): return self.healthy

    def set_tray(self, angle):
        self.calls.append(('tray', angle))
        if not self.tray_works:
            return False
        if angle > 20 and self.tray_deg <= 20:
            self._dumps += 1
            if self.item_leaves_on_dump and self._dumps >= self.dumps_needed:
                self.item_on_tray = False
        self.tray_deg = angle
        return True

    def enable_tray_camera(self, on):
        self.tray_camera = on
        return self.tray_camera_works or not on

    def capture_tray_reference(self):
        self.calls.append(('reference',))
        return self.tray_camera_works

    def tray_state(self):
        return (sm.TRAY_OCCUPIED if self.item_on_tray else sm.TRAY_EMPTY, self.item_stable)

    def classify(self):
        self.calls.append(('classify',))
        return self.classify_script.pop(0) if len(self.classify_script) > 1 else self.classify_script[0]

    def enable_wake(self, on):
        self.wake_enabled = on
        if on:
            self.wake = False

    def wake_requested(self):
        heard, self.wake = self.wake, False
        return heard

    def stop_requested(self):
        flag, self.stop_flag = self.stop_flag, False
        return flag

    def show(self, text): self.shown.append(text)


class Runner:
    def __init__(self, cfg=None):
        self.robot = FakeRobot()
        self.events = []
        self.mission = Mission(self.robot, cfg or MissionConfig(), self.events.append)
        self.t = 0.0
        self.states = [self.mission.state]

    def tick(self, seconds=DT):
        for _ in range(max(1, int(round(seconds / DT)))):
            self.t += DT
            self.mission.tick(self.t)
            if self.mission.state != self.states[-1]:
                self.states.append(self.mission.state)

    def until(self, state, limit_s=600.0):
        end = self.t + limit_s
        while self.mission.state != state and self.t < end:
            self.tick()
        assert self.mission.state == state, 'stuck in %s (%s); path %s' % (
            self.mission.state, self.mission.detail, self.states)

    def wake(self):
        self.tick()                      # S0 entry actions
        self.robot.wake = True
        self.tick()                      # the call is heard: the mission leaves the dock

    def to_item_wait(self):
        self.wake()
        self.until(sm.S3_WAIT_ITEM)
        self.tick()


FULL_PATH = [sm.S0_DOCKED, sm.S1_UNDOCK, sm.S2_TO_PICKUP, sm.S2_ALIGN_PICKUP, sm.S3_WAIT_ITEM, sm.S4_CLASSIFY,
             sm.S4_LEAVE_PICKUP, sm.S5_TO_STATION, sm.S6_ALIGN, sm.S7_DUMP, sm.S7_LEAVE_STATION,
             sm.S8_TO_DOCK, sm.S9_DOCK, sm.S0_DOCKED]


def nav_targets(robot):
    return [c[1] for c in robot.calls if c[0] == 'nav']


@pytest.mark.parametrize('item,station', [
    ('led', 'electrical'), ('resistor', 'electrical'), ('bolt', 'mechanical'), ('multimeter', 'tools')])
def test_full_mission_for_each_station(item, station):
    r = Runner()
    r.robot.classify_script = [(True, item, station, 0.95)]
    assert r.robot.wake_enabled is False
    r.tick(1.0)
    assert r.mission.state == sm.S0_DOCKED and r.robot.wake_enabled and not r.robot.armed
    r.to_item_wait()
    assert r.robot.tray_camera and ('reference',) in r.robot.calls
    r.tick(3.0)
    assert r.mission.state == sm.S3_WAIT_ITEM          # nothing placed yet: keeps waiting
    r.robot.item_on_tray = True
    r.until(sm.S7_DUMP)
    assert r.mission.target == station
    r.until(sm.S0_DOCKED)
    assert r.states == FULL_PATH
    assert nav_targets(r.robot) == ['pickup', station, 'dock']
    assert [c[1] for c in r.robot.calls if c[0] == 'dock'] == ['pickup', station, 'dock']
    assert [c[1] for c in r.robot.calls if c[0] == 'tray'] == [50.0, 0.0]
    assert len([c for c in r.robot.calls if c[0] == 'retreat']) == 3     # dock, pickup, station
    end = [e for e in r.events if e['event'] == 'mission_end'][0]
    assert end['result'] == 'OK' and end['station'] == station and end['item'] == item
    r.tick(1.0)
    assert not r.robot.armed and r.robot.wake_enabled and not r.robot.tray_camera
    assert not r.robot.item_on_tray and r.robot.tray_deg == 0.0


def test_wake_is_ignored_outside_the_dock_and_two_missions_run_back_to_back():
    r = Runner()
    r.to_item_wait()
    r.robot.wake = True                                  # someone calls again mid-mission
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    r.tick(1.0)
    assert r.mission.state == sm.S0_DOCKED and r.mission.missions_started == 1
    r.robot.wake = True
    r.until(sm.S1_UNDOCK)
    assert r.mission.missions_started == 2


def test_unstable_item_is_not_classified_until_still():
    r = Runner()
    r.to_item_wait()
    r.robot.item_on_tray, r.robot.item_stable = True, False
    r.tick(5.0)
    assert r.mission.state == sm.S3_WAIT_ITEM and ('classify',) not in r.robot.calls
    r.robot.item_stable = True
    r.until(sm.S4_LEAVE_PICKUP)


def test_no_item_goes_home():
    r = Runner(MissionConfig(item_wait_s=10.0))
    r.to_item_wait()
    r.until(sm.S0_DOCKED)
    assert sm.S5_TO_STATION not in r.states and sm.S7_DUMP not in r.states
    assert nav_targets(r.robot) == ['pickup', 'dock']
    assert [e for e in r.events if e['event'] == 'mission_end'][0]['result'] == 'NO_ITEM'


def test_unknown_item_is_retried_then_carried_home():
    r = Runner(MissionConfig(classify_retry_wait_s=1.0))
    r.robot.classify_script = [(False, 'bolt', '', 0.4)]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert len([c for c in r.robot.calls if c[0] == 'classify']) == 3      # first try + 2 retries
    assert 'REPOSITION ITEM' in r.robot.shown
    assert sm.S7_DUMP not in r.states and r.robot.item_on_tray             # never dumped a guess
    assert [e for e in r.events if e['event'] == 'mission_end'][0]['result'] == 'UNKNOWN_ITEM'


def test_unknown_item_goes_to_the_manual_sorting_station_when_set():
    r = Runner(MissionConfig(classify_retry_wait_s=1.0, unknown_station='unknown'))
    r.robot.classify_script = [(False, 'bolt', '', 0.4)]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert len([c for c in r.robot.calls if c[0] == 'classify']) == 3      # first try + 2 retries
    assert nav_targets(r.robot) == ['pickup', 'unknown', 'dock']
    assert sm.S7_DUMP in r.states and not r.robot.item_on_tray             # left there for a person
    assert [e for e in r.events if e['event'] == 'dumped'][0]['item'] == 'unknown'


def test_second_classification_succeeds_after_repositioning():
    r = Runner(MissionConfig(classify_retry_wait_s=1.0))
    r.robot.classify_script = [(False, '', '', 0.3), (True, 'pliers', 'tools', 0.9)]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert nav_targets(r.robot) == ['pickup', 'tools', 'dock']


def test_item_taken_back_during_retry_returns_to_waiting():
    r = Runner(MissionConfig(classify_retry_wait_s=1.0))
    r.robot.classify_script = [(False, '', '', 0.3), (True, 'bolt', 'mechanical', 0.9)]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S4_CLASSIFY)
    r.tick(0.3)
    r.robot.item_on_tray = False
    r.until(sm.S3_WAIT_ITEM)
    assert len([c for c in r.robot.calls if c == ('reference',)]) == 1     # reference not retaken with a hand nearby
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert nav_targets(r.robot) == ['pickup', 'mechanical', 'dock']


def test_navigation_is_retried_once():
    r = Runner()
    r.robot.nav_script = [FAILED, OK]
    r.wake()
    r.until(sm.S2_ALIGN_PICKUP)
    assert nav_targets(r.robot) == ['pickup', 'pickup']


def test_pickup_unreachable_returns_to_dock():
    r = Runner()
    r.robot.nav_script = [FAILED, FAILED]
    r.wake()
    r.until(sm.S0_DOCKED)
    assert sm.S3_WAIT_ITEM not in r.states
    assert nav_targets(r.robot) == ['pickup', 'pickup', 'dock']


def test_station_unreachable_is_a_fault():
    r = Runner()
    r.robot.nav_script = [OK, FAILED, FAILED]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.FAULT)
    assert r.mission.detail == 'STATION_UNREACHABLE'
    r.tick()
    assert ('stop',) in r.robot.calls and not r.robot.armed and r.robot.shown[-1].startswith('FAULT')


def test_navigation_timeout_counts_as_a_failure():
    r = Runner(MissionConfig(nav_timeout_s=5.0))
    r.robot.action_ticks = 10 ** 6                      # navigation never finishes
    r.robot.retreat_script = []
    r.tick()
    r.robot.wake = True
    r.until(sm.S1_UNDOCK)
    r.robot.action_ticks = 3
    r.tick()                                            # the undock itself is quick
    r.robot.action_ticks = 10 ** 6
    r.until(sm.S8_TO_DOCK, limit_s=30.0)
    assert ('cancel_nav',) in r.robot.calls and r.mission.detail == 'PICKUP_UNREACHABLE'


def test_station_alignment_failure_is_a_fault():
    r = Runner()
    r.robot.dock_script = [OK, FAILED, FAILED]
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.FAULT)
    assert r.mission.detail == 'STATION_ALIGN_FAILED' and sm.S7_DUMP not in r.states


def test_item_stuck_is_dumped_twice_then_reported():
    r = Runner()
    r.robot.item_leaves_on_dump = False
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert [c[1] for c in r.robot.calls if c[0] == 'tray'] == [50.0, 0.0, 50.0, 0.0]
    assert [e for e in r.events if e['event'] == 'mission_end'][0]['result'] == 'ITEM_STUCK'


def test_second_dump_clears_the_item():
    r = Runner()
    r.robot.dumps_needed = 2
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S0_DOCKED)
    assert [e for e in r.events if e['event'] == 'dumped'][0]['repeats'] == 1
    assert [e for e in r.events if e['event'] == 'mission_end'][0]['result'] == 'OK'


def test_tray_failure_is_a_fault():
    r = Runner()
    r.to_item_wait()
    r.robot.item_on_tray = True
    r.until(sm.S6_ALIGN)
    r.robot.tray_works = False
    r.until(sm.FAULT)
    assert r.mission.detail == 'TRAY_FAILED'


def test_dock_is_tried_three_times_then_faults_and_recovers_on_the_charger():
    r = Runner(MissionConfig(item_wait_s=5.0, dock_requires_charge=True))
    r.robot.dock_script = [OK, FAILED, FAILED, FAILED]
    r.to_item_wait()
    r.until(sm.FAULT)
    assert r.mission.detail == 'DOCKING_FAILED'
    assert [c for c in r.robot.calls if c[0] == 'dock'][1:] == [('dock', 'dock', True)] * 3
    r.tick(5.0)
    assert r.mission.state == sm.FAULT
    r.robot.charging = True                             # a person puts it on the dock
    r.until(sm.S0_DOCKED)
    r.tick()
    assert ('reset_pose',) in r.robot.calls and r.robot.wake_enabled


def test_manual_reset_leaves_fault():
    r = Runner()
    r.robot.arm_works = False
    r.wake()
    r.until(sm.FAULT)
    assert r.mission.detail == 'ARM_FAILED'
    r.tick()
    r.mission.reset()
    r.until(sm.S0_DOCKED)


def test_stop_request_from_any_state():
    for target in (sm.S2_TO_PICKUP, sm.S3_WAIT_ITEM, sm.S5_TO_STATION, sm.S7_DUMP, sm.S8_TO_DOCK):
        r = Runner()
        r.robot.action_ticks = 30
        r.wake()
        r.robot.item_on_tray = True
        r.until(target)
        r.robot.stop_flag = True
        r.tick(0.3)
        assert r.mission.state == sm.FAULT and r.mission.detail == 'STOPPED', target
        assert ('stop',) in r.robot.calls and ('cancel_nav',) in r.robot.calls and not r.robot.armed


def test_lost_localisation_while_driving_is_a_fault_but_a_blip_is_not():
    r = Runner()
    r.robot.action_ticks = 300
    r.wake()
    r.until(sm.S1_UNDOCK)
    r.robot.action_ticks = 3
    r.tick()
    r.robot.action_ticks = 300
    r.until(sm.S2_TO_PICKUP)
    r.robot.healthy = False
    r.tick(3.0)                                         # shorter than the 5 s grace time
    r.robot.healthy = True
    r.tick(3.0)
    assert r.mission.state == sm.S2_TO_PICKUP
    r.robot.healthy = False
    r.until(sm.FAULT, limit_s=10.0)
    assert r.mission.detail == 'LOCALISATION_LOST' and ('cancel_nav',) in r.robot.calls


def test_tray_camera_failure_goes_home():
    r = Runner()
    r.robot.tray_camera_works = False
    r.wake()
    r.until(sm.S0_DOCKED)
    assert sm.S4_CLASSIFY not in r.states
    assert [e for e in r.events if e['event'] == 'mission_end'][0]['result'] == 'TRAY_CAMERA_FAILED'
