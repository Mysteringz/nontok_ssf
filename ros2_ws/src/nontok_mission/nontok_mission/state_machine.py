"""The self-return mission as a state machine (PLAN.md section 7.1). Pure Python.

It talks to the robot only through a RobotApi object, so the whole mission can be tested
with a fake robot. tick() is called about ten times a second and never waits: long actions
(driving, aligning) are started and then polled.
"""
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

RUNNING, OK, FAILED = 'running', 'ok', 'failed'
TRAY_EMPTY, TRAY_OCCUPIED, TRAY_MOVING = 'empty', 'occupied', 'moving'

S0_DOCKED = 'S0_DOCKED'
S1_UNDOCK = 'S1_UNDOCK'
S2_TO_PICKUP = 'S2_TO_PICKUP'
S2_ALIGN_PICKUP = 'S2_ALIGN_PICKUP'
S3_WAIT_ITEM = 'S3_WAIT_ITEM'
S4_CLASSIFY = 'S4_CLASSIFY'
S4_LEAVE_PICKUP = 'S4_LEAVE_PICKUP'
S5_TO_STATION = 'S5_TO_STATION'
S6_ALIGN = 'S6_ALIGN'
S7_DUMP = 'S7_DUMP'
S7_LEAVE_STATION = 'S7_LEAVE_STATION'
S8_TO_DOCK = 'S8_TO_DOCK'
S9_DOCK = 'S9_DOCK'
FAULT = 'FAULT'

DOCK, PICKUP = 'dock', 'pickup'


class RobotApi:
    """What the mission needs from the robot. The real one is in robot_api.py."""

    # navigation between stations (Nav2)
    def start_navigation(self, station: str) -> None: raise NotImplementedError
    def navigation_result(self) -> str: raise NotImplementedError          # RUNNING | OK | FAILED
    def cancel_navigation(self) -> None: raise NotImplementedError
    # tag alignment (dock server)
    def start_dock(self, station: str, require_charge: bool) -> None: raise NotImplementedError
    def dock_result(self) -> str: raise NotImplementedError
    def cancel_dock(self) -> None: raise NotImplementedError
    # short reverse away from a station or the charger
    def start_retreat(self) -> None: raise NotImplementedError
    def retreat_result(self) -> str: raise NotImplementedError
    # base
    def arm(self, on: bool) -> bool: raise NotImplementedError
    def stop(self) -> None: raise NotImplementedError
    def on_charger(self) -> bool: raise NotImplementedError
    def reset_pose_to_dock(self) -> None: raise NotImplementedError
    def localization_healthy(self) -> bool: raise NotImplementedError
    # tray
    def set_tray(self, angle_deg: float) -> bool: raise NotImplementedError
    def enable_tray_camera(self, on: bool) -> bool: raise NotImplementedError
    def capture_tray_reference(self) -> bool: raise NotImplementedError
    def tray_state(self) -> Tuple[str, bool]: raise NotImplementedError   # (TRAY_*, stable)
    def classify(self) -> Tuple[bool, str, str, float]: raise NotImplementedError   # ok, item, station, confidence
    # people
    def enable_wake(self, on: bool) -> None: raise NotImplementedError
    def wake_requested(self) -> bool: raise NotImplementedError          # true once per call
    def stop_requested(self) -> bool: raise NotImplementedError          # true once per request
    def show(self, text: str) -> None: raise NotImplementedError


@dataclass
class MissionConfig:
    nav_timeout_s: float = 120.0
    nav_attempts: int = 2
    align_timeout_s: float = 45.0
    align_attempts: int = 2
    dock_attempts: int = 3
    retreat_timeout_s: float = 15.0
    item_wait_s: float = 120.0
    classify_retries: int = 2
    classify_retry_wait_s: float = 4.0
    tray_up_deg: float = 50.0
    dump_hold_s: float = 1.5
    dump_settle_s: float = 1.0
    dump_repeats: int = 1
    localization_grace_s: float = 5.0
    dock_requires_charge: bool = False     # set true once the charge-sense wiring exists
    unknown_station: str = ''              # where unidentified items go; '' = carry them back to the dock


class Mission:
    def __init__(self, api: RobotApi, cfg: Optional[MissionConfig] = None,
                 log: Optional[Callable[[dict], None]] = None):
        self.api = api
        self.cfg = cfg or MissionConfig()
        self._log = log or (lambda event: None)
        self.state = S0_DOCKED
        self.detail = ''
        self.target: Optional[str] = None      # station for the item on the tray
        self.item = ''
        self.missions_started = 0
        self._entered = 0.0
        self._fresh = True
        self._attempt = 0
        self._step = ''
        self._step_time = 0.0
        self._count = 0
        self._lost_since: Optional[float] = None
        self._reset_requested = False

    # ---- plumbing ----
    def reset(self) -> None:
        """Operator says the robot is back on the dock (used when no charge sensing is fitted)."""
        self._reset_requested = True

    def _go(self, state: str, now: float, detail: str = '') -> None:
        self._log({'event': 'state', 'from': self.state, 'to': state, 'detail': detail,
                   'target': self.target or '', 'item': self.item})
        self.state, self.detail = state, detail
        self._entered, self._fresh = now, True
        self._attempt, self._step, self._count = 0, '', 0
        self._lost_since = None

    def _fail(self, now: float, reason: str) -> None:
        """Abandon the mission: stop safely and wait for a person."""
        self._go(FAULT, now, reason)

    def _first(self) -> bool:
        """True on the first tick in a state."""
        fresh, self._fresh = self._fresh, False
        return fresh

    def tick(self, now: float) -> None:
        if self.state != FAULT and self.api.stop_requested():
            self._fail(now, 'STOPPED')
        handler = getattr(self, '_' + self.state.lower())
        handler(now)

    # ---- shared behaviours ----
    def _lost(self, now: float) -> bool:
        """True once localisation has been unhealthy for longer than the grace time."""
        if self.api.localization_healthy():
            self._lost_since = None
            return False
        if self._lost_since is None:
            self._lost_since = now
        return now - self._lost_since > self.cfg.localization_grace_s

    def _navigate(self, now: float, station: str) -> str:
        """Drive to a station's approach pose, with retries. RUNNING, OK or FAILED."""
        cfg = self.cfg
        if self._first():
            self._attempt, self._step_time = 1, now
            self.api.show('TO ' + station.upper())
            self.api.start_navigation(station)
            return RUNNING
        if self._lost(now):
            self.api.cancel_navigation()
            self._fail(now, 'LOCALISATION_LOST')
            return RUNNING
        result = self.api.navigation_result()
        if result == RUNNING and now - self._step_time > cfg.nav_timeout_s:
            self.api.cancel_navigation()
            result = FAILED
        if result == FAILED:
            self._log({'event': 'nav_failed', 'station': station, 'attempt': self._attempt})
            if self._attempt >= cfg.nav_attempts:
                return FAILED
            self._attempt += 1
            self._step_time = now
            self.api.start_navigation(station)
            return RUNNING
        return result

    def _align(self, now: float, station: str, attempts: int, require_charge: bool = False) -> str:
        """Tag alignment at a station, with retries. RUNNING, OK or FAILED."""
        if self._first():
            self._attempt, self._step_time = 1, now
            self.api.show('ALIGN ' + station.upper())
            self.api.start_dock(station, require_charge)
            return RUNNING
        result = self.api.dock_result()
        if result == RUNNING and now - self._step_time > self.cfg.align_timeout_s:
            self.api.cancel_dock()
            result = FAILED
        if result == FAILED:
            self._log({'event': 'align_failed', 'station': station, 'attempt': self._attempt})
            if self._attempt >= attempts:
                return FAILED
            self._attempt += 1
            self._step_time = now
            self.api.start_dock(station, require_charge)
            return RUNNING
        return result

    def _retreat(self, now: float) -> str:
        """Reverse a short fixed distance so Nav2 has room to turn. RUNNING, OK or FAILED."""
        if self._first():
            self._step_time = now
            self.api.start_retreat()
            return RUNNING
        result = self.api.retreat_result()
        if result == RUNNING and now - self._step_time > self.cfg.retreat_timeout_s:
            return FAILED
        return result

    # ---- states ----
    def _s0_docked(self, now: float) -> None:
        if self._first():
            self.target, self.item = None, ''
            self.api.arm(False)
            self.api.enable_tray_camera(False)
            self.api.reset_pose_to_dock()
            self.api.enable_wake(True)
            self.api.show('READY')
            return
        if self.api.wake_requested():
            self.missions_started += 1
            self._log({'event': 'mission_start', 'number': self.missions_started})
            self._go(S1_UNDOCK, now)

    def _s1_undock(self, now: float) -> None:
        if self._fresh:
            self.api.enable_wake(False)
            if not self.api.arm(True):
                self._fail(now, 'ARM_FAILED')
                return
        result = self._retreat(now)
        if result == OK:
            self._go(S2_TO_PICKUP, now)
        elif result == FAILED:
            self._fail(now, 'UNDOCK_FAILED')

    def _s2_to_pickup(self, now: float) -> None:
        result = self._navigate(now, PICKUP)
        if result == OK:
            self._go(S2_ALIGN_PICKUP, now)
        elif result == FAILED:
            self._go(S8_TO_DOCK, now, 'PICKUP_UNREACHABLE')

    def _s2_align_pickup(self, now: float) -> None:
        result = self._align(now, PICKUP, self.cfg.align_attempts)
        if result == OK:
            self._go(S3_WAIT_ITEM, now)
        elif result == FAILED:
            self._go(S8_TO_DOCK, now, 'PICKUP_ALIGN_FAILED')

    def _s3_wait_item(self, now: float) -> None:
        if self._first():
            if not (self.api.enable_tray_camera(True) and self.api.capture_tray_reference()):
                self._go(S4_LEAVE_PICKUP, now, 'TRAY_CAMERA_FAILED')
                return
            self.api.show('PLACE ITEM')
            return
        state, stable = self.api.tray_state()
        if state == TRAY_OCCUPIED and stable:
            self._go(S4_CLASSIFY, now)
        elif now - self._entered > self.cfg.item_wait_s:
            self._go(S4_LEAVE_PICKUP, now, 'NO_ITEM')

    def _s4_classify(self, now: float) -> None:
        cfg = self.cfg
        if self._first():
            self._step = 'classify'
        if self._step == 'wait':
            # The person was asked to reposition the item: wait for it to be still again.
            state, stable = self.api.tray_state()
            if state == TRAY_EMPTY and stable:
                self._go(S3_WAIT_ITEM, now, 'ITEM_REMOVED')
                self._fresh = False            # keep the empty-tray reference already taken
                self.api.show('PLACE ITEM')
                return
            if now - self._step_time < cfg.classify_retry_wait_s or not (state == TRAY_OCCUPIED and stable):
                if now - self._step_time > cfg.item_wait_s:
                    self._unknown_item(now)
                return
            self._step = 'classify'
        ok, item, station, confidence = self.api.classify()
        self._log({'event': 'classified', 'ok': ok, 'item': item, 'station': station,
                   'confidence': round(float(confidence), 3), 'try': self._count + 1})
        if ok:
            self.target, self.item = station, item
            self._go(S4_LEAVE_PICKUP, now, item)
            return
        self._count += 1
        if self._count > cfg.classify_retries:
            self._unknown_item(now)   # never guess: manual-sorting station, or home
            return
        self.api.show('REPOSITION ITEM')
        self._step, self._step_time = 'wait', now

    def _unknown_item(self, now: float) -> None:
        """Could not name the item: take it to the manual-sorting station if there is one."""
        if self.cfg.unknown_station:
            self.target, self.item = self.cfg.unknown_station, 'unknown'
        self._go(S4_LEAVE_PICKUP, now, 'UNKNOWN_ITEM')

    def _s4_leave_pickup(self, now: float) -> None:
        if self._fresh:
            self.api.enable_tray_camera(False)
        result = self._retreat(now)
        if result == OK:
            self._go(S5_TO_STATION if self.target else S8_TO_DOCK, now, self.detail)
        elif result == FAILED:
            self._fail(now, 'RETREAT_FAILED')

    def _s5_to_station(self, now: float) -> None:
        result = self._navigate(now, self.target)
        if result == OK:
            self._go(S6_ALIGN, now)
        elif result == FAILED:
            self._fail(now, 'STATION_UNREACHABLE')

    def _s6_align(self, now: float) -> None:
        result = self._align(now, self.target, self.cfg.align_attempts)
        if result == OK:
            self._go(S7_DUMP, now)
        elif result == FAILED:
            self._fail(now, 'STATION_ALIGN_FAILED')

    def _s7_dump(self, now: float) -> None:
        cfg = self.cfg
        if self._first():
            self.api.show('DUMP ' + (self.target or '').upper())
            self.api.enable_tray_camera(True)
            self._step = 'up'
        if self._step == 'up':
            if not self.api.set_tray(cfg.tray_up_deg):
                self.api.set_tray(0.0)
                self._fail(now, 'TRAY_FAILED')
                return
            self._step, self._step_time = 'hold', now
        elif self._step == 'hold' and now - self._step_time >= cfg.dump_hold_s:
            if not self.api.set_tray(0.0):
                self._fail(now, 'TRAY_FAILED')
                return
            self._step, self._step_time = 'settle', now
        elif self._step == 'settle' and now - self._step_time >= cfg.dump_settle_s:
            state, _ = self.api.tray_state()
            if state == TRAY_EMPTY:
                self._log({'event': 'dumped', 'item': self.item, 'station': self.target, 'repeats': self._count})
                self._go(S7_LEAVE_STATION, now)
            elif self._count < cfg.dump_repeats:
                self._count += 1
                self._step = 'up'
            else:
                self._log({'event': 'dump_incomplete', 'item': self.item, 'station': self.target})
                self._go(S7_LEAVE_STATION, now, 'ITEM_STUCK')

    def _s7_leave_station(self, now: float) -> None:
        if self._fresh:
            self.api.enable_tray_camera(False)
        result = self._retreat(now)
        if result == OK:
            self._go(S8_TO_DOCK, now, self.detail)
        elif result == FAILED:
            self._fail(now, 'RETREAT_FAILED')

    def _s8_to_dock(self, now: float) -> None:
        result = self._navigate(now, DOCK)
        if result == OK:
            self._go(S9_DOCK, now, self.detail)
        elif result == FAILED:
            self._fail(now, 'DOCK_UNREACHABLE')

    def _s9_dock(self, now: float) -> None:
        result = self._align(now, DOCK, self.cfg.dock_attempts, self.cfg.dock_requires_charge)
        if result == OK:
            self._log({'event': 'mission_end', 'result': self.detail or 'OK', 'item': self.item,
                       'station': self.target or ''})
            self._go(S0_DOCKED, now)
        elif result == FAILED:
            self._fail(now, 'DOCKING_FAILED')

    def _fault(self, now: float) -> None:
        if self._first():
            self.api.cancel_navigation()
            self.api.cancel_dock()
            self.api.stop()
            self.api.arm(False)
            self.api.enable_tray_camera(False)
            self.api.set_tray(0.0)
            self.api.show('FAULT ' + self.detail[:10])
            self._log({'event': 'fault', 'reason': self.detail, 'item': self.item, 'station': self.target or ''})
            self._reset_requested = False
            return
        # The operator carries the robot back to the dock: charging, or a manual reset, restarts it.
        if self._reset_requested or self.api.on_charger():
            self._reset_requested = False
            self._go(S0_DOCKED, now)

