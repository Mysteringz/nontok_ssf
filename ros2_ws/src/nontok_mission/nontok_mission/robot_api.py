"""The real RobotApi: connects the mission to Nav2, the dock server, the base, the tray and
the wake word (PLAN.md section 13.12)."""
import math
import os
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import BaseStatus, LocalizationStatus, TrayState
from nontok_interfaces.srv import ClassifyItem, SetTray
from nontok_localization.tag_geometry import TagMap
from nontok_navigation.stations import approach_pose, load_stations

from .state_machine import FAILED, OK, RUNNING, TRAY_EMPTY, TRAY_MOVING, TRAY_OCCUPIED, RobotApi


class MissionIo(Node):
    """Topics, services and actions of the mission. Spun in a background thread."""

    def __init__(self):
        super().__init__('mission_node')
        self.status = None
        self.localization = None
        self.tray = None
        self.wake_flag = False
        self.stop_flag = False
        self.reset_flag = False
        self.create_subscription(BaseStatus, 'base/status', lambda m: setattr(self, 'status', m), 5)
        self.create_subscription(LocalizationStatus, 'localization/status',
                                 lambda m: setattr(self, 'localization', m), 5)
        self.create_subscription(TrayState, 'tray/state', lambda m: setattr(self, 'tray', m), 5)
        self.create_subscription(Empty, 'wake', lambda m: setattr(self, 'wake_flag', True), 5)
        self.oled = self.create_publisher(String, 'base/oled', 5)
        self.create_service(Trigger, 'mission/stop', self.on_stop)
        self.create_service(Trigger, 'mission/reset', self.on_reset)
        self.arm_client = self.create_client(SetBool, 'base/arm')
        self.stop_client = self.create_client(Trigger, 'base/stop')
        self.tray_client = self.create_client(SetTray, 'base/set_tray')
        self.tray_enable = self.create_client(SetBool, 'tray/enable')
        self.tray_reference = self.create_client(Trigger, 'tray/capture_reference')
        self.classify_client = self.create_client(ClassifyItem, 'tray/classify')
        self.voice_enable = self.create_client(SetBool, 'voice/enable')
        self.reset_pose = self.create_client(Trigger, 'localization/reset_to_dock')
        self.undock_client = self.create_client(Trigger, 'undock')
        self.dock_client = ActionClient(self, DockToTags, 'dock_to_tags')

    def on_stop(self, request, response):
        self.stop_flag = True
        response.success = True
        return response

    def on_reset(self, request, response):
        self.reset_flag = True
        response.success = True
        return response

    def call(self, client, request, timeout_s: float = 3.0):
        """Blocking service call from the mission thread; None on timeout or missing service."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service %s is not available' % client.srv_name)
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda f: done.set())
        if not done.wait(timeout_s):
            self.get_logger().warn('Service %s timed out' % client.srv_name)
            return None
        return future.result()


class RosRobotApi(RobotApi):
    def __init__(self, io: MissionIo, navigator: BasicNavigator, charge_threshold_v: float = 4.0):
        self.io = io
        self.navigator = navigator
        self.charge_threshold_v = charge_threshold_v
        loc = get_package_share_directory('nontok_localization')
        nav = get_package_share_directory('nontok_navigation')
        self.tag_map = TagMap.load(os.path.join(loc, 'config', 'tags.yaml'))
        self.stations = load_stations(os.path.join(nav, 'config', 'stations.yaml'))
        self._dock_state = FAILED
        self._dock_handle = None
        self._retreat_state = FAILED
        self._nav_active = False

    # ---- navigation ----
    def start_navigation(self, station: str) -> None:
        x, y, yaw = approach_pose(self.stations[station], self.tag_map)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self._nav_active = bool(self.navigator.goToPose(goal))

    def navigation_result(self) -> str:
        if not self._nav_active:
            return FAILED
        if not self.navigator.isTaskComplete():
            return RUNNING
        self._nav_active = False
        return OK if self.navigator.getResult() == TaskResult.SUCCEEDED else FAILED

    def cancel_navigation(self) -> None:
        if self._nav_active:
            self.navigator.cancelTask()
            self._nav_active = False

    # ---- alignment ----
    def start_dock(self, station: str, require_charge: bool) -> None:
        self._dock_state, self._dock_handle = RUNNING, None
        if not self.io.dock_client.wait_for_server(timeout_sec=2.0):
            self._dock_state = FAILED
            return
        goal = DockToTags.Goal(station=station, require_charge=require_charge)
        self.io.dock_client.send_goal_async(goal).add_done_callback(self._on_dock_accepted)

    def _on_dock_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._dock_state = FAILED
            return
        self._dock_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: setattr(self, '_dock_state', OK if f.result().result.success else FAILED))

    def dock_result(self) -> str:
        return self._dock_state

    def cancel_dock(self) -> None:
        if self._dock_handle is not None and self._dock_state == RUNNING:
            self._dock_handle.cancel_goal_async()
        self._dock_state = FAILED

    # ---- retreat ----
    def start_retreat(self) -> None:
        self._retreat_state = RUNNING
        if not self.io.undock_client.wait_for_service(timeout_sec=2.0):
            self._retreat_state = FAILED
            return
        self.io.undock_client.call_async(Trigger.Request()).add_done_callback(
            lambda f: setattr(self, '_retreat_state', OK if f.result().success else FAILED))

    def retreat_result(self) -> str:
        return self._retreat_state

    # ---- base ----
    def arm(self, on: bool) -> bool:
        reply = self.io.call(self.io.arm_client, SetBool.Request(data=on))
        return bool(reply and reply.success)

    def stop(self) -> None:
        self.io.call(self.io.stop_client, Trigger.Request())

    def on_charger(self) -> bool:
        return self.io.status is not None and self.io.status.charge_v >= self.charge_threshold_v

    def reset_pose_to_dock(self) -> None:
        self.io.call(self.io.reset_pose, Trigger.Request())

    def localization_healthy(self) -> bool:
        return self.io.localization is not None and self.io.localization.healthy

    # ---- tray ----
    def set_tray(self, angle_deg: float) -> bool:
        reply = self.io.call(self.io.tray_client, SetTray.Request(angle_deg=float(angle_deg)), timeout_s=7.0)
        return bool(reply and reply.success)

    def enable_tray_camera(self, on: bool) -> bool:
        self.io.tray = None
        reply = self.io.call(self.io.tray_enable, SetBool.Request(data=on), timeout_s=5.0)
        return bool(reply and reply.success)

    def capture_tray_reference(self) -> bool:
        reply = self.io.call(self.io.tray_reference, Trigger.Request(), timeout_s=6.0)
        return bool(reply and reply.success)

    def tray_state(self):
        tray = self.io.tray
        if tray is None or not tray.has_reference:
            return TRAY_MOVING, False      # no information yet: neither "empty" nor "occupied"
        name = {TrayState.EMPTY: TRAY_EMPTY, TrayState.OCCUPIED: TRAY_OCCUPIED}.get(tray.state, TRAY_MOVING)
        return name, bool(tray.stable)

    def classify(self):
        reply = self.io.call(self.io.classify_client, ClassifyItem.Request(), timeout_s=10.0)
        if reply is None:
            return False, '', '', 0.0
        return bool(reply.success and reply.station in self.stations), reply.item, reply.station, reply.confidence

    # ---- people ----
    def enable_wake(self, on: bool) -> None:
        self.io.wake_flag = False
        if self.io.voice_enable.service_is_ready():
            self.io.voice_enable.call_async(SetBool.Request(data=on))

    def wake_requested(self) -> bool:
        heard, self.io.wake_flag = self.io.wake_flag, False
        return heard

    def stop_requested(self) -> bool:
        flag, self.io.stop_flag = self.io.stop_flag, False
        return flag

    def show(self, text: str) -> None:
        self.io.oled.publish(String(data=text))
