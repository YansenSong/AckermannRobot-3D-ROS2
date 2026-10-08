"""Robot-side sequential mission executor and JSON rosbridge interface."""

import json
import math
import os
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_status.msg import NavigationStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .store import MissionStore, utc_now


NAV_TIMEOUT = 600.0
DOCK_TIMEOUT = 120.0
ACTIVE = ("RUNNING", "PAUSED")


def validated_mission(raw):
    if not isinstance(raw, dict):
        raise ValueError("mission must be an object")
    mission_id = str(raw.get("id", "")).strip()
    name = str(raw.get("name", "")).strip()
    steps = raw.get("steps")
    if not mission_id or len(mission_id) > 100 or not name or len(name) > 120:
        raise ValueError("mission id/name is missing or too long")
    if not isinstance(steps, list) or len(steps) > 200:
        raise ValueError("mission steps must be a list of at most 200 items")
    result = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            raise ValueError(f"step {index + 1} must be an object")
        kind = step.get("type")
        item = {"id": str(step.get("id") or uuid.uuid4()), "type": kind}
        if kind == "waypoint":
            pose = step.get("pose")
            if not isinstance(pose, dict):
                raise ValueError(f"step {index + 1} has no waypoint pose")
            values = {key: float(pose.get(key, default)) for key, default in
                      (("x", None), ("y", None), ("z", 0), ("w", 1))}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError(f"step {index + 1} has an invalid pose")
            if abs(values["z"] ** 2 + values["w"] ** 2 - 1) > 0.1:
                raise ValueError(f"step {index + 1} has an invalid orientation")
            item.update(pose=values, waypointId=step.get("waypointId"),
                        waypointName=str(step.get("waypointName", ""))[:120])
        elif kind == "wait":
            seconds = float(step.get("seconds", 0))
            if not math.isfinite(seconds) or seconds <= 0 or seconds > 86400:
                raise ValueError(f"step {index + 1} has invalid wait seconds")
            item["seconds"] = seconds
        elif kind not in ("home", "dock", "undock"):
            raise ValueError(f"step {index + 1} has unknown type")
        result.append(item)
    return {"id": mission_id, "name": name, "steps": result}


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        default_db = os.path.join(os.path.expanduser("~"), ".ros", "ackermann_missions.sqlite3")
        self.declare_parameter("database_path", default_db)
        self.store = MissionStore(self.get_parameter("database_path").value)
        self.run = self.store.latest_run()
        self.pose = None
        self.deadline = None
        self.step_started = None
        self.nav_seen_active = False
        self.goal_sent_ns = None
        self.owned_goals = set()
        self.last_wait_persist = 0.0
        self.stop_owned = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mission/state", qos)
        self.ack_pub = self.create_publisher(String, "/mission/ack", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.hold_pub = self.create_publisher(Bool, "/mission/hold", 10)
        self.dock_pub = self.create_publisher(Bool, "/dock_trigger", 10)
        self.undock_pub = self.create_publisher(Bool, "/undock_robot", 10)
        self.create_subscription(String, "/mission/command", self.on_command, 10)
        self.create_subscription(NavigationStatus, "/navigation/state", self.on_nav, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(String, "/dock_trigger_status", self.on_dock, 10)
        self.create_subscription(Odometry, "/liorf_localization/mapping/odometry", self.on_odom, 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(2.0, self.publish_state)
        if self.run and (self.run["status"] in ACTIVE or self.run["hold_active"]):
            self._stop_motion()
            if self.run["status"] == "RUNNING":
                self._transition("PAUSED", "mission manager restarted; resume required")
        self.publish_state()

    def _event(self, reason):
        self.store.add_event(self.run["task_id"], self.run["status"],
                             self.run["step_index"], reason, self.pose)

    def _transition(self, status, reason, **fields):
        self.run = self.store.update_run(self.run["task_id"], status=status, reason=reason, **fields)
        self._event(reason)
        self.publish_state()

    def _stop_motion(self):
        self.hold_pub.publish(Bool(data=True))
        self.stop_owned = True
        if self.run and not self.run["hold_active"]:
            self.run = self.store.update_run(self.run["task_id"], hold_active=True)

    def _release_motion(self):
        if self.stop_owned:
            self.hold_pub.publish(Bool(data=False))
            self.stop_owned = False
            if self.run and self.run["hold_active"]:
                self.run = self.store.update_run(self.run["task_id"], hold_active=False)

    def publish_state(self):
        run = dict(self.run) if self.run else None
        if run:
            run["events"] = self.store.events(run["task_id"])
            run["missionId"] = run["mission_id"]
            run["stepIndex"] = run["step_index"]
            run["taskId"] = run["task_id"]
            run["status"] = run["status"].lower()
        history = self.store.recent_runs(10)
        for item in history:
            item["events"] = self.store.events(item["task_id"], limit=50)
        payload = {"schema_version": 1, "online": True, "server_time": utc_now(),
                   "missions": self.store.missions(), "run": run,
                   "history": history}
        self.state_pub.publish(String(data=json.dumps(payload)))

    def on_command(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError("command must be an object")
            request_id = command.get("request_id")
            action = command.get("command")
            if action == "save":
                mission = validated_mission(command.get("mission"))
                self.store.save_mission(mission)
            elif action == "delete":
                mission_id = str(command.get("mission_id", ""))
                if not self.store.delete_mission(mission_id):
                    raise ValueError("unknown mission")
            elif action == "start":
                if self.run and self.run["status"] in ACTIVE:
                    raise ValueError("a mission is already active")
                mission = self.store.mission(str(command.get("mission_id", "")))
                if not mission or not mission["steps"]:
                    raise ValueError("mission is missing or has no steps")
                self._release_motion()
                self.run = self.store.new_run(str(uuid.uuid4()), mission)
                self.owned_goals.clear()
                self._event("task started")
                self._begin_step()
            elif action in ("pause", "resume", "cancel", "retry", "skip", "release_hold"):
                self._control(action, command)
            elif action != "query":
                raise ValueError("unknown command")
            self.publish_state()
            ack = {"request_id": request_id, "ok": True}
            if action == "start":
                ack["task_id"] = self.run["task_id"]
            self.ack_pub.publish(String(data=json.dumps(ack)))
        except (ValueError, TypeError, KeyError, OverflowError, json.JSONDecodeError) as exc:
            self.ack_pub.publish(String(data=json.dumps({"request_id": request_id,
                                                        "ok": False, "error": str(exc)})))

    def _control(self, action, command):
        if not self.run or command.get("task_id") != self.run["task_id"]:
            raise ValueError("unknown or stale task_id")
        status = self.run["status"]
        if action == "pause" and status == "RUNNING":
            remaining = max(0.0, self.deadline - time.monotonic()) if self.deadline and self._kind() == "wait" else 0.0
            self._stop_motion()
            self.deadline = None
            self._transition("PAUSED", "paused by operator", remaining_seconds=remaining)
        elif action == "resume" and status == "PAUSED":
            self._release_motion()
            self._transition("RUNNING", "resumed by operator")
            self._begin_step(resume=True)
        elif action == "cancel" and status in ACTIVE:
            self._stop_motion()
            self.deadline = None
            self._transition("CANCELLED", "cancelled by operator")
        elif action == "retry" and status == "FAILED":
            self._release_motion()
            self._transition("RUNNING", "retry requested", attempt=self.run["attempt"] + 1)
            self._begin_step()
        elif action == "skip" and status in ("RUNNING", "PAUSED", "FAILED"):
            was_paused = status == "PAUSED"
            self._stop_motion()
            self.deadline = None
            self._event("step skipped by operator")
            if self.run["step_index"] + 1 >= len(self.run["steps"]):
                self._transition("SUCCEEDED", "last step skipped")
            else:
                self._transition("PAUSED" if was_paused else "RUNNING", "next step",
                                 step_index=self.run["step_index"] + 1,
                                 remaining_seconds=0.0)
                if not was_paused:
                    self._release_motion()
                    self._begin_step()
        elif action == "release_hold" and status in ("SUCCEEDED", "FAILED", "CANCELLED") and self.run["hold_active"]:
            self._release_motion()
            self._event("drive hold released by operator")
        else:
            raise ValueError(f"cannot {action} a {status.lower()} task")

    def _kind(self):
        return self.run["steps"][self.run["step_index"]]["type"]

    def _begin_step(self, resume=False):
        step = self.run["steps"][self.run["step_index"]]
        kind = step["type"]
        self.step_started = time.monotonic()
        self.nav_seen_active = False
        self.goal_sent_ns = None
        self._event(f"step {self.run['step_index'] + 1} started: {kind}")
        if kind == "wait":
            seconds = self.run["remaining_seconds"] if resume else step["seconds"]
            self.deadline = time.monotonic() + seconds
            self.last_wait_persist = 0.0
            self.run = self.store.update_run(self.run["task_id"], remaining_seconds=seconds)
        elif kind in ("waypoint", "home"):
            pose = step["pose"] if kind == "waypoint" else {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_sent_ns = goal.header.stamp.sec * 1000000000 + goal.header.stamp.nanosec
            goal.pose.position.x = pose["x"]
            goal.pose.position.y = pose["y"]
            goal.pose.orientation.z = pose["z"]
            goal.pose.orientation.w = pose["w"]
            self.owned_goals.add(self._goal_key(goal))
            self.deadline = time.monotonic() + NAV_TIMEOUT
            self.goal_pub.publish(goal)
        else:
            self.deadline = time.monotonic() + DOCK_TIMEOUT
            (self.dock_pub if kind == "dock" else self.undock_pub).publish(Bool(data=True))
        self.publish_state()

    def _complete_step(self):
        self._event(f"step {self.run['step_index'] + 1} completed")
        self.deadline = None
        if self.run["step_index"] + 1 >= len(self.run["steps"]):
            self._transition("SUCCEEDED", "all steps completed", remaining_seconds=0.0)
            return
        self._transition("RUNNING", "next step", step_index=self.run["step_index"] + 1,
                         remaining_seconds=0.0)
        self._begin_step()

    def _fail(self, reason):
        self._stop_motion()
        self.deadline = None
        self._transition("FAILED", reason)

    def tick(self):
        if not self.run or self.run["status"] != "RUNNING" or self.deadline is None:
            return
        if self._kind() == "wait" and time.monotonic() - self.last_wait_persist >= 1.0:
            self.run = self.store.update_run(
                self.run["task_id"], remaining_seconds=max(0.0, self.deadline - time.monotonic()))
            self.last_wait_persist = time.monotonic()
        if time.monotonic() >= self.deadline:
            if self._kind() == "wait":
                self._complete_step()
            else:
                self._fail(f"{self._kind()} timed out")

    def on_goal(self, message):
        if not self.run or self.run["status"] != "RUNNING" or self.deadline is None:
            return
        if self._goal_key(message) not in self.owned_goals:
            self._fail("external navigation goal interrupted mission")

    @staticmethod
    def _goal_key(message):
        p = message.pose
        s = message.header.stamp
        return (s.sec, s.nanosec, p.position.x, p.position.y,
                p.orientation.z, p.orientation.w)

    def on_nav(self, message):
        if not self.run or self.run["status"] != "RUNNING" or self.deadline is None or self._kind() not in ("waypoint", "home"):
            return
        stamp_ns = message.stamp.sec * 1000000000 + message.stamp.nanosec
        if self.goal_sent_ns is None or stamp_ns < self.goal_sent_ns:
            return
        if message.state == NavigationStatus.PLANNING and message.detail.startswith("new goal received"):
            self.nav_seen_active = True
        elif self.nav_seen_active and message.state == NavigationStatus.ARRIVED:
            self._complete_step()
        elif self.nav_seen_active and message.state == NavigationStatus.FAILED:
            self._fail(message.detail or "navigation failed")

    def on_dock(self, message):
        if not self.run or self.run["status"] != "RUNNING" or self.deadline is None or self._kind() not in ("dock", "undock"):
            return
        status = message.data.lower()
        if status == ("docked" if self._kind() == "dock" else "idle"):
            self._complete_step()
        elif status == "failed":
            self._fail("dock controller reported failure")

    def on_odom(self, message):
        p = message.pose.pose
        values = {"x": p.position.x, "y": p.position.y,
                  "z": p.orientation.z, "w": p.orientation.w}
        if all(math.isfinite(value) for value in values.values()):
            self.pose = values

    def destroy_node(self):
        if self.run and self.run["status"] == "RUNNING":
            self._stop_motion()
            self._transition("PAUSED", "mission manager stopped; resume required")
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
