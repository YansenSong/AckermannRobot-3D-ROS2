"""Robot-side sequential mission executor and JSON rosbridge interface."""

import json
import math
import os
import re
import signal
import time
import uuid
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_status.msg import NavigationStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .store import MissionStore, utc_now
from .scheduler import next_run_at, parse_utc


NAV_TIMEOUT = 600.0
DOCK_TIMEOUT = 120.0
ACTIVE = ("RUNNING", "PAUSED")
REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9._:-]{1,128}$")


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
    map_id = raw.get("map_id")
    map_version_id = raw.get("map_version_id")
    if map_id is not None and (not isinstance(map_id, str) or not map_id.strip() or len(map_id) > 128):
        raise ValueError("mission map_id is invalid")
    if map_version_id is not None and (
        not isinstance(map_version_id, str) or not map_version_id.strip() or len(map_version_id) > 128
    ):
        raise ValueError("mission map_version_id is invalid")
    if (map_id is None) != (map_version_id is None):
        raise ValueError("mission map_id and map_version_id must be provided together")
    return {
        "id": mission_id, "name": name, "steps": result,
        "map_id": map_id.strip() if map_id else None,
        "map_version_id": map_version_id.strip() if map_version_id else None,
    }


def mission_database_path():
    """Resolve MissionStore from explicit configuration or the ROS data root."""
    configured = os.environ.get("ROBOTPILOT_MISSION_DB") or os.environ.get(
        "OPENAMR_MISSION_DB"
    )
    if configured:
        return os.path.expanduser(configured)
    default_ros_home = os.path.join(os.path.expanduser("~"), ".ros")
    ros_home = os.environ.get("ROS_HOME", default_ros_home)
    return os.path.join(os.path.expanduser(ros_home), "ackermann_missions.sqlite3")


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        default_db = mission_database_path()
        self.declare_parameter("database_path", default_db)
        self.robot_id = os.environ.get("ROBOT_ID", "robot-001")
        self.store = MissionStore(self.get_parameter("database_path").value, self.robot_id)
        self.run = self.store.latest_run()
        self.active_request_id = None
        self.pose = None
        self.pose_received_at = None
        self.deadline = None
        self.step_started = None
        self.nav_seen_active = False
        self.goal_sent_ns = None
        self.owned_goals = set()
        self.last_wait_persist = 0.0
        self.stop_owned = False
        self.software_stop_state = None
        self.software_stop_received_at = None
        self.map_identity = None
        self.map_identity_received_at = None
        self.fault_candidates = {}
        self.battery_state = None
        self.battery_received_at = None
        self.area_control = None
        self.area_control_received_at = None
        self.heartbeat_seq = 0
        self.last_sent_seq = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mission/state", qos)
        self.ack_pub = self.create_publisher(String, "/mission/ack", 10)
        self.outbox_pub = self.create_publisher(String, "/robot/events/outbox", 10)
        self.heartbeat_pub = self.create_publisher(String, "/robot/heartbeat", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.hold_pub = self.create_publisher(Bool, "/mission/hold", 10)
        self.dock_pub = self.create_publisher(Bool, "/dock_trigger", 10)
        self.undock_pub = self.create_publisher(Bool, "/undock_robot", 10)
        self.create_subscription(String, "/mission/command", self.on_command, 10)
        self.create_subscription(String, "/robot/events/ack", self.on_outbox_ack, 10)
        self.create_subscription(
            String, "/safety/software_stop/state", self.on_software_stop_state, 10
        )
        self.create_subscription(String, "/ackermann/routes/catalog", self.on_route_catalog, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.create_subscription(String, "/battery/state", self.on_battery_state, 10)
        self.create_subscription(String, "/area_rules/control", self.on_area_control, 10)
        self.create_subscription(NavigationStatus, "/navigation/state", self.on_nav, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(String, "/dock_trigger_status", self.on_dock, 10)
        self.create_subscription(Odometry, "/liorf_localization/mapping/odometry", self.on_odom, 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(2.0, self.publish_state)
        self.create_timer(5.0, self.publish_outbox)
        self.create_timer(2.0, self.publish_heartbeat)
        self.store.recover_claimed_schedules(self.robot_id)
        self.create_timer(1.0, self.scheduler_tick)
        if self.run and (self.run["status"] in ACTIVE or self.run["hold_active"]):
            self._stop_motion()
            if self.run["status"] == "RUNNING":
                self._transition("PAUSED", "mission manager restarted; resume required")
        self.publish_heartbeat()
        self.publish_state()

    def _event(self, reason):
        self.store.add_event(self.run["task_id"], self.run["status"],
                             self.run["step_index"], reason, self.pose,
                             request_id=(
                                 self.active_request_id
                                 or self.run.get("origin_request_id")
                             ))

    def _transition(self, status, reason, **fields):
        self.run = self.store.update_run(
            self.run["task_id"], status=status, reason=reason,
            event_reason=reason, event_pose=self.pose,
            event_request_id=(self.active_request_id or self.run.get("origin_request_id")),
            **fields,
        )
        self.publish_state()

    def publish_outbox(self):
        events = self.store.pending_events(limit=100)
        if events:
            self.last_sent_seq = events[-1]["source_seq"]
            self.outbox_pub.publish(String(data=json.dumps({
                "schema_version": 1, "robot_id": self.robot_id, "events": events,
            }, separators=(",", ":"))))

    def publish_heartbeat(self):
        self.heartbeat_seq += 1
        self.heartbeat_pub.publish(String(data=json.dumps({
            "schema_version": 1, "robot_id": self.robot_id,
            "source": "mission_manager", "heartbeat_seq": self.heartbeat_seq,
            "observed_at": utc_now(),
            "robot_mode": os.environ.get("ROBOT_MODE", "unknown"),
            "network_transport": "simulated" if os.environ.get("ROBOT_MODE") == "simulation" else "unknown",
            "status": "online",
        }, separators=(",", ":"))))

    def on_outbox_ack(self, message):
        try:
            ack = json.loads(message.data)
            if ack.get("schema_version") != 1 or ack.get("robot_id") != self.robot_id:
                return
            if (isinstance(ack.get("through_seq"), bool)
                    or not isinstance(ack.get("through_seq"), int)
                    or ack["through_seq"] > self.last_sent_seq):
                return
            self.store.acknowledge_events(ack.get("through_seq"))
        except (ValueError, TypeError, AttributeError, json.JSONDecodeError):
            return

    def on_diagnostics(self, message):
        now = time.monotonic()
        for item in message.status:
            if not item.name or len(item.name) > 256:
                continue
            level = item.level[0] if isinstance(item.level, (bytes, bytearray)) else int(item.level)
            if level not in (0, 1, 2, 3):
                continue
            if level == 0:
                self.fault_candidates.pop(item.name, None)
                self.store.observe_fault(item.name, 0, item.message)
                continue
            candidate = self.fault_candidates.get(item.name)
            if candidate is None or candidate[0:2] != (level, item.message) or now - candidate[2] > 5:
                self.fault_candidates[item.name] = (level, item.message, now)
                continue
            if now - candidate[2] >= 3:
                self.store.observe_fault(item.name, level, item.message)

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

    def on_software_stop_state(self, message):
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        if not isinstance(state, dict) or not isinstance(state.get("active"), bool):
            return
        self.software_stop_state = state
        self.software_stop_received_at = time.monotonic()

    def on_battery_state(self, message):
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        if not isinstance(state, dict) or not isinstance(state.get("available"), bool):
            return
        self.battery_state = state
        self.battery_received_at = time.monotonic()

    def on_area_control(self, message):
        try:
            state = json.loads(message.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        if not isinstance(state, dict) or not isinstance(state.get("ready"), bool) or not isinstance(state.get("stop"), bool):
            return
        self.area_control = state
        self.area_control_received_at = time.monotonic()

    def motion_guard_block_reason(self):
        if (
            self.software_stop_state is None
            or self.software_stop_received_at is None
            or time.monotonic() - self.software_stop_received_at > 3.0
        ):
            return "software stop state is unavailable or stale"
        if self.software_stop_state.get("result") != "confirmed":
            return "software stop state is not confirmed"
        if not self.software_stop_state.get("durable", False):
            return "software stop state is not durable"
        if self.software_stop_state["active"]:
            return "software stop is active"
        if (
            self.battery_state is None
            or self.battery_received_at is None
            or time.monotonic() - self.battery_received_at > 5.0
        ):
            return "battery state is unavailable or stale"
        if not self.battery_state["available"]:
            return "battery is unavailable"
        if (os.environ.get("ROBOT_MODE") == "hardware"
                and (self.battery_state.get("simulated") is True
                     or self.battery_state.get("source") == "simulated")):
            return "simulated battery cannot authorize hardware motion"
        if self.battery_state.get("low_battery") is not False:
            return "battery is low or its charge level is unknown"
        if self.battery_state.get("charging") is not False:
            return "battery is charging or charge state is unknown"
        if self.area_control_received_at is not None:
            if time.monotonic() - self.area_control_received_at > 3.0:
                return "area control is stale"
            if not self.area_control["ready"] or self.area_control["stop"]:
                return "area control blocks motion"
        if self.store.active_critical_faults():
            return "critical diagnostics are active"
        if self.store.outbox_status()["capacity_warning"]:
            return "robot event outbox is above the safe capacity threshold"
        if (
            self.pose is None
            or self.pose_received_at is None
            or time.monotonic() - self.pose_received_at > 3.0
        ):
            return "localization pose is unavailable or stale"
        return ""

    def on_route_catalog(self, message):
        try:
            active = json.loads(message.data).get("active_files", {})
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        map_id = active.get("map_id")
        map_version_id = active.get("map_version_id") or map_id
        if not isinstance(map_id, str) or not map_id or map_id == "Null":
            return
        if not isinstance(map_version_id, str) or not map_version_id:
            return
        self.map_identity = {"map_id": map_id, "map_version_id": map_version_id}
        self.map_identity_received_at = time.monotonic()

    def map_binding_block_reason(self, map_id, map_version_id):
        if (
            not self.map_identity
            or self.map_identity_received_at is None
            or time.monotonic() - self.map_identity_received_at > 30.0
        ):
            return "current 2D map identity is unavailable or stale"
        if not map_id or not map_version_id:
            return "mission has no bound map identity"
        if map_id != self.map_identity["map_id"]:
            return "mission map_id does not match the current 2D map"
        if map_version_id != self.map_identity["map_version_id"]:
            return "mission map_version_id does not match the current 2D map version"
        return ""

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
                   "event_sync": self.store.outbox_status(),
                   "history": history, "schedules": self.store.schedules(self.robot_id),
                   "schedule_runs": self.store.schedule_runs(self.robot_id, limit=500)}
        self.state_pub.publish(String(data=json.dumps(payload)))

    def _validated_schedule(self, raw, *, created_by, schedule_id=None):
        if not isinstance(raw, dict):
            raise ValueError("schedule must be an object")
        name = str(raw.get("name", "")).strip()
        mission_id = str(raw.get("mission_id", "")).strip()
        if not name or len(name) > 120:
            raise ValueError("schedule name is missing or too long")
        mission = self.store.mission(mission_id)
        if not mission or not mission.get("steps"):
            raise ValueError("schedule mission is missing or has no steps")
        recurrence = raw.get("recurrence")
        timezone_name = raw.get("timezone")
        local_time = raw.get("local_time")
        start_date = raw.get("start_date")
        weekdays = raw.get("weekdays", [])
        enabled = raw.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ValueError("schedule enabled must be a boolean")
        if not isinstance(weekdays, list):
            raise ValueError("schedule weekdays must be a list")
        now = datetime.now(timezone.utc)
        next_run = (
            next_run_at(now, recurrence, timezone_name, local_time, start_date, weekdays)
            if enabled else None
        )
        if enabled and next_run is None:
            raise ValueError("schedule has no future run time")
        return {
            "schedule_id": schedule_id or str(raw.get("schedule_id") or uuid.uuid4()),
            "name": name,
            "mission_id": mission_id,
            "recurrence": recurrence,
            "timezone": timezone_name,
            "local_time": local_time,
            "start_date": start_date,
            "weekdays": weekdays,
            "enabled": enabled,
            "next_run_at": next_run,
            "created_by": created_by,
        }

    def scheduler_tick(self):
        now = datetime.now(timezone.utc)
        now_iso = now.isoformat()
        for schedule in self.store.due_schedules(self.robot_id, now_iso):
            scheduled_for = schedule["next_run_at"]
            following = next_run_at(
                now, schedule["recurrence"], schedule["timezone"], schedule["local_time"],
                schedule.get("start_date"), schedule.get("weekdays"),
            )
            if not self.store.claim_schedule_run(
                self.robot_id, schedule["schedule_id"], scheduled_for, following,
            ):
                continue
            lag = (now - parse_utc(scheduled_for)).total_seconds()
            if lag > 60:
                self.store.finish_schedule_run(
                    self.robot_id, schedule["schedule_id"], scheduled_for,
                    "skipped", "misfire exceeded 60 seconds",
                )
                self.publish_state()
                continue
            reason = self.motion_guard_block_reason()
            mission = self.store.mission(schedule["mission_id"])
            if reason:
                status = "skipped"
            elif self.run and self.run["status"] in ACTIVE:
                reason, status = "another mission is active", "skipped"
            elif not mission:
                reason, status = "scheduled mission no longer exists", "rejected"
            else:
                reason = self.map_binding_block_reason(
                    mission.get("map_id"), mission.get("map_version_id")
                )
                status = "skipped" if reason else "started"
            task_id = None
            if not reason:
                try:
                    task_id = self._start_mission(schedule["mission_id"])
                except ValueError as exc:
                    reason, status = str(exc), "rejected"
            self.store.finish_schedule_run(
                self.robot_id, schedule["schedule_id"], scheduled_for,
                status, reason, task_id,
            )
            self.publish_state()

    def _start_mission(self, mission_id, origin_request_id=None):
        if self.run and self.run["status"] in ACTIVE:
            raise ValueError("a mission is already active")
        mission = self.store.mission(str(mission_id))
        if not mission or not mission["steps"]:
            raise ValueError("mission is missing or has no steps")
        block_reason = self.motion_guard_block_reason()
        if block_reason:
            raise ValueError(block_reason)
        block_reason = self.map_binding_block_reason(
            mission.get("map_id"), mission.get("map_version_id")
        )
        if block_reason:
            raise ValueError(block_reason)
        self.run = self.store.new_run(
            str(uuid.uuid4()), mission, origin_request_id=origin_request_id,
            robot_pose=self.pose, record_start_event=True,
        )
        self._release_motion()
        self.owned_goals.clear()
        self._begin_step()
        return self.run["task_id"]

    def on_command(self, message):
        request_id = None
        origin_request_id = None
        reserved_request_id = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError("command must be an object")
            request_id = command.get("request_id")
            action = command.get("command")
            if action != "query" and request_id is not None:
                if not isinstance(request_id, str) or not REQUEST_ID_RE.fullmatch(request_id):
                    raise ValueError("request_id is invalid")
                is_new, prior_ack = self.store.reserve_command(request_id, command)
                if not is_new:
                    ack = prior_ack or {
                        "request_id": request_id, "ok": False,
                        "error": "command result is unknown after interruption; inspect robot state",
                        "status": "unknown",
                    }
                    self.ack_pub.publish(String(data=json.dumps(ack)))
                    return
                reserved_request_id = request_id
            candidate_origin = command.get("origin_request_id")
            if isinstance(candidate_origin, str) and REQUEST_ID_RE.fullmatch(candidate_origin):
                origin_request_id = candidate_origin
            self.active_request_id = origin_request_id
            if action in {"start", "resume", "retry", "skip", "release_hold"}:
                block_reason = self.motion_guard_block_reason()
                if block_reason:
                    raise ValueError(block_reason)
            if action == "save":
                mission = validated_mission(command.get("mission"))
                if mission.get("map_id") is not None:
                    block_reason = self.map_binding_block_reason(
                        mission.get("map_id"), mission.get("map_version_id")
                    )
                    if block_reason:
                        raise ValueError(block_reason)
                self.store.save_mission(mission)
            elif action == "delete":
                mission_id = str(command.get("mission_id", ""))
                if not self.store.delete_mission(mission_id):
                    raise ValueError("unknown mission")
            elif action == "start":
                self._start_mission(
                    str(command.get("mission_id", "")), origin_request_id
                )
            elif action == "schedule.save":
                schedule_id = command.get("schedule_id")
                existing = self.store.schedule(self.robot_id, schedule_id) if schedule_id else None
                expected_revision = command.get("expected_revision")
                if existing and expected_revision != existing["revision"]:
                    raise ValueError("schedule revision changed")
                schedule = self._validated_schedule(
                    command.get("schedule"),
                    created_by=str(command.get("created_by") or "operator"),
                    schedule_id=schedule_id,
                )
                if existing:
                    result_schedule = self.store.update_schedule(
                        self.robot_id, schedule_id, schedule, expected_revision
                    )
                    if result_schedule is None:
                        raise ValueError("schedule revision changed")
                else:
                    result_schedule = self.store.create_schedule(self.robot_id, schedule)
                command["_schedule_id"] = result_schedule["schedule_id"]
            elif action == "schedule.delete":
                schedule_id = str(command.get("schedule_id", ""))
                expected_revision = command.get("expected_revision")
                existing = self.store.schedule(self.robot_id, schedule_id)
                if not existing:
                    raise ValueError("unknown schedule")
                if expected_revision != existing["revision"]:
                    raise ValueError("schedule revision changed")
                if not self.store.delete_schedule(self.robot_id, schedule_id, expected_revision):
                    raise ValueError("schedule revision changed")
            elif action in ("pause", "resume", "cancel", "retry", "skip", "release_hold"):
                self._control(action, command)
            elif action != "query":
                raise ValueError("unknown command")
            self.publish_state()
            ack = {
                "request_id": request_id,
                "origin_request_id": origin_request_id,
                "ok": True,
            }
            if action == "start":
                ack["task_id"] = self.run["task_id"]
            if action == "schedule.save":
                ack["schedule_id"] = command["_schedule_id"]
            if reserved_request_id:
                self.store.finish_command(reserved_request_id, ack)
            self.ack_pub.publish(String(data=json.dumps(ack)))
        except (ValueError, TypeError, KeyError, OverflowError, json.JSONDecodeError) as exc:
            ack = {"request_id": request_id, "origin_request_id": origin_request_id,
                   "ok": False, "error": str(exc)}
            if reserved_request_id:
                self.store.finish_command(reserved_request_id, ack)
            self.ack_pub.publish(String(data=json.dumps(ack)))
        finally:
            self.active_request_id = None

    def _control(self, action, command):
        if not self.run or command.get("task_id") != self.run["task_id"]:
            raise ValueError("unknown or stale task_id")
        status = self.run["status"]
        if action in ("resume", "retry", "skip"):
            block_reason = self.map_binding_block_reason(
                self.run.get("map_id"), self.run.get("map_version_id")
            )
            if block_reason:
                raise ValueError(block_reason)
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
        if (
            all(math.isfinite(value) for value in values.values())
            and abs(values["z"] ** 2 + values["w"] ** 2 - 1.0) <= 0.1
        ):
            self.pose = values
            self.pose_received_at = time.monotonic()

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
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
