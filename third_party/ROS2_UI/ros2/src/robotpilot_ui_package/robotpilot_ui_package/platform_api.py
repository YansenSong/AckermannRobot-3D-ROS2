"""Versioned robot API backed by the robot-side mission manager."""

import copy
import base64
import binascii
import hashlib
import json
import math
import os
import sqlite3
import threading
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from flask import Response, abort, g, jsonify, request, send_file, stream_with_context
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

try:
    from nav_status.msg import NavigationStatus
except ImportError:
    NavigationStatus = None

from .data_paths import setting
from .auth import COOKIE_NAME, require_role


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def json_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class PlatformStore:
    """Audit, faults, and platform settings; mission execution stays in ROS."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, mode=0o700, exist_ok=True)
        if not os.path.exists(self.path):
            try:
                descriptor = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
                os.close(descriptor)
            except FileExistsError:
                pass
        os.chmod(self.path, 0o600)
        with self.connect() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS commands (
                    command_id TEXT PRIMARY KEY,
                    idempotency_key TEXT NOT NULL UNIQUE,
                    body_hash TEXT NOT NULL,
                    actor TEXT NOT NULL,
                    robot_id TEXT NOT NULL,
                    action TEXT NOT NULL,
                    status TEXT NOT NULL,
                    result_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS faults (
                    fault_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    fault_code TEXT NOT NULL,
                    fault_source TEXT NOT NULL,
                    description TEXT NOT NULL,
                    severity TEXT NOT NULL,
                    first_seen TEXT NOT NULL,
                    last_seen TEXT NOT NULL,
                    resolved_at TEXT,
                    acknowledged_by TEXT,
                    acknowledged_at TEXT,
                    cleared_at TEXT
                );
                CREATE INDEX IF NOT EXISTS faults_robot_time ON faults(robot_id, first_seen DESC);
                CREATE TABLE IF NOT EXISTS config_versions (
                    version INTEGER PRIMARY KEY AUTOINCREMENT,
                    config_json TEXT NOT NULL,
                    active INTEGER NOT NULL DEFAULT 0,
                    actor TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS audit_entries (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    request_id TEXT NOT NULL,
                    actor TEXT NOT NULL,
                    robot_id TEXT,
                    method TEXT NOT NULL,
                    path TEXT NOT NULL,
                    status_code INTEGER NOT NULL,
                    timestamp TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS audit_entries_time ON audit_entries(timestamp DESC);
                CREATE TABLE IF NOT EXISTS platform_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    fingerprint TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    timestamp TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS platform_events_robot_id ON platform_events(robot_id, id);
            """)
            if db.execute("SELECT COUNT(*) FROM config_versions").fetchone()[0] == 0:
                db.execute(
                    "INSERT INTO config_versions(config_json, active, actor, created_at) VALUES (?, 1, ?, ?)",
                    (json.dumps({"low_battery_threshold": 20}), "system", utc_now()),
                )

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.row_factory = sqlite3.Row
        try:
            yield db
            db.commit()
        except Exception:
            db.rollback()
            raise
        finally:
            db.close()

    def audit(self, request_id, actor, robot_id, method, path, status_code):
        with self.connect() as db:
            db.execute(
                """INSERT INTO audit_entries(request_id, actor, robot_id, method, path,
                   status_code, timestamp) VALUES (?, ?, ?, ?, ?, ?, ?)""",
                (request_id, actor, robot_id, method, path, status_code, utc_now()),
            )

    def append_state_event(self, robot_id, payload):
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        fingerprint_payload = dict(payload)
        fingerprint_payload.pop("observed_at", None)
        fingerprint = hashlib.sha256(json.dumps(
            fingerprint_payload, sort_keys=True, separators=(",", ":")
        ).encode()).hexdigest()
        with self.connect() as db:
            latest = db.execute(
                "SELECT fingerprint FROM platform_events WHERE robot_id = ? ORDER BY id DESC LIMIT 1",
                (robot_id,),
            ).fetchone()
            if latest and latest["fingerprint"] == fingerprint:
                return None
            cursor = db.execute(
                "INSERT INTO platform_events(robot_id, event_type, fingerprint, payload_json, timestamp) "
                "VALUES (?, 'robot_state', ?, ?, ?)",
                (robot_id, fingerprint, encoded, utc_now()),
            )
            db.execute(
                "DELETE FROM platform_events WHERE robot_id = ? AND id NOT IN "
                "(SELECT id FROM platform_events WHERE robot_id = ? ORDER BY id DESC LIMIT 10000)",
                (robot_id, robot_id),
            )
            return cursor.lastrowid

    def events_after(self, robot_id, event_id=0):
        with self.connect() as db:
            return [dict(row) for row in db.execute(
                "SELECT id, event_type, payload_json FROM platform_events "
                "WHERE robot_id = ? AND id > ? ORDER BY id ASC LIMIT 1000",
                (robot_id, event_id),
            )]

    def latest_event_id(self, robot_id):
        with self.connect() as db:
            row = db.execute(
                "SELECT MAX(id) AS event_id FROM platform_events WHERE robot_id = ?", (robot_id,)
            ).fetchone()
            return int(row["event_id"] or 0)

    def reserve_command(self, key, body_hash, actor, robot_id, action):
        with self.connect() as db:
            command_id = str(uuid.uuid4())
            now = utc_now()
            result = {"command_id": command_id, "status": "pending", "request_id": command_id}
            inserted = db.execute(
                """INSERT OR IGNORE INTO commands(command_id, idempotency_key, body_hash, actor, robot_id,
                   action, status, result_json, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (command_id, key, body_hash, actor, robot_id, action, "pending", json.dumps(result), now, now),
            )
            row = dict(db.execute("SELECT * FROM commands WHERE idempotency_key = ?", (key,)).fetchone())
            if row["body_hash"] != body_hash or row["actor"] != actor or row["robot_id"] != robot_id:
                raise ValueError("idempotency key was used for a different request")
            return row, inserted.rowcount == 1

    def finish_command(self, command_id, status, result):
        with self.connect() as db:
            db.execute(
                "UPDATE commands SET status = ?, result_json = ?, updated_at = ? WHERE command_id = ?",
                (status, json.dumps(result), utc_now(), command_id),
            )

    def command(self, command_id, robot_id):
        with self.connect() as db:
            row = db.execute(
                "SELECT * FROM commands WHERE command_id = ? AND robot_id = ?", (command_id, robot_id)
            ).fetchone()
            return dict(row) if row else None

    def observe_fault(self, robot_id, name, message, level):
        code = "DIAG-" + hashlib.sha1(name.encode()).hexdigest()[:12].upper()
        now = utc_now()
        with self.connect() as db:
            active = db.execute(
                """SELECT fault_id FROM faults WHERE robot_id = ? AND fault_code = ?
                   AND fault_source = ? AND resolved_at IS NULL ORDER BY first_seen DESC LIMIT 1""",
                (robot_id, code, name),
            ).fetchone()
            if level == 0:
                if active:
                    db.execute("UPDATE faults SET resolved_at = ?, last_seen = ? WHERE fault_id = ?",
                               (now, now, active["fault_id"]))
                return
            severity = {1: "WARNING", 2: "ERROR", 3: "ERROR"}.get(level, "WARNING")
            if active:
                db.execute(
                    "UPDATE faults SET description = ?, severity = ?, last_seen = ? WHERE fault_id = ?",
                    (message, severity, now, active["fault_id"]),
                )
            else:
                db.execute(
                    """INSERT INTO faults(fault_id, robot_id, fault_code, fault_source,
                       description, severity, first_seen, last_seen) VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                    (str(uuid.uuid4()), robot_id, code, name, message, severity, now, now),
                )

    def faults(self, robot_id, active_only=False, limit=100):
        with self.connect() as db:
            query = "SELECT * FROM faults WHERE robot_id = ?"
            if active_only:
                query += " AND resolved_at IS NULL AND cleared_at IS NULL"
            query += " ORDER BY first_seen DESC LIMIT ?"
            return [dict(row) for row in db.execute(query, (robot_id, limit))]

    def fault(self, robot_id, fault_id):
        with self.connect() as db:
            row = db.execute("SELECT * FROM faults WHERE robot_id = ? AND fault_id = ?", (robot_id, fault_id)).fetchone()
            return dict(row) if row else None

    def ack_fault(self, robot_id, fault_id, actor):
        with self.connect() as db:
            db.execute(
                "UPDATE faults SET acknowledged_by = ?, acknowledged_at = ? WHERE robot_id = ? AND fault_id = ?",
                (actor, utc_now(), robot_id, fault_id),
            )

    def clear_fault(self, robot_id, fault_id):
        with self.connect() as db:
            db.execute(
                "UPDATE faults SET cleared_at = ? WHERE robot_id = ? AND fault_id = ? AND resolved_at IS NOT NULL",
                (utc_now(), robot_id, fault_id),
            )

    def config(self, version=None):
        with self.connect() as db:
            row = db.execute(
                "SELECT * FROM config_versions WHERE version = ?" if version else
                "SELECT * FROM config_versions WHERE active = 1 ORDER BY version DESC LIMIT 1",
                (version,) if version else (),
            ).fetchone()
            if not row:
                return None
            item = dict(row)
            item["config"] = json.loads(item.pop("config_json"))
            return item

    def config_versions(self):
        with self.connect() as db:
            return [dict(row) for row in db.execute(
                "SELECT version, active, actor, created_at FROM config_versions ORDER BY version DESC LIMIT 100"
            )]

    def create_config(self, config, actor, expected_version):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            latest = db.execute("SELECT MAX(version) FROM config_versions").fetchone()[0]
            if latest != expected_version:
                raise ValueError("config version changed")
            cursor = db.execute(
                "INSERT INTO config_versions(config_json, actor, created_at) VALUES (?, ?, ?)",
                (json.dumps(config), actor, utc_now()),
            )
            return cursor.lastrowid

    def activate_config(self, version):
        with self.connect() as db:
            row = db.execute("SELECT version FROM config_versions WHERE version = ?", (version,)).fetchone()
            if not row:
                return False
            db.execute("UPDATE config_versions SET active = 0 WHERE active = 1")
            db.execute("UPDATE config_versions SET active = 1 WHERE version = ?", (version,))
            return True


class RobotBridge(Node):
    def __init__(self, store, robot_id):
        super().__init__("ui_platform_bridge")
        self.store = store
        self.robot_id = robot_id
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.acks = {}
        self.mission_state = None
        self.mission_seen = 0
        self.pose = None
        self.pose_seen = 0
        self.odom_seen = 0
        self.battery = None
        self.battery_seen = 0
        self.navigation = None
        self.navigation_seen = 0
        self.diagnostics = {}
        self.fault_candidates = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(String, "/mission/command", 10)
        self.create_subscription(String, "/mission/state", self._mission_state, qos)
        self.create_subscription(String, "/mission/ack", self._mission_ack, 10)
        self.create_subscription(Odometry, "/liorf_localization/mapping/odometry", self._pose, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._odom, 10)
        self.create_subscription(Float32, "/battery_status", self._battery, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostic, 10)
        if NavigationStatus is not None:
            self.create_subscription(NavigationStatus, "/navigation/state", self._navigation, 10)

    def _navigation(self, message):
        labels = {0: "WAITING_FOR_GOAL", 1: "PLANNING", 2: "MOVING", 3: "ARRIVED", 4: "FAILED"}
        with self.lock:
            self.navigation = {"state": labels.get(message.state, "UNKNOWN"), "detail": message.detail}
            self.navigation_seen = time.monotonic()

    def _mission_state(self, message):
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(state, dict) and state.get("schema_version") == 1:
            with self.lock:
                self.mission_state = state
                self.mission_seen = time.monotonic()

    def _mission_ack(self, message):
        try:
            ack = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self.condition:
            self.acks[ack.get("request_id")] = ack
            self.condition.notify_all()

    def _pose(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        position = message.pose.pose.position
        with self.lock:
            self.pose = {"x": position.x, "y": position.y, "yaw": yaw, "frame_id": message.header.frame_id}
            self.pose_seen = time.monotonic()

    def _odom(self, _message):
        with self.lock:
            self.odom_seen = time.monotonic()

    def _battery(self, message):
        with self.lock:
            self.battery = message.data
            self.battery_seen = time.monotonic()

    def _diagnostic(self, message):
        now = time.monotonic()
        with self.lock:
            odom_online = now - self.odom_seen < 2
            for entry in message.status:
                level = entry.level[0] if isinstance(entry.level, (bytes, bytearray)) else int(entry.level)
                if (odom_online and entry.name == "ekf_filter_node: odometry/filtered topic status"
                        and entry.message == "No events recorded."):
                    self.store.observe_fault(self.robot_id, entry.name, entry.message, 0)
                    continue
                self.diagnostics[entry.name] = {
                    "level": level, "message": entry.message, "last_seen": now
                }
                if level == 0:
                    self.fault_candidates.pop(entry.name, None)
                    self.store.observe_fault(self.robot_id, entry.name, entry.message, 0)
                else:
                    first, last = self.fault_candidates.get(entry.name, (now, now))
                    if now - last > 5:
                        first = now
                    self.fault_candidates[entry.name] = (first, now)
                    if now - first >= 3:
                        self.store.observe_fault(self.robot_id, entry.name, entry.message, level)

    def snapshot(self):
        with self.lock:
            return {
                "mission": copy.deepcopy(self.mission_state),
                "mission_online": time.monotonic() - self.mission_seen < 6,
                "pose": copy.deepcopy(self.pose),
                "pose_online": time.monotonic() - self.pose_seen < 3,
                "odom_online": time.monotonic() - self.odom_seen < 3,
                "battery": self.battery,
                "battery_online": time.monotonic() - self.battery_seen < 10,
                "navigation": copy.deepcopy(self.navigation),
                "navigation_online": time.monotonic() - self.navigation_seen < 3,
                "diagnostics": copy.deepcopy(self.diagnostics),
            }

    def command(self, command, request_id, timeout=3):
        with self.condition:
            self.acks.pop(request_id, None)
        self.command_pub.publish(String(data=json.dumps({**command, "request_id": request_id})))
        deadline = time.monotonic() + timeout
        with self.condition:
            while request_id not in self.acks:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            return self.acks.pop(request_id)


def register_platform_api(app, bridge_provider, store, robot_id, auth_store=None):
    prefix = "/api/v1/robots/<robot_id>"

    def bridge_or_503():
        bridge = bridge_provider()
        if bridge is None:
            abort(503, "Robot API is not ready.")
        return bridge

    def mission_snapshot():
        snapshot = bridge_or_503().snapshot()
        if not snapshot["mission_online"]:
            abort(503, "Mission manager is offline.")
        return snapshot["mission"] or {}

    def dispatch(robot_id, action, command):
        bridge = bridge_or_503()
        key = request.headers.get("Idempotency-Key", "")
        if not 8 <= len(key) <= 128:
            abort(400, "Idempotency-Key must be 8–128 characters.")
        body_hash = json_hash({"path": request.path, "command": command})
        try:
            row, is_new = store.reserve_command(key, body_hash, g.identity["username"], robot_id, action)
        except ValueError as exc:
            abort(409, str(exc))
        if not is_new:
            return jsonify(json.loads(row["result_json"])), 202 if row["status"] == "pending" else 200
        if not bridge.snapshot()["mission_online"]:
            result = {"command_id": row["command_id"], "status": "rejected", "error": "Mission manager offline"}
            store.finish_command(row["command_id"], "rejected", result)
            return jsonify(result), 503
        ack = bridge.command(command, row["command_id"])
        if ack is None:
            result = {"command_id": row["command_id"], "status": "pending", "request_id": row["command_id"]}
            return jsonify(result), 202
        if not ack.get("ok"):
            result = {"command_id": row["command_id"], "status": "rejected", "error": ack.get("error", "Robot rejected command")}
            store.finish_command(row["command_id"], "rejected", result)
            return jsonify(result), 409
        result = {"command_id": row["command_id"], "status": "accepted", "request_id": row["command_id"]}
        if ack.get("task_id"):
            result["task_id"] = ack["task_id"]
        store.finish_command(row["command_id"], "accepted", result)
        return jsonify(result), 202

    @app.get(prefix + "/status")
    @require_role("Viewer")
    def platform_status(robot_id):
        snapshot = bridge_or_503().snapshot()
        mission = snapshot["mission"] or {}
        return jsonify({
            "robot_id": robot_id, "observed_at": utc_now(),
            "online": snapshot["pose_online"] or snapshot["odom_online"],
            "pose": {**(snapshot["pose"] or {}), "stale": not snapshot["pose_online"]},
            "odometry": {"stale": not snapshot["odom_online"]},
            "battery": {"percent": snapshot["battery"], "stale": not snapshot["battery_online"]},
            "navigation": {**(snapshot["navigation"] or {}), "stale": not snapshot["navigation_online"]},
            "task": mission.get("run"), "task_stale": not snapshot["mission_online"],
            "fault_counts": {
                "active": len(store.faults(robot_id, active_only=True)),
            },
        })

    @app.get(prefix + "/health")
    @require_role("Viewer")
    def platform_health(robot_id):
        snapshot = bridge_or_503().snapshot()
        return jsonify({
            "robot_id": robot_id, "observed_at": utc_now(),
            "localization_online": snapshot["pose_online"],
            "odometry_online": snapshot["odom_online"],
            "mission_manager_online": snapshot["mission_online"],
            "navigation_online": snapshot["navigation_online"],
            "diagnostics": [
                {"name": name, "level": item["level"], "message": item["message"]}
                for name, item in snapshot["diagnostics"].items()
                if time.monotonic() - item["last_seen"] < 5
            ],
        })

    @app.get(prefix + "/events")
    @require_role("Viewer")
    def platform_events(robot_id):
        bridge = bridge_or_503()
        session_token = request.cookies.get(COOKIE_NAME)
        try:
            last_event_id = max(0, int(request.headers.get("Last-Event-ID", "0")))
        except ValueError:
            abort(400, "Last-Event-ID must be an integer.")

        def stream():
            cursor = last_event_id
            if cursor == 0:
                latest_id = store.latest_event_id(robot_id)
                if latest_id:
                    cursor = latest_id - 1
            for _ in range(150):
                if auth_store is not None and auth_store.session(session_token) is None:
                    break
                for event in store.events_after(robot_id, cursor):
                    cursor = event["id"]
                    yield (f"id: {cursor}\nevent: {event['event_type']}\n"
                           f"data: {event['payload_json']}\n\n")
                snapshot = bridge.snapshot()
                mission = snapshot["mission"] or {}
                payload = {
                    "robot_id": robot_id, "observed_at": utc_now(),
                    "pose": snapshot["pose"], "pose_stale": not snapshot["pose_online"],
                    "task": mission.get("run"), "task_stale": not snapshot["mission_online"],
                    "active_faults": store.faults(robot_id, active_only=True),
                }
                event_id = store.append_state_event(robot_id, payload)
                if event_id is not None:
                    cursor = event_id
                    yield f"id: {event_id}\nevent: robot_state\ndata: {json.dumps(payload)}\n\n"
                elif not store.events_after(robot_id, cursor):
                    yield ": keepalive\n\n"
                time.sleep(2)

        response = Response(stream_with_context(stream()), mimetype="text/event-stream")
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    @app.get(prefix + "/missions")
    @require_role("Viewer")
    def platform_missions(robot_id):
        return jsonify({"missions": mission_snapshot().get("missions", [])})

    @app.get(prefix + "/missions/<mission_id>")
    @require_role("Viewer")
    def platform_mission(robot_id, mission_id):
        mission = next((item for item in mission_snapshot().get("missions", []) if item.get("id") == mission_id), None)
        if not mission:
            abort(404, "Mission not found.")
        return jsonify(mission)

    @app.post(prefix + "/missions")
    @require_role("Operator")
    def platform_save_mission(robot_id):
        mission = request.get_json(silent=True)
        if not isinstance(mission, dict) or not mission.get("id"):
            abort(400, "Mission id is required.")
        return dispatch(robot_id, "mission.save", {"command": "save", "mission": mission})

    @app.put(prefix + "/missions/<mission_id>")
    @require_role("Operator")
    def platform_update_mission(robot_id, mission_id):
        mission = request.get_json(silent=True)
        if not isinstance(mission, dict) or mission.get("id") != mission_id:
            abort(400, "Mission id mismatch.")
        return dispatch(robot_id, "mission.save", {"command": "save", "mission": mission})

    @app.delete(prefix + "/missions/<mission_id>")
    @require_role("Operator")
    def platform_delete_mission(robot_id, mission_id):
        return dispatch(robot_id, "mission.delete", {"command": "delete", "mission_id": mission_id})

    @app.post(prefix + "/tasks")
    @require_role("Operator")
    def platform_start_task(robot_id):
        payload = request.get_json(silent=True) or {}
        mission_id = payload.get("mission_id")
        if not isinstance(mission_id, str) or not mission_id:
            abort(400, "mission_id is required.")
        return dispatch(robot_id, "task.start", {"command": "start", "mission_id": mission_id})

    @app.get(prefix + "/tasks")
    @require_role("Viewer")
    def platform_tasks(robot_id):
        state = mission_snapshot()
        return jsonify({"current": state.get("run"), "history": state.get("history", [])})

    @app.get(prefix + "/tasks/<task_id>")
    @require_role("Viewer")
    def platform_task(robot_id, task_id):
        state = mission_snapshot()
        tasks = [state.get("run")] + state.get("history", [])
        task = next((item for item in tasks if item and item.get("task_id") == task_id), None)
        if not task:
            abort(404, "Task not found.")
        return jsonify(task)

    @app.post(prefix + "/tasks/<task_id>/commands")
    @require_role("Operator")
    def platform_task_command(robot_id, task_id):
        payload = request.get_json(silent=True) or {}
        action = payload.get("action")
        if action not in {"pause", "resume", "cancel", "retry", "skip"}:
            abort(400, "Unsupported task action.")
        return dispatch(robot_id, "task." + action, {"command": action, "task_id": task_id})

    @app.get(prefix + "/commands/<command_id>")
    @require_role("Viewer")
    def platform_command(robot_id, command_id):
        row = store.command(command_id, robot_id)
        if not row:
            abort(404, "Command not found.")
        return jsonify(json.loads(row["result_json"]))

    @app.get(prefix + "/faults")
    @require_role("Viewer")
    def platform_faults(robot_id):
        active = request.args.get("active") == "true"
        return jsonify({"faults": store.faults(robot_id, active_only=active)})

    @app.get(prefix + "/faults/<fault_id>")
    @require_role("Viewer")
    def platform_fault(robot_id, fault_id):
        fault = store.fault(robot_id, fault_id)
        if not fault:
            abort(404, "Fault not found.")
        return jsonify(fault)

    @app.post(prefix + "/faults/<fault_id>/ack")
    @require_role("Operator")
    def platform_ack_fault(robot_id, fault_id):
        if not store.fault(robot_id, fault_id):
            abort(404, "Fault not found.")
        store.ack_fault(robot_id, fault_id, g.identity["username"])
        return jsonify(store.fault(robot_id, fault_id))

    @app.post(prefix + "/faults/<fault_id>/clear")
    @require_role("Engineer")
    def platform_clear_fault(robot_id, fault_id):
        fault = store.fault(robot_id, fault_id)
        if not fault:
            abort(404, "Fault not found.")
        if fault["resolved_at"] is None:
            abort(409, "Active fault must be resolved on the robot first.")
        store.clear_fault(robot_id, fault_id)
        return jsonify(store.fault(robot_id, fault_id))

    @app.get(prefix + "/config")
    @require_role("Viewer")
    def platform_config(robot_id):
        return jsonify({"scope": "platform", **store.config()})

    @app.get(prefix + "/config/versions")
    @require_role("Viewer")
    def platform_config_versions(robot_id):
        return jsonify({"versions": store.config_versions()})

    @app.put(prefix + "/config")
    @require_role("Engineer")
    def platform_create_config(robot_id):
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict) or set(payload) != {"low_battery_threshold"}:
            abort(400, "Only low_battery_threshold is supported in platform config.")
        threshold = payload["low_battery_threshold"]
        if isinstance(threshold, bool) or not isinstance(threshold, int) or not 1 <= threshold <= 99:
            abort(400, "low_battery_threshold must be an integer from 1 to 99.")
        try:
            expected_version = int(request.headers.get("If-Match", "").strip('"'))
        except ValueError:
            abort(412, "Config version changed; reload before saving.")
        try:
            version = store.create_config(payload, g.identity["username"], expected_version)
        except ValueError:
            abort(412, "Config version changed; reload before saving.")
        return jsonify({"scope": "platform", **store.config(version)}), 201

    @app.post(prefix + "/config/versions/<int:version>/activate")
    @require_role("Engineer")
    def platform_activate_config(robot_id, version):
        if not store.activate_config(version):
            abort(404, "Config version not found.")
        return jsonify({"scope": "platform", **store.config(version)})

    log_root = Path(setting("LOG_ROOT", "~/.ros/log")).expanduser().resolve()

    def log_id(path):
        relative = str(path.relative_to(log_root)).encode("utf-8")
        return base64.urlsafe_b64encode(relative).decode("ascii").rstrip("=")

    def log_path(encoded):
        try:
            relative = base64.urlsafe_b64decode(encoded + "=" * (-len(encoded) % 4)).decode("utf-8")
            path = (log_root / relative).resolve()
        except (ValueError, UnicodeDecodeError, binascii.Error):
            abort(404, "Log not found.")
        if not path.is_relative_to(log_root) or not path.is_file() or path.suffix != ".log":
            abort(404, "Log not found.")
        return path

    @app.get(prefix + "/logs")
    @require_role("Viewer")
    def platform_logs(robot_id):
        module = request.args.get("module", "")[:100]
        task_id = request.args.get("task_id", "")[:100]
        fault_code = request.args.get("fault_code", "")[:100]
        files = []
        if log_root.is_dir():
            for path in log_root.rglob("*.log"):
                if len(files) >= 2000:
                    break
                if path.is_symlink() or not path.resolve().is_relative_to(log_root):
                    continue
                if module and module.lower() not in path.name.lower():
                    continue
                if (task_id or fault_code) and path.stat().st_size > 2 * 1024 * 1024:
                    continue
                if task_id or fault_code:
                    body = path.read_text(encoding="utf-8", errors="replace")
                    if task_id and task_id not in body:
                        continue
                    if fault_code and fault_code not in body:
                        continue
                stat = path.stat()
                files.append({
                    "log_id": log_id(path), "name": path.name, "size": stat.st_size,
                    "modified_at": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(),
                })
        files.sort(key=lambda item: item["modified_at"], reverse=True)
        return jsonify({"logs": files[:100]})

    @app.get(prefix + "/logs/<log_id>/download")
    @require_role("Operator")
    def platform_download_log(robot_id, log_id):
        return send_file(log_path(log_id), as_attachment=True, download_name="robot.log")
