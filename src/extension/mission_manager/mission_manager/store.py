"""SQLite storage for mission definitions and execution history."""

import json
import os
import sqlite3
from datetime import datetime, timezone


ACTIVE_STATUSES = ("RUNNING", "PAUSED")
MISSION_SCHEMA_VERSION = 2


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class MissionStore:
    def __init__(self, path):
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.row_factory = sqlite3.Row
        current_schema = self.db.execute("PRAGMA user_version").fetchone()[0]
        if current_schema > MISSION_SCHEMA_VERSION:
            self.db.close()
            raise RuntimeError(
                "Mission database schema is newer than this RobotPilot "
                f"build ({current_schema} > {MISSION_SCHEMA_VERSION})."
            )
        self.db.execute("PRAGMA journal_mode=WAL")
        try:
            self.db.executescript(
                """
            BEGIN IMMEDIATE;
            CREATE TABLE IF NOT EXISTS missions (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                steps_json TEXT NOT NULL,
                map_id TEXT,
                map_version_id TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS runs (
                task_id TEXT PRIMARY KEY,
                mission_id TEXT NOT NULL,
                mission_name TEXT NOT NULL,
                steps_json TEXT NOT NULL,
                map_id TEXT,
                map_version_id TEXT,
                status TEXT NOT NULL,
                step_index INTEGER NOT NULL,
                attempt INTEGER NOT NULL,
                remaining_seconds REAL NOT NULL,
                started_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                ended_at TEXT,
                reason TEXT NOT NULL,
                hold_active INTEGER NOT NULL DEFAULT 0,
                origin_request_id TEXT
            );
            CREATE TABLE IF NOT EXISTS run_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                status TEXT NOT NULL,
                step_index INTEGER NOT NULL,
                reason TEXT NOT NULL,
                robot_pose_json TEXT,
                request_id TEXT
            );
            CREATE INDEX IF NOT EXISTS run_events_task_id
                ON run_events(task_id, id);
            CREATE TABLE IF NOT EXISTS schedules (
                robot_id TEXT NOT NULL,
                schedule_id TEXT NOT NULL,
                name TEXT NOT NULL,
                mission_id TEXT NOT NULL,
                recurrence TEXT NOT NULL,
                timezone TEXT NOT NULL,
                local_time TEXT NOT NULL,
                start_date TEXT,
                weekdays_json TEXT NOT NULL,
                enabled INTEGER NOT NULL DEFAULT 1,
                next_run_at TEXT,
                last_run_at TEXT,
                last_status TEXT,
                misfire_policy TEXT NOT NULL DEFAULT 'skip',
                overlap_policy TEXT NOT NULL DEFAULT 'skip',
                created_by TEXT NOT NULL,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                revision INTEGER NOT NULL DEFAULT 1,
                PRIMARY KEY (robot_id, schedule_id)
            );
            CREATE INDEX IF NOT EXISTS schedules_due
                ON schedules(robot_id, enabled, next_run_at);
            CREATE TABLE IF NOT EXISTS schedule_runs (
                robot_id TEXT NOT NULL,
                schedule_id TEXT NOT NULL,
                scheduled_for TEXT NOT NULL,
                status TEXT NOT NULL,
                attempt INTEGER NOT NULL DEFAULT 1,
                task_id TEXT,
                reason TEXT NOT NULL DEFAULT '',
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                PRIMARY KEY (robot_id, schedule_id, scheduled_for)
            );
            CREATE INDEX IF NOT EXISTS schedule_runs_history
                ON schedule_runs(robot_id, schedule_id, scheduled_for DESC);
                """
            )
            with self.db:
                columns = {row["name"] for row in self.db.execute("PRAGMA table_info(runs)")}
                if "hold_active" not in columns:
                    self.db.execute("ALTER TABLE runs ADD COLUMN hold_active INTEGER NOT NULL DEFAULT 0")
                if "origin_request_id" not in columns:
                    self.db.execute("ALTER TABLE runs ADD COLUMN origin_request_id TEXT")
                event_columns = {row["name"] for row in self.db.execute("PRAGMA table_info(run_events)")}
                if "request_id" not in event_columns:
                    self.db.execute("ALTER TABLE run_events ADD COLUMN request_id TEXT")
                for table in ("missions", "runs"):
                    columns = {row["name"] for row in self.db.execute(f"PRAGMA table_info({table})")}
                    for column in ("map_id", "map_version_id"):
                        if column not in columns:
                            self.db.execute(f"ALTER TABLE {table} ADD COLUMN {column} TEXT")
                self.db.execute(f"PRAGMA user_version = {MISSION_SCHEMA_VERSION}")
        except Exception:
            self.db.rollback()
            self.db.close()
            raise
        self.schema_version = MISSION_SCHEMA_VERSION

    def close(self):
        self.db.close()

    @staticmethod
    def _mission(row):
        if row is None:
            return None
        item = dict(row)
        item["steps"] = json.loads(item.pop("steps_json"))
        return item

    @staticmethod
    def _run(row):
        if row is None:
            return None
        item = dict(row)
        item["steps"] = json.loads(item.pop("steps_json"))
        return item

    def missions(self):
        rows = self.db.execute("SELECT * FROM missions ORDER BY created_at, id")
        return [self._mission(row) for row in rows]

    def mission(self, mission_id):
        row = self.db.execute("SELECT * FROM missions WHERE id = ?", (mission_id,)).fetchone()
        return self._mission(row)

    def save_mission(self, mission):
        now = utc_now()
        with self.db:
            self.db.execute(
                """INSERT INTO missions(id, name, steps_json, map_id, map_version_id, created_at, updated_at)
                   VALUES (?, ?, ?, ?, ?, ?, ?)
                   ON CONFLICT(id) DO UPDATE SET
                     name = excluded.name,
                     steps_json = excluded.steps_json,
                     map_id = excluded.map_id,
                     map_version_id = excluded.map_version_id,
                     updated_at = excluded.updated_at""",
                (mission["id"], mission["name"], json.dumps(mission["steps"]), mission.get("map_id"),
                 mission.get("map_version_id"), now, now),
            )
        return self.mission(mission["id"])

    def delete_mission(self, mission_id):
        with self.db:
            cursor = self.db.execute("DELETE FROM missions WHERE id = ?", (mission_id,))
        return cursor.rowcount > 0

    def new_run(self, task_id, mission, origin_request_id=None):
        now = utc_now()
        with self.db:
            self.db.execute(
                """INSERT INTO runs(task_id, mission_id, mission_name, steps_json, map_id, map_version_id,
                   status, step_index, attempt, remaining_seconds,
                   started_at, updated_at, ended_at, reason, origin_request_id)
                   VALUES (?, ?, ?, ?, ?, ?, 'RUNNING', 0, 1, 0, ?, ?, NULL, 'task started', ?)""",
                (task_id, mission["id"], mission["name"], json.dumps(mission["steps"]), mission.get("map_id"),
                 mission.get("map_version_id"), now, now, origin_request_id),
            )
        return self.run(task_id)

    def run(self, task_id):
        row = self.db.execute("SELECT * FROM runs WHERE task_id = ?", (task_id,)).fetchone()
        return self._run(row)

    def latest_run(self):
        row = self.db.execute("SELECT * FROM runs ORDER BY started_at DESC, rowid DESC LIMIT 1").fetchone()
        return self._run(row)

    def recent_runs(self, limit=20):
        rows = self.db.execute(
            "SELECT * FROM runs ORDER BY started_at DESC, rowid DESC LIMIT ?", (limit,)
        )
        return [self._run(row) for row in rows]

    def update_run(self, task_id, *, status=None, step_index=None, attempt=None,
                   remaining_seconds=None, reason=None, hold_active=None):
        current = self.run(task_id)
        if current is None:
            raise ValueError("unknown task_id")
        next_status = status if status is not None else current["status"]
        ended_at = utc_now() if next_status in ("SUCCEEDED", "FAILED", "CANCELLED") else None
        with self.db:
            self.db.execute(
                """UPDATE runs SET status = ?, step_index = ?, attempt = ?,
                   remaining_seconds = ?, updated_at = ?, ended_at = ?, reason = ?,
                   hold_active = ?
                   WHERE task_id = ?""",
                (
                    next_status,
                    step_index if step_index is not None else current["step_index"],
                    attempt if attempt is not None else current["attempt"],
                    remaining_seconds if remaining_seconds is not None else current["remaining_seconds"],
                    utc_now(),
                    ended_at,
                    reason if reason is not None else current["reason"],
                    int(hold_active) if hold_active is not None else current["hold_active"],
                    task_id,
                ),
            )
        return self.run(task_id)

    def add_event(
        self, task_id, status, step_index, reason, robot_pose=None, request_id=None
    ):
        with self.db:
            self.db.execute(
                """INSERT INTO run_events(task_id, timestamp, status, step_index,
                   reason, robot_pose_json, request_id) VALUES (?, ?, ?, ?, ?, ?, ?)""",
                (
                    task_id, utc_now(), status, step_index, reason,
                    json.dumps(robot_pose) if robot_pose is not None else None,
                    request_id,
                ),
            )

    def events(self, task_id, limit=200):
        rows = self.db.execute(
            """SELECT timestamp, status, step_index, reason, robot_pose_json, request_id
               FROM run_events WHERE task_id = ? ORDER BY id DESC LIMIT ?""",
            (task_id, limit),
        )
        return [
            {
                "timestamp": row["timestamp"],
                "status": row["status"],
                "step_index": row["step_index"],
                "reason": row["reason"],
                "robot_pose": json.loads(row["robot_pose_json"]) if row["robot_pose_json"] else None,
                "request_id": row["request_id"],
            }
            for row in reversed(list(rows))
        ]

    @staticmethod
    def _schedule(row):
        if row is None:
            return None
        item = dict(row)
        item["enabled"] = bool(item["enabled"])
        item["weekdays"] = json.loads(item.pop("weekdays_json"))
        return item

    def schedules(self, robot_id):
        rows = self.db.execute(
            "SELECT * FROM schedules WHERE robot_id = ? ORDER BY created_at, schedule_id",
            (robot_id,),
        )
        return [self._schedule(row) for row in rows]

    def schedule(self, robot_id, schedule_id):
        row = self.db.execute(
            "SELECT * FROM schedules WHERE robot_id = ? AND schedule_id = ?",
            (robot_id, schedule_id),
        ).fetchone()
        return self._schedule(row)

    def create_schedule(self, robot_id, value):
        now = utc_now()
        with self.db:
            self.db.execute(
                """INSERT INTO schedules(robot_id, schedule_id, name, mission_id, recurrence,
                   timezone, local_time, start_date, weekdays_json, enabled, next_run_at,
                   misfire_policy, overlap_policy, created_by, created_at, updated_at, revision)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'skip', 'skip', ?, ?, ?, 1)""",
                (robot_id, value["schedule_id"], value["name"], value["mission_id"],
                 value["recurrence"], value["timezone"], value["local_time"], value.get("start_date"),
                 json.dumps(value.get("weekdays", [])), int(value["enabled"]), value.get("next_run_at"),
                 value["created_by"], now, now),
            )
        return self.schedule(robot_id, value["schedule_id"])

    def update_schedule(self, robot_id, schedule_id, value, expected_revision):
        now = utc_now()
        with self.db:
            cursor = self.db.execute(
                """UPDATE schedules SET name = ?, mission_id = ?, recurrence = ?, timezone = ?,
                   local_time = ?, start_date = ?, weekdays_json = ?, enabled = ?, next_run_at = ?,
                   updated_at = ?, revision = revision + 1
                   WHERE robot_id = ? AND schedule_id = ? AND revision = ?""",
                (value["name"], value["mission_id"], value["recurrence"], value["timezone"],
                 value["local_time"], value.get("start_date"), json.dumps(value.get("weekdays", [])),
                 int(value["enabled"]), value.get("next_run_at"), now, robot_id, schedule_id,
                 expected_revision),
            )
            if cursor.rowcount != 1:
                return None
        return self.schedule(robot_id, schedule_id)

    def delete_schedule(self, robot_id, schedule_id, expected_revision):
        with self.db:
            cursor = self.db.execute(
                "DELETE FROM schedules WHERE robot_id = ? AND schedule_id = ? AND revision = ?",
                (robot_id, schedule_id, expected_revision),
            )
        return cursor.rowcount == 1

    def due_schedules(self, robot_id, now):
        rows = self.db.execute(
            """SELECT * FROM schedules WHERE robot_id = ? AND enabled = 1
               AND next_run_at IS NOT NULL AND next_run_at <= ? ORDER BY next_run_at""",
            (robot_id, now),
        )
        return [self._schedule(row) for row in rows]

    def claim_schedule_run(self, robot_id, schedule_id, scheduled_for, next_run_at):
        now = utc_now()
        self.db.execute("BEGIN IMMEDIATE")
        current = self.schedule(robot_id, schedule_id)
        if not current or not current["enabled"] or current["next_run_at"] != scheduled_for:
            self.db.rollback()
            return False
        inserted = self.db.execute(
            """INSERT OR IGNORE INTO schedule_runs(robot_id, schedule_id, scheduled_for,
               status, created_at, updated_at) VALUES (?, ?, ?, 'claimed', ?, ?)""",
            (robot_id, schedule_id, scheduled_for, now, now),
        )
        if inserted.rowcount != 1:
            self.db.rollback()
            return False
        enabled = current["recurrence"] != "once"
        self.db.execute(
            """UPDATE schedules SET enabled = ?, next_run_at = ?, last_run_at = ?,
               last_status = 'claimed', revision = revision + 1, updated_at = ?
               WHERE robot_id = ? AND schedule_id = ?""",
            (int(enabled), next_run_at, scheduled_for, now, robot_id, schedule_id),
        )
        self.db.commit()
        return True

    def finish_schedule_run(self, robot_id, schedule_id, scheduled_for, status, reason="", task_id=None):
        with self.db:
            self.db.execute(
                """UPDATE schedule_runs SET status = ?, reason = ?, task_id = ?, updated_at = ?
                   WHERE robot_id = ? AND schedule_id = ? AND scheduled_for = ?
                   AND status = 'claimed'""",
                (status, reason, task_id, utc_now(), robot_id, schedule_id, scheduled_for),
            )
            self.db.execute(
                """UPDATE schedules SET last_status = ?, updated_at = ?
                   WHERE robot_id = ? AND schedule_id = ?""",
                (status, utc_now(), robot_id, schedule_id),
            )

    def recover_claimed_schedules(self, robot_id):
        with self.db:
            self.db.execute(
                """UPDATE schedule_runs SET status = 'unknown', reason = 'scheduler restarted after claim',
                   updated_at = ? WHERE robot_id = ? AND status = 'claimed'""",
                (utc_now(), robot_id),
            )

    def schedule_history(self, robot_id, schedule_id, limit=100):
        rows = self.db.execute(
            """SELECT * FROM schedule_runs WHERE robot_id = ? AND schedule_id = ?
               ORDER BY scheduled_for DESC LIMIT ?""",
            (robot_id, schedule_id, limit),
        )
        return [dict(row) for row in rows]

    def schedule_runs(self, robot_id, limit=500):
        rows = self.db.execute(
            """SELECT * FROM schedule_runs WHERE robot_id = ?
               ORDER BY scheduled_for DESC LIMIT ?""",
            (robot_id, limit),
        )
        return [dict(row) for row in rows]
