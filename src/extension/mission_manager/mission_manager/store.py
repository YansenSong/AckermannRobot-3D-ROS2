"""SQLite storage for mission definitions and execution history."""

import json
import os
import sqlite3
from datetime import datetime, timezone


ACTIVE_STATUSES = ("RUNNING", "PAUSED")


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class MissionStore:
    def __init__(self, path):
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.executescript(
            """
            CREATE TABLE IF NOT EXISTS missions (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                steps_json TEXT NOT NULL,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS runs (
                task_id TEXT PRIMARY KEY,
                mission_id TEXT NOT NULL,
                mission_name TEXT NOT NULL,
                steps_json TEXT NOT NULL,
                status TEXT NOT NULL,
                step_index INTEGER NOT NULL,
                attempt INTEGER NOT NULL,
                remaining_seconds REAL NOT NULL,
                started_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                ended_at TEXT,
                reason TEXT NOT NULL,
                hold_active INTEGER NOT NULL DEFAULT 0
            );
            CREATE TABLE IF NOT EXISTS run_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                status TEXT NOT NULL,
                step_index INTEGER NOT NULL,
                reason TEXT NOT NULL,
                robot_pose_json TEXT
            );
            CREATE INDEX IF NOT EXISTS run_events_task_id
                ON run_events(task_id, id);
            """
        )
        columns = {row["name"] for row in self.db.execute("PRAGMA table_info(runs)")}
        if "hold_active" not in columns:
            self.db.execute("ALTER TABLE runs ADD COLUMN hold_active INTEGER NOT NULL DEFAULT 0")
        self.db.commit()

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
                """INSERT INTO missions(id, name, steps_json, created_at, updated_at)
                   VALUES (?, ?, ?, ?, ?)
                   ON CONFLICT(id) DO UPDATE SET
                     name = excluded.name,
                     steps_json = excluded.steps_json,
                     updated_at = excluded.updated_at""",
                (mission["id"], mission["name"], json.dumps(mission["steps"]), now, now),
            )
        return self.mission(mission["id"])

    def delete_mission(self, mission_id):
        with self.db:
            cursor = self.db.execute("DELETE FROM missions WHERE id = ?", (mission_id,))
        return cursor.rowcount > 0

    def new_run(self, task_id, mission):
        now = utc_now()
        with self.db:
            self.db.execute(
                """INSERT INTO runs(task_id, mission_id, mission_name, steps_json,
                   status, step_index, attempt, remaining_seconds,
                   started_at, updated_at, ended_at, reason)
                   VALUES (?, ?, ?, ?, 'RUNNING', 0, 1, 0, ?, ?, NULL, 'task started')""",
                (task_id, mission["id"], mission["name"], json.dumps(mission["steps"]), now, now),
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

    def add_event(self, task_id, status, step_index, reason, robot_pose=None):
        with self.db:
            self.db.execute(
                """INSERT INTO run_events(task_id, timestamp, status, step_index,
                   reason, robot_pose_json) VALUES (?, ?, ?, ?, ?, ?)""",
                (
                    task_id, utc_now(), status, step_index, reason,
                    json.dumps(robot_pose) if robot_pose is not None else None,
                ),
            )

    def events(self, task_id, limit=200):
        rows = self.db.execute(
            """SELECT timestamp, status, step_index, reason, robot_pose_json
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
            }
            for row in reversed(list(rows))
        ]
