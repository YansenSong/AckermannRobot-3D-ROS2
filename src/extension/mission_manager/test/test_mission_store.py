import os
import sqlite3
import tempfile
import threading
import unittest
from unittest.mock import patch

from mission_manager.node import mission_database_path, validated_mission
from mission_manager.store import MissionStore


class MissionStoreTest(unittest.TestCase):
    def test_mission_database_path_honors_explicit_and_ros_home_settings(self):
        with patch.dict(os.environ, {
            "ROBOTPILOT_MISSION_DB": "~/robotpilot/mission.sqlite3",
            "OPENAMR_MISSION_DB": "/legacy/mission.sqlite3",
            "ROS_HOME": "/tmp/ros-home",
        }):
            self.assertEqual(
                mission_database_path(),
                os.path.expanduser("~/robotpilot/mission.sqlite3"),
            )

        with patch.dict(os.environ, {"ROS_HOME": "/tmp/ros-home"}, clear=True):
            self.assertEqual(
                mission_database_path(),
                "/tmp/ros-home/ackermann_missions.sqlite3",
            )

    def test_schedule_claim_is_atomic_idempotent_and_persistent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "schedules.sqlite3")
            store = MissionStore(path)
            schedule = store.create_schedule("robot-001", {
                "schedule_id": "schedule-1", "name": "Morning", "mission_id": "mission-1",
                "recurrence": "once", "timezone": "Asia/Shanghai", "local_time": "08:00",
                "start_date": "2026-10-09", "weekdays": [], "enabled": True,
                "next_run_at": "2026-10-09T00:00:00+00:00", "created_by": "operator",
            })
            self.assertEqual(schedule["revision"], 1)
            store.close()
            barrier = threading.Barrier(2)
            outcomes = []

            def claim_from_instance():
                instance = MissionStore(path)
                barrier.wait()
                outcomes.append(instance.claim_schedule_run(
                    "robot-001", "schedule-1", schedule["next_run_at"], None,
                ))
                instance.close()

            claimers = [threading.Thread(target=claim_from_instance) for _ in range(2)]
            for claimer in claimers:
                claimer.start()
            for claimer in claimers:
                claimer.join()
            self.assertEqual(sorted(outcomes), [False, True])
            store = MissionStore(path)
            self.assertFalse(store.schedule("robot-001", "schedule-1")["enabled"])
            store.recover_claimed_schedules("robot-001")
            self.assertEqual(store.schedule_history("robot-001", "schedule-1")[0]["status"], "unknown")
            store.close()

            reopened = MissionStore(path)
            self.assertEqual(reopened.schedules("robot-001")[0]["name"], "Morning")
            self.assertEqual(reopened.schedule_history("robot-001", "schedule-1")[0]["reason"],
                             "scheduler restarted after claim")
            reopened.close()

    def test_legacy_database_migrates_map_binding_fields_without_losing_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "legacy.sqlite3")
            connection = sqlite3.connect(path)
            connection.executescript("""
                CREATE TABLE missions (
                    id TEXT PRIMARY KEY, name TEXT NOT NULL, steps_json TEXT NOT NULL,
                    created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE runs (
                    task_id TEXT PRIMARY KEY, mission_id TEXT NOT NULL, mission_name TEXT NOT NULL,
                    steps_json TEXT NOT NULL, status TEXT NOT NULL, step_index INTEGER NOT NULL,
                    attempt INTEGER NOT NULL, remaining_seconds REAL NOT NULL, started_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL, ended_at TEXT, reason TEXT NOT NULL,
                    hold_active INTEGER NOT NULL DEFAULT 0
                );
                INSERT INTO missions VALUES ('legacy', 'Legacy', '[]', 'created', 'updated');
                INSERT INTO runs VALUES ('task', 'legacy', 'Legacy', '[]', 'PAUSED', 0, 1, 0,
                    'started', 'updated', NULL, 'legacy task', 1);
            """)
            connection.commit()
            connection.close()

            store = MissionStore(path)
            self.assertEqual(store.schema_version, 2)
            self.assertIsNone(store.mission("legacy")["map_id"])
            self.assertIsNone(store.run("task")["map_version_id"])
            self.assertEqual(store.run("task")["status"], "PAUSED")
            self.assertEqual(store.run("task")["hold_active"], 1)
            store.close()

    def test_future_schema_is_rejected_without_rewriting_version(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "future.sqlite3")
            with sqlite3.connect(path) as db:
                db.execute("PRAGMA user_version = 3")
            with self.assertRaisesRegex(RuntimeError, "schema is newer"):
                MissionStore(path)
            with sqlite3.connect(path) as db:
                self.assertEqual(db.execute("PRAGMA user_version").fetchone()[0], 3)

    def test_schema_migration_rolls_back_after_ddl_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "broken.sqlite3")
            with sqlite3.connect(path) as db:
                db.executescript("""
                    CREATE TABLE missions (
                        id TEXT PRIMARY KEY, name TEXT NOT NULL, steps_json TEXT NOT NULL,
                        map_id TEXT, map_version_id TEXT, created_at TEXT NOT NULL,
                        updated_at TEXT NOT NULL
                    );
                    CREATE VIEW runs AS SELECT NULL AS task_id;
                    CREATE TABLE run_events (
                        id INTEGER PRIMARY KEY, task_id TEXT NOT NULL, timestamp TEXT NOT NULL,
                        status TEXT NOT NULL, step_index INTEGER NOT NULL, reason TEXT NOT NULL,
                        robot_pose_json TEXT
                    );
                    PRAGMA user_version = 1;
                """)
            with self.assertRaises(sqlite3.OperationalError):
                MissionStore(path)
            with sqlite3.connect(path) as db:
                tables = {
                    row[0]
                    for row in db.execute("SELECT name FROM sqlite_master WHERE type = 'table'")
                }
                self.assertNotIn("schedules", tables)
                self.assertEqual(db.execute("PRAGMA user_version").fetchone()[0], 1)

    def test_v1_schema_adds_request_correlation_columns_and_preserves_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "v1.sqlite3")
            with sqlite3.connect(path) as db:
                db.executescript("""
                    CREATE TABLE missions (
                        id TEXT PRIMARY KEY, name TEXT NOT NULL, steps_json TEXT NOT NULL,
                        map_id TEXT, map_version_id TEXT, created_at TEXT NOT NULL,
                        updated_at TEXT NOT NULL
                    );
                    CREATE TABLE runs (
                        task_id TEXT PRIMARY KEY, mission_id TEXT NOT NULL,
                        mission_name TEXT NOT NULL, steps_json TEXT NOT NULL,
                        map_id TEXT, map_version_id TEXT, status TEXT NOT NULL,
                        step_index INTEGER NOT NULL, attempt INTEGER NOT NULL,
                        remaining_seconds REAL NOT NULL, started_at TEXT NOT NULL,
                        updated_at TEXT NOT NULL, ended_at TEXT, reason TEXT NOT NULL,
                        hold_active INTEGER NOT NULL DEFAULT 0
                    );
                    CREATE TABLE run_events (
                        id INTEGER PRIMARY KEY, task_id TEXT NOT NULL,
                        timestamp TEXT NOT NULL, status TEXT NOT NULL,
                        step_index INTEGER NOT NULL, reason TEXT NOT NULL,
                        robot_pose_json TEXT
                    );
                    INSERT INTO missions VALUES
                        ('mission-1', 'Patrol', '[]', 'map-1', 'map-v1', 'created', 'updated');
                    INSERT INTO runs VALUES
                        ('task-1', 'mission-1', 'Patrol', '[]', 'map-1', 'map-v1',
                         'PAUSED', 0, 1, 0, 'started', 'updated', NULL, 'paused', 1);
                    INSERT INTO run_events VALUES
                        (1, 'task-1', 'event-time', 'PAUSED', 0, 'paused', NULL);
                    PRAGMA user_version = 1;
                """)
            store = MissionStore(path)
            self.assertEqual(store.schema_version, 2)
            self.assertEqual(store.run("task-1")["status"], "PAUSED")
            self.assertIsNone(store.run("task-1")["origin_request_id"])
            self.assertIsNone(store.events("task-1")[0]["request_id"])
            store.close()

    def test_run_snapshot_and_history_survive_reopen(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "missions.sqlite3")
            mission = validated_mission({
                "id": "route-1", "name": "Patrol",
                "map_id": "grid-1", "map_version_id": "grid-v1",
                "steps": [{"type": "waypoint", "pose": {"x": 1, "y": 2, "z": 0, "w": 1}},
                          {"type": "wait", "seconds": 5}],
            })
            store = MissionStore(path)
            store.save_mission(mission)
            run = store.new_run("task-1", mission, origin_request_id="web-req-123456")
            store.add_event(
                "task-1", "RUNNING", 0, "started", {"x": 0, "y": 0},
                request_id="web-req-123456",
            )
            store.update_run("task-1", status="PAUSED", remaining_seconds=2)
            store.save_mission({**mission, "steps": []})
            store.close()

            reopened = MissionStore(path)
            self.assertEqual(reopened.run("task-1")["steps"], run["steps"])
            self.assertEqual(reopened.run("task-1")["status"], "PAUSED")
            self.assertEqual(reopened.run("task-1")["remaining_seconds"], 2)
            self.assertEqual(reopened.run("task-1")["map_id"], "grid-1")
            self.assertEqual(
                reopened.run("task-1")["origin_request_id"], "web-req-123456"
            )
            self.assertEqual(reopened.mission("route-1")["map_version_id"], "grid-v1")
            self.assertEqual(reopened.events("task-1")[0]["robot_pose"]["x"], 0)
            self.assertEqual(
                reopened.events("task-1")[0]["request_id"], "web-req-123456"
            )
            reopened.close()

    def test_waypoint_requires_embedded_pose(self):
        with self.assertRaises(ValueError):
            validated_mission({"id": "m", "name": "M", "steps": [
                {"type": "waypoint", "waypointId": 42}]})


if __name__ == "__main__":
    unittest.main()
