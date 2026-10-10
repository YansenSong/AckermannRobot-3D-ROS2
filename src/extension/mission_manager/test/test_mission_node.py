import json
import os
import tempfile
import time
import unittest

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_status.msg import NavigationStatus
from std_msgs.msg import String

from mission_manager.node import MissionManager


class PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class MissionNodeTest(unittest.TestCase):
    @staticmethod
    def software_stop(node, *, active=False, result="confirmed", durable=True):
        node.on_software_stop_state(String(data=json.dumps({
            "active": active,
            "result": result,
            "durable": durable,
            "observed_at": "2026-10-08T00:00:00+00:00",
        })))
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        node.on_odom(odometry)
        node.on_battery_state(String(data=json.dumps({
            "available": True, "low_battery": False, "charging": False,
            "source": "sim", "simulation": True,
        })))

    @staticmethod
    def current_map(node, map_id="grid-1", map_version_id="grid-1"):
        node.on_route_catalog(String(data=json.dumps({
            "active_files": {"map_id": map_id, "map_version_id": map_version_id},
        })))
        node.on_map_bundle(String(data=json.dumps({
            "schema_version": 1, "ready": True,
            "map_id": map_id, "map_version_id": map_version_id,
            "bundle_id": "bundle-test", "globalmap_pcd_sha256": "test-pcd",
        })))

    def test_ekf_no_events_fault_requires_missing_filtered_odom(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            try:
                name = "ekf_filter_node: odometry/filtered topic status"
                node.store.observe_fault(name, 2, "No events recorded.")
                self.assertTrue(node.store.active_critical_faults())
                odom = Odometry()
                odom.header.frame_id = "odom"
                node.on_ekf_odom(odom)
                status = DiagnosticStatus()
                status.name = name
                status.level = b"\x02"
                status.message = "No events recorded."
                diag = DiagnosticArray()
                diag.status = [status]
                node.on_diagnostics(diag)
                self.assertFalse(node.store.active_critical_faults())
                node.ekf_odom_received_at = time.monotonic() - 3
                node.store.observe_fault(name, 2, "No events recorded.")
                node.on_diagnostics(diag)
                self.assertTrue(node.store.active_critical_faults())
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_motion_guard_checks_battery_area_and_critical_faults(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            try:
                self.software_stop(node)
                self.assertEqual(node.motion_guard_block_reason(), "")
                node.on_battery_state(String(data=json.dumps({
                    "available": True, "low_battery": True, "charging": False,
                })))
                self.assertIn("battery is low", node.motion_guard_block_reason())
                node.on_battery_state(String(data=json.dumps({
                    "available": True, "low_battery": False, "charging": True,
                })))
                self.assertEqual(node.motion_guard_block_reason(), "battery is charging or charge state is unknown")
                self.software_stop(node)
                node.battery_received_at = time.monotonic() - 6
                self.assertIn("stale", node.motion_guard_block_reason())
                self.software_stop(node)
                node.on_area_control(String(data=json.dumps({"ready": True, "stop": True})))
                self.assertEqual(node.motion_guard_block_reason(), "area control blocks motion")
                node.on_area_control(String(data=json.dumps({"ready": True, "stop": False})))
                node.store.observe_fault("motor", 2, "fault")
                self.assertEqual(node.motion_guard_block_reason(), "critical diagnostics are active")
                node.store.observe_fault("motor", 0, "ok")
                self.assertEqual(node.motion_guard_block_reason(), "")
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_outbox_ack_cannot_advance_past_published_sequence(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            node.outbox_pub = PublisherSpy()
            try:
                node.store.observe_fault("laser", 2, "offline")
                node.on_outbox_ack(String(data=json.dumps({
                    "schema_version": 1, "robot_id": node.robot_id, "through_seq": 999,
                })))
                self.assertEqual(node.store.outbox_status()["pending_count"], 1)
                node.publish_outbox()
                node.on_outbox_ack(String(data=json.dumps({
                    "schema_version": 1, "robot_id": node.robot_id, "through_seq": 1,
                })))
                self.assertEqual(node.store.outbox_status()["pending_count"], 0)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_autonomy_guard_requires_fresh_localization(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                           f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            try:
                node.on_software_stop_state(String(data=json.dumps({
                    "active": False, "result": "confirmed", "durable": True,
                })))
                node.on_battery_state(String(data=json.dumps({
                    "available": True, "low_battery": False, "charging": False,
                })))
                self.assertEqual(
                    node.motion_guard_block_reason(),
                    "localization pose is unavailable or stale",
                )
                odometry = Odometry()
                odometry.pose.pose.orientation.w = 1.0
                node.on_odom(odometry)
                node.on_battery_state(String(data=json.dumps({
                    "available": True, "low_battery": False, "charging": False,
                })))
                self.assertEqual(node.motion_guard_block_reason(), "")
                node.pose_received_at = time.monotonic() - 4
                self.assertEqual(
                    node.motion_guard_block_reason(),
                    "localization pose is unavailable or stale",
                )
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_wait_can_resume_after_manager_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "missions.sqlite3")
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p", f"database_path:={path}"])
            node = MissionManager()
            node.ack_pub = PublisherSpy()
            self.software_stop(node)
            self.current_map(node)
            try:
                def command(value):
                    node.on_command(String(data=json.dumps(value)))

                command({"command": "save", "mission": {
                    "id": "m", "name": "Wait mission",
                    "map_id": "grid-1", "map_version_id": "grid-1",
                    "steps": [{"type": "wait", "seconds": 0.2}]}})
                command({
                    "command": "start", "mission_id": "m",
                    "request_id": "robot-command-123",
                    "origin_request_id": "web-request-123456",
                })
                task_id = node.run["task_id"]
                self.assertEqual(node.run["status"], "RUNNING")
                self.assertEqual(node.run["origin_request_id"], "web-request-123456")
                self.assertEqual(
                    json.loads(node.ack_pub.messages[-1].data)["origin_request_id"],
                    "web-request-123456",
                )
                self.assertEqual(
                    node.store.events(task_id)[0]["request_id"], "web-request-123456"
                )
                original_event_count = len(node.store.events(task_id))
                command({
                    "command": "start", "mission_id": "m",
                    "request_id": "robot-command-123",
                    "origin_request_id": "web-request-123456",
                })
                self.assertEqual(node.run["task_id"], task_id)
                self.assertEqual(len(node.store.events(task_id)), original_event_count)
                self.assertEqual(json.loads(node.ack_pub.messages[-1].data)["task_id"], task_id)
                command({"command": "pause", "task_id": task_id})
                self.assertEqual(node.run["status"], "PAUSED")
                self.assertEqual(node.run["hold_active"], 1)
                self.assertGreater(node.run["remaining_seconds"], 0)
                self.assertEqual(
                    node.store.events(task_id)[-1]["request_id"], "web-request-123456"
                )
                node.destroy_node()

                node = MissionManager()
                self.software_stop(node)
                self.current_map(node)
                self.assertEqual(node.run["status"], "PAUSED")
                self.assertTrue(node.stop_owned)
                command({"command": "resume", "task_id": task_id})
                self.assertEqual(node.run["hold_active"], 0)
                time.sleep(0.25)
                node.tick()
                self.assertEqual(node.run["status"], "SUCCEEDED")
                self.assertTrue(all(event["step_index"] == 0 for event in node.store.events(task_id)))
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_waypoint_waits_for_current_navigation_arrival(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            self.software_stop(node)
            self.current_map(node)
            try:
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "m", "name": "Go", "map_id": "grid-1", "map_version_id": "grid-1",
                    "steps": [{"type": "waypoint",
                    "pose": {"x": 1, "y": 2, "z": 0, "w": 1}}]}})))
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                self.assertEqual(node.run["status"], "RUNNING")
                status = NavigationStatus()
                status.stamp.sec = node.goal_sent_ns // 1000000000
                status.stamp.nanosec = node.goal_sent_ns % 1000000000
                status.state = NavigationStatus.ARRIVED
                node.on_nav(status)
                self.assertEqual(node.run["status"], "RUNNING")
                status.state = NavigationStatus.PLANNING
                status.detail = "new goal received; waiting for global plan"
                node.on_nav(status)
                status.state = NavigationStatus.ARRIVED
                node.on_nav(status)
                self.assertEqual(node.run["status"], "SUCCEEDED")
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                external = PoseStamped()
                external.pose.position.x = 99.0
                external.pose.orientation.w = 1.0
                node.on_goal(external)
                self.assertEqual(node.run["status"], "FAILED")
                self.assertEqual(node.run["hold_active"], 1)
                node.on_command(String(data=json.dumps({"command": "release_hold",
                                                       "task_id": node.run["task_id"]})))
                self.assertEqual(node.run["hold_active"], 0)
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "home", "name": "Home", "map_id": "grid-1", "map_version_id": "grid-1",
                    "steps": [{"type": "home"}]}})))
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "home"})))
                self.assertEqual(node.run["status"], "RUNNING")
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_motion_commands_fail_closed_on_unknown_or_active_software_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            node.ack_pub = PublisherSpy()
            self.current_map(node)
            try:
                save = {"command": "save", "mission": {
                    "id": "m", "name": "Wait", "map_id": "grid-1", "map_version_id": "grid-1",
                    "steps": [{"type": "wait", "seconds": 1}]}}
                node.on_command(String(data=json.dumps(save)))
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                self.assertIn("unavailable or stale", json.loads(node.ack_pub.messages[-1].data)["error"])
                self.assertIsNone(node.run)

                self.software_stop(node, active=True)
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                self.assertEqual(json.loads(node.ack_pub.messages[-1].data)["error"], "software stop is active")
                self.assertIsNone(node.run)

                self.software_stop(node, durable=False)
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                self.assertEqual(json.loads(node.ack_pub.messages[-1].data)["error"], "software stop state is not durable")
                self.assertIsNone(node.run)

                self.software_stop(node)
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "m"})))
                task_id = node.run["task_id"]
                node.on_command(String(data=json.dumps({"command": "pause", "task_id": task_id})))
                self.software_stop(node, active=True)
                node.on_command(String(data=json.dumps({"command": "resume", "task_id": task_id})))
                self.assertEqual(json.loads(node.ack_pub.messages[-1].data)["error"], "software stop is active")
                self.assertEqual(node.run["status"], "PAUSED")
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_start_and_resume_require_matching_map_identity_and_version(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            node.ack_pub = PublisherSpy()
            self.software_stop(node)
            self.current_map(node)
            try:
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "map-task", "name": "Map bound", "map_id": "grid-1",
                    "map_version_id": "grid-1", "steps": [{"type": "wait", "seconds": 1}],
                }})))
                node.map_bundle = None
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "map-task"})))
                self.assertIn("map bundle is unavailable or unverified",
                              json.loads(node.ack_pub.messages[-1].data)["error"])
                self.current_map(node)
                self.current_map(node, "grid-2", "grid-2")
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "map-task"})))
                self.assertEqual(
                    json.loads(node.ack_pub.messages[-1].data)["error"],
                    "mission map_id does not match the current 2D map",
                )
                self.assertIsNone(node.run)

                self.current_map(node)
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "map-task"})))
                task_id = node.run["task_id"]
                node.on_command(String(data=json.dumps({"command": "pause", "task_id": task_id})))
                self.current_map(node, "grid-1", "revision-2")
                node.on_command(String(data=json.dumps({"command": "resume", "task_id": task_id})))
                self.assertEqual(
                    json.loads(node.ack_pub.messages[-1].data)["error"],
                    "mission map_version_id does not match the current 2D map version",
                )
                self.assertEqual(node.run["status"], "PAUSED")
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_active_mission_pauses_when_map_bundle_becomes_unverified(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            self.software_stop(node)
            self.current_map(node)
            try:
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "map-task", "name": "Map bound", "map_id": "grid-1",
                    "map_version_id": "grid-1", "steps": [{"type": "wait", "seconds": 30}],
                }})))
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "map-task"})))
                self.assertEqual(node.run["status"], "RUNNING")
                node.on_map_bundle(String(data=json.dumps({
                    "schema_version": 1, "ready": False, "reason": "active 2D map changed",
                })))
                node.tick()
                self.assertEqual(node.run["status"], "PAUSED")
                self.assertEqual(node.run["hold_active"], 1)
                self.assertIn("map bundle", node.run["reason"])
                self.assertGreater(node.run["remaining_seconds"], 0)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_schedule_crud_is_robot_persisted_and_revision_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                             f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            node.ack_pub = PublisherSpy()
            self.software_stop(node)
            self.current_map(node)
            try:
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "scheduled", "name": "Scheduled", "map_id": "grid-1",
                    "map_version_id": "grid-1", "steps": [{"type": "wait", "seconds": 1}],
                }})))
                schedule_body = {
                    "name": "Tomorrow", "mission_id": "scheduled", "recurrence": "once",
                    "timezone": "UTC", "local_time": "10:00", "start_date": "2099-01-01",
                    "weekdays": [], "enabled": True,
                }
                node.on_command(String(data=json.dumps({
                    "command": "schedule.save", "schedule": schedule_body,
                    "created_by": "operator-a", "request_id": "save-1",
                })))
                ack = json.loads(node.ack_pub.messages[-1].data)
                self.assertTrue(ack["ok"])
                schedule_id = ack["schedule_id"]
                saved = node.store.schedule("robot-001", schedule_id)
                self.assertEqual(saved["created_by"], "operator-a")
                self.assertEqual(saved["revision"], 1)
                self.assertEqual(saved["recurrence"], "once")

                node.on_command(String(data=json.dumps({
                    "command": "schedule.save", "schedule_id": schedule_id,
                    "expected_revision": 1,
                    "schedule": {**schedule_body, "enabled": False},
                    "request_id": "update-1",
                })))
                self.assertTrue(json.loads(node.ack_pub.messages[-1].data)["ok"])
                self.assertEqual(node.store.schedule("robot-001", schedule_id)["revision"], 2)

                node.on_command(String(data=json.dumps({
                    "command": "schedule.delete", "schedule_id": schedule_id,
                    "expected_revision": 1, "request_id": "stale-delete",
                })))
                self.assertFalse(json.loads(node.ack_pub.messages[-1].data)["ok"])
                self.assertIsNotNone(node.store.schedule("robot-001", schedule_id))
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_due_schedule_starts_mission_once_and_records_task_id(self):
        with tempfile.TemporaryDirectory() as directory:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p",
                           f"database_path:={os.path.join(directory, 'missions.sqlite3')}"])
            node = MissionManager()
            self.software_stop(node)
            self.current_map(node)
            try:
                mission = {
                    "id": "scheduled", "name": "Scheduled", "map_id": "grid-1",
                    "map_version_id": "grid-1", "steps": [{"type": "wait", "seconds": 30}],
                }
                node.store.save_mission(mission)
                from datetime import datetime, timedelta, timezone
                due = (datetime.now(timezone.utc) - timedelta(seconds=1)).isoformat()
                node.store.create_schedule("robot-001", {
                    "schedule_id": "schedule-due", "name": "Due now", "mission_id": "scheduled",
                    "recurrence": "once", "timezone": "UTC", "local_time": "00:00",
                    "start_date": "2026-10-08", "weekdays": [], "enabled": True,
                    "next_run_at": due, "created_by": "test",
                })

                node.scheduler_tick()
                history = node.store.schedule_history("robot-001", "schedule-due")
                self.assertEqual(history[0]["status"], "started")
                self.assertEqual(history[0]["task_id"], node.run["task_id"])
                self.assertEqual(node.run["mission_id"], "scheduled")
                self.assertFalse(node.store.schedule("robot-001", "schedule-due")["enabled"])
                node.scheduler_tick()
                self.assertEqual(len(node.store.schedule_history("robot-001", "schedule-due")), 1)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir


if __name__ == "__main__":
    unittest.main()
