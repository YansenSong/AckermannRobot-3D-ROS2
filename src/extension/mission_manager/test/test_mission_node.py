import json
import os
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_status.msg import NavigationStatus
from std_msgs.msg import String

from mission_manager.node import MissionManager


class MissionNodeTest(unittest.TestCase):
    def test_wait_can_resume_after_manager_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "missions.sqlite3")
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = directory
            rclpy.init(args=["--ros-args", "-p", f"database_path:={path}"])
            node = MissionManager()
            try:
                def command(value):
                    node.on_command(String(data=json.dumps(value)))

                command({"command": "save", "mission": {
                    "id": "m", "name": "Wait mission",
                    "steps": [{"type": "wait", "seconds": 0.2}]}})
                command({"command": "start", "mission_id": "m"})
                task_id = node.run["task_id"]
                self.assertEqual(node.run["status"], "RUNNING")
                command({"command": "pause", "task_id": task_id})
                self.assertEqual(node.run["status"], "PAUSED")
                self.assertEqual(node.run["hold_active"], 1)
                self.assertGreater(node.run["remaining_seconds"], 0)
                node.destroy_node()

                node = MissionManager()
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
            try:
                node.on_command(String(data=json.dumps({"command": "save", "mission": {
                    "id": "m", "name": "Go", "steps": [{"type": "waypoint",
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
                    "id": "home", "name": "Home", "steps": [{"type": "home"}]}})))
                node.on_command(String(data=json.dumps({"command": "start", "mission_id": "home"})))
                self.assertEqual(node.run["status"], "RUNNING")
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir


if __name__ == "__main__":
    unittest.main()
