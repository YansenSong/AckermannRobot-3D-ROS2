import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

from flask import Flask

from robotpilot_ui_package.auth import install_auth
from robotpilot_ui_package.platform_api import PlatformStore, RobotBridge, register_platform_api


class FakeBridge:
    def __init__(self):
        self.calls = []

    def snapshot(self):
        return {
            "mission_online": True,
            "mission": {"missions": [{"id": "patrol", "name": "Patrol", "steps": []}], "run": None, "history": []},
            "pose": {"x": 1, "y": 2, "yaw": 0},
            "pose_online": True,
            "odom_online": True,
            "battery": 80,
            "battery_online": True,
            "navigation": {"state": "WAITING_FOR_GOAL", "detail": ""},
            "navigation_online": True,
            "diagnostics": {},
        }

    def command(self, command, request_id):
        self.calls.append((command, request_id))
        return {"request_id": request_id, "ok": True, "task_id": "task-123"}


class PlatformApiTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.app = Flask(__name__)
        self.bridge = FakeBridge()
        self.store = PlatformStore(str(Path(self.directory.name) / "platform.sqlite3"))
        register_platform_api(self.app, lambda: self.bridge, self.store, "robot-001")
        install_auth(self.app, "open", str(Path(self.directory.name) / "auth.sqlite3"), "robot-001")
        self.client = self.app.test_client()
        self.base = "/api/v1/robots/robot-001"

    def tearDown(self):
        self.directory.cleanup()

    def test_task_command_is_idempotent_and_correlated(self):
        headers = {"Idempotency-Key": "start-patrol-001"}
        first = self.client.post(self.base + "/tasks", json={"mission_id": "patrol"}, headers=headers)
        self.assertEqual(first.status_code, 202)
        self.assertEqual(first.json["task_id"], "task-123")
        second = self.client.post(self.base + "/tasks", json={"mission_id": "patrol"}, headers=headers)
        self.assertEqual(second.status_code, 200)
        self.assertEqual(len(self.bridge.calls), 1)
        changed = self.client.post(self.base + "/tasks", json={"mission_id": "other"}, headers=headers)
        self.assertEqual(changed.status_code, 409)
        command_id = first.json["command_id"]
        self.assertEqual(self.client.get(self.base + "/commands/" + command_id).json["task_id"], "task-123")

    def test_robot_scope_faults_and_config_versions(self):
        self.assertEqual(self.client.get("/api/v1/robots/other/status").status_code, 403)
        self.assertTrue(self.client.get(self.base + "/status").json["online"])
        self.store.observe_fault("robot-001", "lidar", "disconnected", 2)
        fault = self.client.get(self.base + "/faults?active=true").json["faults"][0]
        self.assertEqual(self.client.post(self.base + "/faults/" + fault["fault_id"] + "/clear").status_code, 409)
        self.store.observe_fault("robot-001", "lidar", "connected", 0)
        self.assertEqual(self.client.post(self.base + "/faults/" + fault["fault_id"] + "/clear").status_code, 200)
        self.assertEqual(self.client.put(self.base + "/config", json={"low_battery_threshold": 30}).status_code, 412)
        result = self.client.put(
            self.base + "/config", json={"low_battery_threshold": 30}, headers={"If-Match": '"1"'}
        )
        self.assertEqual(result.status_code, 201)
        self.assertEqual(self.client.post(self.base + "/config/versions/2/activate").json["config"]["low_battery_threshold"], 30)

    def test_ros_diagnostic_level_bytes_are_normalized(self):
        bridge = SimpleNamespace(
            lock=threading.RLock(), odom_seen=0, store=self.store,
            robot_id="robot-001", diagnostics={}, fault_candidates={},
        )
        message = SimpleNamespace(status=[SimpleNamespace(
            name="camera", message="OK", level=b"\x00",
        )])
        RobotBridge._diagnostic(bridge, message)
        self.assertEqual(bridge.diagnostics["camera"]["level"], 0)
        self.assertEqual(bridge.fault_candidates, {})


if __name__ == "__main__":
    unittest.main()
