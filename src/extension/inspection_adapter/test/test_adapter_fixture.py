import json
import os
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from inspection_adapter.node import InspectionAdapter


class AdapterFixtureTest(unittest.TestCase):
    def test_fixture_result_is_explicit_and_duplicate_request_replays_same_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            old = {key: os.environ.get(key) for key in
                   ("INSPECTION_PROVIDER_MODE", "INSPECTION_FIXTURE_SCENARIO", "INSPECTION_ADAPTER_DB")}
            os.environ["INSPECTION_PROVIDER_MODE"] = "fixture"
            os.environ["INSPECTION_FIXTURE_SCENARIO"] = "inconclusive"
            os.environ["INSPECTION_ADAPTER_DB"] = os.path.join(directory, "adapter.sqlite3")
            rclpy.init()
            adapter = InspectionAdapter()
            client = Node("inspection_fixture_test_client")
            received = []
            publisher = client.create_publisher(String, "/inspection/action/request", 10)
            client.create_subscription(String, "/inspection/action/result",
                                       lambda message: received.append(json.loads(message.data)), 10)
            executor = SingleThreadedExecutor()
            executor.add_node(adapter)
            executor.add_node(client)
            request = {"schema_version": 1, "action_run_id": "fixture-action-1",
                       "robot_id": "robot-001", "mission_id": "mission-1", "task_id": "task-1",
                       "step_id": "step-1", "attempt": 1, "map_id": "map-1",
                       "map_version_id": "version-1", "waypoint_id": "wp-1", "asset_ids": [],
                       "kind": "detect", "detector_types": ["fire_smoke"], "parameters": {},
                       "requested_at": "2026-10-10T00:00:00Z", "timeout_ms": 1000,
                       "source_mode": "simulation"}
            try:
                executor.spin_once(timeout_sec=0.05)
                publisher.publish(String(data=json.dumps(request)))
                deadline = time.monotonic() + 2
                while not received and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                self.assertEqual(len(received), 1)
                self.assertEqual(received[0]["outcome"], "INCONCLUSIVE")
                self.assertEqual(received[0]["source_mode"], "fixture")
                self.assertIsNone(received[0]["confidence"])
                result_id = received[0]["result_id"]
                publisher.publish(String(data=json.dumps(request)))
                deadline = time.monotonic() + 2
                while len(received) < 2 and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                self.assertEqual(received[1]["result_id"], result_id)
            finally:
                executor.remove_node(client)
                executor.remove_node(adapter)
                client.destroy_node()
                adapter.db.close()
                adapter.destroy_node()
                executor.shutdown()
                rclpy.shutdown()
                for key, value in old.items():
                    if value is None:
                        os.environ.pop(key, None)
                    else:
                        os.environ[key] = value
