import os
import tempfile
import unittest

from ackermann_mission.node import validated_mission
from ackermann_mission.store import MissionStore


class MissionStoreTest(unittest.TestCase):
    def test_run_snapshot_and_history_survive_reopen(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "missions.sqlite3")
            mission = validated_mission({
                "id": "route-1", "name": "Patrol",
                "steps": [{"type": "waypoint", "pose": {"x": 1, "y": 2, "z": 0, "w": 1}},
                          {"type": "wait", "seconds": 5}],
            })
            store = MissionStore(path)
            store.save_mission(mission)
            run = store.new_run("task-1", mission)
            store.add_event("task-1", "RUNNING", 0, "started", {"x": 0, "y": 0})
            store.update_run("task-1", status="PAUSED", remaining_seconds=2)
            store.save_mission({**mission, "steps": []})
            store.close()

            reopened = MissionStore(path)
            self.assertEqual(reopened.run("task-1")["steps"], run["steps"])
            self.assertEqual(reopened.run("task-1")["status"], "PAUSED")
            self.assertEqual(reopened.run("task-1")["remaining_seconds"], 2)
            self.assertEqual(reopened.events("task-1")[0]["robot_pose"]["x"], 0)
            reopened.close()

    def test_waypoint_requires_embedded_pose(self):
        with self.assertRaises(ValueError):
            validated_mission({"id": "m", "name": "M", "steps": [
                {"type": "waypoint", "waypointId": 42}]})


if __name__ == "__main__":
    unittest.main()
