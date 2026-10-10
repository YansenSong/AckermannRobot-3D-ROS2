import importlib.util
import os
import unittest
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from launch.conditions import IfCondition
from launch_ros.actions import Node


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "navigation.launch.py"
spec = importlib.util.spec_from_file_location("s1_navigation_launch", LAUNCH_FILE)
navigation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(navigation)


class InspectionLaunchTest(unittest.TestCase):
    @staticmethod
    def context(mode, profile, enabled="true"):
        context = LaunchContext()
        context.launch_configurations.update({
            "enable_inspection_adapter": enabled,
            "inspection_provider_mode": mode,
            "inspection_simulation_profile": profile,
        })
        return context

    def test_fixture_requires_explicit_simulation_profile_and_robot_mode(self):
        with patch.dict(os.environ, {"ROBOT_MODE": "simulation"}):
            with self.assertRaisesRegex(RuntimeError, "simulation profile"):
                navigation._validate_inspection_configuration(
                    self.context("fixture", "false"))
            self.assertEqual(navigation._validate_inspection_configuration(
                self.context("fixture", "true")), [])
        with patch.dict(os.environ, {"ROBOT_MODE": "hardware"}):
            with self.assertRaisesRegex(RuntimeError, "ROBOT_MODE=simulation"):
                navigation._validate_inspection_configuration(
                    self.context("fixture", "true"))
            self.assertEqual(navigation._validate_inspection_configuration(
                self.context("external", "false")), [])

    def test_navigation_launch_contains_opt_in_adapter(self):
        description = navigation.generate_launch_description()
        adapter = next(action for action in description.entities
                       if isinstance(action, Node)
                       and action._Node__package == "inspection_adapter")
        self.assertIsInstance(adapter.condition, IfCondition)


if __name__ == "__main__":
    unittest.main()
