"""Mission hold must never clear an operator's independent stop request."""

import importlib.util
from pathlib import Path
import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


SCRIPT = Path(__file__).resolve().parents[1] / "cmd_vel_mux.py"
SPEC = importlib.util.spec_from_file_location("cmd_vel_mux", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def warn(self, *_args):
        pass

    def debug(self, *_args, **_kwargs):
        pass


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class FakeMux:
    stop_requested = False
    mission_hold_requested = False
    manual_msg = None
    neupan_msg = None
    manual_received_at = None
    manual_cmd_timeout = 0.5
    area_rules_required = True
    area_control_timeout = 0.5
    area_control = None
    area_control_received_at = None

    def __init__(self):
        self.pub = FakePublisher()

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return FakeClock()


class MissionHoldTest(unittest.TestCase):
    def test_releasing_mission_hold_does_not_release_operator_stop(self):
        mux = FakeMux()
        MODULE.CmdVelMux.stop_callback(mux, Bool(data=True))
        MODULE.CmdVelMux.mission_hold_callback(mux, Bool(data=True))
        MODULE.CmdVelMux.mission_hold_callback(mux, Bool(data=False))
        MODULE.CmdVelMux.timer_callback(mux)
        self.assertTrue(mux.stop_requested)
        self.assertEqual(len(mux.pub.messages), 1)
        self.assertEqual(mux.pub.messages[0].twist.linear.x, 0.0)

    def test_area_heartbeat_and_speed_limit_constrain_manual_command(self):
        mux = FakeMux()
        mux.manual_msg = Twist()
        mux.manual_msg.linear.x = 0.8
        mux.manual_msg.angular.z = 0.4
        MODULE.CmdVelMux.manual_callback(mux, mux.manual_msg)
        MODULE.CmdVelMux.timer_callback(mux)
        self.assertEqual(mux.pub.messages[-1].twist.linear.x, 0.0)
        MODULE.CmdVelMux.area_control_callback(mux, String(data=(
            '{"ready": true, "stop": false, "speed_limit": 0.2}')))
        MODULE.CmdVelMux.timer_callback(mux)
        self.assertAlmostEqual(mux.pub.messages[-1].twist.linear.x, 0.2)
        self.assertAlmostEqual(mux.pub.messages[-1].twist.angular.z, 0.1)
        self.assertAlmostEqual(mux.manual_msg.linear.x, 0.8)
        mux.area_control_received_at -= 1.0
        MODULE.CmdVelMux.timer_callback(mux)
        self.assertEqual(mux.pub.messages[-1].twist.linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
