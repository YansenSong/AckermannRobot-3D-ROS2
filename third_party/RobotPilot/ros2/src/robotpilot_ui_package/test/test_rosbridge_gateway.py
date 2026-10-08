import unittest

from robotpilot_ui_package.rosbridge_gateway import allowed_origin, required_role


class GatewayPolicyTest(unittest.TestCase):
    def test_default_deny_and_role_categories(self):
        self.assertEqual(required_role({"op": "subscribe", "topic": "/mission/state"}), "Viewer")
        self.assertEqual(required_role({"op": "publish", "topic": "/mission/command"}), "Operator")
        self.assertEqual(required_role({"op": "publish", "topic": "/initialpose"}), "Engineer")
        self.assertIsNone(required_role({"op": "publish", "topic": "/cmd_vel"}))
        self.assertIsNone(required_role({"op": "publish", "topic": "/unknown"}))
        self.assertIsNone(required_role({"op": "call_service", "service": "/rosapi/set_param"}))
        self.assertIsNone(required_role({"op": "advertise_service", "service": "/fake"}))

    def test_origin_must_match(self):
        allowed = {"https://robot.example"}
        self.assertTrue(allowed_origin("https://robot.example", allowed))
        self.assertFalse(allowed_origin("https://robot.example.evil", allowed))
        self.assertFalse(allowed_origin(None, allowed))


if __name__ == "__main__":
    unittest.main()
