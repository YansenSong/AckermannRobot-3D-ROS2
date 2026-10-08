import tempfile
import unittest
from pathlib import Path

from flask import Flask, jsonify

from robotpilot_ui_package.auth import AuthStore, install_auth, require_role


class AuthTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.db_path = str(Path(self.directory.name) / "auth.sqlite3")
        self.app = Flask(__name__)

        @self.app.get("/api/v1/robots/<robot_id>/status")
        @require_role("Viewer")
        def status(robot_id):
            return jsonify({"robot_id": robot_id})

        @self.app.post("/api/v1/robots/<robot_id>/tasks")
        @require_role("Operator")
        def create_task(robot_id):
            return jsonify({"ok": True})

        @self.app.post("/api/v1/robots/<robot_id>/config")
        @require_role("Engineer")
        def update_config(robot_id):
            return jsonify({"ok": True})

        @self.app.post("/api/unguarded")
        def unguarded():
            return jsonify({"ok": True})

        self.store = install_auth(self.app, "local", self.db_path, "robot-001")
        self.store.create_user("viewer", "long-password-123", "Viewer", "robot-001")
        self.store.create_user("operator", "long-password-456", "Operator", "robot-001")
        self.client = self.app.test_client()

    def tearDown(self):
        self.directory.cleanup()

    def login(self, username, password):
        response = self.client.post("/api/v1/auth/login", json={"username": username, "password": password})
        self.assertEqual(response.status_code, 200)
        return response.json["csrf_token"]

    def test_login_scope_role_and_csrf(self):
        self.assertEqual(self.client.get("/api/v1/robots/robot-001/status").status_code, 401)
        self.assertEqual(self.client.post("/api/v1/auth/login", json={"username": "viewer", "password": "wrong"}).status_code, 401)
        csrf = self.login("viewer", "long-password-123")
        self.assertEqual(self.client.get("/api/v1/robots/robot-001/status").status_code, 200)
        self.assertEqual(self.client.get("/api/v1/robots/robot-002/status").status_code, 403)
        self.assertEqual(self.client.post("/api/v1/robots/robot-001/tasks", headers={"X-CSRF-Token": csrf}).status_code, 403)

        csrf = self.login("operator", "long-password-456")
        self.assertEqual(self.client.post("/api/v1/robots/robot-001/tasks").status_code, 403)
        self.assertEqual(self.client.post("/api/v1/robots/robot-001/tasks", headers={"X-CSRF-Token": csrf}).status_code, 200)
        self.assertEqual(self.client.post("/api/v1/robots/robot-001/config", headers={"X-CSRF-Token": csrf}).status_code, 403)
        self.assertEqual(self.client.post("/api/unguarded", headers={"X-CSRF-Token": csrf}).status_code, 403)
        self.assertEqual(self.client.get("/api/v1/auth/csrf").json["csrf_token"], csrf)
        self.assertEqual(self.client.post("/api/v1/auth/logout", headers={"X-CSRF-Token": csrf}).status_code, 200)
        self.assertEqual(self.client.get("/api/v1/auth/me").status_code, 401)

    def test_open_mode_and_invalid_mode(self):
        app = Flask("open-test")
        install_auth(app, "open", self.db_path, "robot-001")
        self.assertEqual(app.test_client().get("/api/v1/auth/me").json["role"], "Admin")
        with self.assertRaises(RuntimeError):
            install_auth(Flask("external-test"), "external", self.db_path, "robot-001")


if __name__ == "__main__":
    unittest.main()
