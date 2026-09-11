#!/usr/bin/env python3
"""Activate the isolated Nav2 lifecycle group after the robot is ready."""

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2StartupGate(Node):
    def __init__(self):
        super().__init__("nav2_startup_gate")
        self.declare_parameter(
            "startup_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("required_topics", ["/odom", "/scan"])
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "rear_axle_link")
        self.declare_parameter("startup_timeout", 90.0)
        self.declare_parameter("check_period", 0.5)

        self.startup_service = str(self.get_parameter("startup_service").value)
        self.required_topics = [
            topic if str(topic).startswith("/") else "/" + str(topic)
            for topic in self.get_parameter("required_topics").value
        ]
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.source_frame = str(self.get_parameter("source_frame").value)
        self.startup_timeout = max(
            float(self.get_parameter("startup_timeout").value), 1.0)
        self.check_period = max(float(self.get_parameter("check_period").value), 0.1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.startup_client = self.create_client(
            ManageLifecycleNodes, self.startup_service)
        self.start_time = time.monotonic()
        self.startup_requested = False
        self.startup_finished = False
        self.timeout_reported = False
        self.last_status = ""
        self.timer = self.create_timer(self.check_period, self._on_timer)

        self.get_logger().info(
            "Waiting for /odom, /scan, TF %s->%s, and %s before starting Ackermann Nav2"
            % (self.target_frame, self.source_frame, self.startup_service))

    def _on_timer(self):
        if self.startup_finished or self.startup_requested:
            return

        missing = self._missing_readiness()
        if missing:
            status = ", ".join(missing)
            if status != self.last_status:
                self.get_logger().info(
                    "Ackermann Nav2 startup gate waiting for: %s" % status)
                self.last_status = status
            if time.monotonic() - self.start_time > self.startup_timeout:
                if not self.timeout_reported:
                    self.get_logger().error(
                        "Timed out waiting for Ackermann Nav2 prerequisites. "
                        "The lifecycle group was not started.")
                    self.timeout_reported = True
                self.timer.cancel()
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.startup_requested = True
        future = self.startup_client.call_async(request)
        future.add_done_callback(self._on_startup_response)
        self.get_logger().info(
            "Ackermann Nav2 prerequisites are ready; requesting lifecycle startup")

    def _missing_readiness(self):
        missing = []
        services = {
            name for name, _ in self.get_service_names_and_types()
        }
        topics = {
            name for name, _ in self.get_topic_names_and_types()
        }

        for topic in self.required_topics:
            if topic not in topics:
                missing.append(topic)

        if self.startup_service not in services:
            missing.append(self.startup_service)
        elif not self.startup_client.service_is_ready():
            missing.append("service_ready:" + self.startup_service)

        try:
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            missing.append(
                "TF %s->%s" % (self.target_frame, self.source_frame))

        return missing

    def _on_startup_response(self, future):
        self.startup_requested = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS service failure path
            self.get_logger().error(
                "Failed to call Ackermann Nav2 lifecycle startup: %s" % exc)
            return

        if response.success:
            self.startup_finished = True
            self.get_logger().info("Ackermann Nav2 lifecycle startup accepted")
        else:
            self.get_logger().error(
                "Ackermann Nav2 lifecycle startup returned success=false")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2StartupGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
