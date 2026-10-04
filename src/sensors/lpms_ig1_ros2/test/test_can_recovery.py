from types import SimpleNamespace

import pytest
import rclpy
from std_srvs.srv import SetBool

from lpms_ig1_ros2 import lpms_ig1_node as driver


class FakeSocket:
    def __init__(self, bind_error=None, receive_error=None):
        self.bind_error = bind_error
        self.receive_error = receive_error
        self.closed = False

    def setblocking(self, _enabled):
        pass

    def bind(self, _address):
        if self.bind_error is not None:
            raise self.bind_error

    def recv(self, _size):
        if self.receive_error is not None:
            raise self.receive_error
        raise BlockingIOError()

    def close(self):
        self.closed = True


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def setup_can(monkeypatch, sockets):
    clock = SimpleNamespace(now=10.0)
    calls = []

    def make_socket(*_args):
        calls.append(1)
        return sockets.pop(0)

    monkeypatch.setattr(driver, "time", SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(
        driver,
        "socket",
        SimpleNamespace(AF_CAN=1, SOCK_RAW=2, CAN_RAW=3, socket=make_socket),
    )
    return clock, calls


def test_missing_interface_retries_and_reports_sample_timeout(monkeypatch, ros_context):
    first = FakeSocket(bind_error=OSError("interface down"))
    second = FakeSocket()
    third = FakeSocket()
    clock, calls = setup_can(monkeypatch, [first, second, third])
    node = driver.LpmsIg1Ros2()
    try:
        status = FakePublisher()
        node.pub_connected = status
        assert node.sock is None
        assert first.closed
        node.poll_can()
        assert len(calls) == 1

        clock.now = 12.1
        node.poll_can()
        assert node.sock is second
        node.publish_connection_status()
        assert status.messages[-1] is False

        node.last_complete_sample_at = clock.now
        node.publish_connection_status()
        assert status.messages[-1] is True

        clock.now = 14.2
        node.publish_connection_status()
        assert second.closed
        assert status.messages[-1] is False
        node.poll_can()
        assert len(calls) == 2

        clock.now = 16.3
        node.poll_can()
        assert node.sock is third
    finally:
        node.destroy_node()


def test_receive_error_and_runtime_reconnect_switch(monkeypatch, ros_context):
    first = FakeSocket(receive_error=OSError("bus off"))
    second = FakeSocket()
    clock, calls = setup_can(monkeypatch, [first, second])
    node = driver.LpmsIg1Ros2()
    try:
        node.poll_can()
        assert node.sock is None
        assert first.closed

        node.set_auto_reconnect(
            SetBool.Request(data=False), SetBool.Response()
        )
        assert node.get_parameter("auto_reconnect").value is False
        clock.now = 20.0
        node.poll_can()
        assert len(calls) == 1

        node.set_auto_reconnect(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert node.get_parameter("auto_reconnect").value is True
        node.poll_can()
        assert node.sock is second
    finally:
        node.destroy_node()
