"""ROS bridge for persistent map rules, Nav2 keepout mask, and drive interlock."""

import copy
import json
import math
from pathlib import Path

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .rules import BLOCKING_TYPES, ROBOT_RADIUS, RuleStore, covers, map_key, rasterize, yaw_of


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AreaRules(Node):
    def __init__(self):
        super().__init__("area_rules")
        default_path = Path.home() / ".local/share/ackermann_robot/area_rules.json"
        self.declare_parameter("storage_path", str(default_path))
        self.declare_parameter("robot_frame", "rear_axle_link")
        self.store = RuleStore(self.get_parameter("storage_path").value)
        self.robot_frame = self.get_parameter("robot_frame").value
        self.grid = None
        self.key = None
        self.rules = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mask_pub = self.create_publisher(OccupancyGrid, "/area_rules/keepout_mask", LATCHED)
        self.info_pub = self.create_publisher(CostmapFilterInfo, "/area_rules/filter_info", LATCHED)
        self.state_pub = self.create_publisher(String, "/area_rules/state", LATCHED)
        self.ack_pub = self.create_publisher(String, "/area_rules/ack", 10)
        self.control_pub = self.create_publisher(String, "/area_rules/control", 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(String, "/area_rules/command", self.on_command, 10)
        self.create_timer(0.1, self.publish_control)
        self.create_timer(1.0, self.check_expiry)
        self.publish_state()
        self.get_logger().info(f"area rules loaded from {self.store.path}")

    def on_map(self, grid):
        key = map_key(grid)
        if key == self.key:
            return
        self.grid = grid
        self.key = key
        self.rules = self.store.state(key)["rules"]
        self.publish_mask()
        self.publish_state()
        self.get_logger().info(f"map {key}: {len(self.rules)} active rules")

    def publish_state(self):
        state = self.store.state(self.key) if self.key else {"version": 0, "rules": []}
        self.state_pub.publish(String(data=json.dumps({
            "ready": self.grid is not None,
            "map_key": self.key,
            **state,
        })))

    def publish_mask(self):
        if self.grid is None:
            return
        mask = OccupancyGrid()
        mask.header.frame_id = self.grid.header.frame_id or "map"
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = copy.deepcopy(self.grid.info)
        mask.data = rasterize(mask.info, self.rules)
        self.mask_pub.publish(mask)
        info = CostmapFilterInfo()
        info.header = mask.header
        info.type = 0  # Nav2 keepout mask
        info.filter_mask_topic = "/area_rules/keepout_mask"
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

    def on_command(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            request_id = command.get("request_id")
            if not self.key or command.get("map_key") != self.key:
                raise ValueError("map changed or is not ready; refresh rules")
            self.store.mutate(self.key, command.get("expected_version"),
                              command.get("action"), command.get("rule"))
            self.rules = self.store.state(self.key)["rules"]
            self.publish_mask()
            self.publish_state()
            result = {"request_id": request_id, "ok": True}
        except (ValueError, TypeError, KeyError, AttributeError, OSError, json.JSONDecodeError) as exc:
            result = {"request_id": request_id, "ok": False, "error": str(exc)}
            self.get_logger().warn(f"area rule command rejected: {exc}")
        self.ack_pub.publish(String(data=json.dumps(result)))

    def check_expiry(self):
        if self.key and self.store.expire(self.key):
            self.rules = self.store.state(self.key)["rules"]
            self.publish_mask()
            self.publish_state()

    def publish_control(self):
        control = {"ready": False, "stop": True, "speed_limit": None, "map_key": self.key}
        if self.grid is not None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.grid.header.frame_id or "map", self.robot_frame, Time())
                stamp = Time.from_msg(transform.header.stamp)
                now = self.get_clock().now()
                if now.nanoseconds > 0 and stamp.nanoseconds > 0 and (
                    abs((now - stamp).nanoseconds) > 1_000_000_000
                ):
                    self.control_pub.publish(String(data=json.dumps(control)))
                    return
                position = transform.transform.translation
                heading = yaw_of(transform.transform.rotation)
                # Check both the current footprint and a short forward envelope.
                points = [(position.x, position.y)]
                for offset in (0.2, 0.4):
                    points.append((position.x + offset * math.cos(heading),
                                   position.y + offset * math.sin(heading)))
                stop = any(covers(rule, x, y, ROBOT_RADIUS)
                           for rule in self.rules if rule["type"] in BLOCKING_TYPES
                           for x, y in points)
                limits = [rule["limit_mps"] for rule in self.rules if rule["type"] == "speed"
                          and any(covers(rule, x, y, ROBOT_RADIUS) for x, y in points)]
                control.update(ready=True, stop=stop,
                               speed_limit=min(limits) if limits else None)
            except TransformException:
                pass
        self.control_pub.publish(String(data=json.dumps(control)))


def main(args=None):
    rclpy.init(args=args)
    node = AreaRules()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
