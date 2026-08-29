#!/usr/bin/env python

"""
NeupanCore is the main ROS2 node for the NeuPAN navigation algorithm.

This node subscribes to laser scan and localization data, executes the NeuPAN
planning algorithm, and publishes velocity commands to control the robot.

Developer: Han Ruihua <hanrh@connect.hku.hk>  Li Chengyang <kevinladlee@gmail.com>
Date: 2025.04.08
"""
import os
import threading
import time
import traceback
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

try:
    from neupan import neupan
    from neupan.util import get_transform
except ImportError as e:
    raise ImportError(
        f"Failed to import 'neupan' package: {e}. "
        "Please install NeuPAN first."
    ) from e

# Import local modules
from neupan_ros2.visualization_manager import VisualizationManager
from neupan_ros2.utils import yaw_to_quat, quat_to_yaw


class NeupanCore(Node):
    """ROS2 node for NeuPAN navigation algorithm.

    This node integrates the NeuPAN planner with ROS2, handling sensor data,
    executing planning, and publishing control commands.
    """

    def __init__(self) -> None:
        super().__init__("neupan_node")

        # Thread lock protecting shared state: robot_state, obstacle_points, stop, arrive
        # These are accessed by both control thread (run) and callback thread (scan/path/goal)
        self._state_lock = threading.Lock()

        # Callback groups for multi-threaded execution
        # Control group: MutuallyExclusive for timer (run) - ensures run() executes alone
        self.control_group = MutuallyExclusiveCallbackGroup()
        # Callback group: Reentrant for all subscriptions - allows concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Package directory for accessing config files and models
        self.pkg_dir = get_package_share_directory("neupan_ros2")

        # Robot identification and configuration directory
        self.declare_parameter("robot_type", "")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_config_dir", "")  # Set by launch file
        self.declare_parameter("planner_config_file", "planner.yaml")
        self.declare_parameter("dune_checkpoint_file", "models/dune_model_5000.pth")

        # Legacy parameter name (for backward compatibility)
        self.declare_parameter("neupan_config_file", "NOT SET")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "laser_link")
        self.declare_parameter("scan_tf_max_age", 0.25)
        self.declare_parameter("scan_data_timeout", 0.5)
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("marker_z", 1.0)
        self.declare_parameter("scan_angle_max", 3.14)
        self.declare_parameter("scan_angle_min", -3.14)
        self.declare_parameter("scan_downsample", 1)
        self.declare_parameter("scan_range_min", 0.1)
        self.declare_parameter("scan_range_max", 5.0)
        self.declare_parameter("refresh_initial_path", False)
        self.declare_parameter("flip_angle", False)
        self.declare_parameter("include_initial_path_direction", False)
        self.declare_parameter("direct_goal_planning", True)
        self.declare_parameter("control_frequency", 50.0)  # Control loop frequency in Hz
        self.declare_parameter("command_rate_limit", True)
        self.declare_parameter("executor_threads", 4)

        # Visualization control parameters
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("enable_dune_markers", True)
        self.declare_parameter("enable_nrmp_markers", True)
        self.declare_parameter("enable_robot_marker", True)

        # Topic names (configurable for flexibility)
        self.declare_parameter("cmd_vel_topic", "/neupan_cmd_vel")
        self.declare_parameter("plan_output_topic", "/neupan_plan")
        self.declare_parameter("ref_state_topic", "/neupan_ref_state")
        self.declare_parameter("initial_path_topic", "/neupan_initial_path")
        self.declare_parameter("dune_markers_topic", "/dune_point_markers")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("nrmp_markers_topic", "/nrmp_point_markers")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_input_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")

        # === Configuration Loading ===
        # Get robot configuration directory (set by launch file)
        robot_config_dir = (
            self.get_parameter("robot_config_dir")
            .get_parameter_value().string_value
        )

        # Validate robot config directory exists
        if not robot_config_dir or not os.path.isdir(robot_config_dir):
            raise ValueError(
                f"Invalid robot_config_dir: '{robot_config_dir}'. "
                "Must be set by launch file to a valid robot config directory."
            )

        # Get robot type for logging
        robot_type = (
            self.get_parameter("robot_type")
            .get_parameter_value().string_value
        )
        robot_description = (
            self.get_parameter("robot_description")
            .get_parameter_value().string_value
        )

        self.get_logger().info(f"Loading robot configuration: {robot_type}")
        self.get_logger().info(f"Description: {robot_description}")
        self.get_logger().info(f"Config directory: {robot_config_dir}")

        # Load planner configuration (relative to robot config dir)
        planner_config_file = (
            self.get_parameter("planner_config_file")
            .get_parameter_value().string_value
        )
        self.planner_config_file = os.path.join(robot_config_dir, planner_config_file)

        # Load DUNE checkpoint (relative to robot config dir)
        dune_checkpoint_file = (
            self.get_parameter("dune_checkpoint_file")
            .get_parameter_value().string_value
        )
        self.dune_checkpoint = os.path.join(robot_config_dir, dune_checkpoint_file)

        # Validate configuration files exist
        if not os.path.isfile(self.planner_config_file):
            raise FileNotFoundError(
                f"Planner config not found: {self.planner_config_file}"
            )
        if not os.path.isfile(self.dune_checkpoint):
            raise FileNotFoundError(
                f"DUNE checkpoint not found: {self.dune_checkpoint}"
            )

        self.get_logger().info(f"Planner config: {self.planner_config_file}")
        self.get_logger().info(f"DUNE checkpoint: {self.dune_checkpoint}")

        # Load other parameters
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self.scan_tf_max_age = (
            self.get_parameter("scan_tf_max_age")
            .get_parameter_value().double_value
        )
        if self.scan_tf_max_age <= 0.0:
            raise ValueError("scan_tf_max_age must be greater than zero")
        self.scan_data_timeout = (
            self.get_parameter("scan_data_timeout")
            .get_parameter_value().double_value
        )
        if self.scan_data_timeout <= 0.0:
            raise ValueError("scan_data_timeout must be greater than zero")
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value

        self.scan_range = np.array([
            self.get_parameter("scan_range_min").get_parameter_value().double_value,
            self.get_parameter("scan_range_max").get_parameter_value().double_value
        ])

        self.scan_angle_range = np.array([
            self.get_parameter("scan_angle_min").get_parameter_value().double_value,
            self.get_parameter("scan_angle_max").get_parameter_value().double_value
        ])

        self.scan_downsample = (
            self.get_parameter("scan_downsample")
            .get_parameter_value().integer_value
        )

        self.refresh_initial_path = (
            self.get_parameter("refresh_initial_path")
            .get_parameter_value().bool_value
        )
        self.flip_angle = (
            self.get_parameter("flip_angle")
            .get_parameter_value().bool_value
        )
        self.include_initial_path_direction = (
            self.get_parameter("include_initial_path_direction")
            .get_parameter_value().bool_value
        )
        self.direct_goal_planning = (
            self.get_parameter("direct_goal_planning")
            .get_parameter_value().bool_value
        )
        self.command_rate_limit = (
            self.get_parameter("command_rate_limit")
            .get_parameter_value().bool_value
        )
        self.executor_threads = (
            self.get_parameter("executor_threads")
            .get_parameter_value().integer_value
        )
        if self.executor_threads < 2:
            raise ValueError("executor_threads must be at least 2")

        self.enable_visualization = (
            self.get_parameter("enable_visualization")
            .get_parameter_value().bool_value
        )
        self.enable_dune_markers = (
            self.get_parameter("enable_dune_markers")
            .get_parameter_value().bool_value
        )
        self.enable_nrmp_markers = (
            self.get_parameter("enable_nrmp_markers")
            .get_parameter_value().bool_value
        )
        self.enable_robot_marker = (
            self.get_parameter("enable_robot_marker")
            .get_parameter_value().bool_value
        )

        if self.refresh_initial_path:
            self.get_logger().info("Refresh initial path is enabled")

        if not self.planner_config_file:
            raise ValueError(
                "No planner config file provided! "
                "Please set the parameter 'config_file'"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)

        # Log robot dimensions for verification
        self.get_logger().info(
            f"Robot dimensions - Length: {self.neupan_planner.robot.length:.3f}m, "
            f"Width: {self.neupan_planner.robot.width:.3f}m"
        )
        self.wheelbase = getattr(self.neupan_planner.robot, 'L', None)
        if self.wheelbase is not None and self.wheelbase > 0.0:
            self.get_logger().info(
                f"Robot wheelbase: {self.wheelbase:.3f}m"
            )
        elif self.neupan_planner.robot.kinematics == 'acker':
            raise ValueError("Ackermann NeuPAN requires a positive wheelbase")
        self.get_logger().info(f"Robot kinematics: {self.neupan_planner.robot.kinematics}")
        self.get_logger().info(
            f"PAN config - horizon: {self.neupan_planner.T} steps, "
            f"step: {self.neupan_planner.dt:.3f}s, "
            f"iterations: {self.neupan_planner.pan.iter_num}, "
            f"DUNE points: {self.neupan_planner.pan.dune_max_num}, "
            f"NRMP points: {self.neupan_planner.pan.nrmp_max_num}"
        )
        self.get_logger().info("NeuPAN planner initialized successfully")

        # Shared state protected by _state_lock (accessed by multiple threads)
        # Write access: scan_callback (obstacle_points), _get_robot_transform (robot_state)
        # Read access: _execute_planning (all), generate_twist_msg (stop, arrive)
        # Planning copies data before execution to minimize lock holding time
        self.obstacle_points: Optional[npt.NDArray] = None  # (2, n) obstacle points in map frame
        self.scan_ready: bool = False  # A scan and a usable TF have been received
        # A single TF miss must not turn into an emergency stop.  Obstacle
        # points are already expressed in the map frame, so the most recent
        # valid set remains usable for a short, explicitly bounded interval.
        self._last_valid_scan_time_ns: Optional[int] = None
        self.robot_state: Optional[npt.NDArray] = None  # (3, 1) [x, y, theta] in map frame
        self.stop: bool = False  # Emergency stop flag from collision detection
        self.arrive: bool = False  # Goal reached flag
        self.goal: Optional[npt.NDArray] = None  # (3, 1) target goal [x, y, theta]
        # Last command in NeuPAN's native [speed, steering_angle] space.
        # The planner's acceleration constraint applies between points inside
        # its horizon, but does not constrain the first command against the
        # command already sent to the vehicle.
        self._last_command = np.zeros(2, dtype=float)
        self._last_command_time = time.monotonic()

        self.vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10
        )
        self.plan_pub = self.create_publisher(
            Path,
            self.get_parameter("plan_output_topic").get_parameter_value().string_value,
            10
        )
        self.ref_state_pub = self.create_publisher(
            Path,
            self.get_parameter("ref_state_topic").get_parameter_value().string_value,
            10
        )
        self.ref_path_pub = self.create_publisher(
            Path,
            self.get_parameter("initial_path_topic").get_parameter_value().string_value,
            10
        )

        # Initialize visualization manager (handles all visualization independently)
        viz_config = {
            'enable_visualization': self.enable_visualization,
            'enable_dune_markers': self.enable_dune_markers,
            'enable_nrmp_markers': self.enable_nrmp_markers,
            'enable_robot_marker': self.enable_robot_marker,
            'map_frame': self.map_frame,
            'marker_size': self.marker_size,
            'marker_z': self.marker_z,
            'dune_markers_topic': (
                self.get_parameter("dune_markers_topic")
                .get_parameter_value().string_value
            ),
            'nrmp_markers_topic': (
                self.get_parameter("nrmp_markers_topic")
                .get_parameter_value().string_value
            ),
            'robot_marker_topic': (
                self.get_parameter("robot_marker_topic")
                .get_parameter_value().string_value
            ),
            'state_lock': self._state_lock
        }
        self.viz_manager = VisualizationManager(self, viz_config)

        # TF listener for coordinate transformations (default 10s buffer)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor data must be processed live.  A deeper queue makes a slow
        # Python callback work through stale scans forever while new scans
        # keep arriving, and their old timestamps then fail the TF age check.
        scan_qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").get_parameter_value().string_value,
            self.scan_callback,
            scan_qos_profile,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Path,
            self.get_parameter("plan_input_topic").get_parameter_value().string_value,
            self.path_callback,
            10,
            callback_group=self.callback_group
        )
        if self.direct_goal_planning:
            self.create_subscription(
                PoseStamped,
                self.get_parameter("goal_topic").get_parameter_value().string_value,
                self.goal_callback,
                10,
                callback_group=self.callback_group
            )
        else:
            self.get_logger().info(
                "Direct goal planning is disabled; using the global planner path only"
            )

        # Control loop timer: frequency configurable via parameter
        self.control_frequency = (
            self.get_parameter("control_frequency")
            .get_parameter_value().double_value
        )
        if self.control_frequency <= 0:
            raise ValueError(
                f"Invalid control_frequency: {self.control_frequency}. "
                "Must be > 0 Hz"
            )

        time_period = 1.0 / self.control_frequency
        self.get_logger().info(
            f"Control loop frequency: {self.control_frequency} Hz "
            f"({time_period*1000:.1f} ms period)"
        )
        if self.command_rate_limit and self.neupan_planner.robot.kinematics == 'acker':
            # max_acce is expressed per second.  Use the elapsed wall time
            # between published planner commands rather than the nominal
            # timer period: a CPU-bound MPC solve can make the effective
            # command period much longer than 1/control_frequency.
            self._command_rate_limit = np.asarray(
                self.neupan_planner.robot.max_acce, dtype=float
            ).reshape(2)
            self._command_rate_limit = np.where(
                np.isfinite(self._command_rate_limit),
                np.maximum(self._command_rate_limit, 0.0),
                np.inf,
            )
            self.get_logger().info(
                "Ackermann command rate limiter enabled - "
                f"max rate d[v,ψ]/dt: {self._command_rate_limit[0]:.3f}, "
                f"{self._command_rate_limit[1]:.3f}"
            )
        else:
            self._command_rate_limit = np.full(2, np.inf, dtype=float)
        self.create_timer(time_period, self.run, callback_group=self.control_group)

    def _lookup_current_robot_pose(self) -> npt.NDArray:
        """Return the current robot pose in ``map`` using the odom chain.

        hdl_localization may publish a slowly updated/delayed map-to-odom
        alignment.  Looking up a directly composed map-to-base transform at
        time zero can therefore return an old robot pose.  Compose the latest
        map alignment with the high-rate odom-to-state-frame transform so the
        planner and scan processing use the same current motion estimate.
        """
        timeout = Duration(seconds=0.05)
        if self.map_frame == self.odom_frame:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(), timeout=timeout
            )
            return self._transform_pose_xy_yaw(transform)

        try:
            map_to_odom = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=timeout,
            )
            odom_to_robot = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=timeout,
            )
            return self._compose_planar_pose(
                self._transform_pose_xy_yaw(map_to_odom),
                self._transform_pose_xy_yaw(odom_to_robot),
            )
        except tf2_ros.TransformException as chain_error:
            # Keep compatibility with deployments that publish only a direct
            # map-to-robot transform.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=timeout,
                )
                self.get_logger().warn(
                    "Using direct map-to-robot TF; odom compensation unavailable: "
                    f"{chain_error}",
                    throttle_duration_sec=2.0,
                )
                return self._transform_pose_xy_yaw(transform)
            except tf2_ros.TransformException as direct_error:
                raise tf2_ros.TransformException(
                    f"No usable robot TF chain: {chain_error}; "
                    f"direct lookup failed: {direct_error}"
                ) from direct_error

    def _get_robot_transform(self) -> bool:
        """Get robot transform from TF and update robot_state.

        Returns:
            bool: True if transform successfully obtained, False otherwise

        """
        try:
            # TF query is thread-safe, no lock needed.
            x, y, yaw = self._lookup_current_robot_pose()
            new_state = np.array([x, y, yaw]).reshape(3, 1)

            # Lock only for writing shared state
            with self._state_lock:
                self.robot_state = new_state

            self.get_logger().info(
                f"Robot state initialized - x: {new_state[0,0]:.2f}m, "
                f"y: {new_state[1,0]:.2f}m, yaw: {new_state[2,0]:.2f}rad",
                once=True
            )
            return True

        except tf2_ros.LookupException:
            self.get_logger().debug(
                f"Waiting for transform from {self.base_frame} to {self.map_frame}",
                throttle_duration_sec=1.0,
            )
            return False
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(
                "ConnectivityException: Transform not available, waiting for connection",
                throttle_duration_sec=1.0
            )
            return False
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f"TF extrapolation error: {e}. Check TF timestamps and buffer size.",
                throttle_duration_sec=1.0
            )
            return False
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"Robot TF unavailable: {e}",
                throttle_duration_sec=1.0,
            )
            return False

    @staticmethod
    def _transform_pose_xy_yaw(transform: Any) -> npt.NDArray:
        """Extract a planar pose from a TransformStamped message."""
        return np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            quat_to_yaw(transform.transform.rotation),
        ], dtype=float)

    @staticmethod
    def _compose_planar_pose(
            parent_pose: npt.NDArray, child_pose: npt.NDArray
    ) -> npt.NDArray:
        """Compose two planar poses, parent->middle and middle->child."""
        parent_yaw = parent_pose[2]
        cos_yaw = np.cos(parent_yaw)
        sin_yaw = np.sin(parent_yaw)
        return np.array([
            parent_pose[0] + cos_yaw * child_pose[0] - sin_yaw * child_pose[1],
            parent_pose[1] + sin_yaw * child_pose[0] + cos_yaw * child_pose[1],
            parent_yaw + child_pose[2],
        ], dtype=float)

    @staticmethod
    def _stamp_nanoseconds(stamp: Any) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _transform_age(self, transform: Any, requested_time: rclpy.time.Time) -> float:
        """Return the absolute age between a TF stamp and a scan stamp."""
        transform_stamp = self._stamp_nanoseconds(transform.header.stamp)
        if transform_stamp == 0 or requested_time.nanoseconds == 0:
            return 0.0
        return abs(requested_time.nanoseconds - transform_stamp) * 1e-9

    def _lookup_scan_pose(
            self, scan_frame: str, scan_time: rclpy.time.Time
    ) -> npt.NDArray:
        """Find the planar map pose of a scan frame without using stale map TF.

        hdl_localization can publish a delayed ``map -> odom`` transform.  A
        direct ``map -> laser`` lookup at the scan timestamp then fails, and
        falling back to the latest composed transform mixes an old map pose
        with a new scan.  The odometry transform is high-rate and buffered, so
        compose the latest map alignment with ``odom -> laser`` at scan time.
        """
        # Do not block a 20 Hz sensor callback waiting for transforms.  If a
        # transform is a few milliseconds late, use the bounded latest-odom
        # fallback below or process the next (newest-only) scan.
        timeout = Duration(seconds=0.0)

        if scan_time.nanoseconds == 0:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, scan_frame, rclpy.time.Time(), timeout=timeout
            )
            return self._transform_pose_xy_yaw(transform)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, scan_frame, scan_time, timeout=timeout
            )
            return self._transform_pose_xy_yaw(transform)
        except tf2_ros.TransformException as direct_error:
            if self.map_frame == self.odom_frame:
                raise direct_error

            try:
                map_to_odom = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.odom_frame,
                    rclpy.time.Time(),
                    timeout=timeout,
                )

                try:
                    odom_to_scan = self.tf_buffer.lookup_transform(
                        self.odom_frame,
                        scan_frame,
                        scan_time,
                        timeout=timeout,
                    )
                    odom_tf_source = "scan timestamp"
                except tf2_ros.TransformException as odom_time_error:
                    # A simulator may stamp the scan a few milliseconds ahead
                    # of the newest odometry message.  Permit only a bounded
                    # latest-odom fallback; never reuse a seconds-old pose.
                    odom_to_scan = self.tf_buffer.lookup_transform(
                        self.odom_frame,
                        scan_frame,
                        rclpy.time.Time(),
                        timeout=timeout,
                    )
                    odom_age = self._transform_age(odom_to_scan, scan_time)
                    if odom_age > self.scan_tf_max_age:
                        raise tf2_ros.TransformException(
                            f"latest {self.odom_frame}->{scan_frame} TF is "
                            f"{odom_age:.3f}s from the scan timestamp; "
                            f"timestamped lookup failed: {odom_time_error}"
                        ) from odom_time_error
                    odom_tf_source = f"latest odom TF ({odom_age:.3f}s)"

                map_pose = self._transform_pose_xy_yaw(map_to_odom)
                odom_scan_pose = self._transform_pose_xy_yaw(odom_to_scan)
                scan_pose = self._compose_planar_pose(map_pose, odom_scan_pose)
                self.get_logger().info(
                    "Timestamped map-to-scan TF unavailable; using "
                    f"map-to-odom + odom-to-scan ({odom_tf_source})",
                    throttle_duration_sec=5.0,
                )
                return scan_pose

            except tf2_ros.TransformException as chain_error:
                # Support other deployments where a direct map transform is
                # available and fresh, while rejecting the stale transform
                # that caused the original moving-scan misalignment.
                try:
                    latest_direct = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        scan_frame,
                        rclpy.time.Time(),
                        timeout=timeout,
                    )
                    direct_age = self._transform_age(latest_direct, scan_time)
                    if direct_age <= self.scan_tf_max_age:
                        self.get_logger().warn(
                            "Using fresh latest map-to-scan TF fallback "
                            f"({direct_age:.3f}s from scan timestamp)",
                            throttle_duration_sec=1.0,
                        )
                        return self._transform_pose_xy_yaw(latest_direct)
                except tf2_ros.TransformException:
                    pass

                raise tf2_ros.TransformException(
                    "No usable scan TF: direct timestamped lookup failed "
                    f"({direct_error}); odom compensation failed "
                    f"({chain_error})"
                ) from chain_error

    def _validate_planning_prerequisites(self) -> bool:
        """Validate all prerequisites for planning are met.

        Returns:
            bool: True if all prerequisites are met, False otherwise

        """
        with self._state_lock:
            if self.robot_state is None:
                self.get_logger().debug("Waiting for robot state", throttle_duration_sec=1.0)
                return False

            # Initialize path from waypoints on first run if no path received
            if (len(self.neupan_planner.waypoints) >= 1
                    and self.neupan_planner.initial_path is None):
                self.neupan_planner.set_initial_path_from_state(
                    self.robot_state
                )
                self.get_logger().info(
                    f'Initialized path with '
                    f'{len(self.neupan_planner.waypoints)} waypoints'
                )

            if self.neupan_planner.initial_path is None:
                self.get_logger().debug("Waiting for initial path", throttle_duration_sec=1.0)
                return False

        return True

    def _execute_planning(self) -> Tuple[Optional[npt.NDArray], Dict[str, Any]]:
        """Execute planning and update state.

        Returns:
            tuple: (action, info) from neupan planner, or (None, None) on failure

        """
        # Publish reference path (generate message needs to read planner state)
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            initial_path = self.neupan_planner.initial_path

        # Publishing is thread-safe, do outside lock
        self.ref_path_pub.publish(self.generate_path_msg(initial_path))

        # Step 1: Fast data copy inside lock to minimize lock holding time
        with self._state_lock:
            # Copy state data for planning
            # (allows other threads to access shared state)
            robot_state_copy = (
                self.robot_state.copy()
                if self.robot_state is not None else None
            )
            obstacle_points_copy = (
                self.obstacle_points.copy()
                if self.obstacle_points is not None else None
            )
            scan_age = None
            if self._last_valid_scan_time_ns is not None:
                scan_age = (now_ns - self._last_valid_scan_time_ns) * 1e-9
            scan_ready = (
                self.scan_ready
                and scan_age is not None
                and 0.0 <= scan_age <= self.scan_data_timeout
            )

            # Check for obstacles
            has_obstacles = obstacle_points_copy is not None

        # A missing scan is a sensor/TF fault, not an obstacle-free scene.
        # Return before the CPU-heavy planner call so scan and TF callbacks
        # can recover instead of being starved by repeated futile solves.
        if not scan_ready:
            info = {
                "stop": True,
                "arrive": False,
                "sensor_fault": True,
                "opt_state_list": [],
                "ref_state_list": [],
            }
            with self._state_lock:
                self.stop = True
                self.arrive = False
            self.get_logger().warn(
                "No recent usable LaserScan; stopping for safety "
                f"(timeout: {self.scan_data_timeout:.2f}s)",
                throttle_duration_sec=1.0,
            )
            return None, info

        # Step 2: Execute planning OUTSIDE lock.
        action, info = self.neupan_planner(
            robot_state_copy, obstacle_points_copy
        )
        info["sensor_fault"] = False
        if info.get("avoidance_seeded", False):
            side = "left" if info.get("avoidance_side", 1) > 0 else "right"
            self.get_logger().info(
                f"Path-blocking obstacle detected; seeding {side} bypass",
                throttle_duration_sec=2.0,
            )

        # Step 3: Write back results inside lock (< 0.1 μs)
        with self._state_lock:
            self.stop = info["stop"]
            self.arrive = info["arrive"]

        # Logging outside lock
        if not has_obstacles:
            self.get_logger().info(
                "LaserScan received but no finite obstacle returns; performing path tracking only",
                throttle_duration_sec=1.0,
            )

        # Log arrival
        if info["arrive"]:
            self.get_logger().info("Arrived at target", once=True)

        # Log stop condition
        if info["stop"] and not info.get("sensor_fault", False):
            # Read min_distance and threshold outside lock
            # (assume read-only access is safe)
            self.get_logger().warn(
                f"Collision risk detected - "
                f"min distance: {self.neupan_planner.min_distance:.2f}m, "
                f"threshold: {self.neupan_planner.collision_threshold:.2f}m",
                throttle_duration_sec=1.0,
            )

        return action, info

    def _publish_planning_results(
            self, action: Optional[npt.NDArray], info: Dict[str, Any]
    ) -> None:
        """Publish planning results and visualization markers.

        Args:
            action: Control action from planner
            info: Planning info dictionary

        """
        # Publish path messages (info is local, thread-safe)
        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))

        # Generate twist message using info dict (avoid reading shared state)
        vel_msg = self.generate_twist_msg(action, info["stop"], info["arrive"])
        self.vel_pub.publish(vel_msg)

        # Visualization (delegated to visualization manager)
        self.viz_manager.publish_visualization(
            self.neupan_planner, self.robot_state
        )

    def run(self) -> None:
        """Execute main control loop at fixed frequency."""
        # Step 1: Get robot transform (locks internally for robot_state write)
        if not self._get_robot_transform():
            return

        # Step 2: Validate planning prerequisites (locks internally for state read)
        if not self._validate_planning_prerequisites():
            return

        # Step 3: Execute planning (locks internally for planning execution)
        action, info = self._execute_planning()

        # Step 4: Publish results (locks internally for marker generation)
        self._publish_planning_results(action, info)

    def scan_callback(self, scan_msg: LaserScan) -> Optional[npt.NDArray]:
        """Process laser scan data and update obstacle points in map frame.

        Args:
            scan_msg: LaserScan message from sensor

        Returns:
            Transformed obstacle points or None if processing failed

        """
        # Quick check if robot state is available (lock briefly)
        with self._state_lock:
            if self.robot_state is None:
                return None

        ranges = np.array(scan_msg.ranges)
        if scan_msg.angle_increment > 0.0:
            angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        else:
            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        if self.flip_angle:
            angles = np.flip(angles)

        # Vectorized filtering: Apply downsampling, range, and angle constraints
        indices = np.arange(len(ranges))
        downsample_mask = (indices % self.scan_downsample) == 0
        # LaserScan uses +inf (and some converters use range_max) for a ray
        # with no return.  Neither represents an obstacle.  Exclude the
        # advertised upper endpoint as well as non-finite values so these
        # rays cannot occupy DUNE's finite obstacle-point budget.
        scan_range_max = min(self.scan_range[1], float(scan_msg.range_max))
        range_mask = (
            np.isfinite(ranges)
            & (ranges >= self.scan_range[0])
            & (ranges < scan_range_max)
        )
        angle_mask = (angles > self.scan_angle_range[0]) & (angles < self.scan_angle_range[1])

        valid_mask = downsample_mask & range_mask & angle_mask
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) == 0:
            # Update obstacle_points with lock
            with self._state_lock:
                self.obstacle_points = None
                # The scan stream is healthy; it simply contains no finite
                # returns inside the configured range.
                self.scan_ready = True
                self._last_valid_scan_time_ns = self.get_clock().now().nanoseconds
            self.get_logger().warn(
                "LaserScan contains no finite returns inside configured range",
                throttle_duration_sec=1.0
            )
            return None

        # Vectorized coordinate computation (faster than loop)
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        point_array = np.vstack([x_coords, y_coords])

        scan_frame = scan_msg.header.frame_id or self.lidar_frame
        scan_time = rclpy.time.Time.from_msg(scan_msg.header.stamp)
        if scan_time.nanoseconds == 0:
            scan_time = rclpy.time.Time()

        try:
            scan_pose = self._lookup_scan_pose(scan_frame, scan_time)
            x, y, yaw = scan_pose

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            transformed_points = rot_matrix @ point_array + trans_matrix

            # Lock only for writing shared state
            with self._state_lock:
                self.obstacle_points = transformed_points
                self.scan_ready = True
                self._last_valid_scan_time_ns = self.get_clock().now().nanoseconds

            self.get_logger().info(
                f"Laser scan initialized with {transformed_points.shape[1]} "
                f"points (frame={scan_frame})", once=True
            )
            return transformed_points

        except tf2_ros.TransformException as e:
            # Keep the last map-frame obstacle set until scan_data_timeout.
            # Transient TF extrapolation is common when Gazebo publishes the
            # scan a few milliseconds before the corresponding odometry TF.
            self.get_logger().warn(
                f"LaserScan TF unavailable; retaining last valid scan: {e}",
                throttle_duration_sec=2.0
            )
            return

    def path_callback(self, path: Path) -> None:
        """Update initial path from received path message.

        Args:
            path: Path message containing waypoints

        """
        n_poses = len(path.poses)
        if n_poses == 0:
            return

        self.get_logger().info(f"Received new path with {n_poses} waypoints")

        # Optimized: single-pass extraction with transpose
        if self.include_initial_path_direction:
            # Extract x, y, and orientation from path pose quaternions
            data = [
                (p.pose.position.x, p.pose.position.y,
                 quat_to_yaw(p.pose.orientation))
                for p in path.poses
            ]
            xs, ys, thetas = np.array(data).T

            # Compute gear for each waypoint:
            # Theta from pose = vehicle heading. Segment direction = atan2(dy, dx).
            # If heading aligns with travel direction → forward (gear=1).
            # If heading opposes travel direction → reverse (gear=-1).
            gears = np.ones(n_poses)  # default forward
            if n_poses > 1:
                # Segment directions: waypoint i → waypoint i+1
                seg_dirs = np.arctan2(np.diff(ys), np.diff(xs))
                for i in range(n_poses - 1):
                    # cos(angle between heading and segment direction)
                    if np.cos(thetas[i] - seg_dirs[i]) < 0:
                        gears[i] = -1
                # Last waypoint inherits the gear of the previous one
                gears[-1] = gears[-2]
        else:
            self.get_logger().debug(
                "Using path gradient for direction "
                "(include_initial_path_direction=False)", once=True
            )

            # Extract x, y in one pass
            coords = [
                (p.pose.position.x, p.pose.position.y) for p in path.poses
            ]
            xs, ys = np.array(coords).T

            # Vectorized gradient computation using np.diff
            dx = np.diff(xs, append=xs[-1])
            dy = np.diff(ys, append=ys[-1])
            thetas = np.arctan2(dy, dx)

            # For the last point, use direction from second-to-last point
            if n_poses > 1:
                thetas[-1] = thetas[-2]

            # Without pose orientations, cannot detect reverse — default all forward
            gears = np.ones(n_poses)

        # Vectorized array construction for better performance
        # Shape: (4, n_poses) — [x, y, theta, gear]
        initial_point_array = np.vstack([xs, ys, thetas, gears])

        # Convert to list of column vectors for planner API compatibility
        initial_point_list = [
            initial_point_array[:, i:i + 1] for i in range(n_poses)
        ]

        with self._state_lock:
            if (self.neupan_planner.initial_path is None
                    or self.refresh_initial_path):
                self.neupan_planner.set_initial_path(initial_point_list)
                # CRITICAL: a new path (possibly from a new goal) means the
                # old arrive/stop flags are stale. Reset so the control loop
                # resumes publishing non-zero velocity.
                self.neupan_planner.reset()
                self.arrive = False
                self.stop = False

    def goal_callback(self, goal: PoseStamped) -> None:
        """Update goal and regenerate initial path.

        Args:
            goal: Goal pose message

        """
        # Extract goal from message (no lock needed)
        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = quat_to_yaw(goal.pose.orientation)

        new_goal = np.array([[x], [y], [theta]])

        self.get_logger().info(
            f"New goal set - x: {x:.2f}m, y: {y:.2f}m, "
            f"theta: {theta:.2f}rad"
        )

        # Check if robot state is ready
        if self.robot_state is None:
            self.get_logger().warn(
                "Goal received but robot state not yet available. "
                "Path planning will start once robot state is received."
            )
            self.goal = new_goal
            return

        # Lock only when accessing shared state and modifying planner
        with self._state_lock:
            self.goal = new_goal

            # CRITICAL: reset arrival/stop flags so the control loop resumes
            # publishing non-zero velocity for the new goal.
            self.arrive = False
            self.stop = False

            self.get_logger().debug(
                f"Current state: {self.robot_state.tolist()}"
            )
            self.get_logger().debug(f"Target goal: {self.goal.tolist()}")

            self.neupan_planner.update_initial_path_from_goal(
                self.robot_state, self.goal
            )
            self.neupan_planner.reset()

    def generate_path_msg(self, path_list: List[npt.NDArray]) -> Path:
        """Generate ROS Path message from list of poses.

        Args:
            path_list: List of pose arrays (3, 1) or (4, 1)
                       containing [x, y, theta, ...]

        Returns:
            Path message with poses

        """
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        if len(path_list) == 0:
            return path

        # Vectorized approach: normalize all points and stack into matrix
        normalized_points = []
        for point in path_list:
            point_arr = np.array(point)
            if point_arr.ndim == 1:
                point_arr = point_arr.reshape(-1, 1)
            # Extract only first 3 elements (x, y, theta) to ensure consistent dimensions
            point_arr = point_arr[:3, :]
            normalized_points.append(point_arr)

        # Stack all points horizontally -> shape: (3, n_poses)
        points_matrix = np.hstack(normalized_points)

        # Vectorized extraction (single op instead of 3 list comps)
        xs = points_matrix[0, :].tolist()
        ys = points_matrix[1, :].tolist()
        yaws = points_matrix[2, :].tolist()

        # Create path message
        for x, y, yaw in zip(xs, ys, yaws):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = yaw_to_quat(yaw)
            path.poses.append(ps)

        return path

    def generate_twist_msg(
            self, vel: Optional[npt.NDArray], stop: bool, arrive: bool
    ) -> Twist:
        """Generate ROS Twist message from velocity command.

        Args:
            vel: NeuPAN command array (2, 1) containing
                 [linear_speed, steering_angle], or None
            stop: Whether the robot should stop (collision risk)
            arrive: Whether the robot has arrived at goal

        Returns:
            Twist message (zero velocity if stopped/arrived or vel is None)

        """
        if vel is None:
            self._last_command[:] = 0.0
            self._last_command_time = time.monotonic()
            return Twist()

        if stop or arrive:
            self._last_command[:] = 0.0
            self._last_command_time = time.monotonic()
            return Twist()

        command = np.asarray(vel, dtype=float).reshape(2)
        speed, steer = command

        # Limit the first command against the command actually sent during
        # the preceding control cycle.  This prevents a fresh MPC solution
        # from jumping straight to the steering bound when the solver's
        # internal horizon starts from a zero/old nominal command.
        if self.neupan_planner.robot.kinematics == 'acker':
            if self.command_rate_limit:
                now = time.monotonic()
                elapsed = max(now - self._last_command_time, 0.0)
                self._last_command_time = now
                max_delta = self._command_rate_limit * elapsed
                robot_min = np.asarray(
                    self.neupan_planner.robot.min_speed, dtype=float
                ).reshape(2)
                robot_max = np.asarray(
                    self.neupan_planner.robot.max_speed, dtype=float
                ).reshape(2)
                lower = np.maximum(
                    robot_min, self._last_command - max_delta
                )
                upper = np.minimum(
                    robot_max, self._last_command + max_delta
                )
                limited_command = np.clip(command, lower, upper)
                if not np.allclose(limited_command, command, atol=1e-9):
                    self.get_logger().warn(
                        "Rate-limited NeuPAN command "
                        f"[v={command[0]:.3f}, ψ={command[1]:.3f}] -> "
                        f"[v={limited_command[0]:.3f}, ψ={limited_command[1]:.3f}]",
                        throttle_duration_sec=1.0,
                    )
                command = limited_command
            self._last_command = command.copy()
            speed, steer = command

        action = Twist()
        action.linear.x = float(speed)
        # NeuPAN's Ackermann command is [v, psi], where psi is the steering
        # angle.  ackermann_steering_controller consumes a body twist, so
        # angular.z must be the yaw rate omega instead.
        if self.neupan_planner.robot.kinematics == 'acker':
            action.angular.z = float(speed * np.tan(steer) / self.wheelbase)
        else:
            action.angular.z = float(steer)
        return action


def main(args=None):
    """Main entry point for NeuPAN node.

    Args:
        args: Command-line arguments (optional)

    """
    rclpy.init(args=args)

    neupan_node = None
    executor = None
    try:
        neupan_node = NeupanCore()

        # Keep dedicated capacity for the control timer, sensor callbacks and
        # TF listener.  NeuPAN optimization is CPU-heavy and may hold one
        # Python worker for substantially longer than the nominal period.
        executor = MultiThreadedExecutor(num_threads=neupan_node.executor_threads)
        executor.add_node(neupan_node)

        viz_status = (
            'enabled' if neupan_node.enable_visualization else 'disabled'
        )
        neupan_node.get_logger().info(
            f"NeuPAN node started - "
            f"Control: {neupan_node.control_frequency}Hz, "
            f"Threads: {neupan_node.executor_threads}, "
            f"Visualization: {viz_status}"
        )
        executor.spin()

    except KeyboardInterrupt:
        if neupan_node:
            neupan_node.get_logger().info(
                "NeuPAN node shutting down due to "
                "KeyboardInterrupt (Ctrl+C)."
            )
        pass
    except Exception as e:
        if neupan_node:
            neupan_node.get_logger().error(
                f'Unhandled exception: {e}\n{traceback.format_exc()}'
            )
        raise
    finally:
        if executor:
            executor.shutdown()
        if neupan_node:
            neupan_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
