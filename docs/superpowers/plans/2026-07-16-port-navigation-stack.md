# Port Navigation Stack — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port LIO-SAM, hdl_localization, ndt_omp, pcd2pgm, neupan_ros2, and robot_slam (Nav2 + bridge) into AckermannRobot-3D, adapting all parameters for the small Ackermann vehicle.

**Architecture:** Copy 4 packages from `~/my_robot` (ndt_omp, LIO-SAM, hdl_localization, pcd2pgm), copy neupan_ros2 from `~/AckermannRobot-2D`, add bridge scripts to ackermann_robot, create new robot_slam package for Nav2 integration.

**Tech Stack:** ROS 2 Humble, colcon, C++17, Python 3.10, Nav2, Gazebo 11, LIO-SAM, NeuPAN

## Global Constraints

- ROS 2 Humble with Gazebo Classic (11.x)
- `/imu/data` topic for IMU (not `/imu_raw`)
- `laser_link` frame for LiDAR (not `lidar3d_link`)
- 16-line LiDAR (N_SCAN=16, Horizon_SCAN=1800)
- Controllers: `ackermann_steering_controller` (ros2_control)
- IMU sensor already present in URDF (gyro_link publishing /imu/data)
- Vehicle: wheelbase 0.593m, max steering 0.52 rad

---

### Task 1: Copy ndt_omp from my_robot

**Files:**
- Copy: `~/my_robot/src/ndt_omp/*` → `src/ndt_omp/`

**Interfaces:**
- Produces: `ndt_omp` shared library — `pclomp::NormalDistributionsTransform` with OpenMP
- Consumed by: hdl_localization (Task 3)

This is a pure C++ library with no ROS dependencies (uses `libpcl-all-dev` directly). No parameter changes needed.

- [ ] **Step 1: Copy the package**

```bash
cp -r ~/my_robot/src/ndt_omp /home/young/AckermannRobot-3D/src/ndt_omp
```

- [ ] **Step 2: Verify files**

```bash
ls /home/young/AckermannRobot-3D/src/ndt_omp/
# Expected: CMakeLists.txt  package.xml  README.md  LICENSE  include/  src/  apps/
```

- [ ] **Step 3: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/ndt_omp
git commit -m "feat: add ndt_omp (OpenMP NDT/GICP library) from my_robot"
```

---

### Task 2: Copy and adapt LIO-SAM

**Files:**
- Copy: `~/my_robot/src/LIO-SAM/*` → `src/LIO-SAM/`
- Modify: `src/LIO-SAM/config/params.yaml` — adapt for small car

**Interfaces:**
- Consumes: `/imu/data` (IMU), `/points_raw` (LiDAR PointCloud2)
- Produces: `map→odom` TF, `/odometry/imu`, saved `GlobalMap.pcd` on shutdown
- Frame: `laser_link` for LiDAR

- [ ] **Step 1: Copy the package**

```bash
cp -r ~/my_robot/src/LIO-SAM /home/young/AckermannRobot-3D/src/LIO-SAM
```

- [ ] **Step 2: Edit params.yaml — adapt topics and frames**

Read `src/LIO-SAM/config/params.yaml` and apply these changes:

Change line 6 — IMU topic:
```yaml
    imuTopic: "/imu/data"
```

Change line 11 — LiDAR frame:
```yaml
    lidarFrame: "laser_link"
```

Change line 28 — 16 lines (was 32):
```yaml
    N_SCAN: 16
```

Change line 29 — 1800 horizontal samples (was 360):
```yaml
    Horizon_SCAN: 1800
```

Change line 44 — LiDAR extrinsic relative to base_link (laser_joint is at x=-0.078, z=0.384):
```yaml
    extrinsicTrans:  [-0.078,  0.0,  0.384]
```

Change lines 59-61 — indoor voxel sizes:
```yaml
    odometrySurfLeafSize: 0.2
    mappingCornerLeafSize: 0.1
    mappingSurfLeafSize: 0.2
```

Change line 72 — shorter keyframe threshold for small car:
```yaml
    surroundingkeyframeAddingDistThreshold: 0.5
```

Change line 75 — smaller search radius:
```yaml
    surroundingKeyframeSearchRadius: 15.0
```

Change line 82 — smaller loop closure radius:
```yaml
    historyKeyframeSearchRadius: 5.0
```

Change line 91 — smaller visualization radius:
```yaml
    globalMapVisualizationSearchRadius: 50.0
```

- [ ] **Step 3: Edit launch/run.launch.py — fix IMU topic remap**

Read `src/LIO-SAM/launch/run.launch.py`. The static_transform_publisher creates `map→odom` identity. No changes needed to the launch file itself — the params file handles all adaptations.

- [ ] **Step 4: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/LIO-SAM
git commit -m "feat: add LIO-SAM adapted for small Ackermann robot (16-line LiDAR, /imu/data)"
```

---

### Task 3: Copy and adapt hdl_localization

**Files:**
- Copy: `~/my_robot/src/hdl_localization/*` → `src/hdl_localization/`
- Modify: `src/hdl_localization/config/params.yaml`

**Interfaces:**
- Consumes: `/points_raw` (LiDAR), `/imu/data` (IMU), `/globalmap` (from globalmap_server)
- Produces: `map→odom` TF, `/odom` (Odometry from NDT matching)
- Frame: `laser_link` (odom_child_frame_id)

- [ ] **Step 1: Copy the package**

```bash
cp -r ~/my_robot/src/hdl_localization /home/young/AckermannRobot-3D/src/hdl_localization
```

- [ ] **Step 2: Edit config/params.yaml**

Change `odom_child_frame_id` (line 11):
```yaml
    odom_child_frame_id: "laser_link"
```

Change `ndt_resolution` (line 20) — finer for small car:
```yaml
    ndt_resolution: 0.5
```

Change `use_imu` (line 38):
```yaml
    use_imu: true
```

The IMU topic remap (`/imu_raw` → `/imu/data`) will be done in the launch file — no change needed in params.yaml.

- [ ] **Step 3: Edit launch/hdl_localization.launch.py — add IMU remap for hdl_localization_node**

Read `src/hdl_localization/launch/hdl_localization.launch.py`. The hdl_localization_node subscribes to IMU. Add a remap:

```python
    hdl_localization = Node(
        package='hdl_localization',
        executable='hdl_localization_node',
        name='hdl_localization',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/imu_raw', '/imu/data'),
        ],
    )
```

Also update the default PCD path to point to the 3D maps directory:
```python
    default_pcd = os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'maps', 'GlobalMap.pcd')
```

- [ ] **Step 4: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/hdl_localization
git commit -m "feat: add hdl_localization adapted for small Ackermann robot (laser_link, /imu/data)"
```

---

### Task 4: Copy pcd2pgm from my_robot

**Files:**
- Copy: `~/my_robot/src/pcd2pgm/*` → `src/pcd2pgm/`

**Interfaces:**
- Produces: `pcd2gridmap` binary — converts 3D PCD to 2D PGM occupancy grid
- Consumed by: manual mapping workflow (user runs after LIO-SAM)

Pure copy, no changes needed.

- [ ] **Step 1: Copy the package**

```bash
cp -r ~/my_robot/src/pcd2pgm /home/young/AckermannRobot-3D/src/pcd2pgm
```

- [ ] **Step 2: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/pcd2pgm
git commit -m "feat: add pcd2pgm (3D PCD to 2D PGM converter)"
```

---

### Task 5: Copy neupan_ros2 from AckermannRobot-2D

**Files:**
- Copy: `~/AckermannRobot-2D/src/neupan_ros2/*` → `src/neupan_ros2/`

**Interfaces:**
- Consumes: `/scan` (LaserScan), `/plan` (Path from Nav2 planner), TF (`map→base_link`)
- Produces: `/neupan_cmd_vel` (Twist), `/neupan_plan` (Path visualization)

Already configured for `ackermann_robot` type. The `config/robots/ackermann_robot/` directory has the correct robot.yaml, planner.yaml, and DUNE model. No modifications needed.

- [ ] **Step 1: Copy the package (excluding build artifacts)**

```bash
mkdir -p /home/young/AckermannRobot-3D/src/neupan_ros2
# Copy everything except train/build, train/install, train/log, __pycache__
rsync -av --exclude='train/build' --exclude='train/install' --exclude='train/log' \
      --exclude='__pycache__' --exclude='*.pyc' \
      ~/AckermannRobot-2D/src/neupan_ros2/ /home/young/AckermannRobot-3D/src/neupan_ros2/
```

- [ ] **Step 2: Verify the ackermann_robot config is present**

```bash
ls /home/young/AckermannRobot-3D/src/neupan_ros2/config/robots/ackermann_robot/
# Expected: robot.yaml  planner.yaml  models/dune_model_5000.pth
```

- [ ] **Step 3: Verify robot.yaml parameters match the vehicle**

Read `src/neupan_ros2/config/robots/ackermann_robot/robot.yaml` — parameters should include:
- `laser_link` as `lidar_frame`
- `wheelbase: 0.593` in planner.yaml
- `length: 0.70`, `width: 0.52` in planner.yaml

- [ ] **Step 4: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/neupan_ros2
git commit -m "feat: add neupan_ros2 with ackermann_robot config from AckermannRobot-2D"
```

---

### Task 6: Add bridge scripts to ackermann_robot

**Files:**
- Create: `src/ackermann_robot/scripts/cmd_vel_stamper.py`
- Create: `src/ackermann_robot/scripts/cmd_vel_mux.py`
- Modify: `src/ackermann_robot/CMakeLists.txt`
- Create: `src/ackermann_robot/config/pcl_to_scan.yaml`

**Interfaces:**
- `cmd_vel_stamper`: consumes Twist (`/cmd_vel`) → produces TwistStamped (`/ackermann_steering_controller/reference`)
- `cmd_vel_mux`: consumes Twist from `/cmd_vel` + `/neupan_cmd_vel` → produces TwistStamped to `/ackermann_steering_controller/reference`
- `pcl_to_scan.yaml`: config for `pointcloud_to_laserscan` node (3D→2D)

- [ ] **Step 1: Create cmd_vel_stamper.py**

```bash
cp ~/AckermannRobot-2D/src/ackermann_robot/scripts/cmd_vel_stamper.py \
   /home/young/AckermannRobot-3D/src/ackermann_robot/scripts/cmd_vel_stamper.py
```

- [ ] **Step 2: Create cmd_vel_mux.py**

```bash
cp ~/AckermannRobot-2D/src/ackermann_robot/scripts/cmd_vel_mux.py \
   /home/young/AckermannRobot-3D/src/ackermann_robot/scripts/cmd_vel_mux.py
```

- [ ] **Step 3: Create pcl_to_scan.yaml**

Write `src/ackermann_robot/config/pcl_to_scan.yaml`:

```yaml
/**:
  ros__parameters:
    use_sim_time: true
    target_frame: laser_link
    # Height slice relative to laser_link (LiDAR at z≈0.384 in base_link)
    # Ground is below, scan horizontal forward
    min_height: -0.5
    max_height: 2.0
    # Full 360° scan
    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.0087   # ~0.5° per ray → 720 points
    scan_time: 0.1            # 10 Hz
    range_min: 0.15
    range_max: 25.0
    concurrency_level: 1
```

- [ ] **Step 4: Update CMakeLists.txt to install new scripts**

Read `src/ackermann_robot/CMakeLists.txt` (line 15-18, the `install(PROGRAMS ...)` block).

Old:
```cmake
install(PROGRAMS
  scripts/arrow_key_control.py
  DESTINATION lib/${PROJECT_NAME}
)
```

New:
```cmake
install(PROGRAMS
  scripts/arrow_key_control.py
  scripts/cmd_vel_stamper.py
  scripts/cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 5: Update package.xml to add pointcloud_to_laserscan dependency**

Read `src/ackermann_robot/package.xml`. Add this depend near other depends:

```xml
  <exec_depend>pointcloud_to_laserscan</exec_depend>
```

- [ ] **Step 6: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/ackermann_robot/scripts/cmd_vel_stamper.py \
        src/ackermann_robot/scripts/cmd_vel_mux.py \
        src/ackermann_robot/config/pcl_to_scan.yaml \
        src/ackermann_robot/CMakeLists.txt \
        src/ackermann_robot/package.xml
git commit -m "feat: add cmd_vel bridge scripts and pointcloud_to_laserscan config"
```

---

### Task 7: Create robot_slam package (Nav2 + launch files)

**Files:**
- Create: `src/robot_slam/package.xml`
- Create: `src/robot_slam/CMakeLists.txt`
- Create: `src/robot_slam/config/nav2_params.yaml`
- Create: `src/robot_slam/launch/slam.launch.py`
- Create: `src/robot_slam/launch/navigation_dwb.launch.py`
- Create: `src/robot_slam/launch/navigation_neupan.launch.py`

**Interfaces:**
- slam.launch.py: launches LIO-SAM (consumes `/points_raw`, `/imu/data`; produces mapping TF, saves PCD)
- navigation_dwb.launch.py: Nav2 full stack with DWB (consumes map.pgm via map_server; produces `/cmd_vel`)
- navigation_neupan.launch.py: Nav2 + NeuPAN (consumes `/scan`, `/plan`; produces `/neupan_cmd_vel`)

- [ ] **Step 1: Create package directory**

```bash
mkdir -p /home/young/AckermannRobot-3D/src/robot_slam/config
mkdir -p /home/young/AckermannRobot-3D/src/robot_slam/launch
```

- [ ] **Step 2: Write package.xml**

Write `src/robot_slam/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_slam</name>
  <version>0.0.1</version>
  <description>Nav2 navigation + LIO-SAM mapping for Ackermann robot with 3D LiDAR</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>lio_sam</exec_depend>
  <exec_depend>hdl_localization</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ackermann_robot</exec_depend>
  <exec_depend>pointcloud_to_laserscan</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

Write `src/robot_slam/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 4: Write nav2_params.yaml**

Write `src/robot_slam/config/nav2_params.yaml`. Use the exact content from AckermannRobot-2D's `nav2_params.yaml` (the DWB version, not neupan version). Key parameters:

```yaml
# ====== AMCL (localization — used when 2D method) ======
amcl:
  ros__parameters:
    use_sim_time: True
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: "/scan"
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    laser_max_range: 25.0
    laser_min_range: 0.15
    set_initial_pose: true
    always_reset_initial_pose: true
    initial_pose: [0.0, 0.0, 0.0]
    tf_broadcast: true
    transform_tolerance: 1.0
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    sigma_hit: 0.2
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    pf_err: 0.05
    pf_z: 0.99

# ====== BT Navigator ======
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom_wheel"
    bt_loop_duration: 10
    default_server_timeout: 20
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_goal_updated_controller_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_truncate_path_local_action_bt_node

# ====== Controller Server (DWB) ======
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.3
      movement_time_allowance: 15.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.3
      stateful: True
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: -0.5
      min_vel_y: 0.0
      max_vel_x: 1.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 1.5
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 1.5
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -1.5
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 20
      sim_time: 1.0
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.3
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.3
      GoalAlign.scale: 32.0
      GoalAlign.forward_point_distance: 0.3
      PathDist.scale: 32.0
      GoalDist.scale: 32.0
      RotateToGoal.scale: 20.0
      RotateToGoal.slowing_factor: 3.0
      RotateToGoal.lookahead_time: 1.0

# ====== Planner Server (SmacPlannerHybrid) ======
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      motion_model_for_search: "REEDS_SHEPP"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 2.0
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 1.05
      reverse_penalty: 2.1
      change_penalty: 0.15
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      retrospective_penalty: 0.1
      lookup_table_size: 20.0
      cache_obstacle_heuristic: true
      smooth_path: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.2
        w_data: 0.3
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2

# ====== Behavior Server ======
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    global_frame: "odom"
    robot_base_frame: "base_link"
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

# ====== Global Costmap ======
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.4
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      always_send_full_costmap: True

# ====== Local Costmap ======
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.4
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.15
          obstacle_max_range: 8.0
          obstacle_min_range: 0.15
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      always_send_full_costmap: True

# ====== Map Server ======
map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: ""

# ====== Velocity Smoother ======
velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [1.5, 0.0, 1.0]
    min_velocity: [-0.5, 0.0, -1.0]
    max_accel: [1.0, 0.0, 1.5]
    max_decel: [-1.0, 0.0, -1.5]
    odom_topic: "/odom_wheel"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

- [ ] **Step 5: Write slam.launch.py — LIO-SAM mapping**

Write `src/robot_slam/launch/slam.launch.py`:

```python
"""
LIO-SAM 3D SLAM — mapping mode with 16-line LiDAR + IMU

Usage:
  ros2 launch robot_slam slam.launch.py

After mapping, Ctrl+C to save GlobalMap.pcd, then convert:
  cd ~/AckermannRobot-3D
  ./src/pcd2pgm/build/pcd2gridmap src/maps/GlobalMap.pcd src/maps/map
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_lio = get_package_share_directory('lio_sam')
    pkg_robot = get_package_share_directory('ackermann_robot')
    params_file = os.path.join(pkg_lio, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) time'
    )

    # Static TF: map → odom (identity, LIO-SAM publishes odom→base_link optimization)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # LIO-SAM pipeline nodes
    imu_preintegration = Node(
        package='lio_sam', executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    image_projection = Node(
        package='lio_sam', executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    feature_extraction = Node(
        package='lio_sam', executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    map_optimization = Node(
        package='lio_sam', executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz for SLAM visualization
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_robot, 'rviz', 'slam_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_to_odom_tf,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
        rviz,
    ])
```

- [ ] **Step 6: Write navigation_dwb.launch.py — Nav2 with DWB**

Write `src/robot_slam/launch/navigation_dwb.launch.py`:

```python
"""
Nav2 navigation — AMCL + DWB local planner (pure Nav2, no NeuPAN)

Usage:
  ros2 launch robot_slam navigation_dwb.launch.py map:=/path/to/map.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map.yaml file'
    )
    nav_param_file = os.path.join(
        get_package_share_directory(pkg_slam), 'config', 'nav2_params.yaml'
    )
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    # Nav2 full stack
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', 'True'),
            ('params_file', nav_param_file),
            ('use_composition', 'False'),
        ],
    )

    # Twist → TwistStamped bridge
    bridge = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Pointcloud to laserscan (3D → 2D for DWB obstacle layer + AMCL)
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(pkg_robot), 'config', 'pcl_to_scan.yaml'
        )],
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan'),
        ],
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([map_arg, bringup, bridge, pcl_to_scan, rviz])
```

- [ ] **Step 7: Write navigation_neupan.launch.py — Nav2 + NeuPAN**

Write `src/robot_slam/launch/navigation_neupan.launch.py`:

```python
"""
Nav2 + NeuPAN navigation — AMCL + NeuPAN local planner + cmd_vel_mux

Components:
  - Nav2 bringup (AMCL, map_server, planner_server, bt_navigator, DWB)
  - NeuPAN local planner (subscribes /scan + /plan, publishes /neupan_cmd_vel)
  - cmd_vel_mux (selects NeuPAN by default)

Usage:
  Terminal 1: ros2 launch ackermann_robot gazebo.launch.py
  Terminal 2: ros2 launch robot_slam navigation_neupan.launch.py map:=src/maps/map.yaml
  Set goal via RViz "2D Goal Pose" tool
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'
    pkg_neupan = 'neupan_ros2'

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map.yaml file'
    )

    # Nav2 params (use neupan-compatible: DWB runs but NeuPAN controls)
    nav_param_file = os.path.join(
        get_package_share_directory(pkg_slam), 'config', 'nav2_params.yaml'
    )
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    # Nav2 full stack
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', 'True'),
            ('params_file', nav_param_file),
            ('use_composition', 'False'),
        ],
    )

    # Pointcloud to laserscan
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(pkg_robot), 'config', 'pcl_to_scan.yaml'
        )],
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan'),
        ],
    )

    # NeuPAN local planner
    robot_config_dir = os.path.join(
        get_package_share_directory(pkg_neupan),
        'config', 'robots', 'ackermann_robot'
    )
    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_type': 'ackermann_robot',
            'robot_description': 'Ackermann vehicle (0.70m, 0.593m wheelbase) in Gazebo',
            'robot_config_dir': robot_config_dir,
            'planner_config_file': 'planner.yaml',
            'dune_checkpoint_file': 'models/dune_model_5000.pth',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'lidar_frame': 'laser_link',
            'scan_topic': '/scan',
            'plan_input_topic': '/plan',
            'cmd_vel_topic': '/neupan_cmd_vel',
            'enable_visualization': False,
        }],
    )

    # cmd_vel mux: selects NeuPAN by default
    mux = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{'use_sim_time': True, 'active_planner': 'neupan'}],
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([
        map_arg, bringup, pcl_to_scan, neupan_node, mux, rviz,
    ])
```

- [ ] **Step 8: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/robot_slam
git commit -m "feat: add robot_slam package (Nav2 SmacPlannerHybrid + bridge + launch files)"
```

---

### Task 8: Create maps directory

**Files:**
- Create: `src/maps/` (empty directory for mapping output)

- [ ] **Step 1: Create directory with README**

```bash
mkdir -p /home/young/AckermannRobot-3D/src/maps
```

Write `src/maps/README.md`:

```markdown
# Maps Directory

Place pre-built maps here for navigation.

## Files expected:
- `GlobalMap.pcd` — LIO-SAM output (3D point cloud)
- `map.pgm` + `map.yaml` — 2D occupancy grid (from pcd2pgm)

## Workflow:
1. Run LIO-SAM: `ros2 launch robot_slam slam.launch.py`
2. Drive the robot around, Ctrl+C LIO-SAM → saves GlobalMap.pcd
3. Convert: `pcd2gridmap GlobalMap.pcd map`
4. Use map.yaml with navigation: `ros2 launch robot_slam navigation_dwb.launch.py map:=src/maps/map.yaml`
```

- [ ] **Step 2: Commit**

```bash
cd /home/young/AckermannRobot-3D
git add src/maps
git commit -m "feat: add maps directory for mapping output"
```

---

### Task 9: Build all packages

**Files:**
- No file changes — build and verify all packages compile

- [ ] **Step 1: Install system dependencies**

```bash
sudo apt update && sudo apt install -y \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-cartographer-ros \
  libpcl-dev
```

- [ ] **Step 2: Build all packages**

```bash
cd /home/young/AckermannRobot-3D
source /opt/ros/humble/setup.bash
colcon build --packages-select ndt_omp lio_sam hdl_localization pcd2pgm
```

Expected: All 4 packages build successfully.

- [ ] **Step 3: Build ackermann_robot (updated with bridge scripts)**

```bash
colcon build --packages-select ackermann_robot
```

Expected: Builds successfully. Verify scripts installed:
```bash
ls install/ackermann_robot/lib/ackermann_robot/
# Expected: arrow_key_control.py  cmd_vel_stamper.py  cmd_vel_mux.py
```

- [ ] **Step 4: Build robot_slam and neupan_ros2**

```bash
colcon build --packages-select neupan_ros2 robot_slam
```

Expected: Both packages install successfully.

- [ ] **Step 5: Full workspace build**

```bash
colcon build
source install/setup.bash
```

Expected: All packages build with no errors.

- [ ] **Step 6: Verify package discovery**

```bash
ros2 pkg list | grep -E "ndt_omp|lio_sam|hdl_localization|pcd2pgm|robot_slam|neupan_ros2"
# Expected: all 6 packages listed
```

- [ ] **Step 7: Commit workspace state**

```bash
cd /home/young/AckermannRobot-3D
git add -A
git commit -m "build: successful full workspace build with all navigation packages"
```

---

### Task 10: Integration smoke test

**Files:** No new files — verify end-to-end flows

- [ ] **Step 1: Verify robot model loads in Gazebo**

```bash
source install/setup.bash
timeout 30 ros2 launch ackermann_robot gazebo.launch.py || true
# Check Gazebo starts and robot spawns. Then Ctrl+C.
```

- [ ] **Step 2: Verify LIO-SAM nodes launch and subscribe to topics**

```bash
# Terminal 1: start Gazebo
ros2 launch ackermann_robot gazebo.launch.py &
sleep 10
# Terminal 2: start LIO-SAM
timeout 30 ros2 launch robot_slam slam.launch.py || true
# Check: lio_sam nodes show "Waiting for point cloud data" / "Waiting for IMU data"
# Check topics: ros2 topic list | grep -E "points_raw|imu/data"
```

- [ ] **Step 3: Verify Nav2 navigation launches**

```bash
# With a pre-existing map:
timeout 30 ros2 launch robot_slam navigation_dwb.launch.py map:=src/ackermann_robot/maps/mini_world.yaml || true
# Check: map_server loads map, planner_server starts, AMCL initializes
```

- [ ] **Step 4: Verify NeuPAN navigation launches**

```bash
timeout 30 ros2 launch robot_slam navigation_neupan.launch.py map:=src/ackermann_robot/maps/mini_world.yaml || true
# Check: neupan_node starts, cmd_vel_mux starts, pcl_to_scan starts
```

- [ ] **Step 5: Commit test results**

```bash
git commit --allow-empty -m "test: integration smoke tests pass for all navigation flows"
```
