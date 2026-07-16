# Port Navigation Stack to AckermannRobot-3D

## Goal

Port the full autonomous navigation stack (SLAM mapping, NDT localization, Nav2 global planning, NeuPAN local planning, and IMU sensing) from `my_robot` and `AckermannRobot-2D` into the AckermannRobot-3D workspace.

## Source Projects

| Source | Role |
|--------|------|
| `~/my_robot` (large 5.24m vehicle) | LIO-SAM, hdl_localization, ndt_omp, pcd2pgm |
| `~/AckermannRobot-2D` (same small vehicle) | NeuPAN configs, Nav2 params, bridge scripts |

## Target Vehicle Parameters (existing, unchanged)

| Parameter | Value |
|-----------|-------|
| Chassis | 0.70m × 0.52m × 0.1m (collision box) |
| Wheelbase | 0.593m |
| Track width | 0.510m (front), 0.519m (rear) |
| Wheel radius | 0.093m |
| Max steering | 0.52 rad (~30°) |
| Min turning radius | 1.05m |
| Max linear speed | 1.5 m/s forward, -0.5 m/s reverse |

## Existing Sensors (unchanged)

| Sensor | Frame | Topic | Specs |
|--------|-------|-------|-------|
| 3D LiDAR | laser_link | /points_raw | 16 lines, 360° H, ±15° V, 20Hz |
| IMU | gyro_link | /imu/data | 100Hz, accel + gyro with noise |
| Depth Camera | camera_optical_link | /depth/* | 640×480, 30Hz |

## Existing Control (unchanged)

- ros2_control with `ackermann_steering_controller`
- Publishes `/odom_wheel` (Odometry), remapped from controller odometry
- Accepts `/ackermann_steering_controller/reference` (TwistStamped)

## Architecture

### Package Layout

```
AckermannRobot-3D/src/
├── ackermann_robot/          # Existing — enhanced with bridge scripts
├── gazebo_worlds/            # Existing
├── ndt_omp/                  # NEW — from my_robot (OpenMP NDT + GICP library)
├── LIO-SAM/                  # NEW — from my_robot (3D LiDAR-IMU SLAM)
├── hdl_localization/         # NEW — from my_robot (NDT 3D localization)
├── pcd2pgm/                  # NEW — from my_robot (3D PCD → 2D PGM converter)
├── robot_slam/               # NEW — Nav2 global planner + bridge scripts
├── neupan_ros2/              # NEW — from AckermannRobot-2D (local planner)
└── maps/                     # NEW — map storage directory
```

### Data Flow

**Mapping phase (offline):**
```
/points_raw + /imu/data → LIO-SAM → GlobalMap.pcd → pcd2pgm → map.pgm + map.yaml
```

**Navigation phase (online):**
```
map.pgm → map_server → /map → global_costmap → SmacPlannerHybrid → /plan
GlobalMap.pcd → hdl_localization (NDT) → map→odom TF (publishes /odom)
/points_raw → pointcloud_to_laserscan → /scan → NeuPAN → /neupan_cmd_vel
                                                         ↓
                                              cmd_vel_mux (Twist→TwistStamped)
                                                         ↓
                                         /ackermann_steering_controller/reference
```

### TF Tree (navigation)

```
map ──(hdl_localization NDT matching)──→ odom ──(controller odom)──→ base_link
                                                                       ├→ laser_link
                                                                       └→ gyro_link
```

### Topic Adaptations

| Source Topic | Target Topic | Adapter |
|-------------|-------------|---------|
| `/imu_data` (lio_sam expects `/imu_raw`) | `/imu/data` | remap in launch |
| `/points_raw` | `/points_raw` | same name |
| `/neupan_cmd_vel` (Twist) | `/ackermann_steering_controller/reference` (TwistStamped) | cmd_vel_mux |
| `/odom_wheel` (from controller) | hdl_localization publishes `/odom` | both coexist |

### LIO-SAM Parameter Changes (from large vehicle to small vehicle)

| Parameter | Old (large car) | New (small car) |
|-----------|----------------|-----------------|
| `imuTopic` | `/imu_raw` | `/imu/data` |
| `lidarFrame` | `lidar3d_link` | `laser_link` |
| `N_SCAN` | 32 | 16 |
| `Horizon_SCAN` | 360 | 1800 |
| `extrinsicTrans` | `[1.30, 0.0, 0.65]` | `[-0.078, 0.001, 0.384]` (laser_joint offset from base_link) |
| `odometrySurfLeafSize` | 0.4 | 0.2 (indoor) |
| `mappingCornerLeafSize` | 0.2 | 0.1 (indoor) |
| `mappingSurfLeafSize` | 0.4 | 0.2 (indoor) |
| `surroundingkeyframeAddingDistThreshold` | 1.0 | 0.5 |
| `surroundingKeyframeSearchRadius` | 50.0 | 15.0 |
| `historyKeyframeSearchRadius` | 15.0 | 5.0 |
| `globalMapVisualizationSearchRadius` | 1000.0 | 50.0 |

### hdl_localization Parameter Changes

| Parameter | Old (large car) | New (small car) |
|-----------|----------------|-----------------|
| `odom_child_frame_id` | `lidar3d_link` | `laser_link` |
| `ndt_resolution` | 1.0 | 0.5 (finer for small car) |
| `use_imu` | true | true |
| IMU topic remap | `/imu_raw` | `/imu/data` |

### Nav2 Parameters (from AckermannRobot-2D)

Planner: `SmacPlannerHybrid` with REEDS_SHEPP motion model
- minimum_turning_radius: 1.05m
- angle_quantization_bins: 72 (5°/bin)
- reverse_penalty: 2.1
- robot_radius: 0.4m
- inflation_radius: 0.35m

Controller: DWB (runs as BT dependency, NeuPAN does actual control)
- max_vel_x: 1.5, min_vel_x: -0.5
- cmd_vel_mux selects NeuPAN by default

### NeuPAN Config (from AckermannRobot-2D, already configured for this vehicle)

- robot.yaml: wheelbase 0.593m, length 0.70m, width 0.52m
- planner.yaml: min_radius 1.05m, receding 15 steps, control 20Hz
- DUNE checkpoint: `ackermann_robot/models/dune_model_5000.pth`
