# NeuPAN Real-Vehicle Configuration

This directory contains the real-vehicle robot configuration for NeuPAN.

## Directory Structure

```
robots/
├── real_vehicle/            # Real ackermann vehicle deployment
│   ├── robot.yaml           # ROS node parameters (TF, topics, scan)
│   ├── planner.yaml         # NeuPAN planner configuration (MPC, geometry, tuning)
│   ├── pcl_to_scan.yaml     # pointcloud_to_laserscan node config
│   └── models/
│       └── dune_model_5000.pth  (place trained model here)
│
└── _template/               # Template for new robots (reference only)
    ├── README.md
    ├── robot.yaml.template
    ├── planner.yaml.template
    └── models/
        └── README.md
```

## Real Vehicle

| Property | Value |
|----------|-------|
| Kinematics | Ackermann |
| Dimensions (L×W) | 0.70m × 0.52m |
| Wheelbase | 0.97m |
| Max steering | 30° (0.524 rad) |
| Min turning radius | 1.68m |
| Max speed | 2.0 m/s forward, -0.5 m/s reverse |
| LiDAR | Hesai PandarXT-16 (16-line 3D) |
| Control | STM32 via UDP (192.168.1.50:5000) |

## Launch

```bash
# Terminal 1: Navigation infrastructure (LiDAR + pcl→scan + Hybrid A* + bridge)
bash scripts/start_vehicle.sh nav /path/to/map.pgm

# Terminal 2: NeuPAN (requires conda 'neupan' environment)
bash scripts/run_neupan.sh
```

## Configuration Files

### robot.yaml

ROS integration parameters for the `neupan_node`:
- `use_sim_time: false` — wall-clock time for real vehicle
- `cmd_vel_topic: '/cmd_vel'` — publishes directly to motion_control bridge_node
- `scan_topic: '/scan'` — from pointcloud_to_laserscan
- `plan_input_topic: '/plan'` — from hybrid_astar_planner
- `lidar_frame: 'base_link'` — scan is output in base_link by pcl_to_scan
- `control_frequency: 20.0` — matches bridge_node publish_rate

### planner.yaml

NeuPAN planner parameters:
- `wheelbase: 0.97` — must match bridge_params.yaml exactly
- `min_radius: 1.68` — wheelbase / tan(30°)
- Collision and acceleration parameters set conservatively for real vehicle safety
- Tuning weights (`adjust` section) are a starting point — adjust per `docs/neupan_tuning.md`

### pcl_to_scan.yaml

Configuration for `pointcloud_to_laserscan` node:
- Input: `/lidar_points` (PointCloud2 from lidar_driver)
- Output: `/scan` (LaserScan in `base_link` frame)
- Height slice: -0.4m to 1.0m relative to base_link

## Prerequisites

### DUNE Model

The DUNE neural network model must be placed in `real_vehicle/models/dune_model_5000.pth`.
The model must be trained for the geometry in `planner.yaml` (0.97m wheelbase, ackermann kinematics).

### Localization

A TF transform from `map` to `base_link` is required for NeuPAN to function.
Until real localization (hdl_localization + EKF) is ready, a static identity transform
is provided by the launch file as a bootstrap:

```
map → base_link (identity, static)
base_link → hesai_lidar (static, LiDAR mounting position)
```

Remove the static TF publisher once localization is operational.

### Control Chain

```
NeuPAN → /cmd_vel (Twist)
  → bridge_node (Ackermann conversion, δ = atan2(ω×0.97, |v|+ε))
  → 26-byte UDP frame → STM32 (192.168.1.50:5000)
  → CAN → RT49 drive + EPS steering + SEB brake
```

Safety: bridge_node has 0.5s command timeout → auto zero-velocity frame.
STM32 has independent 500ms timeout.

## Troubleshooting

### NeuPAN starts but publishes no /cmd_vel
- Check TF tree: `ros2 run tf2_tools view_frames`
- Ensure `map → base_link` transform exists
- Check `/scan` and `/plan` topics are being published
- Check neupan_node log output for errors

### Model loading error
- Verify `models/dune_model_5000.pth` exists
- Check it's a valid PyTorch checkpoint
- Ensure it was trained for 0.97m wheelbase geometry

### No /scan output
- Verify LiDAR is running: `ros2 topic echo /lidar_points --once`
- Check pointcloud_to_laserscan is running with correct remapping
- Verify pcl_to_scan.yaml parameters match LiDAR mounting position

### Vehicle doesn't move despite /cmd_vel output
- Check bridge_node is running and UDP socket is bound to enp5s0
- Verify network: `ip addr show enp5s0` should show 192.168.1.x
- Check STM32 is powered and connected (192.168.1.50:5000)
- Check remote controller is not overriding (遥控器优先级最高)
