# Ackermann Nav2

This package is a second, isolated navigation system for the robot. It does
not launch or communicate with `hybrid_astar_planner`, `neupan_ros2`, or the
NeuPAN command mux.

The Nav2 system uses:

- Smac Hybrid-A* with a Reeds-Shepp motion model;
- Regulated Pure Pursuit with reversing enabled and rotate-in-place disabled;
- namespaced Nav2 nodes, actions, and topics under `/nav2`;
- `/nav2/cmd_vel` as its only navigation command input to the dedicated
  `nav2_cmd_adapter`;
- the existing robot, sensors, localization, odometry, and ros2_control
  infrastructure.

For Nav2 only, HDL localization is connected through a current-time TF bridge
so its scan-stamped pose updates do not intermittently stop the controller.
The Nav2 point-cloud slice also rejects ground-edge and near-body returns.

Build:

```bash
colcon build --packages-select ackermann_nav2
source install/setup.bash
```

Start the Nav2 simulation system only:

```bash
ros2 launch ackermann_nav2 nav2_navigation_sim.launch.py \
  map:=$PWD/maps/mini/map.yaml \
  globalmap_pcd:=$PWD/maps/mini/GlobalMap.pcd
```

Or use the workspace-level launcher, which selects the `maps/mini` maps by
default:

```bash
./scripts/start_ackermann_nav2.sh
```

Different maps can be supplied as the first and second arguments:

```bash
./scripts/start_ackermann_nav2.sh /path/to/map.yaml /path/to/GlobalMap.pcd
```

The namespaced navigation action is `/nav2/navigate_to_pose`. Do not start the
existing `ackermann_bringup/navigation_sim.launch.py` at the same time.
