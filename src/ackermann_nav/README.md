# ackermann_nav

`ackermann_nav` is the second, independent real-vehicle navigation stack in
`real-vehicle-integration`.

It intentionally coexists with the Smac + NeuPAN stack:

- `ackermann_nav`: Nav2 BT Navigator + Smac Hybrid-A* + MPPI + velocity smoother
- `ackermann_bringup` + `neupan_ros2`: Smac Hybrid-A* + NeuPAN

Do not run both navigation stacks at the same time; they use overlapping Nav2
node names and both ultimately target `/ackermann_cmd`.

Vehicle geometry and kinematic limits are not duplicated here. They are read at
launch from the repository-root `config/vehicle.yaml` and injected into Smac,
MPPI, costmap footprints, the velocity smoother, and the Nav2-to-Ackermann
command adapter.

Nav2 controller output uses body-twist semantics (`angular.z = yaw rate`).
`nav2_cmd_adapter.py` converts that yaw rate to the project's canonical
Ackermann command (`angular.z = front-wheel steering angle`) before the command
passes through `motion_interface`.

Recommended full real-vehicle entry point:

```bash
./scripts/start_vehicle.sh nav2 maps/<map_name>
```

Direct navigation-only launch (hardware may already be running):

```bash
ros2 launch ackermann_nav navigation.launch.py \
  vehicle_config:=$(pwd)/config/vehicle.yaml \
  map:=$(pwd)/maps/<map_name>/map.yaml \
  globalmap_pcd:=$(pwd)/maps/<map_name>/GlobalMap.pcd
```

LiDAR/IMU extrinsics must be `VERIFIED` in `config/vehicle.yaml`; localization
will otherwise fail fast.
