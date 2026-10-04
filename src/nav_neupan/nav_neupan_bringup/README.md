# nav_neupan_bringup

Launch package for the Smac Hybrid-A* + NeuPAN real-vehicle navigation stack.
Shared sensors, motion interface, and LIORF localization are provided by
`vehicle_bringup`.

The full real-vehicle launch starts the hardware, shared localization, Smac
planning bridge, point-cloud scan conversion, RViz, and NeuPAN. NeuPAN runs in
the `neupan` Conda environment through the repository script
`scripts/run_neupan.sh`.

The recommended entry point is:

```bash
./scripts/start_vehicle.sh nav maps/<map_name>
```

Direct launch:

```bash
ros2 launch nav_neupan_bringup navigation.launch.py \
  project_dir:=$(pwd) \
  vehicle_config:=$(pwd)/config/vehicle.yaml \
  map:=$(pwd)/maps/<map_name>/map.yaml \
  map_pgm:=$(pwd)/maps/<map_name>/map.pgm \
  globalmap_pcd:=$(pwd)/maps/<map_name>/GlobalMap.pcd \
  start_hardware:=true
```

Do not run this stack at the same time as `nav_nav2`; both feed the shared
`/ackermann_cmd` interface.
