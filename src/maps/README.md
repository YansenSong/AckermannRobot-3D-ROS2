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
