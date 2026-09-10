# Vendored `nav2_smac_planner` provenance

This workspace vendors the official ROS 2 Humble-compatible `nav2_smac_planner`
package. It is used through the installed Humble `nav2_planner`,
`nav2_costmap_2d`, `nav2_map_server`, and `nav2_lifecycle_manager` runtime
packages.

```text
Upstream: https://github.com/ros-navigation/navigation2
Branch: humble
Commit: 3c3db59d6969d8ecee8e68468693d006397f4a0c
Imported package: nav2_smac_planner
Package version: 1.1.20
Local algorithm modifications: None
```

Robot-specific settings live in
[`ackermann_bringup/config/smac_planner.yaml`](../src/ackermann_bringup/config/smac_planner.yaml).
For adapting the planner to another Ackermann chassis, use the reusable
[parameter adaptation guide](ackermann_smac_parameter_adaptation.md).
The `ackermann_smac_bridge` package adapts `/goal_pose`, rear-axle TF, and the
legacy `/plan_path` and `/global_path_remaining_distance` telemetry without
publishing a second `/plan` topic or rewriting Smac pose orientations.

## Ackermann geometry audit

The vehicle geometry was recomputed from
[`chassis.xacro`](../src/ackermann_simulation/robot/xacro/chassis.xacro), not
from the removed planner configuration:

```text
wheelbase = 0.28930 - (-0.30449) = 0.59379 m
front track = 2 * (0.163000 + 0.091957) = 0.509914 m
steering joint limit = +/-0.52 rad
minimum turning radius at rear axle center = 1.29203 m
```

The physical radius is a lower bound. The planner is configured with a
conservative `minimum_turning_radius: 1.320` m so the official Humble smoother
also remains within the discrete-curvature acceptance bound.

The analytic expansion length is scaled with that configured radius:
`analytic_expansion_max_length = 5 * 1.320 = 6.60` m.

The base collision box, wheel collision cylinders, and both steering-link STL
collision meshes were sampled at steering angles `-0.52`, `0`, and `+0.52` rad.
Their sampled XY envelope relative to `rear_axle_link` is:

```text
x = [-0.09300, 0.73248] m
y = [-0.35235, 0.35235] m
```

The configured polygon is therefore conservatively expanded to:

```text
[[-0.10, -0.36], [0.74, -0.36], [0.74, 0.36], [-0.10, 0.36]]
```
