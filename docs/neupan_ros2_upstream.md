# NeuPAN ROS2 Upstream Baseline

## Source

- Source clone: `/home/young/Project/neupan_ros2`
- Upstream repository: `KevinLADLee/neupan_ros2`
- Upstream branch: `main`
- Imported commit: `4ffb7ec2dc45ff7ee9024f64083813237906af98`
- Imported package: `src/neupan_ros2`
- Target project baseline before migration: `5d6fb60666bf2ff9043c705d37993b7dd75a53d7`

The upstream clone was clean before it was copied. It was used as a source
only; it was not reset, checked out, pulled, or otherwise modified.

## Preserved local assets

The pre-migration directory was backed up at:

`/home/young/Project/AckermannRobot_neupan_backup_20260910_195107`

The following files remain in the target package and their SHA256 values were
identical before and after the vendor reset:

- `src/neupan_ros2/config/dune_checkpoint/scout_model_5000.pth`
- `src/neupan_ros2/config/dune_checkpoint/scout_model_5000_v2.pth`

The official source clone already contained files with the same names. The
independent backup and before/after hash comparison were still performed to
make the preservation explicit.

## Local modifications to the vendored package

1. Added `config/robots/ackermann_robot` by copying the official Ranger
   profile, then changed only the current robot frames, topics, and motion
   limits.
2. Added the minimal reverse-gear calculation in `path_callback()`:
   Smac pose heading is compared with each segment travel direction and the
   resulting `gear` is passed as the fourth initial-path component.
No optional `ackermann_robot.launch.py` was added; the project starts the
wrapper through `scripts/run_neupan.sh`.

## Explicitly not ported from the old project fork

- avoidance seed configuration and logic
- scan TF age / stale-scan handling
- direct goal planning switch
- command rate limiter
- executor thread customization
- old `min_speed` baseline configuration
- Ackermann steering-to-yaw-rate conversion inside `neupan_node.py`

The official node still publishes NeuPAN's Ackermann command as
`Twist.linear.x = v` and `Twist.angular.z = ψ`. The project-local
`ackermann_control/neupan_ackermann_adapter.py` performs the separate
conversion to body yaw rate.

The project-local `third_party/NeuPAN` core was intentionally not changed in
this migration. Its existing nested `init_from_yaml()` merge behavior was
checked and is sufficient for the wrapper's `pan.dune_checkpoint` override.
