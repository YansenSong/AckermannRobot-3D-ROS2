# AckermannRobot Nav2

This package is an isolated, forward-only Nav2 baseline for the AckermannRobot.
It uses Smac Hybrid-A* with `DUBIN`, Nav2 MPPI in `Ackermann` mode, the
existing LIORF localization chain, and `/scan` obstacle layers in both local
and global costmaps.

Start the simulation baseline with:

```bash
bash scripts/run_ackermann_nav.sh maps/mini
```

The package does not start the NeuPAN navigation stack, its command mux,
`ackermann_smac_bridge`, or the NeuPAN navigation launch. Do not run the
NeuPAN navigation entry points at the same time because both stacks target the
same ros2_control reference topic.

Dynamic-obstacle support in this baseline is reactive: `/scan` updates the
costmaps, MPPI reacts locally, and the behavior tree replans globally at 1 Hz.
Unknown-object tracking and future trajectory prediction are intentionally out
of scope.
