# lpms_ig1_ros2

LPMS-IG1 CANopen 16-bit driver using Linux SocketCAN directly.

## Topics

- `/imu/data` — `sensor_msgs/msg/Imu`
  - LPMS fused quaternion orientation
  - angular velocity in rad/s
  - acceleration in m/s^2
- `/imu/data_raw` — `sensor_msgs/msg/Imu`
  - gyro + acceleration
  - orientation marked unavailable (`orientation_covariance[0] = -1`)
- `/imu/mag` — `sensor_msgs/msg/MagneticField`
  - magnetic field in Tesla

## Default device settings assumed

- SocketCAN interface: `can0`
- LPMS CANopen node ID: `5`
- 16-bit CAN mapping:
  - `0x185`: calibrated accel XYZ + aligned Gyro II X
  - `0x285`: aligned Gyro II YZ + calibrated mag XY
  - `0x385`: calibrated mag Z + Euler RPY
  - `0x485`: quaternion WXYZ

If the CAN mapping has been changed in IG1Control, update the decoder.

## ROS convention conversions

By default:

1. Acceleration is sign-inverted and converted from g to m/s^2.
   With the sensor stationary and its +Z axis upward, ROS output should be
   approximately `+9.80665 m/s^2` on Z.

2. Gyroscope is converted from deg/s to rad/s.

3. Magnetometer is converted from microtesla to tesla.

4. LPMS magnetic world convention NWU (north-west-up) is converted to
   ROS ENU (east-north-up) for the fused quaternion.

The NWU->ENU conversion can be disabled with:
`convert_nwu_to_enu:=false`.

## Build

Copy this package into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r lpms_ig1_ros2 ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select lpms_ig1_ros2
source install/setup.bash
```

## Bring up CAN

Example for 500 kbit/s:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

## Run

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

Or:

```bash
ros2 run lpms_ig1_ros2 lpms_ig1_node --ros-args \
  -p interface:=can0 \
  -p node_id:=5 \
  -p frame_id:=imu_link
```

## Inspect

```bash
ros2 topic list | grep imu
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/mag --once
```

## Static gyroscope calibration

Place the IMU on a stable surface, keep it completely stationary, and leave the
driver running. In another sourced terminal run:

```bash
ros2 run lpms_ig1_ros2 gyro_calibration
```

The default procedure waits 5 seconds, samples `/imu/data_raw` for 30 seconds,
checks gyroscope motion and acceleration magnitude, and writes a loadable ROS 2
parameter file to:

```text
~/.ros/lpms_ig1_gyro_calibration.yaml
```

Durations and the output path can be changed, for example:

```bash
ros2 run lpms_ig1_ros2 gyro_calibration --ros-args \
  -p warmup_duration:=10.0 \
  -p calibration_duration:=60.0 \
  -p output_file:=/tmp/lpms_ig1_gyro_calibration.yaml
```

Load the generated bias file on the next driver start:

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py \
  params_file:=$HOME/.ros/lpms_ig1_gyro_calibration.yaml
```

With calibration loaded, `/imu/data_raw` retains the converted, uncorrected
gyro readings. Bias subtraction is applied only to `/imu/data`.

## Accelerometer calibration

Run the six-position calibration while the uncalibrated driver is publishing
`/imu/data_raw`:

```bash
ros2 run lpms_ig1_ros2 accel_calibration
```

Follow the prompts and place the sensor in this order, pressing Enter only
after it is stable:

1. +X up
2. -X up
3. +Y up
4. -Y up
5. +Z up
6. -Z up

Each orientation is sampled for 8 seconds by default. A failed motion,
gravity-magnitude, or orientation check repeats only the current position.
The result is written to `~/.ros/lpms_ig1_calibration.yaml`; existing gyro bias
parameters in that file, or in the earlier gyro calibration file, are retained.
A detailed report is written separately to
`~/.ros/lpms_ig1_accel_calibration_report.yaml`.

Load the unified calibration with:

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py \
  params_file:=$HOME/.ros/lpms_ig1_calibration.yaml
```

The calibration measured for this project IMU is versioned at
`config/lpms_ig1_calibration.yaml`, with its acquisition report at
`config/lpms_ig1_accel_calibration_report.yaml`. The real-vehicle bringup uses
the versioned calibration by default so the same sensor can be moved to another
computer without recalibrating. Recalibrate if the physical IMU unit changes.

The topic semantics are:

- `/imu/data_raw`: SI-unit IMU data without user gyro or accelerometer calibration.
- `/imu/data`: gyro bias removed and accelerometer offset/gain applied.

Accelerometer correction is `corrected = (raw - offset) * gain`. It affects
only the ROS acceleration values and does not change the LPMS internal AHRS or
its fused quaternion. Perform magnetometer calibration only after final vehicle
installation, where the local magnetic environment is representative.

A stationary, Z-up sensor should show approximately:

```text
linear_acceleration.z ~= +9.8
angular_velocity.x/y/z ~= 0
```

## Mounting frame

Measurements remain expressed in the LPMS sensor axes. `frame_id` defaults to
`imu_link`.

Describe the physical IMU mounting relative to the robot with a static TF, e.g.
from `base_link` to `imu_link`. For a ROS-standard robot body frame use:

- +X forward
- +Y left
- +Z up

Do not fake the mounting orientation by permuting CAN bytes unless you have a
specific reason; represent mounting with TF or LPMS Object Reset.
