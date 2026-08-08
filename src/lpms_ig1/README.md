# LPMS ROS 2 Example (`lpms_ig1`)

ROS 2 (ament_cmake / colcon) driver package for LP-Research IMU sensors:
**LPMS-IG1 / IG1-RS485 / IG2 / BE1 / NAV3 / CU3-CURS3 / SI1**.

The nodes publish standard [`sensor_msgs/msg/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
data and expose services for gyro calibration, heading reset and autocalibration. Helper nodes
convert quaternion orientation to Euler angles and radians to degrees for easy plotting.

In addition to the serial-port nodes, the package contains **`lpms_can_bridge_node`** — a
CAN-bus bridge that reads LPMS CAN frames directly from a Linux SocketCAN interface
(see [section 6](#6-can-bus-bridge-lpms_can_bridge_node)).

---

## 1. Prerequisites

* Ubuntu with a matching ROS 2 distribution (e.g. **Foxy** on 20.04, **Humble** on 22.04).
  Source ROS 2:

  ```bash
  source /opt/ros/humble/setup.bash    # or foxy, etc.
  ```

* Build tools and dependencies:

  ```bash
  sudo apt install python3-colcon-common-extensions \
      ros-$ROS_DISTRO-rclcpp \
      ros-$ROS_DISTRO-std-msgs \
      ros-$ROS_DISTRO-std-srvs \
      ros-$ROS_DISTRO-sensor-msgs \
      ros-$ROS_DISTRO-tf2
  ```

* **The LPMS OpenSource library must be installed first.** The nodes link against
  `LpmsIG1_OpenSourceLib`. From the repository root:

  ```bash
  cd ~/lpmsig1opensourcelib
  mkdir build && cd build
  cmake ..
  make
  make package
  sudo dpkg -i libLpmsIG1_OpenSource-x.y.z-Linux.deb
  ```

  (See the top-level [`README.md`](../README.md) for full library build details.)

---

## 2. Setting up the colcon workspace

```bash
# Create a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Symlink this example folder into the workspace (recommended over copying)
ln -s ~/lpmsig1opensourcelib/ros2_example lpms_ig1

# Build from the workspace root
cd ~/ros2_ws
colcon build --packages-select lpms_ig1

# Source the workspace overlay (add to ~/.bashrc to make it permanent)
source ~/ros2_ws/install/setup.bash
```

---

## 3. Running the nodes

Connect the sensor, then run the node for your device with `ros2 run`.
Parameters are passed with the `-p name:=value` syntax; adjust `port` and `baudrate` to match
your hardware.

| Sensor      | Command |
| :---------- | :------ |
| IG1         | `ros2 run lpms_ig1 lpms_ig1_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600` |
| IG1-RS485   | `ros2 run lpms_ig1 lpms_ig1_rs485_node --ros-args -p port:=/dev/ttyTHS5 -p baudrate:=115200 -p rs485ControlPin:=388 -p rs485ControlPinToggleWaitMs:=2` |
| IG2         | `ros2 run lpms_ig1 lpms_ig2_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600` |
| BE1 / BE2   | `ros2 run lpms_ig1 lpms_be1_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200` |
| NAV3        | `ros2 run lpms_ig1 lpms_nav3_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200` |
| CU3 / CURS3 | `ros2 run lpms_ig1 lpms_curs3_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600` |
| SI1         | `ros2 run lpms_ig1 lpms_si1_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600` |

Inspect the data (nodes run in the `imu` namespace):

```bash
ros2 topic echo /imu/data      # print IMU messages
ros2 topic list                # list available topics
```

---

## 4. Launch files

Launch files live in [`launch/`](launch/) and are Python launch descriptions. They start the
sensor node in the `imu` namespace together with the `quat_to_euler_node` and
`imudata_rad_to_deg_node` helper nodes. Edit the `port` / `baudrate` params inside the file to
match your setup.

| Launch file | Description |
| :---------- | :---------- |
| `ros2 launch lpms_ig1 lpms_ig1_launch.py`       | IG1 + quat→euler + rad→deg |
| `ros2 launch lpms_ig1 lpms_ig1_rs485_launch.py` | IG1-RS485 (with RS485 control pin params) |
| `ros2 launch lpms_ig1 lpms_ig2_launch.py`       | IG2 |
| `ros2 launch lpms_ig1 lpms_be1_launch.py`       | BE1 |
| `ros2 launch lpms_ig1 lpms_nav3_launch.py`      | NAV3 |
| `ros2 launch lpms_ig1 lpms_curs3_launch.py`     | CU3 / CURS3 |
| `ros2 launch lpms_ig1 lpms_si1_launch.py`       | SI1 |
| `ros2 launch lpms_ig1 lpms_can_bridge_launch.py` | CAN-bus bridge (SocketCAN, no helper nodes) — also available as `lpms_can_bridge_launch.xml` |

Example:

```bash
ros2 launch lpms_ig1 lpms_ig1_launch.py
```

> **Note:** launch files are installed into the package `share/` directory. Re-run
> `colcon build` and re-source `install/setup.bash` after editing them.

---

## 5. Topics, services and parameters

### Published topics

| Topic | Type | Notes |
| :---- | :--- | :---- |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation (unit quaternion), calibrated angular rate, calibrated acceleration |
| `/imu/mag`  | `sensor_msgs/msg/MagneticField` | Magnetometer reading (*IG1 series only*) |

Helper nodes republish orientation in human-friendly units:

| Node | Publishes | Type |
| :--- | :-------- | :--- |
| `quat_to_euler_node` | `/imu/rpy_angles` | `geometry_msgs/msg/Vector3` |
| `imudata_rad_to_deg_node` | `/imu/angular_vel_deg`, `/imu/rpy_deg` | `geometry_msgs/msg/Vector3` |

### Services

| Service | Type | Notes |
| :------ | :--- | :---- |
| `/imu/calibrate_gyroscope` | `std_srvs/srv/Trigger` | Keep sensor stationary ~4 s |
| `/imu/reset_heading` | `std_srvs/srv/Trigger` | Reset yaw to zero |
| `/imu/enable_gyro_autocalibration` | `std_srvs/srv/SetBool` | Toggle gyro autocalibration |
| `/imu/enable_auto_reconnect` | `std_srvs/srv/SetBool` | Toggle library auto-reconnect |

Example service call:

```bash
ros2 service call /imu/calibrate_gyroscope std_srvs/srv/Trigger
```

### Parameters

| Parameter | Type | Default | Notes |
| :-------- | :--- | :------ | :---- |
| `port` | string | `/dev/ttyUSB0` | Serial port of the sensor |
| `baudrate` | int | `921600` (IG1) / `115200` (RS485, BE1, NAV3) | Serial baud rate |
| `frame_id` | string | `imu` | Frame id in published messages |
| `autoreconnect` | bool | `true` | Enable library auto-reconnect |
| `rate` | int | `200` | Internal processing loop rate (Hz); must be ≥ sensor stream rate |
| `rs485ControlPin` | int | `-1` (disabled) | *IG1-RS485 only* — send/receive GPIO control pin |
| `rs485ControlPinToggleWaitMs` | int | `1` | *IG1-RS485 only* — toggle wait time (ms) |

---

## 6. CAN-bus bridge (`lpms_can_bridge_node`)

[`src/lpms_can_bridge_node.cpp`](src/lpms_can_bridge_node.cpp) is an alternative to the
serial-port nodes for sensors configured for **CAN output** (e.g. IG1-CAN / IG2-CAN). Instead of
using the OpenSource library it opens a **raw Linux SocketCAN socket** directly, decodes the LPMS
CAN frames and republishes them as ROS 2 messages. Because of that it:

* has **no dependency** on `LpmsIG1_OpenSourceLib` (only `rclcpp`, `sensor_msgs`,
  `geometry_msgs`, `std_msgs` and pthreads),
* is **Linux only** (SocketCAN),
* offers **no services** — CAN is receive-only here, so gyro calibration / heading reset must be
  done over the sensor's serial link or with LpmsControl.

Internally a dedicated receive thread blocks on the CAN socket (10 ms read timeout) and updates a
mutex-protected state struct, while a wall timer publishes the latest state at a fixed rate. This
decouples the publish rate from the CAN frame rate.

### Bring up the CAN interface

The node does **not** configure the interface — do that once before starting it:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify frames are arriving (from can-utils: sudo apt install can-utils)
candump can0
```

### Running

```bash
# Directly
ros2 run lpms_ig1 lpms_can_bridge_node --ros-args -p channel:=can0 -p publish_rate:=200.0

# Or via launch file (Python or XML variant)
ros2 launch lpms_ig1 lpms_can_bridge_launch.py
ros2 launch lpms_ig1 lpms_can_bridge_launch.xml
```

Both launch files start the node under the name `lpms_can_bridge` with `output='screen'` and set
`channel`, `publish_rate` and `timestamp_scale`. Edit those values to match your setup, then
re-run `colcon build` and re-source `install/setup.bash`.

### Parameters

| Parameter | Type | Default | Notes |
| :-------- | :--- | :------ | :---- |
| `channel` | string | `can0` | SocketCAN interface name |
| `publish_rate` | double | `200.0` | Publish timer rate in Hz (independent of the CAN frame rate) |
| `timestamp_scale` | double | `0.001` | Reserved for sensor-timestamp scaling; currently unused — messages are stamped with the ROS clock |

> **Note:** only the parameters above are declared. Passing undeclared parameters (e.g.
> `interface`, `bitrate`) raises `ParameterNotDeclaredException` on startup — this is why they are
> commented out in the launch files.

### Published topics

Unlike the serial nodes, this node publishes on **absolute topic names** (no `imu` namespace) and
uses a fixed `frame_id` of `imu_link`.

| Topic | Type | Notes |
| :---- | :--- | :---- |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation quaternion, angular velocity (rad/s), linear acceleration (m/s²). Covariances are left at zero |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetic field in tesla |
| `/imu/euler` | `geometry_msgs/msg/Vector3` | Roll / pitch / yaw in **degrees**, straight from the sensor |
| `/imu/temperature` | `std_msgs/msg/Float32` | Only published when a non-zero temperature has been received; no CAN ID currently decodes it |

### Expected CAN frame layout

Frames must have `DLC == 8`; shorter frames are ignored. Each payload is four
**little-endian `int16`** values:

| CAN ID | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Bytes 6–7 |
| :----- | :-------- | :-------- | :-------- | :-------- |
| `0x181` | acc X | acc Y | acc Z | gyro X |
| `0x281` | gyro Y | gyro Z | mag X | mag Y |
| `0x381` | mag Z | euler X | euler Y | euler Z |
| `0x481` | quat W | quat X | quat Y | quat Z |

Scaling applied while decoding (before SI conversion): acceleration `/1000` (g), gyro `/10`
(°/s), magnetometer `/100` (µT), Euler `/100` (°), quaternion `/10000`. Unknown CAN IDs are
silently ignored, so the sensor's CAN output profile must match this table — configure it with
LpmsControl if it does not.

---

## 7. Troubleshooting

**Serial permission / port error**: add your user to the `dialout` group, then log out and
back in:

```bash
sudo adduser <username> dialout
```

**Library not found at build/run time**: make sure the OpenSource library `.deb` was installed
(section 1), then clean and rebuild:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select lpms_ig1
source install/setup.bash
```

**`package not found` when running / launching**: the workspace overlay was not sourced.
Run `source ~/ros2_ws/install/setup.bash` in every new terminal.

**`lpms_can_bridge_node` exits with `SocketCAN initialization failed`**: the interface does not
exist or is down. Check with `ip -details link show can0` and bring it up as shown in
[section 6](#6-can-bus-bridge-lpms_can_bridge_node). Make sure the `channel` parameter matches the
real interface name.

**CAN bridge runs but topics stay at zero**: no matching frames are arriving. Confirm traffic with
`candump can0`; if IDs are present but differ from `0x181` / `0x281` / `0x381` / `0x481`, the
sensor's CAN output profile does not match the layout the node expects. Also verify both ends use
the same bitrate.

---

For more information visit <https://www.lp-research.com> or <https://www.alubi.cn>.
