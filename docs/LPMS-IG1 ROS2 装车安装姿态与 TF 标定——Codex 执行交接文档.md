# LPMS-IG1 ROS2 装车安装姿态与 TF 标定——Codex 执行交接文档

## 1. 当前项目状态

当前项目：

```text
lpms_ig1_ros2
```

硬件：

```text
LPMS-IG1
CAN: can0
CANopen Node ID: 5
```

已经完成：

```text
1. CAN / SocketCAN 数据解析
2. ROS2 标准 IMU topic 发布
3. Gyroscope 静止零偏标定
4. Accelerometer 六面标定
```

当前主要 ROS2 topic：

```text
/imu/data_raw
sensor_msgs/msg/Imu

/imu/data
sensor_msgs/msg/Imu

/imu/mag
sensor_msgs/msg/MagneticField
```

其中约定：

```text
/imu/data_raw
```

为 ROS SI 单位下、未应用用户软件 gyro/accel calibration 的数据。

```text
/imu/data
```

为应用 gyro bias 与 accelerometer offset/gain 后的数据。

当前 frame：

```text
imu_link
```

---

# 2. 本阶段目标

现在用户将 LPMS-IG1 固定安装到车辆。

本阶段只解决：

```text
IMU 安装方向相对于车辆 base_link 的关系
```

即：

```text
base_link -> imu_link
```

static transform。

重点标定：

```text
mount_roll
mount_pitch
```

同时支持用户设置：

```text
mount_yaw
```

但是：

> 静止状态下不能通过重力观测得到 yaw。

因此禁止通过静止加速度计“计算 yaw”。

Yaw 安装角需要：

```text
物理安装测量
```

或者在后续：

```text
Heading / magnetometer / GNSS course
```

标定阶段确定。

---

# 3. ROS 车辆坐标约定

车辆：

```text
base_link
```

采用 ROS REP-103 常见约定：

```text
+X = 车辆前方
+Y = 车辆左方
+Z = 车辆上方
```

即：

```text
       +Z
        ↑
        |
        |
 +Y ← 车体 → 
        |
       +X 前
```

实际 IMU 可以有小角度安装误差。

例如：

```text
Roll mounting error  = +1.2°
Pitch mounting error = -2.5°
Yaw mounting error   = +0.8°
```

这些不应该通过修改 CAN byte order 或交换 XYZ 来修正。

必须通过：

```text
TF
```

描述。

---

# 4. 非常重要：不要修改传感器坐标数据

不要为了让数据“看起来水平”而在：

```text
lpms_ig1_node.py
```

里做：

```text
x/y/z swap
轴符号交换
人工旋转 acceleration
人工旋转 gyro
```

除非发现原驱动本身坐标定义有 bug。

正确结构：

```text
LPMS sensor frame
      |
      |
   imu_link
      |
      | static TF
      |
   base_link
```

---

# 5. 新增标定工具

新增：

```text
lpms_ig1_ros2/
└── mounting_calibration.py
```

setup.py：

```python
"mounting_calibration = lpms_ig1_ros2.mounting_calibration:main",
```

用户执行：

```bash
ros2 run lpms_ig1_ros2 mounting_calibration
```

---

# 6. 使用环境

执行标定时：

```text
IMU 已经最终固定在车上
车辆完全静止
车辆停在尽可能水平的地面
轮胎正常充气
车辆正常载荷状态
```

不要：

```text
顶起某一个轮胎
停在明显斜坡
车体正在晃动
人在车上频繁上下车
```

---

# 7. 输入 topic

订阅：

```text
/imu/data
```

而不是：

```text
/imu/data_raw
```

原因：

此时 gyro 和 accelerometer 的传感器 calibration 已经完成。

安装姿态标定应该基于：

```text
校准后的 acceleration
```

读取：

```python
msg.linear_acceleration.x
msg.linear_acceleration.y
msg.linear_acceleration.z
```

单位：

```text
m/s²
```

同时读取：

```python
msg.angular_velocity
```

用于静止检测。

---

# 8. 本次标定的可观测量

静止车辆只有重力参考。

因此能够可靠估计：

```text
Roll mounting offset
Pitch mounting offset
```

无法通过静止重力估计：

```text
Yaw mounting offset
```

必须在代码和 README 中明确写出这一点。

禁止使用：

```text
atan2(ay, ax)
```

之类的方法把重力横向分量错误解释成 yaw。

---

# 9. 标定过程

程序启动：

```text
============================================================
LPMS-IG1 VEHICLE MOUNTING CALIBRATION
============================================================

Purpose:
Estimate IMU mounting roll and pitch relative to base_link.

IMPORTANT:
Static gravity cannot determine mounting yaw.

Requirements:
- Vehicle stationary
- Flat ground
- IMU permanently installed
- No strong vibration

Press ENTER when ready.
```

---

# 10. Warm-up

参数：

```text
warmup_duration
```

默认：

```text
5.0 sec
```

只接收数据，不用于计算。

---

# 11. 正式采集

参数：

```text
sample_duration
```

默认：

```text
30.0 sec
```

采集：

```text
ax
ay
az

gx
gy
gz
```

不要假设固定采样频率。

记录：

```text
actual_duration
sample_count
sample_rate
```

---

# 12. 静止检查

计算：

```python
gyro_norm = sqrt(
    gx*gx +
    gy*gy +
    gz*gz
)
```

参数：

```text
max_stationary_gyro
```

默认：

```text
0.03 rad/s
```

统计：

```text
motion_count
motion_ratio
```

参数：

```text
max_motion_ratio
```

默认：

```text
0.05
```

如果：

```text
motion_ratio > 5%
```

则 FAIL。

---

# 13. 重力检查

计算每个样本：

```python
a_norm = sqrt(
    ax*ax +
    ay*ay +
    az*az
)
```

平均：

```text
mean_acc_norm
```

应接近：

```text
G = 9.80665 m/s²
```

参数：

```text
gravity_tolerance
```

默认：

```text
0.5 m/s²
```

如果严重偏离则拒绝标定。

---

# 14. 使用平均重力向量

标定数据：

```python
ax = mean(ax_samples)
ay = mean(ay_samples)
az = mean(az_samples)
```

静止情况下，当前 ROS driver 的约定应使：

```text
IMU +Z 朝上时：

az ≈ +9.80665
```

车辆理想 base_link 重力观测应为：

```text
[0, 0, +G]
```

---

# 15. Roll / Pitch 计算

必须特别注意当前 acceleration 的 ROS 符号约定。

对于静止状态、Z-up 且 acceleration 为 proper acceleration：

可以使用与当前坐标定义一致的重力方向计算 mounting tilt。

推荐首先归一化：

```python
norm = sqrt(ax² + ay² + az²)

nx = ax / norm
ny = ay / norm
nz = az / norm
```

然后：

```python
roll_measured =
    atan2(ny, nz)

pitch_measured =
    atan2(
        -nx,
        sqrt(ny*ny + nz*nz)
    )
```

但是：

> Codex 不得只凭公式假设符号正确。

必须添加单元测试或离线测试验证：

```text
[0, 0, +G]
=> roll  = 0
=> pitch = 0
```

并测试：

```text
已知 +5° roll
已知 -5° roll
已知 +5° pitch
已知 -5° pitch
```

确保最终 TF 的方向是正确的。

---

# 16. TF 方向要特别核对

最终要发布：

```text
base_link -> imu_link
```

而重力计算得到的是：

```text
IMU frame 相对于水平车辆 frame 的倾斜
```

不要因为：

```text
sensor rotation
```

与：

```text
TF parent-to-child rotation
```

概念混淆而把角度符号写反。

必须做数学验证。

建议使用：

```text
scipy Rotation
```

不是必须。

也可以自行实现 quaternion。

但必须：

```text
验证 TF 后把 IMU 重力向量旋转到 base_link
```

结果接近：

```text
[0, 0, +G]
```

如果不是，则 TF 符号/方向有问题。

---

# 17. Yaw 的处理

增加参数：

```text
mount_yaw_deg
```

默认：

```text
0.0
```

用户可以根据机械安装情况填写。

例如 IMU X 轴相对车辆前方顺时针/逆时针偏差，需要按照 ROS +Z 右手规则明确输入。

程序输出：

```text
Yaw was NOT estimated from gravity.

Using user-specified mounting yaw:

0.000 deg
```

---

# 18. 不允许自动猜 yaw

禁止根据：

```text
LPMS quaternion yaw
```

直接推导 mounting yaw。

因为 LPMS yaw 涉及：

```text
magnetometer
heading reset
NWU / ENU
当地磁场
磁偏角
```

不能和机械安装 yaw 混为一谈。

---

# 19. 标定输出

例如：

```text
============================================================
LPMS-IG1 VEHICLE MOUNTING RESULT
============================================================

Samples:
3005

Duration:
30.01 sec

Sample rate:
100.1 Hz

Mean acceleration:
X: +0.328 m/s²
Y: -0.171 m/s²
Z: +9.798 m/s²

Magnitude:
9.805 m/s²

Estimated mounting:

Roll:
-0.999 deg

Pitch:
-1.917 deg

Yaw:
+0.000 deg
(user supplied)

Static check:
PASS
============================================================
```

---

# 20. TF 验证

程序在求出 transform 后必须内部进行一次验证。

将平均 acceleration：

```text
a_imu
```

通过候选 TF 旋转到：

```text
base_link
```

得到：

```text
a_base
```

输出：

```text
After mounting correction:

X: +0.003 m/s²
Y: -0.005 m/s²
Z: +9.805 m/s²
```

计算：

```text
horizontal_residual =
sqrt(
    ax_base² +
    ay_base²
)
```

建议：

```text
< 0.15 m/s²  -> PASS
< 0.30 m/s²  -> WARNING
>=0.30       -> FAIL/WARNING
```

阈值做成参数。

---

# 21. 地面倾斜是不可分辨误差

这是一个非常重要的限制。

静止重力标定得到的是：

```text
IMU 相对重力方向的倾斜
```

它无法区分：

```text
IMU 安装歪了 1°
```

和：

```text
地面本身斜了 1°
```

所以输出必须提醒：

```text
Mounting roll/pitch accuracy is limited by vehicle floor/ground level.
```

用户应该尽量寻找水平场地。

---

# 22. 保存 mounting YAML

生成：

```text
~/.ros/lpms_ig1_mounting.yaml
```

例如：

```yaml
lpms_ig1_mounting:
  parent_frame: base_link
  child_frame: imu_link

  translation:
    x: 0.0
    y: 0.0
    z: 0.0

  rotation_rpy_deg:
    roll: -0.999
    pitch: -1.917
    yaw: 0.0
```

注意：

```text
translation
```

不能由 IMU 数据自动求出来。

默认：

```text
0
0
0
```

并要求用户填写实际测量值。

---

# 23. IMU 位置也需要用户测量

增加参数：

```text
mount_x
mount_y
mount_z
```

单位：

```text
meter
```

含义：

```text
imu_link 原点相对于 base_link 原点的位置
```

按照：

```text
X forward
Y left
Z up
```

例如：

```text
IMU 在 base_link 前方 0.42 m
左侧 0.03 m
上方 0.55 m
```

则：

```text
mount_x = 0.42
mount_y = 0.03
mount_z = 0.55
```

程序不能自己估计位置。

---

# 24. 生成 static TF launch

建议自动生成：

```text
lpms_ig1_mounting.launch.py
```

或者输出用户应该执行的：

```bash
ros2 run tf2_ros static_transform_publisher ...
```

最好使用 quaternion 参数，避免 CLI RPY 顺序歧义。

最终目标：

```text
base_link
    |
    └── imu_link
```

---

# 25. 推荐正式 launch 集成方式

主 launch：

```text
lpms_ig1.launch.py
```

支持参数：

```text
publish_static_tf:=true
```

以及：

```text
mounting_file:=...
```

默认：

```text
~/.ros/lpms_ig1_mounting.yaml
```

如果文件不存在：

```text
不要崩溃
```

可以：

```text
WARNING
```

并不发布 TF。

---

# 26. 不允许修改 imu header.frame_id

LPMS node 保持：

```python
msg.header.frame_id = "imu_link"
```

不要因为有了：

```text
base_link -> imu_link
```

就把消息 frame_id 改成：

```text
base_link
```

传感器数据原始表达 frame 仍然是：

```text
imu_link
```

这是正确的 TF 使用方式。

---

# 27. TF 验证命令

Codex 完成后应提供：

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

以及：

```bash
ros2 topic echo /imu/data --once
```

---

# 28. RViz 验证

如果系统已有 RViz：

```bash
rviz2
```

设置：

```text
Fixed Frame:
base_link
```

显示：

```text
TF
Axes
```

应能看到：

```text
imu_link
```

相对：

```text
base_link
```

的位置和姿态。

---

# 29. 可选：发布 mounting 校正后的调试数据

可以实现一个：

```text
/imu/debug/base_accel
```

但不是必须。

不要另造一个：

```text
sensor_msgs/Imu
```

并把 frame_id 写成 base_link，除非所有 vector 和 covariance 都正确进行旋转。

优先使用 TF。

---

# 30. Covariance

本阶段不要擅自修改：

```text
orientation_covariance
angular_velocity_covariance
linear_acceleration_covariance
```

安装 TF 与 covariance 标定是两件事情。

---

# 31. 本阶段不做磁力计 calibration

虽然下一阶段会涉及磁力计，但本阶段不要实现：

```text
hard iron
soft iron
mag bias
mag correction matrix
heading offset
```

先确定机械坐标关系。

---

# 32. 本阶段结束后的下一步

完成 mounting / TF 后，下一阶段为：

```text
车辆磁环境检测
      ↓
Magnetometer calibration
      ↓
Heading / yaw validation
```

LPMS-IG1 官方确实支持：

```text
START_MAG_CALIBRATION 0x54
STOP_MAG_CALIBRATION  0x55
```

并支持设置 magnetometer calibration timeout。

但是暂时不要在本任务中实现这些命令。

后续任务会单独处理。

---

# 33. 为什么磁力计放在 mounting 后面

车辆上会存在：

```text
钢结构
车架
电机
伺服
动力线
DC/DC
电池
高电流回路
```

这些会改变磁场。

所以最终磁环境必须基于：

```text
IMU 的最终安装位置
```

评估。

---

# 34. 地面车辆磁力计的特殊问题

后续 Codex 任务必须注意：

LPMS 官方完整 magnetometer calibration 通常要求传感器：

```text
yaw 旋转多圈
pitch 旋转多圈
roll 旋转多圈
随机多方向旋转
```

完整覆盖三维磁场。官方说明这是 hard-iron / soft-iron 标定方法。

但是：

> 一辆已经装好的地面车辆通常无法做完整 pitch / roll 翻转。

因此不能简单要求用户：

```text
把整辆车翻过去做六面磁标定
```

后续需要单独设计：

```text
ground vehicle 2D magnetic calibration
```

或者使用 LPMS 在安装前完成的 3D calibration，再评估车辆附加磁干扰。

---

# 35. 动态磁干扰不能靠 calibration 解决

下一阶段尤其需要检查：

```text
车辆断电
车辆上电
驱动器上电
电机静止
电机运行
不同油门/电流
```

情况下磁场是否变化。

如果磁场会随着：

```text
motor current
```

明显变化，则：

```text
hard-iron / soft-iron 静态 calibration
```

无法真正解决。

正确处理可能是：

```text
移动 IMU 位置
远离动力线
改善线束布局
不用磁力计参与 yaw
```

这是下一阶段内容。

---

# 36. 参数

新增 mounting calibration node 参数：

```yaml
input_topic: /imu/data

warmup_duration: 5.0

sample_duration: 30.0

minimum_samples: 100

max_stationary_gyro: 0.03

max_motion_ratio: 0.05

gravity_m_s2: 9.80665

gravity_tolerance: 0.5

max_horizontal_residual: 0.30

mount_yaw_deg: 0.0

mount_x: 0.0
mount_y: 0.0
mount_z: 0.0

parent_frame: base_link
child_frame: imu_link

output_file:
  ~/.ros/lpms_ig1_mounting.yaml
```

不要硬编码：

```text
/home/young
```

使用：

```python
Path.home()
```

---

# 37. Ctrl+C

用户按：

```text
Ctrl+C
```

正常退出：

```text
Mounting calibration cancelled by user.
```

不要 traceback。

---

# 38. README 更新

新增：

```text
Vehicle Mounting Calibration
```

说明：

```text
1. 将 IMU 最终固定在车上
2. 车辆停在水平地面
3. 保持静止
4. 运行 mounting_calibration
5. 得到 roll/pitch
6. 手工提供 mounting yaw
7. 测量 XYZ 安装位置
8. 生成 base_link -> imu_link TF
```

同时明确：

```text
Static accelerometer measurements cannot determine yaw.
```

---

# 39. 代码测试

至少给姿态计算函数做以下离线测试。

输入：

```text
[0, 0, +G]
```

预期：

```text
roll  = 0°
pitch = 0°
```

然后人工构造：

```text
+5° roll
-5° roll
+5° pitch
-5° pitch
```

生成对应 gravity vector。

再运行 estimator。

误差应该接近：

```text
0
```

---

# 40. 最重要的 TF 单元测试

创建已知：

```text
base_link -> imu_link
```

例如：

```text
roll  = +5°
pitch = -3°
yaw   = +10°
```

计算静止时 IMU frame 中应该看到的重力。

然后把该重力交给 estimator。

验证：

```text
roll / pitch
```

能够恢复正确安装 tilt。

再应用生成的 TF。

最终：

```text
gravity in base_link
≈ [0, 0, +G]
```

该测试是强制项。

---

# 41. 用户实际验收流程

首先安装 IMU。

然后：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

检查：

```bash
ros2 topic hz /imu/data
```

车辆保持静止。

运行：

```bash
ros2 run lpms_ig1_ros2 mounting_calibration
```

得到：

```text
Calibration status: PASS
```

生成：

```text
~/.ros/lpms_ig1_mounting.yaml
```

然后启动 static TF。

确认：

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

成功。

---

# 42. 最终交付内容

完成后 Codex 必须说明：

1. 修改了哪些文件。
2. 新增了哪些文件。
3. Roll/Pitch 如何计算。
4. 为什么不能静态计算 yaw。
5. TF 方向是如何验证的。
6. 如何配置安装 XYZ。
7. 如何 build。
8. 如何运行 mounting calibration。
9. 如何加载 static TF。
10. 如何验证 TF。
11. 单元测试结果。
12. 下一阶段磁力计标定的接口准备情况。

---

# 43. 本阶段禁止事项

不要：

```text
修改 CAN mapping
交换 IMU XYZ
硬编码安装角
根据 quaternion yaw 自动猜 mounting yaw
修改 LPMS 内部 heading
做磁力计 hard/soft iron calibration
加入 EKF
加入滤波
修改 robot_localization 配置
```

本阶段保持单一目标：

```text
确定并正确表达：

base_link -> imu_link
```

---

# 44. 阶段完成判定

到目前为止整个标定流水线应该成为：

```text
Gyro static bias
        ✅
        ↓
Accelerometer six-position
        ✅
        ↓
Vehicle installation
        ✅
        ↓
Mounting roll/pitch + TF
        ← 本次任务
        ↓
Vehicle magnetic environment survey
        ↓
Magnetometer calibration
        ↓
Heading / yaw calibration
        ↓
robot_localization / navigation
```

完成本任务以后停止。

不要继续自动实现磁力计阶段。

等待下一份交接任务。