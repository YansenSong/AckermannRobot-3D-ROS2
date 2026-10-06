# LPMS-IG1 ROS2 静止陀螺标定脚本——开发交接文档

## 1. 当前项目背景

当前已经完成 LPMS-IG1 IMU 通过 CAN / SocketCAN 接入 ROS2。

硬件：

- IMU：LPMS-IG1
- CAN 接口：`can0`
- CANopen Node ID：`5`
- 对应 CAN ID：
  - `0x185`
  - `0x285`
  - `0x385`
  - `0x485`

当前 ROS2 驱动包：

```text
lpms_ig1_ros2
```

当前已经能够正常发布：

```text
/imu/data
/imu/data_raw
/imu/mag
```

主要消息类型：

```text
/imu/data_raw
sensor_msgs/msg/Imu
```

当前 `/imu/data_raw` 中：

```text
angular_velocity.x
angular_velocity.y
angular_velocity.z
```

已经转换为 ROS 标准单位：

```text
rad/s
```

加速度：

```text
linear_acceleration
```

单位：

```text
m/s^2
```

现在需要进行下一步：

> 在办公桌上将 IMU 平放并保持完全静止，对陀螺仪进行静止零偏测量、评估，并生成可用于后续 ROS2 数据修正的 bias 参数。

---

# 2. 本次开发目标

请为现有 `lpms_ig1_ros2` ROS2 包新增一个：

```text
静止陀螺仪标定 / 评估工具
```

它不是 CAN 驱动，而是一个独立 ROS2 节点。

它应该订阅：

```text
/imu/data_raw
```

自动采集一段时间的数据，例如默认：

```text
30 秒
```

然后计算三轴陀螺仪：

```text
gyro_bias_x
gyro_bias_y
gyro_bias_z
```

以及：

```text
mean
standard deviation
RMS
min
max
sample count
```

同时对静止状态做基本检查。

---

# 3. 使用场景

当前 IMU 会：

1. 放在稳定的办公桌上。
2. IMU 平放。
3. 标定期间完全不移动。
4. 桌面没有明显振动。
5. ROS2 IMU 驱动已经启动。
6. `/imu/data_raw` 已正常发布。

用户执行：

```bash
ros2 run lpms_ig1_ros2 gyro_calibration
```

程序开始后应给出类似提示：

```text
LPMS-IG1 static gyro calibration
Keep the IMU completely stationary.

Waiting for IMU data...
Warm-up: 5.0 s
Calibration: 30.0 s
```

然后自动采样。

---

# 4. 必须采用的数据

订阅：

```text
/imu/data_raw
```

消息：

```python
sensor_msgs.msg.Imu
```

使用：

```python
msg.angular_velocity.x
msg.angular_velocity.y
msg.angular_velocity.z
```

这里已经是：

```text
rad/s
```

不要重新做：

```text
deg/s -> rad/s
```

转换。

同时可以使用：

```python
msg.linear_acceleration.x
msg.linear_acceleration.y
msg.linear_acceleration.z
```

辅助判断 IMU 是否静止。

---

# 5. 标定算法

## 5.1 Warm-up 阶段

程序启动后不要立刻采集。

默认：

```text
warmup_duration = 5 秒
```

Warm-up 阶段只接收数据，不计入标定结果。

参数应该可以通过 ROS2 参数调整。

例如：

```bash
ros2 run lpms_ig1_ros2 gyro_calibration --ros-args \
  -p warmup_duration:=10.0
```

注意：

这里的 warm-up 只是脚本启动后的短等待。

真正严谨使用时，IMU 上电以后用户可能已经提前静置了一段时间。

---

# 6. 正式采样阶段

默认：

```text
calibration_duration = 30 秒
```

采集：

```python
gx
gy
gz
```

保存到内存。

不能假设 IMU 固定输出 100 Hz。

应该基于 ROS 实际收到的消息数量处理。

需要记录：

```text
start timestamp
end timestamp
sample count
effective sample rate
```

实际频率：

```text
sample_count / actual_duration
```

---

# 7. Gyro Bias 定义

静止状态下真实角速度：

```text
ωx = 0
ωy = 0
ωz = 0
```

因此测量平均值即为：

```text
bias_x = mean(gx)
bias_y = mean(gy)
bias_z = mean(gz)
```

单位：

```text
rad/s
```

同时为了方便人工理解，请额外输出：

```text
deg/s
```

计算：

```python
bias_dps = bias_rad_s * 180 / pi
```

但最终 ROS 参数仍优先保存：

```text
rad/s
```

---

# 8. 需要计算的统计量

对 X / Y / Z 每个轴分别计算：

```text
mean
standard deviation
RMS
minimum
maximum
peak-to-peak
```

定义：

```text
bias = mean
```

标准差：

```text
std = sqrt(mean((x - mean)^2))
```

RMS：

```text
rms = sqrt(mean(x^2))
```

peak-to-peak：

```text
max - min
```

要求使用：

```text
Python 标准库
```

或者：

```text
numpy
```

均可。

如果使用 numpy，需要在 `package.xml` 中正确声明依赖。

优先建议使用 Python 标准库，避免增加不必要依赖。

---

# 9. 静止状态检测

标定过程中不要盲目接受任何数据。

至少实现以下两个检测。

## 9.1 Gyro motion check

计算瞬时：

```text
gyro_norm
```

即：

```python
sqrt(gx^2 + gy^2 + gz^2)
```

提供参数：

```text
max_stationary_gyro
```

默认可以设置为：

```text
0.03 rad/s
```

约：

```text
1.72 deg/s
```

如果出现明显超过阈值的数据，认为 IMU 在移动。

不要因为一个偶发样本立刻退出。

可以统计：

```text
motion_sample_count
motion_sample_ratio
```

如果运动样本比例超过：

```text
5%
```

则最后标定失败。

---

# 10. 加速度静止检查

静止情况下：

```text
|a| ≈ 9.80665 m/s²
```

计算：

```python
acc_norm = sqrt(ax^2 + ay^2 + az^2)
```

检查：

```text
abs(acc_norm - 9.80665)
```

默认容差例如：

```text
1.0 m/s²
```

该阈值应做成 ROS 参数：

```text
accel_norm_tolerance
```

如果大量样本超限，应提示：

```text
IMU may be moving or vibrating.
Calibration rejected.
```

注意：

不要根据：

```text
ax ≈ 0
ay ≈ 0
az ≈ 9.8
```

判断静止。

因为 IMU 并不一定绝对水平。

应该只检查：

```text
加速度向量模长
```

---

# 11. 标定结果输出

成功后终端输出要求清晰。

例如：

```text
============================================================
LPMS-IG1 STATIC GYRO CALIBRATION RESULT
============================================================

Duration       : 30.01 s
Samples        : 3001
Sample rate    : 100.0 Hz

Gyroscope bias:

X:
  bias         : +0.000087 rad/s
  bias         : +0.004985 deg/s
  stddev       : 0.000412 rad/s
  RMS          : 0.000421 rad/s
  min          : -0.001745 rad/s
  max          : +0.001745 rad/s

Y:
  ...

Z:
  ...

Motion samples : 0 / 3001
Accel norm mean: 9.79 m/s²

Calibration status: PASS
============================================================
```

---

# 12. 生成 YAML 标定文件

成功以后自动输出一个 ROS2 YAML 文件。

默认路径建议：

```text
~/.ros/lpms_ig1_gyro_calibration.yaml
```

也可以通过参数指定：

```text
output_file
```

YAML 格式：

```yaml
lpms_ig1_node:
  ros__parameters:
    gyro_bias_x: 0.000087
    gyro_bias_y: -0.000035
    gyro_bias_z: 0.000052
```

同时建议保留统计信息作为注释或者额外字段：

```yaml
calibration:
  duration_sec: 30.01
  samples: 3001
  sample_rate_hz: 100.0

  gyro_bias_rad_s:
    x: 0.000087
    y: -0.000035
    z: 0.000052

  gyro_stddev_rad_s:
    x: 0.000412
    y: 0.000398
    z: 0.000405
```

如果想保证此 YAML 可以直接加载到 ROS2 node，则可以只把 ROS 参数部分放在主 YAML 中，再额外生成一个报告文件。

建议：

```text
lpms_ig1_gyro_calibration.yaml
lpms_ig1_gyro_calibration_report.yaml
```

但不是硬性要求。

---

# 13. 修改现有 LPMS ROS2 驱动

除了新增标定工具，还需要修改当前：

```text
lpms_ig1_node.py
```

增加三个 ROS 参数：

```text
gyro_bias_x
gyro_bias_y
gyro_bias_z
```

默认：

```text
0.0
```

单位：

```text
rad/s
```

当前程序已经将 LPMS：

```text
deg/s
```

转换成：

```text
rad/s
```

所以 bias 应在转换完成以后减掉。

逻辑：

```python
gx_ros = gx_raw_rad_s - gyro_bias_x
gy_ros = gy_raw_rad_s - gyro_bias_y
gz_ros = gz_raw_rad_s - gyro_bias_z
```

然后：

```python
imu.angular_velocity.x = gx_ros
imu.angular_velocity.y = gy_ros
imu.angular_velocity.z = gz_ros
```

---

# 14. `/imu/data_raw` 的含义

需要注意：

即使名字叫：

```text
/imu/data_raw
```

它现在已经经过：

```text
CAN 解码
单位转换
ROS 坐标规范转换
```

如果在主驱动里面应用：

```text
gyro bias correction
```

那么需要明确决定：

方案 A：

```text
/imu/data_raw
```

保留未做用户 bias correction 的 gyro，

而：

```text
/imu/data
```

使用 bias corrected gyro。

这种方案更推荐。

原因：

后续可以继续观察真实传感器零偏，并能够重新标定。

建议实现为：

```text
/imu/data_raw:
    gyro = 未减软件 bias 的 gyro

/imu/data:
    gyro = gyro - calibrated_bias
```

这样语义更合理。

---

# 15. 推荐最终话题关系

推荐调整为：

```text
LPMS CAN
   │
   ▼
单位转换
   │
   ├─────────────> /imu/data_raw
   │                 gyro 未做软件 bias correction
   │
   ▼
gyro bias correction
   │
   ▼
LPMS orientation
   │
   ▼
/imu/data
```

磁力计保持：

```text
/imu/mag
```

---

# 16. 标定脚本必须订阅哪个话题

标定工具必须订阅：

```text
/imu/data_raw
```

而不是：

```text
/imu/data
```

原因：

如果 `/imu/data` 已经减掉旧 bias，再对它求平均会导致重复修正。

标定永远应该针对：

```text
未应用当前软件 bias 的 gyro 数据
```

---

# 17. 参数设计

`gyro_calibration` 节点至少支持：

```text
input_topic
warmup_duration
calibration_duration
max_stationary_gyro
accel_norm_tolerance
max_motion_ratio
output_file
```

推荐默认值：

```yaml
input_topic: /imu/data_raw

warmup_duration: 5.0

calibration_duration: 30.0

max_stationary_gyro: 0.03

accel_norm_tolerance: 1.0

max_motion_ratio: 0.05
```

---

# 18. 失败情况

以下情况必须清晰报错。

## 没有收到 IMU

例如 5 秒内没有任何消息：

```text
ERROR:
No IMU messages received on /imu/data_raw
```

退出非零状态。

---

## 标定过程中运动过大

输出：

```text
Calibration FAILED.

Reason:
IMU was not stationary during calibration.

Motion ratio:
12.6 %

Maximum allowed:
5.0 %
```

不要写 YAML bias 文件。

---

## 样本过少

例如：

```text
sample_count < 100
```

认为结果不可靠。

提示并失败。

---

# 19. Ctrl+C

用户按：

```text
Ctrl+C
```

时应正常退出。

不要留下 traceback。

例如：

```text
Calibration cancelled by user.
```

---

# 20. ROS2 包结构

现有：

```text
lpms_ig1_ros2/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── config/
├── launch/
└── lpms_ig1_ros2/
    ├── __init__.py
    └── lpms_ig1_node.py
```

新增：

```text
lpms_ig1_ros2/
└── lpms_ig1_ros2/
    └── gyro_calibration.py
```

然后 `setup.py`：

```python
entry_points={
    "console_scripts": [
        "lpms_ig1_node = lpms_ig1_ros2.lpms_ig1_node:main",
        "gyro_calibration = lpms_ig1_ros2.gyro_calibration:main",
    ],
}
```

---

# 21. 运行方法

完成以后应支持：

```bash
cd ~/ros2_ws

colcon build --symlink-install \
  --packages-select lpms_ig1_ros2

source install/setup.bash
```

先启动 IMU：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

另外一个终端：

```bash
ros2 run lpms_ig1_ros2 gyro_calibration
```

自定义 60 秒：

```bash
ros2 run lpms_ig1_ros2 gyro_calibration \
  --ros-args \
  -p calibration_duration:=60.0
```

---

# 22. 加载标定结果

主驱动应该支持：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py \
  calibration_file:=/home/young/.ros/lpms_ig1_gyro_calibration.yaml
```

如果当前 launch 结构不方便传 calibration file，也可以支持：

```bash
ros2 run lpms_ig1_ros2 lpms_ig1_node \
  --ros-args \
  --params-file ~/.ros/lpms_ig1_gyro_calibration.yaml
```

请优先保证后者能够工作。

如果方便，可以进一步修改：

```text
lpms_ig1.launch.py
```

允许指定：

```text
params_file
```

---

# 23. 验证标定效果

完成标定并加载 bias 后：

```bash
ros2 topic echo /imu/data
```

静止时：

```text
angular_velocity.x
angular_velocity.y
angular_velocity.z
```

长期平均应该更加接近：

```text
0 rad/s
```

但注意：

瞬时数据仍允许有噪声。

例如 LPMS 当前 16-bit CAN gyro 分辨率大约会导致：

```text
0
±0.001745 rad/s
```

附近跳动。

不要为了让输出永远等于 0 而实现 deadband / threshold。

本次任务只做：

```text
bias subtraction
```

不要擅自加入：

```text
低通滤波
dead zone
threshold
Kalman filter
moving average
```

这些以后单独处理。

---

# 24. 验证原始话题没有被污染

加载 calibration 后：

```bash
ros2 topic echo /imu/data_raw
```

仍然应该能看到传感器本来的 gyro bias。

而：

```bash
ros2 topic echo /imu/data
```

应该看到减掉 bias 后的值。

这是重要验收项。

---

# 25. 标定前实际环境

本次测试条件：

```text
地点：
办公桌

IMU：
LPMS-IG1

状态：
平放在桌面

运动：
完全静止

目的：
静态陀螺仪零偏估计
```

当前之前观察到的典型数据大约为：

```text
GYR[dps]

X ≈  0.0
Y ≈ -0.1 ~ +0.1
Z ≈  0.0
```

也就是说转换到 ROS 后大约：

```text
±0.001745 rad/s
```

的量级。

这是正常的量化和噪声范围。

不要以为：

```text
瞬时 ±0.1 dps
```

就是巨大 bias。

真正的 bias 应由几十秒样本的：

```text
平均值
```

决定。

---

# 26. 不属于本次任务

本次不要实现：

```text
Accelerometer six-position calibration
Magnetometer hard-iron calibration
Magnetometer soft-iron calibration
Heading calibration
Temperature compensation
Allan variance
robot_localization
EKF
TF mounting calibration
```

这些后续单独做。

当前唯一目标：

> 准确、安全、可重复地测量静止 gyro bias，并可以把结果作为 ROS2 参数应用到 `/imu/data`。

---

# 27. 代码质量要求

请：

- 保持代码结构清晰。
- 不修改现有 CAN 解码规则，除非发现明确 bug。
- 不引入不必要依赖。
- 使用 ROS2 logger。
- 做好异常处理。
- 输出单位必须明确。
- 注释关键计算。
- 不要硬编码用户 home 路径。
- 用 `Path.home()` 获取用户目录。
- Python 文件通过基本语法检查。
- 保证 `colcon build` 能成功。

---

# 28. 最终交付

完成后请给出：

1. 修改后的文件列表。
2. `gyro_calibration.py`。
3. 修改后的 `lpms_ig1_node.py`。
4. 修改后的 `setup.py`。
5. 如有需要，修改 `launch`。
6. 完整 build 命令。
7. 完整标定命令。
8. 完整加载标定参数命令。
9. 验证 `/imu/data_raw` 与 `/imu/data` 的命令。
10. 简单说明 bias 的计算方法。

---

# 29. 最终验收流程

用户将按如下方式验收：

Terminal 1：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

Terminal 2：

```bash
ros2 topic hz /imu/data_raw
```

确认有正常数据。

然后：

```bash
ros2 run lpms_ig1_ros2 gyro_calibration
```

IMU 保持完全静止至少 30 秒。

程序结束后应：

```text
PASS
```

并生成：

```text
~/.ros/lpms_ig1_gyro_calibration.yaml
```

然后重启驱动并加载参数。

最后比较：

```bash
ros2 topic echo /imu/data_raw
```

和：

```bash
ros2 topic echo /imu/data
```

预期：

```text
/data_raw
仍保留原始静止 gyro bias

/data
减掉标定 bias，长期均值更接近 0
```

不要以单个样本是否为 0 判断成功。

应以至少几十秒数据的平均值判断。

# 核心原则

本次做的是：

```text
静止零偏估计
```

而不是：

```text
让所有静止 gyro 数值强制变成 0
```

正确结果应该保留真实传感器噪声，只消除长期平均偏置。