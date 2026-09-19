# LPMS-IG1 ROS2 六面加速度计标定——Codex 开发交接文档

## 1. 当前状态

项目已经完成：

```text
LPMS-IG1
   ↓ CAN / SocketCAN
lpms_ig1_ros2
   ↓
/imu/data_raw
/imu/data
/imu/mag
```

并且上一阶段已经/正在实现：

```text
静止陀螺仪零偏标定
```

对应参数：

```text
gyro_bias_x
gyro_bias_y
gyro_bias_z
```

当前 ROS2 驱动约定：

```text
/imu/data_raw
```

保存**未应用用户软件标定参数**的 IMU 数据。

```text
/imu/data
```

保存应用软件标定后的 IMU 数据。

必须继续保持这个原则。

---

# 2. 本阶段只做什么

本阶段只实现：

> Accelerometer 六面静态标定

目标是估计三个轴各自的：

```text
zero offset / bias
scale / gain
```

即：

```text
accel_offset_x
accel_offset_y
accel_offset_z

accel_gain_x
accel_gain_y
accel_gain_z
```

单位：

```text
offset: m/s²
gain:   dimensionless
```

暂时不要做：

```text
磁力计标定
Heading 标定
安装方向标定
TF 标定
温度补偿
完整 3x3 misalignment matrix
椭球拟合
EKF
滤波器
```

这些留到 IMU 安装到车辆以后处理。

---

# 3. 为什么现在做加速度计

本阶段 IMU 可以直接在办公桌环境完成标定。

六面标定不依赖：

```text
车辆方向
磁场环境
机器人 base_link
```

只依赖：

```text
重力
```

因此适合装车前完成。

需要依次使 IMU 的六个方向朝上：

```text
+X UP
-X UP

+Y UP
-Y UP

+Z UP
-Z UP
```

每个姿态保持完全静止。

---

# 4. 输入话题

标定程序必须订阅：

```text
/imu/data_raw
```

消息：

```python
sensor_msgs.msg.Imu
```

使用：

```python
msg.linear_acceleration.x
msg.linear_acceleration.y
msg.linear_acceleration.z
```

当前单位已经是：

```text
m/s²
```

并且已经采用当前 ROS 驱动定义的传感器坐标系。

不要再次进行：

```text
g -> m/s²
```

转换。

同时可以读取：

```python
msg.angular_velocity
```

用于判断 IMU 是否真的静止。

---

# 5. ROS 坐标下的六面预期值

当前驱动已经把 LPMS 的加速度符号转换为 ROS IMU 约定。

因此：

## +X 朝上

预期：

```text
ax ≈ +9.80665
ay ≈ 0
az ≈ 0
```

## -X 朝上

预期：

```text
ax ≈ -9.80665
ay ≈ 0
az ≈ 0
```

## +Y 朝上

```text
ax ≈ 0
ay ≈ +9.80665
az ≈ 0
```

## -Y 朝上

```text
ax ≈ 0
ay ≈ -9.80665
az ≈ 0
```

## +Z 朝上

```text
ax ≈ 0
ay ≈ 0
az ≈ +9.80665
```

## -Z 朝上

```text
ax ≈ 0
ay ≈ 0
az ≈ -9.80665
```

注意：

不能要求单个样本严格等于这些值。

标定必须基于每个姿态一段时间内的：

```text
mean
```

---

# 6. 新增工具

新增：

```text
lpms_ig1_ros2/
└── accel_calibration.py
```

setup.py 增加：

```python
"accel_calibration = lpms_ig1_ros2.accel_calibration:main",
```

用户执行：

```bash
ros2 run lpms_ig1_ros2 accel_calibration
```

---

# 7. 标定交互流程

程序启动：

```text
============================================================
LPMS-IG1 SIX-POSITION ACCELEROMETER CALIBRATION
============================================================

This procedure requires six stationary orientations.

1. +X UP
2. -X UP
3. +Y UP
4. -Y UP
5. +Z UP
6. -Z UP
```

然后逐个姿态提示。

例如：

```text
STEP 1 / 6

Place the IMU with +X pointing upward.

Keep the sensor completely stationary.

Press ENTER when ready.
```

用户按 Enter 后：

```text
Checking orientation...
Collecting data: 8.0 seconds
```

完成：

```text
PASS

Samples: 801
Mean acceleration:
X: +9.79 m/s²
Y: +0.05 m/s²
Z: -0.03 m/s²

Continue to next orientation.
```

依次完成六面。

---

# 8. 每个姿态的默认采样时间

参数：

```text
sample_duration
```

默认：

```text
8.0 sec
```

建议允许：

```bash
ros2 run lpms_ig1_ros2 accel_calibration \
  --ros-args \
  -p sample_duration:=10.0
```

每个姿态至少要求：

```text
minimum_samples = 100
```

不要假定固定 100 Hz。

记录：

```text
samples
actual duration
effective sample rate
```

---

# 9. 每个姿态需要记录

保存：

```text
ax[]
ay[]
az[]

gx[]
gy[]
gz[]
```

至少计算：

```text
mean
stddev
min
max
RMS
```

对 acceleration 额外计算：

```text
acc_norm
```

即：

```python
sqrt(ax**2 + ay**2 + az**2)
```

---

# 10. 静止检测

采样过程中必须检查运动。

使用：

```text
gyro_norm
```

定义：

```python
sqrt(gx**2 + gy**2 + gz**2)
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
motion_sample_count
motion_ratio
```

默认：

```text
max_motion_ratio = 0.05
```

如果：

```text
motion_ratio > 5%
```

则该姿态采样失败。

提示：

```text
FAILED:
IMU moved during sampling.

Please keep the sensor stationary and retry this orientation.
```

只重做当前姿态，不要让用户重新做前面已经成功的姿态。

---

# 11. 姿态方向检查

仅仅静止还不够。

必须确认用户确实摆成当前要求的方向。

例如：

```text
+X UP
```

不能误摆成：

```text
+Y UP
```

建议使用采样均值向量：

```python
a = [mean_ax, mean_ay, mean_az]
```

归一化：

```python
a_unit = a / norm(a)
```

当前目标方向例如：

```text
+X = [1, 0, 0]
-X = [-1, 0, 0]
+Y = [0, 1, 0]
...
```

计算夹角：

```python
angle = acos(clamp(dot(a_unit, target), -1, 1))
```

转换为度。

参数：

```text
max_orientation_error_deg
```

默认建议：

```text
12 degrees
```

如果超过：

```text
12°
```

则拒绝这一面。

例如：

```text
Orientation check FAILED.

Requested:
+X UP

Measured direction error:
26.4 deg

Please reposition the IMU and retry.
```

---

# 12. 重力模长检查

每个姿态还需要检查：

```text
|a| ≈ g
```

使用：

```text
G = 9.80665 m/s²
```

参数：

```text
gravity_tolerance
```

建议默认：

```text
1.0 m/s²
```

如果：

```python
abs(mean_acc_norm - G) > gravity_tolerance
```

则提示：

```text
Acceleration magnitude is inconsistent with gravity.

Possible causes:
- sensor is moving
- strong vibration
- invalid data
- severe calibration error
```

对于轻微超限可以 WARNING。

严重超限应 FAIL。

---

# 13. 标定模型

当前阶段只做最简单、稳定、容易解释的：

```text
每轴 offset + gain
```

不要实现完整 3x3 矩阵。

设 X 轴在：

```text
+X UP
```

姿态时测量平均值：

```text
x_plus
```

在：

```text
-X UP
```

姿态时：

```text
x_minus
```

理论真实值分别：

```text
+G
-G
```

其中：

```text
G = 9.80665
```

---

# 14. Offset 计算

定义：

```python
offset_x = (x_plus + x_minus) / 2.0
```

同理：

```python
offset_y = (y_plus + y_minus) / 2.0
offset_z = (z_plus + z_minus) / 2.0
```

单位：

```text
m/s²
```

---

# 15. Gain 计算

定义软件修正后的结果：

```python
corrected = (raw - offset) * gain
```

因此：

```python
gain_x = (2 * G) / (x_plus - x_minus)
```

同理：

```python
gain_y = (2 * G) / (y_plus - y_minus)
gain_z = (2 * G) / (z_plus - z_minus)
```

注意：

这里的：

```text
gain
```

是“校正时乘上的系数”。

不要把它和：

```text
sensor scale error
```

混用。

最终必须明确按照：

```python
corrected_x = (raw_x - offset_x) * gain_x
```

使用。

---

# 16. 示例

假设：

```text
X+ = +9.91
X- = -9.70
```

则：

```text
offset_x
= (9.91 - 9.70) / 2
= +0.105 m/s²
```

注意实际公式是：

```python
(9.91 + (-9.70)) / 2
```

然后：

```text
gain_x
= 19.6133 / (9.91 - (-9.70))
≈ 1.00017
```

修正：

```python
ax_corrected = (ax_raw - 0.105) * 1.00017
```

---

# 17. 防止错误计算

如果出现：

```text
x_plus <= x_minus
```

或者：

```text
abs(x_plus - x_minus)
```

明显过小，则说明六面方向采集错了。

必须失败。

例如：

```text
Calibration FAILED:
Invalid +X / -X measurements.
```

同样检查 Y/Z。

---

# 18. 参数合理性检查

标定完成后对 gain 做 sanity check。

例如：

```text
0.8 < gain < 1.2
```

超出时不要直接静默保存。

至少输出：

```text
WARNING:
Unexpected accelerometer gain on X axis.
```

如果特别异常，例如：

```text
gain < 0.5
gain > 1.5
```

建议直接标定失败。

Offset 也应该输出供人工检查。

不要强制假定 offset 必须小于某个极小值，但例如：

```text
|offset| > 2 m/s²
```

应该给出明显 WARNING。

---

# 19. 六面数据全部采集完成后的验证

根据得到的：

```text
offset_x/y/z
gain_x/y/z
```

重新对六面平均值应用校正：

```python
a_corr = (a_raw - offset) * gain
```

对于：

```text
+X UP
```

修正后预期：

```text
X ≈ +G
Y ≈ 0
Z ≈ 0
```

以此类推。

输出 validation table：

```text
============================================================
VALIDATION
============================================================

Pose     X         Y         Z         |a|
------------------------------------------------------------
+X      +9.807    +0.041    -0.026     9.807
-X      -9.807    +0.032    -0.018     9.807

+Y      +0.054    +9.807    +0.031     9.807
-Y      +0.049    -9.807    +0.028     9.807

+Z      +0.062    -0.019    +9.807     9.807
-Z      +0.058    -0.014    -9.807     9.807
```

---

# 20. Cross-axis residual

虽然本次不做 3x3 misalignment correction，但需要测量：

```text
非目标轴残差
```

例如：

```text
+X UP
```

理想：

```text
Y = 0
Z = 0
```

如果修正后仍出现：

```text
Y = 0.7 m/s²
```

说明可能存在：

```text
摆放不正
传感器轴不垂直
安装面不平
cross-axis sensitivity
```

统计：

```text
maximum_off_axis_residual
```

如果超过：

```text
0.5 m/s²
```

先 WARNING。

不要在这一步擅自做矩阵标定。

---

# 21. 输出结果

成功后输出：

```text
============================================================
LPMS-IG1 ACCELEROMETER CALIBRATION RESULT
============================================================

Offset [m/s²]

X: +0.0321
Y: -0.0184
Z: +0.0743

Correction gain

X: 1.00142
Y: 0.99891
Z: 1.00213

Maximum off-axis residual:
0.083 m/s²

Calibration status:
PASS
============================================================
```

---

# 22. YAML 参数

不要覆盖上一阶段的 gyro calibration。

目标是最终形成统一参数文件：

```text
~/.ros/lpms_ig1_calibration.yaml
```

结构：

```yaml
lpms_ig1_node:
  ros__parameters:

    gyro_bias_x: 0.000012
    gyro_bias_y: -0.000031
    gyro_bias_z: 0.000018

    accel_offset_x: 0.0321
    accel_offset_y: -0.0184
    accel_offset_z: 0.0743

    accel_gain_x: 1.00142
    accel_gain_y: 0.99891
    accel_gain_z: 1.00213
```

非常重要：

如果文件中已经有：

```text
gyro_bias_x/y/z
```

必须保留。

不要因为做 accelerometer calibration 就把 gyro 参数覆盖掉。

推荐实现：

```text
读取已有 calibration YAML
       ↓
保留已有参数
       ↓
只更新 accel_* 参数
       ↓
重新写回
```

如果文件不存在，则新建。

---

# 23. 单独生成详细报告

建议另外生成：

```text
~/.ros/lpms_ig1_accel_calibration_report.yaml
```

保存：

```text
时间
每面采样数
每面采样频率
每面 mean
每面 stddev
每面 norm
orientation error
motion ratio
offset
gain
validation result
```

方便以后追踪。

主 ROS 参数文件不要塞太多统计字段。

---

# 24. 修改主驱动

修改：

```text
lpms_ig1_node.py
```

增加参数：

```text
accel_offset_x
accel_offset_y
accel_offset_z

accel_gain_x
accel_gain_y
accel_gain_z
```

默认：

```text
offset = 0.0
gain   = 1.0
```

---

# 25. 加速度修正顺序

当前 CAN 驱动已经完成：

```text
LPMS raw CAN
   ↓
LPMS signed value
   ↓
g
   ↓
ROS acceleration sign convention
   ↓
m/s²
```

软件 accelerometer calibration 必须在以上步骤之后执行。

即：

```python
ax_uncalibrated_ros = ...
ay_uncalibrated_ros = ...
az_uncalibrated_ros = ...
```

然后：

```python
ax_corrected = (
    ax_uncalibrated_ros - accel_offset_x
) * accel_gain_x

ay_corrected = (
    ay_uncalibrated_ros - accel_offset_y
) * accel_gain_y

az_corrected = (
    az_uncalibrated_ros - accel_offset_z
) * accel_gain_z
```

---

# 26. `/imu/data_raw` 必须保持未校正

这是强制要求。

```text
/imu/data_raw
```

应该发布：

```text
未应用 gyro software bias
未应用 accel offset/gain
```

但：

```text
已经完成 CAN 解码
已经转换到 ROS SI 单位
已经使用当前 ROS sensor frame
```

即：

```text
raw ≠ CAN raw bytes
```

而是：

```text
ROS 标准单位下的未软件校准传感器数据
```

---

# 27. `/imu/data`

```text
/imu/data
```

应发布：

```text
gyro:
raw gyro - gyro bias

accelerometer:
(raw accel - accel offset) * accel gain

orientation:
LPMS 输出的 fused quaternion
```

不要使用六面加速度计标定重新计算 quaternion。

不要修改 LPMS 自己的姿态融合结果。

---

# 28. 一个重要的一致性说明

当前：

```text
orientation
```

来自 LPMS 内部 AHRS。

而：

```text
linear_acceleration
```

将在 ROS 端做额外软件标定。

因此：

```text
/imu/data.orientation
```

仍然是 LPMS 内部自己的融合姿态。

不要声称：

```text
LPMS AHRS 也使用了这次 ROS 软件 accel calibration
```

它没有。

本次参数只影响 ROS 发布的 acceleration 数值。

如果以后需要真正影响 LPMS 内部 AHRS，需要单独研究设备内部标定命令。

本阶段不要执行设备 flash calibration 命令。

---

# 29. 标定程序默认参数

建议：

```yaml
input_topic: /imu/data_raw

sample_duration: 8.0

minimum_samples: 100

max_stationary_gyro: 0.03

max_motion_ratio: 0.05

max_orientation_error_deg: 12.0

gravity_m_s2: 9.80665

gravity_tolerance: 1.0

output_file:
  ~/.ros/lpms_ig1_calibration.yaml

report_file:
  ~/.ros/lpms_ig1_accel_calibration_report.yaml
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

# 30. Ctrl+C

任何阶段按：

```text
Ctrl+C
```

应正常退出：

```text
Accelerometer calibration cancelled by user.
```

不要打印 Python traceback。

已经完成的临时六面数据不需要保存为正式 calibration。

---

# 31. 推荐用户实际操作顺序

用户会按如下方式操作。

启动 IMU：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

新终端：

```bash
ros2 run lpms_ig1_ros2 accel_calibration
```

然后程序依次要求：

```text
+X UP
-X UP
+Y UP
-Y UP
+Z UP
-Z UP
```

用户每次摆好以后：

```text
等待完全静止
按 Enter
```

程序自动采样。

---

# 32. 标定完成后加载参数

例如：

```bash
ros2 run lpms_ig1_ros2 lpms_ig1_node \
  --ros-args \
  --params-file ~/.ros/lpms_ig1_calibration.yaml
```

如果现有 launch 已支持 calibration file，则也应该支持：

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py \
  calibration_file:=$HOME/.ros/lpms_ig1_calibration.yaml
```

---

# 33. 标定效果检查

标定完成以后，将 IMU：

```text
+Z 朝上
平放
静止
```

检查：

```bash
ros2 topic echo /imu/data --once
```

预期：

```text
linear_acceleration.x ≈ 0
linear_acceleration.y ≈ 0
linear_acceleration.z ≈ +9.80665
```

不要要求：

```text
x/y == 0.000000
z == 9.806650
```

存在正常噪声。

---

# 34. 比较 raw 与 corrected

检查：

```bash
ros2 topic echo /imu/data_raw --once
```

再：

```bash
ros2 topic echo /imu/data --once
```

应能看到：

```text
/data_raw
保持标定前的软件原始值

/data
应用 offset / gain 修正
```

---

# 35. 可选验证工具

如果开发成本不高，可以给：

```text
accel_calibration
```

增加：

```text
--verify-only
```

或者 ROS 参数：

```text
verify_only:=true
```

模式。

该模式读取现有 YAML，不重新求 calibration。

依次让用户做六面姿态，然后报告校正后的误差。

这是可选项，不应阻塞主任务。

---

# 36. 本阶段不要做磁力计标定

这一点非常重要。

当前办公桌环境可能与车辆环境完全不同。

装车以后附近可能有：

```text
钢制车架
电机
舵机
驱动器
DC/DC
电池大电流线
CAN 线
电源线
螺丝
磁铁
音响
```

这些都会改变局部磁场。

因此现在不要把办公桌磁力计标定结果作为最终车辆参数。

最多可以观察：

```text
magnetic field magnitude
```

但不要建立最终：

```text
hard iron
soft iron
heading offset
```

参数。

---

# 37. 本阶段不要做安装方向标定

车辆还没安装 IMU，因此现在无法最终定义：

```text
base_link -> imu_link
```

不要通过软件交换：

```text
XYZ
```

来假装完成安装方向校准。

安装后应使用：

```text
TF
```

明确描述传感器相对车辆的位置和旋转。

---

# 38. 装车后还需要完成的工作

本任务结束后，用户会把 LPMS-IG1 安装到车辆。

下一阶段计划：

```text
1. 确认 imu_link 相对 base_link 的安装方向

2. 创建/验证 static TF

3. 装车静止 gyro bias 复检

4. 装车加速度重力方向复检

5. 磁力计 hard-iron calibration

6. 磁力计 soft-iron calibration

7. 检查车辆电机通电前/后的磁场变化

8. Heading / yaw 校准

9. 检查 ENU / NWU 转换

10. 最后接 robot_localization / EKF
```

这些不属于本次 Codex 任务。

---

# 39. 代码质量要求

必须：

- Python 代码清晰。
- 不引入不必要依赖。
- 优先 Python 标准库。
- 使用 ROS2 logger。
- 正确处理 ROS shutdown。
- 正确处理 Ctrl+C。
- 不硬编码用户目录。
- 保持现有 CAN 解码逻辑不变。
- 保持 `/imu/data_raw` 未应用用户软件标定。
- 保持 `/imu/data` 应用软件标定。
- YAML 更新时不能丢失 gyro calibration。
- 明确所有数值单位。
- 编译通过。
- 更新 README。

---

# 40. README 需要新增内容

补充：

```text
Accelerometer Calibration
```

包含：

运行：

ros2 run lpms_ig1_ros2 accel_calibration

六面顺序：

+X
-X
+Y
-Y
+Z
-Z

以及解释：

```text
/imu/data_raw = 未应用软件 calibration
/imu/data     = 应用 gyro + accel calibration
```

并明确：

```text
Magnetometer calibration should be performed after final vehicle installation.
```

---

# 41. 最终需要 Codex 汇报

完成任务后请给出：

1. 修改了哪些文件。
2. 新增了哪些文件。
3. 六面标定算法。
4. offset 计算公式。
5. gain 计算公式。
6. YAML 结构。
7. build 命令。
8. 标定命令。
9. 参数加载命令。
10. raw/corrected 数据验证命令。
11. 六面实际操作说明。
12. 是否通过基础测试。

---

# 42. 最终验收流程

## Terminal 1

```bash
ros2 launch lpms_ig1_ros2 lpms_ig1.launch.py
```

## Terminal 2

确认：

```bash
ros2 topic hz /imu/data_raw
```

然后：

```bash
ros2 run lpms_ig1_ros2 accel_calibration
```

完成：

```text
+X
-X
+Y
-Y
+Z
-Z
```

六个方向。

程序应该输出：

```text
Calibration status: PASS
```

并生成：

```text
~/.ros/lpms_ig1_calibration.yaml
```

该文件必须同时保留之前的：

```text
gyro_bias_x
gyro_bias_y
gyro_bias_z
```

以及新增：

```text
accel_offset_x
accel_offset_y
accel_offset_z

accel_gain_x
accel_gain_y
accel_gain_z
```

---

# 43. 标定后最终验证

加载 YAML 后，IMU +Z 向上平放。

检查：

```bash
ros2 topic echo /imu/data_raw --once
```

和：

```bash
ros2 topic echo /imu/data --once
```

预期：

```text
/imu/data_raw
仍为未应用软件 offset/gain 的 acceleration

/imu/data
应用六面 calibration
```

`/imu/data` 静止时：

```text
|a| ≈ 9.80665 m/s²
```

且平放时：

```text
X ≈ 0
Y ≈ 0
Z ≈ +9.80665
```

---

# 44. 核心原则

本阶段目标不是：

```text
让所有数据看起来完美
```

而是通过六个已知的重力参考点，估计：

```text
每轴零偏
每轴比例误差
```

并做简单、透明、可逆的软件修正：

```python
corrected = (raw - offset) * gain
```

不要为了得到漂亮数据加入：

```text
deadband
moving average
low-pass filter
人为归零
```

正常传感器噪声应该继续存在。

---

# 45. 本阶段完成的判定

完成以下项目即可结束本阶段：

```text
Gyro static bias          DONE
Accelerometer six-face    DONE
```

然后用户即可把 IMU 正式安装到车辆。

装车以后再进行：

```text
Mounting / TF
Magnetometer
Heading
Vehicle-environment validation
```