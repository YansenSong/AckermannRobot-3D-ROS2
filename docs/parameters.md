# 一、机器人实际参数基准

这部分建议作为 Hybrid A* 和 NeuPAN 的共同“真值”。

| 参数             |              当前/URDF |            我建议规划统一采用 | 说明              |
| -------------- | -------------------: | -------------------: | --------------- |
| 轴距 `wheelbase` | 0.593 / 实际约0.59379 m |          **0.593 m** | 当前误差极小，不值得单独折腾  |
| 左右转向关节极限       |            ±0.52 rad |           物理保持 ±0.52 | 这是单个前轮关节极限      |
| 自行车模型中心转角      |            当前按 ±0.52 |       **±0.414 rad** | 配合 Rmin=1.35 m  |
| 理论几何极限 Rmin    |             当前1.05 m |              ≈1.29 m | 内轮刚好达到0.52      |
| 规划使用 Rmin      |               1.05 m |           **1.35 m** | 给控制误差留余量        |
| chassis 长度     |               0.70 m |                    — | 仅底盘本体           |
| collision 整车长度 |             ≈0.780 m | **0.80 m footprint** | 包含车轮            |
| collision 整车宽度 |             ≈0.569 m | **0.58 m footprint** | 包含车轮            |
| 后轮最大角速度        |            ±10 rad/s |                    — | ros2_control限制  |
| 后轮半径           |              0.093 m |                    — | 直线约0.93 m/s轮缘速度 |
| 建议规划 vmax      |       当前NeuPAN 2 m/s |    **0.70~0.75 m/s** | 保证弯道也不易饱和       |
| 初期参考速度         |              0.8 m/s |    **0.45~0.50 m/s** | 参数稳定后再升到0.6~0.7 |

这些原始几何和控制约束分别来自 Xacro 和 Ackermann controller。

---

# 二、Hybrid A* 参数分级

当前实际运行配置在 `standalone_planner/config/planner_params.yaml`。

## 一级：直接决定机器人能不能走、怎么走

| 参数                       |            当前值 |                建议值 | 参数意义 / 调整理由                              |
| ------------------------ | -------------: | -----------------: | ---------------------------------------- |
| `minimum_turning_radius` |       **1.05** |           **1.35** | 最重要参数。决定运动基元的最大曲率。当前1.05比物理Ackermann能力激进 |
| `vehicle_length`         |           0.70 |           **0.80** | Hybrid碰撞检测矩形长度。应该包含轮子，而不是只看底盘            |
| `vehicle_width`          |           0.52 |           **0.58** | 同上。当前明显低估轮胎外包络                           |
| `rear_axle_offset_x`     |       -0.30449 |             **保持** | 后轴相对base_link位置，与URDF完全对应                |
| `robot_state_frame`      | rear_axle_link |             **保持** | 正确。Ackermann运动模型应以后轴中心作状态参考              |
| `motion_model`           |    REEDS_SHEPP |             **保持** | 室内Ackermann需要前进+倒车                       |
| `reverse_penalty`        |        **5.0** |   **2.0~2.5，先2.3** | 当前倒车代价太高，狭窄空间中即使倒车更合理也会强行找前进路径           |
| `gear_change_penalty`    |        **4.0** |   **1.5~2.0，先1.8** | 当前严重压制前进↔倒车换挡                            |
| `change_penalty`         |        **2.0** |   **1.1~1.3，先1.2** | 抑制左右方向频繁切换；当前实现中作用非常强，2.0容易把正常S弯也惩罚过度    |
| `non_straight_penalty`   |            1.2 | **1.05~1.15，先1.1** | 转弯相对直行的代价。Ackermann室内导航不宜过分厌恶转弯          |

这些 penalty 并不是抽象的“偏好”。你这份代码里，倒车直接乘 `reverse_penalty`，换挡又继续乘 `gear_change_penalty`；左右运动基元切换还同时涉及 `change_penalty` 和 `non_straight_penalty`。所以现在 `reverse=5 + gear=4` 的组合确实非常狠。

## 二级：明显影响路径形态、规划成功率和精细程度

| 参数                              |     当前 |            建议 | 含义                                 |
| ------------------------------- | -----: | ------------: | ---------------------------------- |
| `angle_quantization`            |     72 |     **先保持72** | 5°一个航向桶。稳定后可以试96，即3.75°，路径更细但计算量增加 |
| `tolerance`                     | 0.10 m |    **保持0.10** | 目标位置容差，两格0.05m地图，很合理               |
| `analytic_expansion_ratio`      |    3.5 |   **3.0~4.0** | 越接近目标时尝试Reeds-Shepp解析连接的频度         |
| `analytic_expansion_max_length` |  5.0 m |     **4~5 m** | 解析连接允许的最大距离。配合1.35m Rmin当前5m仍可用    |
| `max_iterations`                |  50000 |  **80000 初期** | footprint和Rmin变大后搜索更困难，可以暂时提高      |
| `max_on_approach_iterations`    |   1000 | **1000~1500** | 已接近目标后允许继续搜索多少次                    |
| `allow_unknown`                 |  false |   **保持false** | 室内静态地图不建议进入未知区域                    |

Hybrid A* 的运动基元会同时依据 `Rmin`、地图 resolution 和 angle bin 自动决定每一步移动长度，所以 `Rmin` 改成1.35后不用另找“step size”参数。

## 三级：现在不是主要矛盾

| 参数                      |    当前 |          建议 | 说明                                                     |
| ----------------------- | ----: | ----------: | ------------------------------------------------------ |
| `cost_penalty`          |   2.0 | **暂时1.0即可** | 当前静态PGM几乎是二值地图，没有inflation距离场，因此它并不能像注释写的那样真正“让车走通道中央” |
| `use_smoother`          | false | **继续false** | 现在绝对不要因为路径不好看就打开                                       |
| `smooth_max_time`       |   0.1 |           — | smoother关闭时无影响                                         |
| `smooth_max_iterations` |    50 |           — | smoother关闭时无影响                                         |
| `resolution`            |  0.05 |     跟地图yaml | launch已从地图metadata覆盖它                                  |

这里有个很关键的实现问题：PGM adapter 只是把黑色变成 occupied、白色变成 free，并没有生成离障碍越近代价越高的 inflation cost。于是普通自由栅格的 cost 都是0，**单纯提高 `cost_penalty` 并不会让全局路径自动远离墙面**。

如果你现在最典型的问题之一是“Hybrid A* 总喜欢贴着墙走”，正确方向不是继续把 `cost_penalty` 从2改成5、10，而是给 PGM 生成**距离障碍物的渐变代价场**。这比 penalty 调值重要很多。

---

# 三、Hybrid A* 的 smoother 目前不要打开

这个我单独强调一下。

当前 smoother 初始化时真正传入的主要只有 `max_time` 和 `max_iterations`；默认 `max_curvature` 并没有根据你的 `1/Rmin` 设置。

而且更加麻烦的是，平滑完成后代码重新使用

`atan2(dy, dx)`

生成每个点的姿态。对于 **Reeds-Shepp 倒车段**，机器人车头朝向本来应该和运动方向相反；这么重新计算以后，很可能把倒车段变成“车头朝路径切向”。NeuPAN 又恰好依据“pose orientation 和 segment direction 是否相反”判断前进还是倒车，因此会把原本倒车段误判成前进。

所以：

**当前 `use_smoother:false` 是正确的。**

先不要动它。后面如果要平滑，我建议做“保持 gear 的分段曲率约束平滑”，而不是直接开启现有 smoother。

---

# 四、NeuPAN 一级参数：最应该先动

当前 Ackermann 专用配置是 `config/robots/ackermann_robot/planner.yaml`。

| 参数                    |       当前 |               第一版建议 | 作用                                           |
| --------------------- | -------: | ------------------: | -------------------------------------------- |
| `ref_speed`           |  **0.8** |            **0.50** | 参考速度。现在接近整车硬件极限，调参阶段太快                       |
| `robot.max_speed[0]`  |  **2.0** |            **0.75** | 最大线速度；2m/s和当前10rad/s车轮限制不匹配                  |
| `robot.max_speed[1]`  | **0.52** |           **0.414** | NeuPAN中的第二控制量是自行车模型转角ψ，不应直接等于物理内轮上限          |
| `robot.wheelbase`     |    0.593 |         **保持0.593** | 和controller统一                                |
| `ipath.min_radius`    | **1.05** |            **1.35** | 必须和Hybrid A*统一                               |
| `robot.max_acce[0]`   |      1.0 |    **0.6~0.8，先0.7** | 线速度变化率                                       |
| `robot.max_acce[1]`   |      0.5 | **0.30~0.40，先0.35** | 实际是转向角变化率 rad/s，不是配置注释写的rad/s²               |
| `adjust.d_min`        |     0.15 |            **0.20** | 优化器期望保持的最小障碍安全间距                             |
| `collision_threshold` |     0.05 | **0.08~0.10，先0.08** | 最后的硬停止阈值                                     |
| `adjust.eta`          |  **0.5** |        **2~5，先3.0** | 对安全约束 slack 的L1惩罚；当前0.5太容易允许优化器“借 slack”贴近障碍 |
| `adjust.q_s`          |      0.8 |   **[1.0,1.0,0.3]** | 状态跟踪权重。降低yaw相对权重，有利于局部绕障时别死抱全局航向             |
| `adjust.p_u`          |      1.0 |             **0.7** | 参考速度跟踪权重，稍降让避障时更愿意减速                         |
| `adjust.d_max`        |      1.2 |           **保持1.2** | 障碍影响范围，目前比较合理                                |

NeuPAN 上游代码明确将 `eta` 定义为 slack 的 L1 regularization gain，`d_max/d_min` 是最大/最小安全距离，`q_s` 是状态代价，`p_u` 是速度代价。并且现在已经支持把 `q_s` 设置成 `[x,y,theta]` 三维权重。

---

# 五、NeuPAN 有一个非常严重的 `ind_range` 配置问题

你现在写的是：

```yaml
ind_range: 0   # 搜索剩余全部路径
```

但是上游 NeuPAN 实现不是这么解释的。

它实际上执行：

```python
start = max(cur_index, 0)
end = min(cur_index + ind_range, len(self.cur_curve))

for index in range(start, end):
    ...
```

所以当 `ind_range=0` 时：

```python
range(start, start)
```

根本一个点都不搜索。

这意味着 `point_index` 很可能长期不更新，同时 `check_arrive()` 又依赖 `point_index` 接近路径末尾才能判断到达。这个配置非常可能造成**参考点推进异常、路径索引滞后、终点判断异常**。

我建议你先直接改成：

```yaml
ind_range: 50
```

按当前 Hybrid A* 大约 0.1m 量级的路径点间距，相当于向前寻找约几米范围，已经足够。

更干净的办法是改 NeuPAN 源码，让：

```python
ind_range <= 0
```

显式表示“搜索到曲线末尾”。但仅改 YAML 的话，先用 **50**。

这是我认为你现在 NeuPAN 行为异常里，最值得优先验证的一个点。

---

# 六、`ipath.interval: 0.1` 在你当前组合中基本不是你以为的作用

你配置了：

```yaml
ipath:
  interval: 0.1
```

但是你的 ROS 节点收到 Hybrid A* `/plan` 后会调用 `set_initial_path()`，而 NeuPAN 上游的 `set_initial_path()` 又会重新执行：

```python
self.interval = self.cal_average_interval(path)
```

即使用**输入 Hybrid A* 路径的平均点间距覆盖 YAML 的 interval**。

因此在你这套“Hybrid A* → NeuPAN”架构下：

**调 `ipath.interval: 0.1` 基本不能解决输入全局路径过密/过稀问题。**

如果要控制 NeuPAN 接收到的点距，更合理的是在两者之间做一次明确的 path resampling，比如固定 **0.10~0.15m**。

---

# 七、NeuPAN 的 `min_speed` 目前存在实现/API问题

你的 YAML 写着：

```yaml
max_speed: [2, 0.52]
min_speed: [-0.5, -0.52]
```

看上去想达到：

* 最大前进2m/s
* 最大倒车0.5m/s

但是我查看当前上游 NeuPAN 的 `robot` 实现后发现，它并没有处理 `min_speed`，而是：

```python
self.max_speed = ...
self.speed_bound = self.max_speed

cp.abs(self.indep_u) <= self.speed_bound
```

也就是上下限完全对称。

所以按照这版实现：

```yaml
min_speed: [-0.5, ...]
```

**并不能限制倒车为 -0.5。**

更麻烦的是，你仓库自己的 `neupan_node.py` 在 command rate limiter 里又访问：

```python
self.neupan_planner.robot.min_speed
```

但当前上游 `robot` 类里没有这个成员。

如果你现在仿真没在这里直接报 `AttributeError`，说明你实际 Python 环境中的 NeuPAN 版本和我检查到的当前上游实现并不完全一样。

而仓库的 `setup.py` 又没有 pin NeuPAN 版本，README 基本是直接让用户 `pip install neupan`，这会让环境变化非常隐蔽。

所以这里我的建议不是“再调一个数字”，而是：

**先固定 NeuPAN 版本，并把 `min_speed` 真正实现为优化约束。**

修完以后可以用：

```yaml
max_speed: [0.75, 0.414]
min_speed: [-0.35, -0.414]
```

这样倒车明显慢于前进，更适合你这台小型 Ackermann 车。

---

# 八、NeuPAN 二级参数：优化实时性和轨迹稳定性

| 参数                       |      当前 |             建议起点 | 说明                     |
| ------------------------ | ------: | ---------------: | ---------------------- |
| `receding`               |      15 |           **12** | MPC预测步数                |
| `step_time`              |     0.2 |        **保持0.2** | 12步就是2.4s预测域           |
| `pan.iter_num`           |       2 |      **先1，稳定后2** | PAN迭代次数，直接影响CPU时间      |
| `dune_max_num`           |     200 |      **100~120** | DUNE处理的障碍点上限           |
| `nrmp_max_num`           |      10 |         **8~10** | 真正进入NRMP优化的障碍约束数量      |
| `arrive_threshold`       | **0.5** |    **0.20~0.25** | 当前半米就判到达，对0.8m级机器人过于宽松 |
| `arrive_index_threshold` |       3 |          **保持3** | 与终点索引配合                |
| `close_threshold`        |     0.1 |    **0.10~0.15** | 当前点匹配阈值                |
| `control_frequency`      |    20Hz | **先10Hz，根据实测再升** | CPU求解跟不上20Hz时，写20没有意义  |
| `scan_downsample`        |       1 |          **1或2** | 360点激光用2通常仍够局部避障       |
| `scan_range_max`         |     25m |   **室内可降至8~12m** | 局部避障没必要长期吃25m范围障碍点     |

你自己的 ROS wrapper 已经明确承认 NeuPAN solve 是 CPU-heavy，而且实际 command limiter 也只能根据真实 wall-time 做处理。

因此建议调参时暂时：

```yaml
time_print: true
```

先记录规划耗时。只有当 NeuPAN 求解的绝大多数周期能在 **50ms以内**完成时，20Hz才真正有意义；如果是80~120ms，则老老实实跑8~10Hz反而更稳定。

---

# 九、`avoidance_seed_*` 我目前不建议你花时间调

你配置中有：

```yaml
avoidance_seed_enabled: true
avoidance_seed_distance: 1.2
avoidance_seed_clearance: 0.15
avoidance_seed_steer: 0.25
```

但是我检查当前上游 NeuPAN 时，这几个顶层参数进入 `**kwargs` 后没有看到对应逻辑真正消费它们；而你的 ROS wrapper 只是检查 `info.get("avoidance_seeded")`，本身并没有实现 seed 算法。

因此在当前仓库 + 当前上游实现的组合下，这几个很可能属于**无效参数/no-op**。

所以它们应该归在三级，而且在确认实际安装的 NeuPAN fork 前，不建议通过它们解释机器人绕障效果。

---

# 十、NeuPAN footprint 怎么改，有个模型训练问题

从真实 Gazebo collision 来说，我建议：

```yaml
robot:
  length: 0.80
  width: 0.58
```

但你自己的配置文件已经特别写了：

> DUNE 权重与车辆几何尺寸相关，几何参数必须与训练配置一致。

所以这里分两种处理。

**短期、不重新训练 DUNE：**

NeuPAN 暂时保持 checkpoint 训练时使用的 `0.70 × 0.52`（前提是这个checkpoint确实按这个尺寸训练），但把：

```yaml
d_min: 0.20
collision_threshold: 0.08
```

提高，给未建模的轮胎尺寸提供额外安全余量。

与此同时 Hybrid A* 可以立即改成：

```yaml
vehicle_length: 0.80
vehicle_width: 0.58
```

因为它没有学习模型依赖。

**长期正确做法：**

把 NeuPAN/DUNE 的机器人几何也改成真实约 **0.80 × 0.58 m**，重新生成/训练与这个 footprint 对应的 DUNE checkpoint。

这会比用一个0.52m宽的神经距离模型，再靠 `d_min` 一直补偿可靠得多。

---

# 十一、还有一个不是“算法参数”，但必须修的控制问题

`cmd_vel_mux.py` 每50ms把**最后一次 NeuPAN 命令重新发布时间戳后继续发给控制器**。

控制器虽然配置：

```yaml
reference_timeout: 2.0
```

但因为 mux 一直重新发布旧命令，controller看到的消息永远是新的。

结果就是：

**NeuPAN如果卡死、优化一次跑很久、甚至停止更新，机器人仍可能继续执行最后一次非零速度。**

这也特别容易表现成：

> “NeuPAN 怎么已经看到障碍了车还一直转/一直往前？”

但实际上可能不是优化器参数，而是旧指令在被 mux 不断重放。

建议给 mux 增加 NeuPAN command age，例如：

```text
0.2~0.3 s 未收到新的 /neupan_cmd_vel
→ 强制发布 zero Twist
```

这个我会放在“一级系统参数”，优先级非常高。

---

# 十二、我建议你先采用的第一版参数

Hybrid A* 我会先这样：

```yaml
minimum_turning_radius: 1.35
angle_quantization: 72
motion_model: "REEDS_SHEPP"

robot_state_frame: rear_axle_link
goal_pose_is_base_link: true
rear_axle_offset_x: -0.30449
vehicle_length: 0.80
vehicle_width: 0.58

max_iterations: 80000
max_on_approach_iterations: 1200
tolerance: 0.10
allow_unknown: false

change_penalty: 1.2
non_straight_penalty: 1.1
reverse_penalty: 2.3
gear_change_penalty: 1.8

# 当前二值PGM下意义有限
cost_penalty: 1.0

analytic_expansion_ratio: 3.5
analytic_expansion_max_length: 5.0

use_smoother: false
```

NeuPAN 在**不重新训练 DUNE footprint**的第一轮，可以先这样：

```yaml
receding: 12
step_time: 0.2
ref_speed: 0.50

collision_threshold: 0.08

robot:
  kinematics: 'acker'

  # 在修复 min_speed 实现前，注意上游约束仍可能是对称的
  max_speed: [0.75, 0.414]
  min_speed: [-0.35, -0.414]

  max_acce: [0.7, 0.35]

  # 如果当前DUNE checkpoint是按这组几何训练，第一轮先别改
  length: 0.70
  width: 0.52
  wheelbase: 0.593

ipath:
  min_radius: 1.35
  ind_range: 50
  loop: false

  arrive_threshold: 0.25
  close_threshold: 0.12
  arrive_index_threshold: 3

pan:
  iter_num: 1
  dune_max_num: 120
  nrmp_max_num: 10

adjust:
  q_s: [1.0, 1.0, 0.3]
  p_u: 0.7
  eta: 3.0
  d_max: 1.2
  d_min: 0.20
```

等第一轮能做到“**直线稳定 + 圆弧稳定 + 倒车正确 + 动态障碍能绕**”以后，再把 `iter_num` 从1升到2、`ref_speed` 从0.50逐渐升到0.60/0.70，而不是一开始就追0.8m/s。

---

# 十三、我给这些问题排的实际优先顺序

如果按调试收益排序，我会这样处理：

| 优先级    | 要处理的问题                                     | 为什么                     |
| ------ | ------------------------------------------ | ----------------------- |
| **P0** | `ind_range:0` → 50或修源码                     | 当前实现下可能根本不推进路径索引        |
| **P0** | Rmin `1.05→1.35`，NeuPAN steer `0.52→0.414` | 当前路径曲率可能超出真实前轮能力        |
| **P0** | Hybrid footprint `0.70×0.52→0.80×0.58`     | 当前碰撞模型明显小于Gazebo机器人     |
| **P0** | NeuPAN vmax `2.0→0.75`，ref `0.8→0.5`       | 控制接口物理上就达不到2m/s         |
| **P0** | 修 `min_speed` / 固定 NeuPAN 版本               | 配置与上游API现在不一致           |
| **P0** | cmd_vel_mux增加旧指令超时                         | 否则planner卡顿会被误认为控制/避障问题 |
| **P1** | `reverse/gear/change` penalty降低            | 改善狭窄区域和Reeds-Shepp换向    |
| **P1** | NeuPAN `eta↑, d_min↑, q_s分维度`              | 改善绕障、贴墙和航向僵硬            |
| **P1** | `receding/dune_max_num/iter_num`降低         | 先保证局部规划真正实时             |
| **P2** | 给Hybrid地图增加inflation/clearance cost        | 真正解决全局路径贴墙              |
| **P2** | 重训0.80×0.58 footprint的DUNE                 | 长期统一真实碰撞模型              |
| **P3** | angle bins 72→96等精细优化                      | 前面的物理问题没修前意义不大          |

还有一个集成层面的缺口：当前 `ackermann_bringup/navigation.launch.py` 会启动 Hybrid A*、scan转换、mux等，但没有直接把 Ackermann NeuPAN node 一起拉起；而 `neupan_ros2` 的 launch 目录里目前也没有专门的 `ackermann_robot.launch.py`。
所以长期我建议把它做成一个明确的完整导航 launch，否则很容易出现“改了 `ackermann_robot/planner.yaml`，实际运行的却是另一套 NeuPAN config”这种非常难排查的问题。

**如果只让我选三个最可能直接改善你当前仿真行为的修改，我会先改：`Rmin=1.35m`、Hybrid footprint=`0.80×0.58m`、NeuPAN `ind_range=50 + ref_speed=0.5/max_speed=0.75`。** 然后再动 `eta/q_s/reverse_penalty`。这样调参顺序才不会用算法权重去补偿一个错误的车辆模型。

继续完善这套导航参数

* 生成可直接替换的 YAML
