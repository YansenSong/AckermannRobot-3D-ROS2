# 转向 EPS 字段刻度排查记录

> 状态:已定位并修复(待后续与 STM32 侧复核)
> 日期:2026-08-05
> 现象:发布 `/cmd_vel` 的 `angular.z` 后,前轮只"晃了一下",达不到预期转角。

---

## 1. 现象

运行运动控制桥后,向 STM32 下发转向指令:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.3}}"
```

- `angular.z = 0.3 rad`(即前轮转角 17.19°)没有转出预期角度;
- 前轮仅轻微抖动一下,随即回中,看不到任何保持的转角;
- 同时段下发速度 `linear.x = 0.2` 车轮正常转动 —— 说明 UDP 链路、校验和、使能位均正常,
  问题只出在"转向字段的解析"。

## 2. 排查结论

**根因是 EPS 字段的刻度少了 10 倍**,而不是 rad 转角度的换算逻辑。

当前 repo(修复前)发送 `EPS_RAW = 转角(°) × 10000`;
实际 STM32 固件按 **`转角(°) / 0.1° × 10000 = 转角(°) × 100000`** 解释。

因此 `angular.z=0.3`(17.19°):

| 实现 | 计算 | 发送字段 | STM32 读到 |
|---|---|---|---|
| 参考版 `uart_vehicle_bridge`(正常) | `17.19 / 0.1 × 10000` | `1,720,000`(`00 1A 3E C0`) | **17.2°** |
| 当前 repo 修复前(故障) | `17.19 × 10000` | `171,887`(`00 02 9F 6F`) | **~1.7°** |

1.7° 的微小偏转在实车上就表现为"晃了一下、没有角度"。

### 2.1 参考实现对比

`/home/t/Sensors/src/uart_vehicle_bridge` 中已验证可用的换算链路:

```python
EPS_DEG_PER_RAW = 0.1     # degrees per EPS raw unit
eps_raw = int(round(delta_deg / 0.1))        # 30° → 300 (0.1° 单位)
eps_raw_scaled = int(round(eps_raw * 10000)) # → 3,000,000 上线
```

即线缆字段 = `角度(°)/0.1° × 10000`。

### 2.2 与协议文档的矛盾(关键遗留问题)

`docs/普通车运控_以太网UDP通信协议说明_V1.0.md` 第 59 / 71 行及第 6 节示例报文写的是
**"eps_raw = 目标转角(°) × 10000"**,30° 示例为 `300000`(`00 04 93 E0`)。

- 若按文档实现(`× 10000`),实车转向失灵(本次事故);
- 参考版按 `× 100000` 实现,实车转向正常。

**结论:协议文档对 EPS 字段的描述与 STM32 实际固件不符。** 需要与 STM32 侧/供应商核对:
是固件按 0.1°/count 的 raw 缩放(文档 EPS 注意事项里提到过该可能性),还是文档笔误。
在双方确认前,请以"参考版 `× 100000`"为准。

## 3. 修复内容

改动文件:

- `src/motion_control/motion_control/bridge_node.py`
  - 恢复两段式刻度:`EPS_DEG_PER_RAW = 0.1`,`_compute_eps_raw` 返回 `deg / 0.1`,
    `_build_frame` 再 `× EPS_SCALE(10000)` → 字段 = `角度 × 100000`;
  - **保留**"`angular.z` 即前轮转角(rad)"的契约,不做 Ackermann 换算,
    因此 `v=0` 的纯转向指令也能出角度(与 NeuPAN 输出的"前轮转角"一致)。
- `src/motion_control/test/test_bridge_node.py`
  - 更新为参考刻度:`30° → 300(0.1° 单位)`,`build_frame` 后字段 = `3,000,000`;
  - 期望报文由 `00 04 93 E0` 改为 `00 2D C6 C0`,4 个单测全部通过。

重新编译后生效:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motion_control
```

> 注意:修复前 `install/` 里是 `0d4b85e` 构建的旧拷贝(普通文件,非符号链接),
> 改 `src/` 后运行中的节点仍加载旧逻辑;用 `--symlink-install` 重建后 install 符号链接到
> `src/`,重启即生效。

## 4. 后续待排查清单

- [ ] 与 STM32 侧核对 EPS 字段的真实刻度(`×10000` 还是 `×100000`),修正
      `docs/普通车运控_以太网UDP通信协议说明_V1.0.md` 第 59 / 71 行及第 6 节示例报文。
- [ ] 确认 30° 单侧最大转角在 STM32 侧是否有独立 clamp/回中逻辑(排除大角度保护)。
- [ ] 核对转向正负方向与实车标定是否一致(文档:正负由实车标定确定)。
- [ ] 若后续固件更新为文档口径(`× 10000`),需同步更新本桥的 `EPS_DEG_PER_RAW`/刻度。

## 5. 相关文件

- 协议文档:`docs/普通车运控_以太网UDP通信协议说明_V1.0.md`
- 运动控制桥:`src/motion_control/motion_control/bridge_node.py`
- 单测:`src/motion_control/test/test_bridge_node.py`
- 参考实现(可用):`/home/t/Sensors/src/uart_vehicle_bridge/uart_vehicle_bridge/bridge_node.py`
- 引入回归的提交:`0d4b85e fix: correct real vehicle steering control`(把刻度从参考版改成 `× 10000`)
