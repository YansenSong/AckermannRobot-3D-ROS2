# Ackermann Robot — Gazebo Classic 11 世界环境清单

> 最后更新: 2026-07-11
> 
> **所有世界文件、模型和地图集中存放在本目录**，与 `ackermann_robot` 功能包分离。
> 原 `mini.world` / `empty.world` / `mini_world.yaml` 仍留在 `src/ackermann_robot/` 中不动。

## 快速启动

```bash
# 设置模型路径（本目录下的 models + Gazebo 系统默认路径）
export GAZEBO_MODEL_PATH=/home/young/AckermannRobot/src/gazebo_worlds/models:/usr/share/gazebo-11/models

# 手动测试某个世界:
gazebo /home/young/AckermannRobot/src/gazebo_worlds/worlds/<文件名>
```

---

## 1. ✅ office_world.world — 开放式办公室

| 项目 | 说明 |
|------|------|
| **来源** | ros2-demo (Satone7) |
| **大小** | 12m × 12m 开放式办公区 |
| **障碍物** | 办公桌、大理石桌、会议桌、书柜、柜子、纸箱、隔断墙、铰链门 |
| **动态元素** | 2 个站立人物（person_standing） |
| **照明** | 点光源，室内照明 |
| **地面** | nist_elevated_floor_120 地板砖（3×3 排列） |
| **模型依赖** | 见下方「模型缺失清单」 |
| **配套地图** | 暂无生成好的地图，需 SLAM 或手动生成 |

---

## 2. ✅ mobile_robot_world.world — 户外住宅区

| 项目 | 说明 |
|------|------|
| **来源** | ros2-demo (Satone7) |
| **环境** | 户外+室内混合场景 |
| **障碍物** | 咖啡桌、书柜、柜子、纸箱、路锥 |
| **动态元素** | 1 个站立人物 |
| **装饰** | 橡树、路灯杆、一座房子 |
| **模型依赖** | 见下方「模型缺失清单」 |
| **配套地图** | 暂无 |

---

## 3. ✅ hospital.world — AWS 医院（单层）

| 项目 | 说明 |
|------|------|
| **来源** | Dataset-of-Gazebo-Worlds |
| **环境** | 完整的医院场景，含多房间、走廊、护士站 |
| **模型** | 100 个医院模型 → 已解压到 `src/gazebo_worlds/models/hospital/` |
| **动态元素** | 无（静态场景） |
| **配套地图** | 暂无现成地图文件（世界较大，建议用 gmapping 建图） |
| **状态** | **立即可用** ✅（模型已就位） |

---

## 4. ✅ hospital_two_floors.world — AWS 医院（双层）

| 项目 | 说明 |
|------|------|
| **来源** | Dataset-of-Gazebo-Worlds |
| **环境** | 两层的医院场景，含电梯、坡道 |
| **模型** | 同 hospital.world（100 个模型，已解压） |
| **配套地图** | 暂无 |
| **状态** | **立即可用** ✅（模型已就位） |

---

## 5. ✅ factory.world — AWS 工厂/仓储

| 项目 | 说明 |
|------|------|
| **来源** | Dataset-of-Gazebo-Worlds |
| **环境** | 工厂车间，含货盘、零件箱、传送带、机械臂工作台 |
| **模型** | 已复制到 `models/factory_models/` |
| **注意** | 部分模型 URI 使用 Fuel 在线资源（例如 `arm_part`、`coke_can` 等），需要联网加载或额外下载 |
| **配套地图** | 暂无 |
| **状态** | **基本可用** ✅（本地+在线模型混合） |

---

## 6. ✅ dynamic_room.world — 动态障碍物房间

| 项目 | 说明 |
|------|------|
| **来源** | Dataset-of-Gazebo-Worlds |
| **环境** | 带有动态障碍物的房间 |
| **模型** | 引用 `dynamic_obstacle`、`empty_room` → 已解压到 models |
| **配套地图** | 暂无 |
| **状态** | **立即可用** ✅（模型已就位） |

---

## 7. ✅ experiment_rooms/ — 实验房间（4种）

| 项目 | 说明 |
|------|------|
| **来源** | Dataset-of-Gazebo-Worlds |
| **模型依赖** | **无** — 所有几何体都是内联的（inline SDF），不需要额外模型 |
| **版本** | 每个房间有静态版和动态障碍物版（world_dynamic） |
| **配套地图** | room1.yaml ~ room4.yaml → 已复制到 `maps/`目录 |
| **状态** | **完全立即可用** ✅ |

### 各房间详情

| 房间 | 静态版 | 动态版 | 说明 |
|------|--------|--------|------|
| room1 | `experiment_rooms/room1/world.world` | — | 简单房间 |
| room2 | `experiment_rooms/room2/world.world` | `experiment_rooms/room2/world_dynamic.world` | 房间+动态障碍物 |
| room3 | `experiment_rooms/room3/world.world` | `experiment_rooms/room3/world_dynamic.world` | 房间+动态障碍物 |
| room4 | `experiment_rooms/room4/world.world` | `experiment_rooms/room4/world_dynamic.world` | 房间+动态障碍物 |

---

## 8. ✅ mini.world — 原有小型迷宫

| 项目 | 说明 |
|------|------|
| **来源** | 项目原有文件 |
| **环境** | 小型迷宫（围墙+桌子+方块障碍物） |
| **模型依赖** | **无** — 所有几何体内联，不需额外模型 |
| **配套地图** | mini_world.yaml → 已存在 |
| **状态** | **完全立即可用** ✅ |

---

## 9. ✅ empty.world — 空世界

| 项目 | 说明 |
|------|------|
| **来源** | 项目原有文件 |
| **环境** | 完全空白，只有地面和光照 |
| **用途** | 纯传感器测试、快速启动 |
| **状态** | **立即可用** ✅ |

---

## 10. 📦 配套地图 Nav2 可直接用的地图

| 地图文件 | 对应世界 | 说明 |
|---------|---------|------|
| `maps/mini_world.yaml` | mini.world | 已有 |
| `maps/empty_room.yaml` | dynamic_room.world | 新增 |
| `maps/room1.yaml` | experiment_rooms/room1 | 新增 |
| `maps/room2.yaml` | experiment_rooms/room2 | 新增 |
| `maps/room3.yaml` | experiment_rooms/room3 | 新增 |
| `maps/room4.yaml` | experiment_rooms/room4 | 新增 |
| `maps/room_with_walls_1.yaml` | 单独模型场景 | 新增 |
| `maps/room_with_walls_2.yaml` | 单独模型场景 | 新增 |
| `maps/star_room_with_walls.yaml` | 单独模型场景 | 新增 |
| `maps/turtlebot3_world.yaml` | turtlebot3 世界 | 新增 |

---

## ⚠️ 模型缺失清单（针对 office_world / mobile_robot_world）

以下模型是 ros2-demo 世界的引用的 **Gazebo 模型数据库** 中的标准模型，当前系统未预装。启动时会自动从 Fuel 下载到 `~/.gazebo/models/`，也可手动安装：

```bash
# 从 Gazebo 模型数据库自动下载（首次启动世界时 Gazebo 会自动拉取）
# 或从 GitHub 批量下载:
git clone https://github.com/osrf/gazebo_models /tmp/gazebo_models
mkdir -p ~/.gazebo/models
for m in table bookshelf cabinet cafe_table cardboard_box grey_wall \
         hinged_door person_standing table_marble nist_elevated_floor_120 \
         oak_tree lamp_post house_1 construction_cone; do
  cp -r "/tmp/gazebo_models/$m" ~/.gazebo/models/
done
```

之后将 `GAZEBO_MODEL_PATH` 包含 `~/.gazebo/models`:
```bash
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:/home/young/AckermannRobot/src/gazebo_worlds/models:/usr/share/gazebo-11/models
```

---

## 快速对比总表

| 世界文件 | 模型依赖 | 配套地图 | 启动复杂度 | 推荐用途 |
|---------|---------|---------|-----------|---------|
| `mini.world` | 无 | ✅ mini_world.yaml | ⭐ 即开即用 | 快速导航测试 |
| `empty.world` | 无 | ❌ | ⭐ 即开即用 | 传感器调试 |
| `experiment_rooms/room*/world*.world` | 无 | ✅ room*.yaml | ⭐ 即开即用 | 导航算法对比 |
| `hospital.world` | 100模型已就位 | ❌ | ⭐ 即开即用 | 室内复杂导航 |
| `hospital_two_floors.world` | 同上 | ❌ | ⭐ 即开即用 | 多层导航 |
| `factory.world` | 已就位+在线混合 | ❌ | ⭐⭐ | 仓储物流场景 |
| `dynamic_room.world` | 已就位 | ✅ empty_room.yaml | ⭐ 即开即用 | 动态避障 |
| `office_world.world` | 需补充7个模型 | ❌ | ⭐⭐⭐ | 办公室导航 |
| `mobile_robot_world.world` | 需补充7个模型 | ❌ | ⭐⭐⭐ | 户外住宅导航 |
