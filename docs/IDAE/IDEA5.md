六、第四个值得做的大升级：从 Location Repository 升级成 Semantic World Model

现在 Chat2Drive 的世界认知大致是：

{
   "办公楼": Pose,
   "大门": Pose,
   "实验室": Pose
}

也就是：

String → Coordinate

这很好用，但 Agent 实际上对环境几乎“一无所知”。

AgenticRobot 的 semantic navigation 则是：

floor
room
object
 ↓
FsrVlnClient.query()
 ↓
semantic map / room / object
 ↓
target object
 ↓
center_map
 ↓
Pose

这样用户能说：

“带我去二楼茶水间。”

“去实验室那台打印机附近。”

“带我去有沙发的休息区域。”

这时 Location 不再只是：

dict[str, Pose]

而应该慢慢演变成：

WorldModel
├── named_places
├── rooms
├── roads
├── parking_spots
├── landmarks
├── detected_objects
├── vehicle_pose
└── dynamic_objects

特别是 Chat2Drive 本身是“对话驾驶”，这个升级的意义比在普通机器人上还大：

LLM 真正有了一个可以推理的“驾驶世界”。