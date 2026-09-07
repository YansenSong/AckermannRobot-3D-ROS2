五、第三个我非常建议你马上加：Relative Navigation

这个功能对 Chat2Drive 特别搭。

AgenticRobot 支持：

forward,left,degrees

例如：

2.0, 0.5, 30

代表：

向当前车头方向前进 2m
向左偏移 0.5m
最终再旋转 30°

它读取：

map → base_link TF

把局部坐标偏移转换到 map 全局坐标系，再生成 PoseStamped。

这对 Chat2Drive 会产生一类现在做不了、但特别自然的对话：

“再往前开一点。”

“靠左一点。”

“往前十米停一下。”

“往前开到那个路口附近。”

现在你的地点导航只能：

地点名 → locations.json

有了 relative navigation，就是：

自然语言
 ↓
相对运动参数
 ↓
current pose
 ↓
TF transform
 ↓
absolute pose
 ↓
Autoware goal

我会把这个放在 第一批新功能，因为实现成本远低于语义导航，但 Demo 效果特别明显。