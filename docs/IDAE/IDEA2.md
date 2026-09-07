三、AgenticRobot 最值得你借的第一个设计：Skill，而不是更多 @tool

现在 Chat2Drive 的思路基本是：

tools = [
    set_navigation_goal,
    emergency_stop,
    set_speed_limit,
    cancel_current_goal,
    resume_navigation,
]

也就是说 Agent 的能力边界基本等于一个 Python function list。

AgenticRobot 则开始把一个能力描述成：

Skill
├── 什么时候应该使用
├── 输入是什么
├── 首选接口是什么
├── Workflow
├── Safety Rules
└── Example

比如语义导航 Skill 明确规定：

User:
“去二楼实验室找打印机”

Skill:
semantic-navigation

结构化：
floor = 二楼
room = 实验室
object = 打印机

执行：
POST /api/semantic_nav

然后：
monitor result
→ success / failed / ambiguous

而不是仅仅：

@tool
def go_somewhere(...)

这个设计特别适合 Chat2Drive。

我会把你现在五个工具重构成：

skills/
  navigation/
    named_navigation/
    relative_navigation/
    semantic_navigation/

  vehicle_control/
    speed_control/
    emergency_stop/
    autonomous_mode/

  trip/
    multipoint_trip/
    cancel_trip/
    resume_trip/

  vehicle_state/
    query_speed/
    query_eta/
    query_navigation_state/

于是以后加能力不再是在 tools.py 里面继续堆：

@tool
@tool
@tool
@tool
@tool
...

而是拥有真正意义上的 Capability Registry / Skill Registry。