5. RAI 的 Action 生命周期，比你现在导航模型更值得借鉴

RAI 对 ROS Action 的处理非常标准：

start action
 ↓
action_id
 ↓
feedback
 ↓
result

同时可以：
cancel action

而且 action feedback/result 有单独存储，通过 ID 关联。

Nav2 Toolkit 也是一样：

navigate_to_pose()
     ↓
return immediately

get_feedback()
get_result()
cancel()

这其实正好可以解决我上次提到 Chat2Drive 当前比较大的一个架构问题：

你现在比较接近：

publish goal
 ↓
sleep 1.5s
 ↓
切 autonomous
 ↓
RouteState 到达

以后建议抽象成：

NavigationTask
  id: nav_a832
  status: ACCEPTED

          ↓

PLANNING
          ↓

EXECUTING
          ↓

feedback:
  distance: 183m
  eta: 91s

          ↓

SUCCEEDED

或者：

CANCEL_REQUESTED
↓
CANCELLED

最好让所有 Skill 都共享同一套生命周期：

CREATED
ACCEPTED
RUNNING
SUCCEEDED
FAILED
CANCELLED
TIMEOUT

那么将来 Task DAG 就特别自然：

Task Graph
    ↓
Skill
    ↓
ExecutionHandle
    ↓
feedback/result