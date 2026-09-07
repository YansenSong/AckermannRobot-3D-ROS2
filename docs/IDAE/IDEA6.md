七、第五个值得借的核心：把“多点导航”升级成 Task Graph

你当前：

A → B → C

做得很好。

但它只能表达线性队列：

deque[A, B, C]

而实际自然语言任务会越来越像：

“先去 A 接张三，然后去 B。到了 B 等五分钟，如果李四还没来，就先去 C，途中最高速度保持 2m/s，到地方提醒我。”

这已经不能很好地表达成 waypoint queue 了。

AgenticRobot 的 DAG demo 是：

{
  "nodes": [
    {
      "id": "task_1",
      "skill": "navigation",
      "target": "A",
      "depends_on": []
    },
    {
      "id": "task_2",
      "skill": "...",
      "depends_on": ["task_1"]
    }
  ]
}

甚至多机器人时：

Robot 11 navigate ──→ wave ───┐
                              ├──→ final action
Robot 12 navigate ──→ highfive┘

真正的 scheduler 会找出所有依赖已经完成的 ready nodes，然后并行执行；节点失败则停止后续调度。

这个设计其实完全不需要等你做“多车”。

你可以先做：

Single-Vehicle Task DAG
用户：
“先去图书馆，再去食堂，路上慢一点”

             ┌─ set_speed(2m/s)
START ───────┤
             └─ navigate(图书馆)
                       ↓
                navigate(食堂)
                       ↓
                  COMPLETE

进一步：

navigate(A)
    ↓
wait(passenger_boarded)
    ↓
navigate(B)

这时 Chat2Drive 从：

“LLM 可以调用驾驶工具”

开始变成：

“LLM 可以规划并监督一段完整行程。”

这才是我理解的 Chat2Drive 走向 Agentic Driving 最关键的一步。