另一个问题是导航里现在还有一些基于固定时间的流程控制：

等待路径规划 1.5 秒
↓
切 AUTO

以及到途经点：

等待 3 秒
↓
前往下一站

Prototype 没问题，但如果开始 Agentic 化，我建议换成：

publish goal
 ↓
WAIT route_ready
 ↓
engage autonomous
 ↓
WAIT route_arrived
 ↓
WAIT dwell_condition
 ↓
next step

也就是：

Sleep-driven → Event-driven

这刚好对应 AgenticRobot workflow 里强调的：

prerequisites
monitoring signals
success criteria
abort conditions