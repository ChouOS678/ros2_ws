# Multi-Agent Working State Recording & Replay (ROS2 Simulation Robot)

## 1. 总体设计思路
- 采用 `状态流 + 事件流` 双轨记录。
- 用 ROS2 原生机制做分层承载：
`Topic` 承载周期状态和异步事件，`Action` 承载长任务执行轨迹，`Service` 承载瞬时查询，`Lifecycle/Diagnostics` 承载节点健康。
- 用统一时间线字段把 Agent 内部、Agent 间通信、任务执行、环境变化串到同一条 trace。
- 双写存储：
`ros2 bag` 保存原始证据可回放，`JSONL + SQLite` 保存结构化数据可统计、告警、报表。

## 2. 状态模型设计
- Agent 级字段：
`agent_id, role, state, current_goal, current_subtask, progress, health, last_heartbeat_ts`
- 建议状态枚举：
`idle, planning, waiting, executing, blocked, failed, done`
- 任务级字段：
`task_id, parent_task_id, owner_agent, dependencies[], result, failure_reason`
- 通信级字段：
`sender, receiver, message_type, phase(request|feedback|result), latency_ms, timeout_ms, retry_count`
- 仿真上下文字段：
`sim_time, wall_time, robot_pose{x,y,yaw}, world_state_summary, object_state, collision_event, occlusion_event, replanning_event`
- 全局关联字段：
`trace_id, correlation_id`

## 3. ROS2 通信与记录架构
- `Topic`:
`/agents/status` 周期状态流（10Hz 或按 Agent 频率）
`/agents/events` 事件流（状态迁移、任务事件、冲突、超时）
`/world_model/events` 环境突发事件（鬼探头、摩擦骤降、遮挡）
`/monitor/summary` 聚合监控输出
- `Action`:
`/agents/task_action`（建议）用于长任务。原因：天然包含 `goal/feedback/result`，便于记录执行全过程、重试与取消。
- `Service`:
`/agents/query_state` 与 `/monitor/query_state`。原因：查询是瞬时请求，不需要持续订阅。
- `Lifecycle/Diagnostics`:
`/diagnostics` 输出节点健康、异常计数。Lifecycle 适合控制关键节点状态切换（configure/activate/deactivate）。

## 4. 消息模型设计
- `AgentStatus`（建议 msg）：
`header, agent_id, role, state, current_goal, current_subtask, progress, health, last_heartbeat_ts, task_id, parent_task_id, owner_agent, dependencies, queue_backlog, blocked_reason, trace_id, correlation_id, sim_time, wall_time, robot_pose, world_state_summary`
- `AgentEvent`（建议 msg）：
`header, event_id, event_type, sender, receiver, message_type, phase, latency_ms, timeout_ms, retry_count, task_id, trace_id, correlation_id, result, failure_reason, details_json, sim_time, wall_time`
- `TaskExecutionSnapshot`（建议 msg）：
`task_id, owner_agent, stage, stage_progress, dependency_wait_ms, handoff_latency_ms, action_feedback, status`
- `AgentHealth/Diagnostics`：
建议通过 `diagnostic_msgs/DiagnosticArray` 承载：
`heartbeat_gap, queue_backlog, stuck_duration, error_count`
- `QueryState.srv`（建议）：
Request:
`agent_id(optional), task_id(optional), trace_id(optional)`
Response:
`ok, message, status_json, recent_events_json`

## 5. 状态流与事件流设计
- 状态流记录内容（周期）：
`state, current_task, current_subtask, progress, health, queue_backlog, dependency_waiting, world_state_summary`
- 事件流记录内容（变化触发）：
`task_assigned, task_started, task_finished, task_failed, replanned, timeout, conflict_detected, resource_locked, resource_released, state_transition`
- 职责划分：
状态流解决“当前快照”问题；
事件流解决“为什么变化、何时变化、谁触发”的因果链问题。

## 6. monitor/logger node 设计
- 输入：
`/agents/status, /agents/events, /world_model/events, /marl/reward_breakdown, /odom`
- 输出：
`/monitor/summary, /diagnostics, JSONL, SQLite`
- 内部逻辑：
维护 Agent 最新状态缓存；
检测 heartbeat 丢失；
检测 waiting 超时；
检测 blocked/stuck；
聚合为统一时间线快照；
写入结构化存储。
- 推荐实现：
单独节点 `monitor_logger_node.py`，订阅并聚合，1Hz 发布 summary，服务化查询。

## 7. 存储与回放方案
- 原始记录（rosbag）：
`/agents/status, /agents/events, /tf, /odom, /detections, /planning_result, /world_model/events, /marl/reward_breakdown, action feedback/result`
- 结构化存储（monitor/logger 双写）：
`agent_status.jsonl, agent_event.jsonl, monitor_summary.jsonl, timeline.db(SQLite)`
- 分工：
`rosbag` = 原始证据、时序回放、重现现场；
`JSONL/SQLite` = 指标计算、报表、告警、查询。

## 8. 关键指标体系
- 执行效率：
`task completion time, subtask waiting time, replanning time`
- 协作效率：
`dependency wait time, handoff latency, conflict resolution count`
- 通信健康：
`message latency, dropped messages(proxy), timeout/retry`
- Agent 健康：
`heartbeat gap, queue backlog, action stuck duration`
- 场景影响：
`env-change-triggered failures, perception-jitter-triggered replanning`
- 全局结果：
`success rate, deadlock count, starvation count`
- 计算来源：
状态流给出驻留时长和健康趋势；
事件流给出任务边界、重试、冲突、迁移；
Action feedback/result 给出长任务过程与结果边界。

## 9. 推荐架构图（文本）
```text
Agent Nodes (planner/chassis/perception/nav/manip)
  -> publish /agents/status (Topic)
  -> publish /agents/events (Topic)
  -> execute /agents/task_action (Action)
  -> serve   /agents/query_state (Service)

World Model Mutator
  -> publish /world_model/events

Monitor/Logger Node
  <- subscribe /agents/status
  <- subscribe /agents/events
  <- subscribe /world_model/events
  <- subscribe /odom /tf /action feedback
  -> publish   /monitor/summary
  -> publish   /diagnostics
  -> write JSONL + SQLite

ros2 bag recorder
  <- records raw topics/actions for replay evidence

Dashboard / Offline Analyzer
  <- read /monitor/summary (online)
  <- read JSONL + SQLite (offline analytics)
  <- optional rosbag replay + timeline alignment
```

## 10. 反模式与风险
- 只记录 print log：
不可结构化聚合，无法准确算指标，无法自动告警。
- 只记录 heartbeat 不记录状态机：
只能知道“活着”，不知道“在做什么、卡在哪里”。
- 只能看到 task failed：
没有 state_transition 与依赖链，无法定位根因。
- 无 task_id/trace_id：
跨 Agent 无法串联协作链，复盘断裂。
- 回放只有传感器无决策记录：
只能看到“发生了什么”，看不到“为什么这样决策”。

## 11. 最终落地建议
- 先落 JSON-Topic 版本（当前仓库已实现）快速打通。
- 第二阶段升级到自定义 `msg/srv/action` 接口包，替换 JSON 字符串。
- 将所有关键 Agent 接入统一 `trace_id/correlation_id`。
- 配置固定 rosbag 录制模板 + monitor 双写。
- 每周基于 SQLite 跑指标报表，持续压低 deadlock 与 starvation。
