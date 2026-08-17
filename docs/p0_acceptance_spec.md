# P0 端到端验收规范

状态：冻结基线 v1
日期：2026-08-17

## 1. 目标

本规范定义当前随机三障碍通行与固定 `space_2` 停车任务的单次端到端验收契约。目标是把已有的 Gazebo 人工运行结果升级为可重复、可测量、可回放的证据链，而不是扩展导航能力。

被验收链路为：

```text
/scan -> lidar_safety_node -> /safety/status

/scan + /odom + /parking/selected_target
  -> axis_aligned_planner_node -> /navigation/plan

/navigation/plan + /odom + /safety/status
  -> axis_aligned_follower_node
  -> /navigation/desired_cmd_vel

/navigation/desired_cmd_vel + /safety/status
  -> safety_arbiter_node -> /cmd_vel

/navigation/safety_arbiter_status
  -> cmd_vel_watchdog_node
```

P0 的基准场景为 `layout_seed=42`、固定目标 `space_2`、`stop_at_cp1=false`。

## 2. 非目标

P0 不实现以下能力：

- 多随机种子成功率统计。
- 动态停车位检测或运行期目标切换。
- 任意角度、曲线或倒车泊车。
- TF2、SLAM、Navigation2 或真机部署。
- VLM 与当前停车主链路的整合。
- 将现有 String/JSON 状态接口重构为自定义 ROS 消息。

## 3. 安全不变量

- 现有生产 launch 的 `enable_motion` 默认值必须保持 `false`。
- 只有显式请求运动的验收命令可以传入 `enable_motion=true`。
- 验收监视器不得发布 `/cmd_vel` 或 `/navigation/desired_cmd_vel`。
- 验收监视器不得修改生产节点参数、调用运动服务或注入安全状态。
- 正常状态下 `/cmd_vel` 发布者必须只有 `safety_arbiter_node`。
- watchdog 接管表示验收失败，不得将其零速度发布者计作正常第二发布者。
- 验收器失效只能产生 `ERROR`，不能影响安全主链路或产生 `PASS`。

## 4. 权威数据与时间

### 4.1 数据来源

| 数据 | 话题 | 用途 |
|---|---|---|
| 雷达 | `/scan` | 证明规划和安全输入存在且新鲜 |
| 控制位姿 | `/odom` | 复核 CP2、控制误差和停稳状态 |
| 仿真真值 | `/gazebo/model_states` | 独立复核实际停车位置、朝向和漂移 |
| 停车目标 | `/parking/selected_target` | 目标 ID、坐标、朝向和停车框尺寸 |
| 规划 | `/navigation/plan` | 路线、CP2、停车目标和计划稳定性 |
| 跟踪器状态 | `/navigation/control_status` | 任务状态和故障原因 |
| 期望速度 | `/navigation/desired_cmd_vel` | 跟踪器输出和完成后零速 |
| 安全状态 | `/safety/status` | 雷达有效性、急停和旋转安全 |
| 仲裁状态 | `/navigation/safety_arbiter_status` | armed、锁存、最终命令和发布者数量 |
| watchdog 状态 | `/navigation/cmd_vel_watchdog_status` | 心跳和应急接管 |
| 最终速度 | `/cmd_vel` | 实际发送给仿真的控制命令 |
| 碰撞状态 | `/acceptance/collision_status` | 五个 contact sensor 的存活状态和累计碰撞数 |
| 碰撞事件 | `/acceptance/collision_events` | 机器人与瓶子、墙体等非预期接触；零碰撞时允许无消息 |

当前 String/JSON 状态没有消息时间戳。验收器必须为每条消息记录本地 monotonic 接收时间，并保留 ROS 时间用于与 rosbag 对齐。所有数值必须为有限数，缺字段、错误类型、NaN、Inf 和未知状态均属于 `TELEMETRY_INVALID`。

### 4.2 坐标权威性

- 当前目标和规划使用 `odom` frame。
- `/odom` 用于复核控制器看到的误差。
- Gazebo world pose 用于复核机器人实际外廓是否位于场地停车框内。
- 验收实现必须在运行前验证 `odom` 与 Gazebo world 的初始对齐关系，并把变换写入结果。
- 未建立或无法验证该关系时，结果必须是 `ERROR / FRAME_ALIGNMENT_INVALID`，不能只用 odom 产生 PASS。

### 4.3 时间边界

| 名称 | v1 值 | 定义 |
|---|---:|---|
| readiness timeout | 30.0 s | 进程启动后等待必要话题和有效状态的最长 wall time |
| motion start timeout | 10.0 s | READY 后等待首个非零 `/cmd_vel` 的最长 wall time |
| controller mission timeout | 120.0 s | 与 follower 当前配置一致 |
| monitor grace | 5.0 s | 允许监视器观察 follower 超时终态的额外时间，不放宽任务成功期限 |
| post-complete hold | 1.0 s | `parking_complete` 后持续观察零速、漂移和碰撞的时间 |

生产节点仍使用自身现有超时逻辑。验收器的额外时间只用于观察和分类结果，不能覆盖生产故障。

## 5. 验收状态机

```text
PREPARING
  -> READY
  -> RUNNING
  -> TERMINAL_CANDIDATE
  -> POST_COMPLETE_HOLD
  -> FINALIZING
  -> PASS | FAIL | ERROR
```

### 5.1 PREPARING

以下条件必须在 readiness timeout 内同时满足：

- `/scan`、`/odom` 和所有状态话题已出现并保持新鲜。
- 停车目标 `valid=true`、`frame_id=odom`、目标 ID 为 `space_2`。
- `/navigation/plan` 有效并包含恰好一个 `cp2` 和一个最终 `parking_goal`。
- `cp2.stop_required=false`，其他航点 `stop_required=true`。
- `parking_goal` 的 ID、中心、尺寸和最终朝向与目标一致。
- `/safety/status` 满足 `valid is true` 且 `emergency_stop is false`。
- follower 和 arbiter 均已启用。
- arbiter 未锁存，watchdog 未接管。
- `/cmd_vel` 正常发布者数量为 1。
- odom 与 Gazebo world 的初始对齐已验证。

启动期间允许短暂 `WAITING` 和 `waiting_for_*`，超过期限则为 `READINESS_TIMEOUT`。

### 5.2 READY 与 RUNNING

- READY 后首个超过零速容差的 `/cmd_vel` 标记任务开始。
- 运动开始后，计划的规范化内容必须保持不变。
- 运动期间计划无效、跟踪器故障、安全失效、仲裁器锁存、watchdog 接管或进程退出立即产生失败候选。
- 记录路线、所选通道、计划最小净空、状态转换和所有安全事件。

### 5.3 CP2 独立复核

由于 follower 在同一控制周期递归进入 CP2 后的线段，不能依赖 `target_role=cp2` 作为唯一证据。验收器必须根据冻结计划和 odom 独立确认一次有向 CP2 穿越。

穿越采样必须同时满足：

- 沿线剩余距离绝对值不大于 `0.015 m`。
- 横向误差不大于 `0.030 m`。
- 航向误差不大于 `0.020 rad`。
- 实际角速度绝对值不大于 `0.020 rad/s`。
- 穿越方向与 CP2 前后共线线段方向一致。

没有满足条件的 CP2 证据时，不得 PASS。

### 5.4 终态候选

唯一正确的任务终态为：

```text
/navigation/control_status
state  == "COMPLETE"
reason == "parking_complete"
```

`COMPLETE / cp1_reached`、其他 COMPLETE reason、任意 `FAULT`、arbiter `LATCHED` 或 watchdog emergency 均为失败。

### 5.5 完成后保持

收到正确终态后继续观察至少 `1.0 s`。整个窗口内必须满足：

- `|odom linear speed| <= 0.005 m/s`。
- `|odom angular speed| <= 0.020 rad/s`。
- `/navigation/desired_cmd_vel` 的线速度和角速度绝对值不大于 `0.0001`。
- `/cmd_vel` 的线速度和角速度绝对值不大于 `0.0001`。
- follower 持续为 `COMPLETE / parking_complete`。
- arbiter 不锁存，watchdog 不接管。
- 没有新碰撞。
- Gazebo 真值中心漂移不大于 `0.005 m`，yaw 漂移不大于 `0.010 rad`。

## 6. 停车几何

### 6.1 固定基准

| 参数 | v1 值 |
|---|---:|
| 停车目标中心 | `(0.8015, 0.0)` |
| 停车框内沿尺寸 | `0.297 x 0.210 m` |
| 最终朝向 | `pi rad` |
| 机器人保守外廓 | `0.210 x 0.178 m` |
| 中心位置误差上限 | `0.015 m` |
| 最终 yaw 误差上限 | `0.050 rad` |

目标消息、计划和场地真值必须与该基准一致；不一致属于配置或契约错误，而不是新的测试场景。

### 6.2 四边余量

令停车框中心和朝向为 `(gx, gy, psi_p)`，机器人中心和朝向为 `(rx, ry, psi_r)`。先把机器人中心转换到停车框局部坐标：

```text
dx = rx - gx
dy = ry - gy
cx =  cos(psi_p) * dx + sin(psi_p) * dy
cy = -sin(psi_p) * dx + cos(psi_p) * dy
delta = normalize(psi_r - psi_p)
```

将机器人局部四角 `(+-Lr/2, +-Wr/2)` 按 `delta` 旋转并平移到 `(cx, cy)`，得到 `xmin`、`xmax`、`ymin`、`ymax`。四边余量定义为：

```text
rear  = xmin + Lp / 2
front = Lp / 2 - xmax
right = ymin + Wp / 2
left  = Wp / 2 - ymax
```

odom 和 Gazebo 真值必须分别计算四边余量。每一边都必须不小于 `-1e-9 m`；`1e-9` 只处理浮点误差，不是物理容差。完成后保持窗口中的每边最小余量写入结果。

完全居中且方向一致时，理论纵向单边余量为 `0.0435 m`，横向单边余量为 `0.0160 m`。

## 7. 碰撞判定

- 碰撞证据必须来自 Gazebo contact 数据，不得通过轨迹外观推断。
- 正常轮子与地面接触必须过滤。
- 机器人与瓶子、场地墙体或其他非预期实体的接触均为失败。
- 当前停车标线只有 visual、没有 collision；是否越过标线由停车框内沿四边余量判定，不改变场地物理模型。
- 记录首次碰撞时间、双方 collision 名称、接触点和总事件数。
- 五个原始 contact sensor 必须持续提供 heartbeat，`/acceptance/collision_status` 必须报告全部传感器新鲜。
- contact 数据缺失、插件未加载、任一传感器不新鲜或过滤规则无法确认时，结果为 `ERROR / COLLISION_EVIDENCE_MISSING`。

## 8. PASS 条件

一次运行只有同时满足以下条件才能 PASS：

1. 使用 seed 42、`space_2`、`stop_at_cp1=false` 和显式 `enable_motion=true`。
2. readiness 全部满足且消息契约无错误。
3. 计划在运动期间有效且不变。
4. CP2 独立穿越复核通过。
5. follower 到达 `COMPLETE / parking_complete`，且未出现 FAULT。
6. arbiter 从未锁存，watchdog 从未接管。
7. 非预期碰撞数为 0。
8. odom 与 Gazebo 真值的最终中心误差均不大于 `0.015 m`。
9. odom 与 Gazebo 真值的最终 yaw 误差均不大于 `0.050 rad`。
10. odom 与 Gazebo 真值计算的四边余量均不小于 `-1e-9 m`。
11. 完成后保持窗口的零速、漂移和状态条件全部满足。
12. rosbag 正常关闭，所有必录话题至少包含一条有效消息。
13. 结构化结果通过 `docs/schemas/p0_result.schema.json` 校验。

控制任务成功但证据缺失时必须判 ERROR，不能判 PASS。

## 9. 结果分类和退出码

`PASS` 表示任务和证据均合格；`FAIL` 表示被测任务违反验收标准；`ERROR` 表示环境、验收器或证据链无法给出可靠任务结论。

| 退出码 | 代码 | 分类 |
|---:|---|---|
| 0 | `PASS` | PASS |
| 10 | `CONFIG_INVALID` | ERROR |
| 11 | `PREFLIGHT_FAILED` | ERROR |
| 12 | `LAUNCH_FAILED` | ERROR |
| 13 | `PROCESS_EXITED` | ERROR |
| 14 | `READINESS_TIMEOUT` | ERROR |
| 15 | `TELEMETRY_INVALID` | ERROR |
| 16 | `EVIDENCE_INCOMPLETE` | ERROR |
| 17 | `REPORT_FAILED` | ERROR |
| 18 | `UNEXPECTED_SHUTDOWN` | ERROR |
| 19 | `FRAME_ALIGNMENT_INVALID` | ERROR |
| 20 | `MISSION_TIMEOUT` | FAIL |
| 21 | `CONTROLLER_FAULT` | FAIL |
| 22 | `SAFETY_CHAIN_FAULT` | FAIL |
| 23 | `COLLISION_DETECTED` | FAIL |
| 24 | `WRONG_TERMINAL_STATE` | FAIL |
| 25 | `FINAL_POSITION_FAILED` | FAIL |
| 26 | `FINAL_YAW_FAILED` | FAIL |
| 27 | `PARKING_ENVELOPE_FAILED` | FAIL |
| 28 | `NOT_STATIONARY` | FAIL |
| 29 | `POST_COMPLETE_MOTION` | FAIL |
| 30 | `CP2_VALIDATION_FAILED` | FAIL |
| 31 | `PLAN_CHANGED` | FAIL |
| 32 | `COLLISION_EVIDENCE_MISSING` | ERROR |

结果必须保存全部 failure。`primary_code` 的优先级为：碰撞、安全链路、计划变化、控制器故障、CP2、超时、停车几何、停稳、环境与报告错误。同一根因产生的次生安全锁存不得覆盖首次根因。

## 10. 必录证据与产物

每次运行目录必须包含：

```text
artifacts/<run_id>/
├── manifest.json
├── result.json
├── events.jsonl
├── rosbag/
├── pose.csv
├── cmd_vel.csv
├── safety.csv
├── parking_margins.csv
├── junit.xml
├── report.md
└── logs/
```

rosbag v1 必录话题：

```text
/clock
/scan
/odom
/parking/candidates
/parking/selected_target
/navigation/plan
/navigation/control_status
/navigation/desired_cmd_vel
/navigation/safety_arbiter_status
/navigation/cmd_vel_watchdog_status
/safety/status
/cmd_vel
/gazebo/model_states
/gazebo/contacts/base
/gazebo/contacts/lidar
/gazebo/contacts/wheel_left
/gazebo/contacts/wheel_right
/gazebo/contacts/caster
/acceptance/collision_status
/acceptance/collision_events
/tf
/tf_static
```

除 `/acceptance/collision_events` 外，必录话题必须至少包含一条有效消息。零碰撞运行允许 collision events 为零条；五个原始 contact heartbeat、collision status 中的传感器新鲜度和 `collision_count=0` 共同构成零碰撞证据。

视频不是机器验收依据。seed 42 首次自动 PASS 后应单独录制一段展示视频或 GIF，但不要求每次 CI 录制。

## 11. 结果格式

`result.json` 必须通过 `docs/schemas/p0_result.schema.json` 校验。Schema 只验证结构和类型；跨字段规则，例如 `PASS` 必须零碰撞、bag 完整且全部余量合格，由判定核心验证。

结果结构在 PASS、FAIL 和 ERROR 下保持一致。某项指标因启动失败或证据缺失而无法观测时必须写为 `null`，不得用 `0` 伪装为有效测量；对应原因必须同时出现在 `failures` 中。

`docs/examples/p0_result.example.json` 仅演示格式，不代表已经完成真实验收，不得作为项目运行证据。

## 12. 实施顺序

1. 实现纯 Python 消息契约、几何和 verdict 核心及单元测试。
2. 实现不发布控制消息的 ROS monitor，并用 synthetic publishers 做 launch 测试。
3. 实现 runner、rosbag 生命周期和结果落盘，先在 `enable_motion=false` 下验证证据链。
4. 增加并验证 Gazebo contact 观测和 world/odom 对齐。
5. 经明确确认后，以 `enable_motion=true` 执行 seed 42 自动验收。
6. 真实 PASS 后更新根 README、GitHub About 和展示材料。

任何后续阈值变化都必须升级规范或在结果中记录新的契约版本，不能静默修改历史标准。
