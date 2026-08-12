# 2026-08-12 开发日志：CP2 连续通过与动态停车目标

## 本轮目标

将 CP2 从任务终点改为障碍区出口的连续通过点，并把完整任务延伸到停车位。当前先选择固定 2 号停车框，但目标通过独立接口提供，不在规划器和跟踪器中写死坐标。

## 当前路线

规划器生成轴对齐路线：

```text
start -> CP1 -> 可选 lane_entry -> CP2
      -> 可选 parking_transition -> parking_approach
      -> parking_goal -> 最终朝向调整
```

- CP2 仍为 `(0.20, selected_y)`，表示机器人沿安全通道完整离开障碍带。
- CP2 为连续通过点，前后线段必须共线，不在此停车。
- CP2 通过时还要求实际航向误差和角速度不超过配置阈值，否则锁定故障并停车。
- 当前 2 号停车目标由 `parking_target_node` 发布。
- 当前停车中心为 `(0.8015, 0.0)`，驶入朝向为 `0 rad`，最终朝向为 `pi rad`。
- 停车框按标线内沿定义为 `0.297 x 0.210 m`。只有最终朝向正确、机器人停稳且按实际朝向投影后的 `0.210 x 0.178 m` 保守矩形外廓完整位于框内时，任务才进入 `COMPLETE`。

## 接口与安全

```text
parking_target_node -> /parking/candidates
parking_target_node -> /parking/selected_target
axis_aligned_planner_node -> /navigation/plan
axis_aligned_follower_node -> /navigation/desired_cmd_vel
safety_arbiter_node -> /cmd_vel
```

现有安全仲裁与 watchdog 链路保持不变。停车目标缺失、无效、超时或运动中计划变化时仍按保守原则停车。

## 当前验证状态

已完成以下非运动验证：

```text
robot_perception 与 robot_simulation 构建成功
47 tests, 0 errors, 0 failures, 1 skipped
即 46 passed, 1 skipped
obstacle_navigation.launch.py 参数解析成功
obstacle_planning.launch.py 参数解析成功
```

两条警告来自测试依赖的弃用接口，不是项目测试失败。本轮没有启动 ROS 持续节点、Gazebo 或运动验证，因此仍不能声称新路线已经在仿真中完成停车。

用户随后按提供的独立终端命令进行仿真，并反馈“没有问题”。本会话没有采集该次运行的最终里程计、控制状态或 `/cmd_vel` 记录，因此这里只记录用户反馈，不把它表述为带遥测证据的自动化运动验收。

## 实现细节

### 动态停车目标

新增 `parking_target_node`，从 YAML 中读取停车位集合，通过以下话题发布候选目标和当前选择：

```text
/parking/candidates
/parking/selected_target
```

当前配置仅包含 `space_2`，但规划器不再直接写死停车中心。启动参数 `parking_space_id` 用于选择目标，为后续增加多个停车框和上层目标选择器保留接口。

当前停车位字段包括：

```text
id
center_x / center_y
entry_yaw / final_yaw
length / width
approach_distance
```

### 路线与状态机

- 所有平移段仍沿 X 或 Y 轴。
- `parking_transition` 位于 `(0.4015, selected_y)`，确保机器人通过 CP2 后再改变方向。
- `parking_approach` 位于 `(0.4015, 0.0)`。
- `parking_goal` 位于 `(0.8015, 0.0)`。
- CP2 前后必须共线，跟踪器以严格航向和角速度阈值确认连续通过。
- 到达停车中心后先停稳，再连续旋转到最终朝向，进入角度容差后再次停稳。
- `COMPLETE / parking_complete` 被锁存，后续持续输出零期望速度。

### 失败安全

- 停车目标缺失、格式错误、坐标系不匹配或超时：规划无效。
- 运动中计划变化或失效：锁定故障并停车。
- CP2 通过时航向或角速度不稳定：锁定 `cp2_heading_unstable`。
- 停车框不足以容纳车体，或停车目标不支持轴对齐驶入：拒绝规划。
- 最终姿态下车体外廓不在停车框内：锁定 `parking_envelope_outside_space`。
- 最终旋转空间不安全：锁定故障或等待旋转安全确认。
- 安全仲裁和 `/cmd_vel` watchdog 架构保持不变。

## 主要文件

- `src/robot_perception/robot_perception/parking_target_node.py`
- `src/robot_perception/robot_perception/axis_aligned_planner_node.py`
- `src/robot_perception/robot_perception/axis_aligned_follower_node.py`
- `src/robot_perception/config/parking_targets.yaml`
- `src/robot_perception/config/axis_aligned_planner.yaml`
- `src/robot_perception/config/axis_aligned_follower.yaml`
- `src/robot_perception/launch/obstacle_navigation.launch.py`
- `src/robot_perception/launch/obstacle_planning.launch.py`
- `src/robot_perception/test/test_parking_target.py`
- `src/robot_perception/test/test_axis_aligned_planner.py`
- `src/robot_perception/test/test_axis_aligned_follower.py`

## 仍有限制

- 当前配置只定义并选择固定 2 号停车框，尚未接入真实的多停车框检测和选择策略。
- 停车目标只支持四个轴对齐基准方向和正向驶入，不支持任意斜线、曲线或倒车泊车。
- CP2 后路径只依据当前场地静态空闲假设和边界规划，没有实现通用局部障碍绕行。
- 尚未完成多随机种子统计验证。
- 本会话没有保存用户仿真运行的最终位姿、框内余量和状态话题证据。
- 尚未接入带时间同步的 TF，规划器仍使用里程计和手工雷达偏移。
- 尚未完成真机部署。
