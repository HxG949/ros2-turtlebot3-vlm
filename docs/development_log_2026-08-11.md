# 2026-08-11 开发日志：轴对齐随机障碍通行与 CP2

## 本轮目标

在 ROS2 Humble 和 Gazebo Classic 中实现三瓶随机障碍区域的低速安全通行，并把障碍区对面的检查点定义为 CP2。本阶段任务在 CP2 结束，不规划或执行 CP2 到最终停车位的路线。

## 最终路线

规划器只生成以下轴对齐航点：

```text
start -> CP1 -> lane_entry（需要横移时）-> CP2
```

当前固定几何：

```text
start = (-0.9015, -0.845)
CP1   = (-0.3900, -0.845)
CP2.x =  0.2000
障碍带 X = [-0.20, 0.00]
```

CP2 的 Y 坐标由安全通道决定。跟踪器要求最后一个航点角色必须是 `cp2`，到达后进入 `COMPLETE` 并持续发布零期望速度。

## 通道选择逻辑

旧策略选择最小净空最大的通道，可能为了进入宽间隙中心而横移过多。新策略为：

1. 当前通道满足 `0.070 m` 安全余量时保持当前通道。
2. 当前通道不安全时，只沿 `+Y` 方向寻找最近的安全候选通道。
3. 新通道首次选择额外要求 `0.050 m` 感知储备，即观测净空至少为 `0.120 m`。
4. 通道提交后，只要仍满足原有 `0.070 m` 安全余量就保持，不追逐更大净空。
5. 候选通道间隔为 `0.020 m`。

该策略保留安全余量，同时避免继续横移到整个间隙中心。

## 感知稳定性修复

- 规划启动前要求同一路径连续稳定 `0.6 s`，避免第一帧不完整扫描触发运动。
- 量程下限 `range_min + 0.015 m` 内的饱和回波不进入规划点云。
- 机器人包络半径内的近场自反射点不进入规划点云。
- 上述过滤只影响规划点云；独立雷达安全节点仍接收原始近场射线。
- 旋转安全要求至少 3 条连续危险或未知射线，并用 5 帧确认处理瞬态噪声。

## 控制与安全链路

```text
/scan -> lidar_safety_node -> /safety/status
/scan + /odom -> axis_aligned_planner_node -> /navigation/plan
/navigation/plan + /safety/status + /odom
  -> axis_aligned_follower_node
  -> /navigation/desired_cmd_vel
  -> safety_arbiter_node
  -> /cmd_vel
```

`safety_arbiter_node` 是正常运行时唯一的 `/cmd_vel` 发布者。`cmd_vel_watchdog_node` 监控仲裁器心跳，在仲裁器失联时接管并发布零速度。

安全消息只有同时满足以下条件才允许运动：

```text
valid is True
emergency_stop is False
```

字段缺失、类型错误、消息无效或超时均按保守原则停车。

## 关键参数

```text
最大线速度：0.06 m/s
最大角速度：0.30 rad/s
减速距离：0.20 m
位置容差：0.015 m
横向误差上限：0.06 m
任务超时：60 s
规划安全余量：0.070 m
新通道感知储备：0.050 m
前方急停距离：0.175 m
旋转安全距离：0.180 m
```

## Seed 42 最终验证

瓶子布局：

```text
bottle_1=(-0.082, -0.964)
bottle_2=(-0.129, -0.562)
bottle_3=(-0.069,  0.359)
```

最终通道与结果：

```text
selected_y = -0.315 m
横移量 = 0.530 m
旧最大净空策略横移量 = 0.730 m
减少横移 = 0.200 m

控制状态 = COMPLETE / mission_complete
最终位置 = (0.1913, -0.3164)
CP2目标 = (0.2000, -0.3150)
位置误差约 = 0.0089 m
最终 /cmd_vel = 0
```

Gazebo 和 RViz 中完成了人工视觉核验：机器人无碰撞穿过障碍带，在 CP2 附近停车，CP2 后没有继续生成或执行路径。

## 自动化验证

最终提交前验证命令：

```bash
colcon build --packages-select robot_perception robot_simulation --symlink-install
source install/setup.bash
colcon test --packages-select robot_perception --event-handlers console_direct+
colcon test-result --verbose
```

最终结果：两个包构建成功；`42 tests, 0 errors, 0 failures, 1 skipped`，即 `41 passed, 1 skipped`。两条警告来自测试依赖的弃用接口，不是项目测试失败。

## 仍有限制

- 最终逻辑目前只完成 seed 42 的完整运动和视觉验证，不能声称多随机种子成功率。
- 较高通道可能因路线较长触发 `60 s` 任务超时；超时会安全停车。
- 尚未实现 CP2 到最终停车位的规划和控制。
- 尚未完成最终停车框位置与朝向验证。
- 未使用 SLAM、Navigation2 或端到端 VLA。
- 尚未完成真机部署。

## 主要文件

- `src/robot_perception/robot_perception/axis_aligned_planner_node.py`
- `src/robot_perception/robot_perception/axis_aligned_follower_node.py`
- `src/robot_perception/robot_perception/lidar_safety_node.py`
- `src/robot_perception/robot_perception/safety_arbiter_node.py`
- `src/robot_perception/robot_perception/cmd_vel_watchdog_node.py`
- `src/robot_perception/launch/obstacle_navigation.launch.py`
- `src/robot_perception/config/axis_aligned_planner.yaml`
- `src/robot_perception/config/axis_aligned_follower.yaml`
- `src/robot_simulation/launch/bottle_world.launch.py`
