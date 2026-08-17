# 项目状态

更新时间：2026-08-17

实现基线：`f289f48 Add reproducible P0 parking acceptance`

当前主线：seed 42 随机障碍通行与固定 `space_2` 停车的 P0 自动验收

## 项目定位

本项目基于 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 和 TurtleBot3 Burger，研究半结构化小型场地中的分段导航、雷达障碍通行、安全仲裁和精确停车。

当前不能表述为多随机种子稳定性已完成、真机部署、Nav2、SLAM、通用多车位泊车或完整 VLA 系统。

## 已实现能力

- 使用 `layout_seed=42` 复现三个瓶子的障碍场景。
- 从固定起点依次经过 CP1、动态安全通道、CP2 和停车路径。
- CP2 作为连续通过点，在线验证位置、横向误差、航向和角速度。
- 使用固定目标 `space_2` 完成最终位置与朝向闭环停车。
- follower 只发布 `/navigation/desired_cmd_vel`。
- safety arbiter 是正常状态下唯一的 `/cmd_vel` 发布者。
- watchdog 在 arbiter 失联时执行零速度应急接管。
- 五个 Gazebo contact sensor 覆盖底盘、雷达、左右轮和脚轮。
- C++ collision observer 过滤正常地面接触并统计非预期碰撞。
- 只读 acceptance monitor 在线检查任务、安全、碰撞、位姿和停车余量。
- runner 自动管理仿真、规划、控制、rosbag、关闭顺序和结果落盘。
- 生成 `result.json`、Markdown、JUnit、CSV、日志和 rosbag 证据。

## 当前生产链路

```text
/scan -> lidar_safety_node -> /safety/status

/scan + /odom + /parking/selected_target
  -> axis_aligned_planner_node -> /navigation/plan

/navigation/plan + /odom + /safety/status
  + /navigation/safety_arbiter_status
  -> axis_aligned_follower_node
  -> /navigation/desired_cmd_vel

/navigation/desired_cmd_vel + /safety/status
  -> safety_arbiter_node -> /cmd_vel

/navigation/safety_arbiter_status
  -> cmd_vel_watchdog_node

Gazebo contact topics
  -> robot_collision_observer
  -> /acceptance/collision_status
```

## 安全边界

- 所有运动默认禁用，必须显式传入 `--enable-motion`。
- acceptance monitor 不发布速度、参数或安全状态。
- arbiter 启动前要求零命令资格和连续 0.5 秒新鲜输入。
- follower 必须观察到 arbiter `ACTIVE` 才能开始任务。
- 验收模式要求 arbiter、monitor 和 rosbag 订阅稳定后才能运动。
- watchdog 启动前要求连续 0.5 秒 arbiter 心跳。
- 运行期命令和心跳超时保持严格，不因启动确认而放宽。
- 数据无效、消息超时、安全异常、计划变化或碰撞均保守停止。

## P0 验收状态

正式 GUI 运行：`20260817T120500.185272Z`

| 指标 | 结果 |
|---|---:|
| Verdict | `PASS` |
| Exit code | `0` |
| 碰撞数 | `0` |
| CP2 | 通过 |
| 最终位置误差 | `0.00893 m` |
| 最终 yaw 误差 | `0.02154 rad` |
| 最小停车余量 | `0.00993 m` |
| 停车后静止保持 | 通过 |
| odom/world 对齐 | 通过 |

运行产物保存在执行设备的 `artifacts/`，该目录不上传 Git。表中结论来自当次正式 `result.json`，不是多次运行成功率。

## 自动测试

最近验证结果：

```text
175 tests, 0 errors, 0 failures, 2 skipped
```

两个 skipped 项为版权检查，测试依赖弃用警告不属于项目失败。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

GUI 正式验收：

```bash
ros2 run robot_acceptance robot_acceptance_run \
  --enable-motion --seed 42 --gui
```

Headless 正式验收：

```bash
ros2 run robot_acceptance robot_acceptance_run \
  --enable-motion --seed 42
```

单次运行通常约需 2 分钟。runner 默认只在结束时向终端打印产物目录；运行期间终端无新增输出不代表卡死。

## 结果解释

- `PASS / 0`：任务和完整在线证据满足 P0。
- `READINESS_TIMEOUT / 14`：运动前证据或安全条件未准备完成。
- `EVIDENCE_INCOMPLETE / 16`：证据不完整，或运动关闭的 dry-run，不能声明 PASS。
- `PLAN_CHANGED / 31`：运动后执行路线发生变化，系统按安全策略失败。

## 尚未完成

- 尚未定义多随机种子集合、重复次数和成功率统计方法。
- 尚未完成系统化异常注入测试。
- 当前目标源只配置 `space_2`，不是通用停车位检测和选择。
- 路线仍限制为轴对齐平移、原地旋转和正向驶入。
- 规划器仍依赖 odom 与手工雷达偏移，没有完整 TF 时间同步。
- 尚未验证真机、Nav2、SLAM 或 VLM 闭环决策。

## 下一阶段

1. 冻结多种子测试集合、重复次数和统计口径。
2. 增加消息超时、节点退出、无安全通道和碰撞等异常注入。
3. 统计成功率、失败分类、任务时间、CP2 误差和停车误差。
4. 保持 seed 42 P0 验收作为每次变更后的回归基线。
