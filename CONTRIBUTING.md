# 协作与开发规则

本文记录需要跨设备保持一致的项目协作习惯、安全边界和验证流程。

## 沟通方式

- 默认使用中文沟通。
- 开始开发步骤前说明当前小目标、相关理论、准备修改或运行的内容、风险和成功标准。
- 启动持续进程、启用非零运动、提交或推送前，先明确说明影响并获得用户确认。
- 复杂任务维护 `完成`、`进行中`、`待处理`、`阻塞` 状态。
- 完成后报告实际结果、测试证据、能力边界和下一步，不只描述实现意图。

## 开始工作前

先检查真实工作区，不依赖旧对话或交接文档推断当前状态：

```bash
git status --short --branch
git log --oneline -5
colcon test-result --verbose
```

- 阅读 `docs/KNOWLEDGE_BASE.md` 和 `docs/PROJECT_STATUS.md`。
- 阅读当前任务对应的规范、配置、测试和最近开发日志。
- 工作区存在其他修改时不得擅自回滚、覆盖或清理。

## 运动与安全

- `enable_motion` 默认必须为 `false`。
- 只有用户明确同意运动验证时才能使用 `--enable-motion`。
- 正常运行只能有一个最终 `/cmd_vel` 发布者。
- acceptance monitor 必须保持只读，不能发布速度或修改生产状态。
- 数据无效、消息超时、目标变化、计划变化、安全链异常或碰撞时优先停车。
- 不得为通过测试而移除故障锁存、watchdog 或放宽运行期安全阈值。
- GUI 和 headless 运动测试都必须等待 runner 完成安全关闭。

## 实现原则

- 先读取现有代码和配置，再决定实现方案。
- 优先采用满足需求的最小改动。
- 不添加没有实际需求的兼容层、抽象或依赖。
- 生产规划、控制和验收监控保持职责分离。
- 新增状态字段、话题或结果字段时同步契约、Schema 和测试。
- 复杂算法需要注释其约束或原因，避免重复代码表面含义的注释。

## 验证流程

Python/ROS 包改动至少执行相关包构建和测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <packages>
source install/setup.bash
colcon test --packages-select <packages> --event-handlers console_direct+
colcon test-result --verbose
```

运动链路变更按以下顺序验证：

1. 纯逻辑或 synthetic 测试。
2. 相关 ROS 包构建和完整测试。
3. `enable_motion=false` 的证据链 dry-run。
4. 明确获准后执行固定 seed 的正式运动验收。
5. 检查 `result.json`、rosbag 完整性、安全状态和碰撞结果。

## Git 规则

- 提交前检查 `git status`、`git diff`、`git diff --check` 和最近提交风格。
- 只暂存本次任务相关文件。
- 不提交 `build/`、`install/`、`log/`、`artifacts/`、缓存或凭据。
- 不使用破坏性 reset、强制推送或跳过检查，除非用户明确要求并理解风险。
- 未经明确请求不 amend、commit 或 push。
- 推送后确认当前分支与远程跟踪分支同步。

## 知识库更新

发生以下变化时更新 `docs/PROJECT_STATUS.md`：

- 当前主线或下一阶段变化。
- 新增或删除已验证能力。
- 架构、话题、状态机或安全策略变化。
- 测试总数和正式验收结果变化。
- 已知限制得到解决或发现新的可复现风险。

更新要求：

- 写明日期和实现基线提交。
- 区分代码已实现、测试已通过、单次验收通过和统计验证完成。
- 记录关键命令、结果和能力边界。
- 详细个人笔记保留在 `local_knowledge/`，不直接上传。

## 表述边界

- 未进行多种子重复实验时，不声称成功率或完整 L3。
- 未做真机测试时，不声称真机部署。
- 未实现时，不声称 Nav2、SLAM、通用泊车或完整 VLA。
- VLM 当前不能覆盖雷达安全或直接控制 `/cmd_vel`。
- 失败运行必须保留真实分类，不能通过人工观察改写为 PASS。
