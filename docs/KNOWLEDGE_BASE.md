# 项目知识库索引

更新时间：2026-08-17

本目录保存适合跨设备和多人协作的项目知识。代码、自动测试和正式验收证据是最终事实依据，文档不得覆盖与其冲突的结果。

## 信息可信顺序

1. 当前 Git 提交中的代码与配置。
2. 自动测试结果和正式验收 `result.json`。
3. 冻结的规范、Schema 和运行契约。
4. 项目状态、开发日志和协作文档。
5. 对话、人工观察和未归档说明。

只有机器证据满足验收条件时才能记录 `PASS`。用户观察或单独日志不能替代正式证据。

## 云端知识结构

| 文件 | 用途 | 更新时机 |
|---|---|---|
| `docs/PROJECT_STATUS.md` | 当前能力、验证结果、限制和下一步 | 里程碑、验证结果或主线变化后 |
| `docs/p0_acceptance_spec.md` | P0 验收要求和安全不变量 | 验收契约经过评审并明确变更后 |
| `docs/schemas/p0_result.schema.json` | `result.json` 机器可读结构 | 结果契约变更时 |
| `docs/examples/p0_result.example.json` | 非真实结果示例 | Schema 示例需要同步时 |
| `docs/development_log_*.md` | 按日期保存阶段开发事实 | 阶段工作完成后 |
| `docs/ros2_environment_setup.md` | ROS 2 环境安装与使用记录 | 环境基线变化时 |
| `CONTRIBUTING.md` | 协作习惯、安全边界和验证流程 | 团队工作方式变化时 |

## 本机私有层

以下路径通过 `.gitignore` 保持本机私有，不属于云端知识库：

- `local_knowledge/`：详细项目理解、理论卡片和个人继续开发位置。
- `handoff/`：阶段交接 Prompt。
- `artifacts/`：rosbag、运行日志、CSV 和正式结果产物。

私有层不得保存为云端事实的唯一来源。需要跨设备恢复的重要状态，应脱敏后更新到 `docs/PROJECT_STATUS.md`。

## 更新规则

- 每项状态必须注明适用日期和可追溯 Git 提交。
- 明确区分“已实现”“已测试”“单次验收通过”和“尚未验证”。
- 新能力应同时记录实现位置、测试方式、能力边界和剩余风险。
- 修改安全链、运动门禁、验收阈值或消息契约时，必须同步规范和测试。
- 不同步 Token、密码、私有环境变量、账号凭据或未经筛选的运行日志。
- 不在知识库中声称尚未完成的多种子成功率、真机、Nav2、SLAM 或完整 VLA 能力。

## 新设备恢复

```bash
git clone https://github.com/HxG949/ros2-turtlebot3-vlm.git
cd ros2-turtlebot3-vlm
git pull origin main
```

建议依次阅读：

1. `docs/KNOWLEDGE_BASE.md`
2. `docs/PROJECT_STATUS.md`
3. `CONTRIBUTING.md`
4. 当前任务对应的规范和开发日志
