# ROS2环境搭建日志

## 2026-07-27 ROS2 Humble基础环境配置

### 完成内容

- 在Ubuntu 22.04.5 LTS环境下完成ROS2 Humble Hawksbill安装。
- 配置ROS2官方软件源并完成ROS2 Desktop版本安装。
- 配置ROS2环境变量，使系统能够自动加载ROS2 Humble环境。
- 安装Git、colcon等ROS2开发所需工具。
- 创建ROS2工作空间：
  - ros2_ws
  - src目录
- 安装TurtleBot3 ROS2相关软件包。
- 配置TurtleBot3 Burger作为默认仿真机器人模型。
- 完成ROS2官方通信测试：
  - 使用talker节点发布消息。
  - 使用listener节点接收消息。
  - 验证ROS2节点通信机制正常。
- 完成TurtleBot3 Gazebo仿真环境测试：
  - 成功启动turtlebot3_world仿真环境。
  - 成功加载TurtleBot3 Burger机器人模型。
  - 使用teleop_keyboard节点控制机器人运动。
  - 验证ROS2通过/cmd_vel话题控制机器人运动流程正常。

### 遇到的问题

- 第一次启动TurtleBot3 Gazebo仿真时，机器人模型自动生成失败，出现spawn_entity.py进程退出问题。
- 通过检查ROS2服务列表确认Gazebo的/spawn_entity服务正常运行。
- 分析问题后发现主要原因可能是Gazebo启动阶段服务初始化时间不足。
- 等待Gazebo完全启动后，手动调用spawn_entity.py成功加载机器人模型。

### 当前环境状态

- Ubuntu 22.04.5 LTS环境配置完成。
- ROS2 Humble开发环境运行正常。
- Gazebo仿真环境运行正常。
- TurtleBot3机器人可以在Gazebo中被控制移动。
- 当前环境已经具备后续ROS2机器人开发基础，可继续进行节点开发、SLAM、Navigation2以及ROS2与VLM结合实验。

## 2026-08-02 ROS2 + VLM项目阶段开发

### 今日目标

- 整理并验证截至今日的ROS2 + VLM项目进度。
- 在Gazebo中建立从视觉感知、雷达安全判断、行为决策到运动控制的基础链路。
- 根据校级电赛《B题-打靶小车》修正项目目标，先实现基础停车和三瓶避障任务。
- 将当前代码和真实开发边界同步到GitHub，避免把规划中的功能写成已经完成。

### 完成内容

#### 1. Gazebo虚拟摄像头接入

- 使用带摄像头的TurtleBot3 Burger Cam仿真模型。
- 验证`/camera/image_raw`图像话题可以稳定发布，图像大小为320×240，频率约为30 Hz。
- 确认视觉节点可以通过ROS2订阅Gazebo相机图像。
- 相机参数优化暂时停止，当前继续使用Burger Cam默认配置，避免过早增加系统复杂度。

#### 2. VLM运行环境与BLIP模型验证

- 在工作空间中创建独立Python虚拟环境`.venv`，避免污染系统Python和ROS2环境。
- 完成PyTorch、Transformers、Pillow、OpenCV等依赖配置。
- 当前环境检查结果：
  - Python 3.10.12。
  - PyTorch 2.11.0+cu128。
  - Transformers 4.57.6。
  - CUDA可以被PyTorch正常识别。
- 下载并缓存`Salesforce/blip-image-captioning-base`模型。
- 使用GTX 1660 Ti完成BLIP GPU推理验证。
- 单次测试中首次推理约762 ms，模型预热后约401 ms；这些数据只代表当前仿真环境下的功能测试，不代表完整性能实验结果。

#### 3. VLM感知节点

- 实现`vlm_inference_node`。
- 订阅Gazebo图像话题`/camera/image_raw`。
- 定时调用BLIP模型生成图像描述，并发布到`/vlm/perception_result`。
- 当前主要参数：
  - 推理间隔2.0 s。
  - 最大生成长度30 tokens。
  - 目标物体关键字为`bottle`。
  - GPU环境下启用FP16推理。
- 感知结果使用JSON字符串传递，便于后续节点读取模型描述、目标匹配状态和推理时间。

#### 4. 激光雷达安全节点

- 实现`lidar_safety_node`，订阅`/scan`并发布`/safety/status`。
- 将雷达扫描划分为前方、左侧和右侧区域，输出各区域最近有效距离。
- 当前紧急停车距离为0.35 m。
- 验证障碍物进入前方危险范围后可以触发`emergency_stop`。
- 雷达安全判断独立于VLM，保证高层语义判断错误或超时时，底层仍可以优先停车。

#### 5. 高层决策节点

- 实现`decision_node`。
- 同时订阅`/vlm/perception_result`和`/safety/status`，发布`/robot/decision`。
- 当前可以生成`stop`、`forward`、`turn_left`和`turn_right`等离散决策。
- 决策优先级以安全为先：雷达紧急停车、消息超时或数据无效都会输出停车命令。
- 当前VLM只提供场景语义和目标物体提示，不直接生成机器人速度。

#### 6. 运动控制节点

- 实现`motion_controller_node`，将`/robot/decision`转换为`/cmd_vel`速度命令。
- 增加以下安全限制：
  - 默认`enabled=false`，启动后不允许发布非零速度。
  - 决策消息超过1.0 s未更新时自动停车。
  - 最大线速度0.15 m/s，最大角速度0.5 rad/s。
- 在明确启用运动后完成过一次短时间真实右转测试，随后恢复默认禁用状态。

#### 7. 一键启动文件

- 创建`semantic_navigation.launch.py`，可以统一启动VLM感知、雷达安全、决策和运动控制节点。
- 创建`competition_parking.launch.py`，可以启动固定目标停车决策与运动控制节点。
- 两个启动文件默认都设置`enable_motion=false`，防止启动后机器人意外移动。

#### 8. Gazebo仿真场景

- 创建`robot_simulation`功能包，统一存放Gazebo模型、世界文件和启动文件。
- 创建三瓶语义场景原型`semantic_bottle.world`，用于验证相机、雷达和基础语义链路。
- 创建正式竞赛基础场地`target_car_basic.world`：
  - 场地尺寸为2.10 m × 2.10 m。
  - 包含围挡、起始区、固定2号停车区和目标区域标线。
  - 当前不生成矿泉水瓶，用于先实现基本要求（1）的无障碍停车。
- 当前三瓶原型的尺寸和生成方式还没有按竞赛规格完成，因此不能作为正式随机避障场景。

#### 9. 固定2号位停车决策

- 实现`parking_decision_node`，订阅`/odom`并向`/robot/decision`发布离散运动指令。
- 使用有限状态机完成朝向目标、前进、最终转向和停车判断。
- 当前目标为固定2号打靶位：
  - 目标位置`x=0.8015 m`、`y=0.0 m`。
  - 最终朝向为π rad。
- 干运行验证中，起点到目标距离约1.901 m，初始应左转约26.4°。
- 已验证默认禁用运动时`/cmd_vel`保持全零，节点能够干净退出。
- 尚未执行完整的自动运动停车测试，因此不能声明已经满足5 cm停车误差要求。

#### 10. Git与GitHub同步

- 建立远程仓库`HxG949/ros2-turtlebot3-vlm`并推送`main`分支。
- 首次项目提交为`131775f Add ROS2 perception and competition simulation`。
- 通过GitHub远程接口二次确认：
  - 远程最新提交完整SHA为`131775fea9fbe7dfb46076000b9d3e292b2c7748`。
  - 远程`src`目录同时包含`robot_perception`和`robot_simulation`。
- 将`.venv`、`build`、`install`、`log`和Python缓存加入`.gitignore`，防止本地环境和生成文件进入仓库。

### 遇到的问题与解决方法

- 默认TurtleBot3 Burger模型没有提供项目需要的相机图像。
  - 改用Burger Cam仿真模型，并通过话题列表和图像频率确认相机接入成功。
- ROS2使用系统Python，而VLM依赖需要独立版本管理。
  - 在工作空间创建`.venv`，使用该环境构建和运行感知包，避免直接修改系统Python。
- Hugging Face主站下载模型不稳定。
  - 使用可访问的镜像完成模型下载，并保留本地缓存供离线加载。
- 初始Pillow版本与Transformers处理流程不兼容。
  - 升级虚拟环境中的Pillow后重新验证模型加载和推理。
- 首次构建后ROS2入口可能仍调用错误的Python解释器。
  - 在激活`.venv`的环境中重新执行`colcon build`，使Python节点使用正确环境。
- 仿真模型和相机插件启动顺序不稳定时，可能出现话题暂时不可用。
  - 等待Gazebo完全启动后再检查话题，避免在初始化阶段误判失败。
- 直接输出完整`LaserScan`数组会产生大量终端文本，难以观察关键状态。
  - 使用独立安全节点只发布前、左、右最近距离和紧急停车状态。
- 决策节点意外退出或消息停止时，机器人可能保持旧速度。
  - 在运动控制节点中加入消息超时看门狗，并在关闭节点时发布零速度。
- GitHub不再支持账户密码进行HTTPS推送。
  - 使用Personal Access Token在本地交互终端中完成认证，Token没有写入代码或日志。
- GitHub仓库简介目前提到了SLAM和Navigation2，但代码中尚未实现这两项。
  - 本日志明确标记实际进度；后续应将仓库简介改为与当前实现一致的描述。

### 构建与测试结果

- `robot_perception`和`robot_simulation`均可通过`colcon build`完成构建。
- `robot_perception`测试结果为3项测试、0个错误、0个失败、1项跳过。
- Gazebo相机话题、雷达安全状态、决策话题和速度话题均完成过独立验证。
- 固定2号位停车当前只完成决策干运行和零速度安全验证，没有完成整段运动精度测试。

### 当前实现边界

- 已完成ROS2仿真中的VLM图像描述、雷达安全判断、离散决策、运动控制和竞赛基础场地。
- 项目当前属于“VLM辅助的ROS2机器人高层语义决策实验”，不能表述为完整VLA系统。
- VLM还没有稳定识别仿真中的矿泉水瓶，语义结果不能独立承担安全避障。
- 正式三瓶随机场景尚未实现；竞赛规定的瓶子尺寸为高17 cm、直径7 cm，后续需要按区域进行二维随机生成。
- 固定2号位停车尚未完成带运动的闭环验证，也没有证明停车误差小于5 cm。
- 激光云台、自动瞄准、激光点火、弹药补充区和5个目标全流程尚未实现。
- SLAM、Navigation2、实物机器人部署和系统化性能实验尚未完成。

### 下一步计划

- 第一优先级：在无障碍正式场地中低速验证固定2号位完整停车流程，并记录实际停车误差。
- 第二优先级：按17 cm × 7 cm规格建立三瓶二维随机生成场景。
- 第三优先级：将雷达安全链路接入竞赛停车流程，验证紧急停车优先级。
- 完成基础停车与避障后，再开始激光云台、自动瞄准和扩展任务。
