# Ackermann 覆盖导航项目说明文档

本文档用于说明项目怎么跑、各模块为什么这样连接、ROS 接口和参数在哪里、如何继续开发，以及后续建议。当前项目默认面向 Ubuntu 20.04 + ROS Noetic + Gazebo 仿真环境。

## 1. 项目概览

本工作空间实现了一套 Ackermann 小车全覆盖导航仿真流程：

1. PyQt5 GUI 加载地形图，交互式绘制覆盖区域。
2. GUI 调用覆盖路径规划服务，生成 `nav_msgs/Path`。
3. GUI 将规划路径发布到 `/plan_path`。
4. MPC 跟踪节点订阅路径和定位状态，输出 `/ackermann_car/cmd_vel`。
5. `cmd_vel_to_tricycle.py` 将线速度/角速度转换为三轮车插件使用的控制量。
6. Gazebo 中的小车执行路径，RViz 和 GUI 显示状态。

主包位于：

```bash
src/ackermann_car_gazebo
```

核心入口：

```bash
./install.sh
./run.sh
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

主配置文件：

```bash
src/ackermann_car_gazebo/config/coverage_navigation_params.yaml
```

## 2. 快速上手

### 2.1 安装依赖并构建

目标机器需要先安装 ROS Noetic 本体，脚本不负责从零安装 ROS。

在工作空间根目录执行：

```bash
./install.sh
```

脚本会做这些事情：

- 检查 `/opt/ros/noetic/setup.bash` 是否存在。
- 安装 apt/ROS 依赖，包括 Gazebo、RViz、PyQt5、catkin tools、rosdep、OSQP。
- 初始化或更新 rosdep。
- 执行 `rosdep install`。
- 清理旧 catkin 构建产物并执行 `catkin build`。

构建完成后，在当前 shell 中执行：

```bash
source devel/setup.bash
```

### 2.2 一键启动

```bash
./run.sh
```

`run.sh` 会先尝试清理残留的 Gazebo 进程，再 source ROS 和工作空间环境，最后在新终端中启动：

```bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

如果目标机器没有支持的新终端程序，可以手动执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

### 2.3 GUI 基本操作流程

1. 启动 GUI。
2. 点击 GUI 中的仿真启动按钮，启动 Gazebo、车辆、RViz 和定位节点。
3. 在地图上绘制覆盖区域。
4. 调整扫描宽度、覆盖率、转弯半径等规划参数。
5. 点击规划按钮，GUI 调用 `/coverage_path_planner/plan_coverage_path`。
6. 确认路径显示正常后点击导航。
7. GUI 发布 `/plan_path`，并按配置启动 `mpc_path_tracker.launch`。
8. `/mpc_controller` 输出 `/ackermann_car/cmd_vel`，车辆开始跟踪。
9. 状态栏观察 `/tracking_status`，正常会经历 `tracking`，到终点后变为 `arrived`。

## 3. 系统流程

### 3.1 Launch 关系

主入口是：

```bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

它会：

- 加载 `coverage_navigation_params.yaml`。
- 启动 `coverage_path_planner.launch`，节点名为 `/coverage_path_planner`。
- 发布静态 `map -> odom` TF。
- 启动 PyQt5 GUI，节点名为 `/coverage_navigation_gui`。

GUI 按钮会按配置启动两个 launch：

- `gazebo_ackermann_car.launch`：启动 Gazebo、车辆、传感器模拟、EKF、RViz。
- `mpc_path_tracker.launch`：为了兼容旧 GUI 入口保留该文件名，但实际启动的是 `mpc_controller_node`，节点名为 `/mpc_controller`。

### 3.2 数据流

主要数据流如下：

```text
GUI 绘制区域
  -> /coverage_path_planner/plan_coverage_path
  -> nav_msgs/Path
  -> /plan_path
  -> /mpc_controller
  -> /ackermann_car/cmd_vel
  -> /ackermann_car/cmd_vel_to_tricycle
  -> /ackermann_car/tricycle_cmd
  -> Gazebo 车辆插件
```

定位状态数据流如下：

```text
Gazebo /gazebo/model_states
  -> dual_rtk_publisher.py
  -> /ackermann_car/rtk_front/fix + /ackermann_car/rtk_rear/fix
  -> dual_rtk_imu_ekf_node + /ackermann_car/imu/data
  -> /ackermann_car/ekf_state
  -> GUI 和 /mpc_controller
```

MPC 会优先使用 `/ackermann_car/ekf_state`，如果 EKF 超时则 fallback 到 `/ackermann_car/odom`，再 fallback 到 `/odom`。

## 4. 模块原理

### 4.1 Gazebo 车辆、URDF 和世界

车辆模型由 `urdf/ackermann_car.urdf.xacro` 描述，`gazebo_ackermann_car.launch` 会通过 xacro 生成 `robot_description` 并调用 `spawn_model` 放入 Gazebo。默认世界是：

```bash
src/ackermann_car_gazebo/worlds/terrain.world
```

GUI 使用的地形渲染图配置在：

```yaml
coverage_navigation_gui:
  render_path: "package://ackermann_car_gazebo/worlds/render.png"
  map_world_size_m: 129.0
```

当前地图和规划默认使用 ENU 笛卡尔坐标：`x` 为 East，`y` 为 North，`z` 为 Up。

### 4.2 PyQt5 覆盖导航 GUI

GUI 脚本是：

```bash
src/ackermann_car_gazebo/scripts/coverage_navigation_gui.py
```

它负责：

- 加载地形图并完成像素坐标到世界坐标的转换。
- 订阅 EKF/odom 状态并显示车辆位置。
- 收集用户绘制的多边形覆盖区域。
- 调用覆盖路径规划服务。
- 发布 latched `/plan_path`。
- 点击停止时向 `/ackermann_car/cmd_vel` 发布零速度。
- 显示 `/tracking_status`。
- 按 YAML 中的 launch 配置启动仿真和跟踪节点。

注意：GUI 的 topic 和服务名应通过 `coverage_navigation_params.yaml` 修改，不建议直接改脚本中的默认值。

### 4.3 覆盖路径规划器

规划代码主要在：

```bash
src/ackermann_car_gazebo/src/coverage_path_planner.cpp
src/ackermann_car_gazebo/src/coverage_path_planner_node.cpp
src/ackermann_car_gazebo/include/ackermann_car_gazebo/coverage_path_planner.hpp
```

节点 `/coverage_path_planner` 提供私有服务 `plan_coverage_path`，因此完整服务名是：

```bash
/coverage_path_planner/plan_coverage_path
```

规划器输入一个多边形区域，并根据扫描宽度、覆盖率和转弯半径生成覆盖路径。实现上会：

- 清理和校验多边形。
- 对复杂区域做子区域分解。
- 通过OBB算法确定扫描方向
- 按扫描方向生成覆盖直线。
- 使用 Dubins 风格的转弯连接扫描线。
- 为子区域入口/出口生成候选路径变体。
- 根据连接代价选择子区域顺序。
- 子区域数量不超过 `exact_connection_limit` 时使用精确 DP，超过后使用贪心连接，避免组合爆炸。

规划结果会返回路径、覆盖线数量、分解数量、线间距、总代价、内部路径代价、连接代价、起点接入代价等指标，GUI 可用于展示和调参。

### 4.4 OSQP MPC 路径跟踪器

当前跟踪节点是：

```bash
src/ackermann_car_gazebo/src/mpc_controller_node.cpp
```

它是独立 ROS 节点， `mpc_path_tracker.launch` 保留旧文件名作为 GUI 入口，但实际启动：

```text
node name: /mpc_controller
type: mpc_controller_node
```

MPC 算法核心来自开源 MPC local planner 的 QP 思路，并移除了原来的 `pluginlib`、`nav_core`、`costmap_2d` 依赖。当前模型：

- 状态量：`[x, y, yaw]`
- 控制量：`[v, yaw_rate]`
- 输出：`geometry_msgs/Twist.linear.x` 和 `geometry_msgs/Twist.angular.z`
- 求解器：OSQP

为了避免覆盖航带之间按欧氏最近点跳线，路径跟踪使用有序路径投影：

- `nav_msgs/Path` 按消息顺序构建弧长 `s`。
- 当前投影只允许沿路径单调前进。
- 搜索窗口从当前 segment 向前展开，不回退到已经走过的路径。
- 如果横向偏离超过 `off_path_stop_distance`，发布零速度和 `error:off_path`，而不是直接追到远处最近航带。

前视距离公式：

```text
lookahead = clamp(abs(speed) * lookahead_time,
                  min_lookahead_dist,
                  max_lookahead_dist)
```

到达条件：

```text
终点欧氏距离 <= goal_tolerance
并且
剩余弧长 <= goal_tolerance
```

到达后持续发布零速度和 `arrived`。

### 4.5 cmd_vel 到三轮车控制转换

转换脚本是：

```bash
src/ackermann_car_gazebo/scripts/cmd_vel_to_tricycle.py
```

该节点位于 `/ackermann_car` 命名空间内，默认接口：

- 订阅：`cmd_vel`，解析为 `/ackermann_car/cmd_vel`
- 发布：`tricycle_cmd`，解析为 `/ackermann_car/tricycle_cmd`

转换逻辑：

```text
steering_angle = atan(wheelbase * angular_z / linear_x)
```

再根据 `max_steering_angle` 限幅。MPC 和该脚本中的 `wheelbase`、`max_steering_angle` 应保持一致。

### 4.6 双 RTK + IMU EKF

仿真 RTK 发布脚本：

```bash
src/ackermann_car_gazebo/scripts/dual_rtk_publisher.py
```

它订阅 `/gazebo/model_states`，根据车辆模型位姿和前后天线偏移，发布：

- `/ackermann_car/rtk_front/fix`
- `/ackermann_car/rtk_rear/fix`

EKF 节点：

```bash
src/ackermann_car_gazebo/src/dual_rtk_imu_ekf_node.cpp
```

它融合前后 RTK 和 IMU，输出：

```bash
/ackermann_car/ekf_state
```

输出消息类型是 `ackermann_car_gazebo/RtkImuEkfState`，包含 `x`、`y`、`yaw`、`vx`、`vy`。MPC 使用该状态作为首选定位输入。

### 4.7 声呐和 RViz 可视化

声呐 Marker 脚本：

```bash
src/ackermann_car_gazebo/scripts/sonar_range_marker.py
```

默认在 `/ackermann_car` 命名空间内：

- 订阅：`sonar/range`
- 发布：`sonar/marker`

RViz 配置：

```bash
src/ackermann_car_gazebo/rviz/ackermann_car.rviz
```

`gazebo_ackermann_car.launch` 默认会启动 RViz。

## 5. ROS 接口

### 5.1 核心节点

| 节点                                   | 来源                           | 作用                              |
| -------------------------------------- | ------------------------------ | --------------------------------- |
| `/coverage_navigation_gui`           | `coverage_navigation_gui.py` | GUI、规划请求、路径发布、状态显示 |
| `/coverage_path_planner`             | `coverage_path_planner_node` | 覆盖路径规划服务                  |
| `/mpc_controller`                    | `mpc_controller_node`        | OSQP MPC 路径跟踪                 |
| `/ackermann_car/cmd_vel_to_tricycle` | `cmd_vel_to_tricycle.py`     | Twist 到三轮车控制转换            |
| `/ackermann_car/dual_rtk_publisher`  | `dual_rtk_publisher.py`      | Gazebo 位姿到双 RTK               |
| `/ackermann_car/dual_rtk_imu_ekf`    | `dual_rtk_imu_ekf_node`      | 双 RTK + IMU 定位融合             |
| `/ackermann_car/sonar_range_marker`  | `sonar_range_marker.py`      | 声呐 RViz Marker                  |

### 5.2 Topic

| Topic                            | 类型                                    | 方向                    | 说明                                         |
| -------------------------------- | --------------------------------------- | ----------------------- | -------------------------------------------- |
| `/plan_path`                   | `nav_msgs/Path`                       | GUI -> MPC              | 规划路径，GUI latched 发布                   |
| `/ackermann_car/cmd_vel`       | `geometry_msgs/Twist`                 | MPC/GUI -> 转换节点     | 线速度和角速度命令；GUI 停止按钮也会发零速度 |
| `/tracking_status`             | `std_msgs/String`                     | MPC -> GUI              | 跟踪状态                                     |
| `/ackermann_car/ekf_state`     | `ackermann_car_gazebo/RtkImuEkfState` | EKF -> GUI/MPC          | 首选定位状态                                 |
| `/ackermann_car/odom`          | `nav_msgs/Odometry`                   | Gazebo/插件 -> GUI/MPC  | 第一备用状态                                 |
| `/odom`                        | `nav_msgs/Odometry`                   | Gazebo/插件 -> GUI/MPC  | 第二备用状态                                 |
| `/ackermann_car/tricycle_cmd`  | `geometry_msgs/Twist`                 | 转换节点 -> Gazebo 插件 | 三轮车控制命令                               |
| `/ackermann_car/rtk_front/fix` | `sensor_msgs/NavSatFix`               | RTK 仿真 -> EKF         | 前 RTK                                       |
| `/ackermann_car/rtk_rear/fix`  | `sensor_msgs/NavSatFix`               | RTK 仿真 -> EKF         | 后 RTK                                       |
| `/ackermann_car/imu/data`      | `sensor_msgs/Imu`                     | Gazebo 插件 -> EKF      | IMU                                          |
| `/ackermann_car/sonar/range`   | `sensor_msgs/Range`                   | Gazebo 插件 -> Marker   | 声呐距离                                     |
| `/ackermann_car/sonar/marker`  | `visualization_msgs/Marker`           | Marker -> RViz          | 声呐扇形可视化                               |

### 5.3 Service

服务名：

```bash
/coverage_path_planner/plan_coverage_path
```

类型：

```bash
ackermann_car_gazebo/PlanCoveragePath
```

请求核心字段：

| 字段                        | 说明                           |
| --------------------------- | ------------------------------ |
| `header`                  | 坐标系和时间戳                 |
| `polygon`                 | 覆盖区域多边形点               |
| `use_robot_start`         | 是否使用机器人当前位置接入路径 |
| `robot_start`             | 机器人起点位姿                 |
| `use_robot_start_heading` | 是否使用起点朝向               |
| `scan_width`              | 扫描宽度                       |
| `coverage_ratio`          | 覆盖率                         |
| `turn_radius`             | 转弯半径                       |
| `turn_step`               | 转弯曲线采样步长               |
| `start_connection_weight` | 起点接入代价权重               |

响应核心字段：

| 字段                         | 说明                     |
| ---------------------------- | ------------------------ |
| `success`                  | 是否规划成功             |
| `message`                  | 状态或错误说明           |
| `path`                     | 生成的 `nav_msgs/Path` |
| `num_lines`                | 覆盖扫描线数量           |
| `decomposition_count`      | 子区域数量               |
| `line_spacing`             | 扫描线间距               |
| `total_path_cost`          | 总路径代价               |
| `internal_path_cost`       | 子区域内部路径代价       |
| `smooth_connection_cost`   | 区域/航带连接代价        |
| `start_connection_cost`    | 起点接入代价             |
| `selected_variant_indices` | 各子区域选中的路径变体   |
| `connection_order`         | 子区域连接顺序           |

### 5.4 自定义消息

`ackermann_car_gazebo/RtkImuEkfState`：

```text
std_msgs/Header header
float64 x
float64 y
float64 yaw
float64 vx
float64 vy
```

其中 `x/y/yaw` 是地图坐标系下的二维位姿，`vx/vy` 是平面速度。

### 5.5 跟踪状态语义

| 状态                   | 含义                                |
| ---------------------- | ----------------------------------- |
| `idle`               | 无有效路径，或收到空路径            |
| `tracking`           | 正在跟踪路径                        |
| `arrived`            | 已到达终点，节点发布零速度          |
| `error:no_state`     | EKF/odom/fallback odom 都超时或无效 |
| `error:off_path`     | 车辆横向偏离当前有序路径超过阈值    |
| `error:optimization` | OSQP 求解未成功                     |

## 6. 参数配置

统一参数入口：

```bash
src/ackermann_car_gazebo/config/coverage_navigation_params.yaml
```

正常使用时由 `coverage_navigation_gui.launch` 一次性加载，不建议其他 launch 重复加载或写死同名参数。

### 6.1 全局参数

```yaml
use_sim_time: true
```

仿真使用 Gazebo `/clock`。未来接真实车时通常改为 `false`，并移除 Gazebo 相关启动流程。

### 6.2 `coverage_navigation_gui`

负责 GUI、地图显示、topic/service 名称和按钮启动的 launch 命令。最常改的字段：

- `render_path`：GUI 背景图。
- `map_world_size_m`：背景图对应的真实世界边长。
- `planning_service`：规划服务名。
- `plan_topic`：路径发布 topic。
- `cmd_vel_topic`：停止按钮和控制输出 topic。
- `sim_launch` / `sim_launch_args`：GUI 启动仿真的命令。
- `tracker_launch` / `tracker_launch_args`：GUI 启动跟踪器的命令。

### 6.3 `coverage_path_planner`

覆盖规划默认参数。GUI 请求中提供的参数会覆盖部分默认值。

- `scan_width`：扫描宽度。
- `coverage_ratio`：覆盖率，越大扫描线越密。
- `turn_radius`：Dubins 掉头半径，应符合车辆最小转弯能力。
- `turn_step`：转弯曲线离散步长。
- `straight_step`：直线段采样步长。
- `start_connection_weight`：起点接入代价权重。
- `exact_connection_limit`：使用精确 DP 的最大子区域数。
- `nonadjacent_penalty`：非相邻区域连接惩罚。

### 6.4 `mpc_controller`

MPC 跟踪器参数。调参时建议先只改少数关键项：

- `target_speed`：目标巡航速度。
- `max_linear_velocity`：最大线速度。
- `max_angular_velocity`：最大角速度，同时受 `wheelbase` 和 `max_steering_angle` 限制。
- `goal_tolerance`：到达终点阈值。
- `off_path_stop_distance`：偏离路径停车阈值。
- `lookahead_time`、`min_lookahead_dist`、`max_lookahead_dist`：前视距离。
- `predict_time_domain`、`control_time_domain`、`control_dt`：MPC 预测/控制时域。
- `q_matrix_x/y/theta`：状态误差权重。
- `r_matrix_v/w`：控制量权重。
- `delta_*`：单步控制增量约束。

调参建议：

- 车辆抄近道或跳到相邻航带：优先减小 `max_lookahead_dist`，降低 `target_speed`，收紧 `off_path_stop_distance`。
- 转弯太激进：降低 `max_angular_velocity` 或 `max_steering_angle`，增大规划器 `turn_radius`。
- 跟踪抖动：增大 `r_matrix_w` 或适当减小 `q_matrix_theta`。
- 速度变化太慢：放宽 `delta_linear_velocity_max`；速度刹不住则检查 `delta_linear_velocity_min`。

### 6.5 `ackermann_car`

该组参数在车辆命名空间下使用：

- `cmd_vel_to_tricycle`：轮距、最大转角、输入/输出 topic。
- `dual_rtk_publisher`：Gazebo 模型名、经纬高原点、前后 RTK 天线偏移。
- `dual_rtk_imu_ekf`：传感器 topic、坐标系、同步窗口、噪声、RTK 门限和 URDF 几何校验。
- `sonar_range_marker`：声呐 marker 输入输出和显示参数。

## 7. 使用方法

### 7.1 常用命令

安装和构建：

```bash
./install.sh
```

加载环境：

```bash
source devel/setup.bash
```

一键启动：

```bash
./run.sh
```

手动启动 GUI：

```bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

只启动规划服务和 GUI：

```bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch start_planner:=true start_gui:=true
```

单独启动仿真：

```bash
roslaunch ackermann_car_gazebo gazebo_ackermann_car.launch
```

单独启动 MPC 跟踪器：

```bash
roslaunch ackermann_car_gazebo mpc_path_tracker.launch
```

### 7.2 停止和重启

正常停止使用各终端中的 `Ctrl+C`。

Gazebo 偶尔会留下残留进程，重启前可以执行：

```bash
pkill -TERM -x gzclient || true
pkill -TERM -x gzserver || true
pkill -TERM -x gazebo || true
```

`run.sh` 已经内置了该清理步骤。

### 7.3 接口自检

查看节点：

```bash
rosnode list
```

查看 topic 连接：

```bash
rostopic info /plan_path
rostopic info /ackermann_car/cmd_vel
rostopic info /tracking_status
rostopic info /ackermann_car/ekf_state
```

查看状态：

```bash
rostopic echo /tracking_status
```

查看控制输出：

```bash
rostopic echo /ackermann_car/cmd_vel
```

查看规划服务：

```bash
rosservice info /coverage_path_planner/plan_coverage_path
```

查看参数：

```bash
rosparam get /mpc_controller
rosparam get /coverage_path_planner
rosparam get /coverage_navigation_gui
```

查看计算图：

```bash
rqt_graph
```

## 8. 开发方法

### 8.1 构建

推荐使用 catkin tools：

```bash
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

如果修改了消息或服务，重新构建后一定要重新 source：

```bash
catkin build ackermann_car_gazebo
source devel/setup.bash
```

### 8.2 测试

当前 CMake 中有一个 gtest：

```bash
src/ackermann_car_gazebo/test/test_coverage_path_planner.cpp
```

运行方式：

```bash
catkin run_tests ackermann_car_gazebo
catkin_test_results
```

或者：

```bash
catkin build ackermann_car_gazebo --catkin-make-args run_tests
catkin_test_results
```

### 8.3 增加依赖

增加 C++/Python/ROS 依赖时至少检查三处：

1. `src/ackermann_car_gazebo/package.xml`
2. `src/ackermann_car_gazebo/CMakeLists.txt`
3. 根目录 `install.sh`

如果依赖可以由 rosdep 解析，优先写入 `package.xml`。如果是项目关键依赖，并且希望一键安装脚本显式安装，也同步加入 `install.sh`。

### 8.4 增加或修改参数

推荐流程：

1. 在节点代码中读取私有参数 `~param_name`，给出合理默认值。
2. 在 `coverage_navigation_params.yaml` 对应节点名分组中添加该参数。
3. 在本文档的参数章节补充含义。
4. 避免在 launch 文件里重复写同一个参数。

### 8.5 增加节点或脚本

C++ 节点：

- 在 `CMakeLists.txt` 添加 `add_executable`。
- 添加 `add_dependencies`，确保消息/服务生成顺序正确。
- 添加 `target_link_libraries`。
- 如需安装，加入 `install(TARGETS ...)`。

Python 脚本：

- 确保文件有 shebang。
- 确保可执行权限。
- 加入 `catkin_install_python(PROGRAMS ...)`。

Launch：

- 优先让主配置 YAML 提供参数。
- 如果是 GUI 按钮启动的流程，同步检查 `coverage_navigation_gui` 分组中的 `sim_launch` 或 `tracker_launch`。

### 8.6 修改规划器

规划器核心是纯 C++ 类 `CoveragePathPlanner`，服务节点只负责 ROS 消息转换。建议保持这种边界：

- 算法逻辑放在 `coverage_path_planner.cpp`。
- ROS topic/service 参数转换放在 `coverage_path_planner_node.cpp`。
- 新增算法行为优先增加 gtest。

重点测试场景：

- 三角形、矩形、凹多边形。
- 很小的区域或重复点。
- 多子区域连接顺序。
- 带机器人起点和起点朝向的规划。

### 8.7 修改 MPC 跟踪器

`mpc_controller_node.cpp` 目前包含两层：

- `MpcController`：QP/MPC 计算。
- `MpcControllerNode`：ROS 订阅、发布、参数、状态机。

修改建议：

- 保持外部 topic 不变，避免破坏 GUI。
- 保持状态字符串语义不变。
- 路径投影继续使用单调弧长策略，避免覆盖航带之间跳线。
- 修改 OSQP 约束后，必须测试 `error:optimization` 场景和速度/角速度限幅。
- Ackermann 车不要引入原地旋转到目标逻辑，输出仍保持 Twist，由转换节点处理前轮转角。

### 8.8 版本控制注意事项

当前工作空间中可能存在 `build/`、`devel/`、`logs/` 等 catkin 生成物。这些目录不应作为源码交接重点。建议后续整理 `.gitignore`，避免把构建产物、日志和 Python 缓存提交到仓库。

## 9. 常见问题和排障

### 9.1 `roslaunch` 找不到包

通常是没有 source 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find ackermann_car_gazebo
```

### 9.2 GUI 打开但地图不显示

检查：

```bash
rosparam get /coverage_navigation_gui/render_path
```

默认应为：

```text
package://ackermann_car_gazebo/worlds/render.png
```

如果换地图，确保图片存在且 `map_world_size_m` 与 Gazebo 世界尺度一致。

### 9.3 Gazebo 启动失败或卡住

常见原因是旧进程残留。优先使用：

```bash
./run.sh
```

或手动清理：

```bash
pkill -KILL -x gzclient || true
pkill -KILL -x gzserver || true
pkill -KILL -x gazebo || true
```

### 9.4 MPC 一直 `error:no_state`

检查定位输入：

```bash
rostopic echo /ackermann_car/ekf_state
rostopic echo /ackermann_car/odom
rostopic echo /odom
rosparam get /mpc_controller/state_timeout
```

如果 Gazebo 仿真暂停，`use_sim_time` 和消息时间也可能导致状态看起来不更新。

### 9.5 MPC 报 `error:off_path`

含义是车辆距离当前有序路径投影超过 `off_path_stop_distance`。排查方向：

- 车辆初始位置是否离路径起点太远。
- GUI 发布的 `/plan_path` 是否是期望区域。
- `map -> odom` TF 和坐标系是否一致。
- `off_path_stop_distance` 是否过小。

不要简单改回全局最近点追踪，否则覆盖航带之间容易抄近道。

### 9.6 MPC 报 `error:optimization`

表示 OSQP 未返回 solved。排查方向：

- OSQP 依赖是否安装完整：`ros-noetic-osqp-vendor` 和 `ros-noetic-osqp`。
- `predict_time_domain`、`control_time_domain` 是否设置过大。
- 速度、角速度和增量约束是否互相矛盾。
- `q_matrix_*` 和 `r_matrix_*` 是否为非负且合理。

### 9.7 车辆不转弯或转弯半径不对

检查三处参数是否一致：

```bash
rosparam get /mpc_controller/wheelbase
rosparam get /mpc_controller/max_steering_angle
rosparam get /ackermann_car/cmd_vel_to_tricycle/wheelbase
rosparam get /ackermann_car/cmd_vel_to_tricycle/max_steering_angle
```

同时检查规划器的 `turn_radius` 是否小于车辆实际可实现转弯半径。

### 9.8 `install.sh` 失败

先确认 ROS Noetic 存在：

```bash
ls /opt/ros/noetic/setup.bash
```

再单独检查 rosdep：

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

如果 apt 源缺包，先修复系统 ROS apt 源，再重新运行 `./install.sh`。

## 10. 文件索引

| 路径                                                                | 说明                                    |
| ------------------------------------------------------------------- | --------------------------------------- |
| `install.sh`                                                      | 一键安装依赖、rosdep、构建工作空间      |
| `run.sh`                                                          | 清理 Gazebo 残留并新终端启动 GUI 主入口 |
| `src/ackermann_car_gazebo/config/coverage_navigation_params.yaml` | 统一参数入口                            |
| `src/ackermann_car_gazebo/launch/coverage_navigation_gui.launch`  | GUI 主入口                              |
| `src/ackermann_car_gazebo/launch/gazebo_ackermann_car.launch`     | Gazebo 仿真入口                         |
| `src/ackermann_car_gazebo/launch/mpc_path_tracker.launch`         | 兼容旧名称的 MPC 跟踪入口               |
| `src/ackermann_car_gazebo/scripts/coverage_navigation_gui.py`     | PyQt5 覆盖导航 GUI                      |
| `src/ackermann_car_gazebo/src/coverage_path_planner.cpp`          | 覆盖路径规划算法                        |
| `src/ackermann_car_gazebo/src/coverage_path_planner_node.cpp`     | 覆盖路径规划 ROS 服务节点               |
| `src/ackermann_car_gazebo/src/mpc_controller_node.cpp`            | OSQP MPC 路径跟踪节点                   |
| `src/ackermann_car_gazebo/scripts/cmd_vel_to_tricycle.py`         | Twist 到三轮车控制转换                  |
| `src/ackermann_car_gazebo/scripts/dual_rtk_publisher.py`          | Gazebo 位姿到双 RTK 仿真                |
| `src/ackermann_car_gazebo/src/dual_rtk_imu_ekf_node.cpp`          | 双 RTK + IMU EKF                        |
| `src/ackermann_car_gazebo/scripts/sonar_range_marker.py`          | 声呐 RViz marker                        |
| `src/ackermann_car_gazebo/msg/RtkImuEkfState.msg`                 | EKF 输出消息                            |
| `src/ackermann_car_gazebo/srv/PlanCoveragePath.srv`               | 覆盖路径规划服务                        |
| `src/ackermann_car_gazebo/test/test_coverage_path_planner.cpp`    | 覆盖路径规划单元测试                    |
