# CovNav-Ackermann

[简体中文](README.md) | **English**

> Full-coverage navigation simulation project for an Ackermann-steering car · ROS Noetic + Gazebo

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420.svg)
![Sim](https://img.shields.io/badge/simulation-Gazebo-2C2C2C.svg)

A ROS1 Noetic + Gazebo simulation workspace for Ackermann-steering car full-coverage navigation — **draw a region on the map, and the car automatically plans a switchback coverage path and tracks it precisely with MPC**.

This project implements a complete simulation pipeline for Ackermann-car coverage navigation: the user loads a terrain map in the PyQt5 GUI and interactively draws the coverage region, a C++ coverage path planning service is called to generate a `nav_msgs/Path`, and an OSQP-based MPC tracking node then outputs velocity commands that drive the car in Gazebo through the coverage job.

The localization chain simulates a real agricultural-machine / sanitation-vehicle scenario: `NavSatFix` data for front and rear dual RTK antennas is generated from Gazebo ground truth and fused with the IMU by a custom EKF to output the full vehicle state, rather than using the simulation ground truth directly.

Planning and tracking each have their highlights: the planner combines polygon sub-region decomposition, OBB sweep-direction selection, Dubins-style turn connections, and bitmask-DP region ordering; the tracker uses ordered arc-length path projection to prevent "shortcut jumps" between coverage swaths.

The project targets Ubuntu 20.04 + ROS Noetic + Gazebo simulation and is positioned as a learning and experimental effort; it has not yet been connected to a real vehicle (real-vehicle integration, Mbus chassis communication, and vision-based obstacle avoidance are all on the development plan).

<div align="center">
  <img src="docs/pipeline.png" alt="System overview pipeline" width="1100">
</div>

<p align="center"><b>Figure 1</b> · Overall CovNav-Ackermann pipeline: dual RTK satellites and the IMU feed an EKF, the PyQt5 GUI outlines the coverage polygon, the planner generates a boustrophedon path, and the MPC controller drives the Ackermann car via /ackermann_car/cmd_vel to leave coverage tracks in the field.</p>

## Features

- **Interactive region drawing in the PyQt5 GUI**: loads the terrain render, converts between pixel and world coordinates automatically, lets you draw a polygonal coverage region with the mouse, and plan with one click after adjusting swath width, coverage ratio, and turn radius; the GUI can also launch the Gazebo simulation and the MPC tracker directly per the YAML config (`scripts/coverage_navigation_gui.py`).
- **Coverage path planning** (`src/coverage_path_planner.cpp`):
  - **Polygon decomposition**: cleans, validates, and decomposes complex/concave regions into sub-regions, generating coverage scan lines for each;
  - **OBB sweep direction**: determines the optimal sweep direction with the OBB algorithm and generates equally spaced coverage lines along it;
  - **Dubins turns**: connects adjacent scan lines with Dubins-style turn curves whose radius respects the Ackermann vehicle's minimum turning capability;
  - **DP region ordering**: generates candidate path variants for sub-region entries/exits and selects the order by connection cost; with no more than `exact_connection_limit` sub-regions it uses exact bitmask DP, degrading to greedy beyond that to avoid combinatorial explosion.
- **OSQP MPC path tracking**: a linearized MPC with state `[x, y, yaw]` and controls `[v, yaw_rate]`, solved by OSQP and output as `geometry_msgs/Twist`.
- **Ordered path projection prevents swath jumping**: arc length is built in message order, so the projection point only advances monotonically along the path and never goes back; a lateral deviation beyond `off_path_stop_distance` stops the car with `error:off_path`, preventing the vehicle from jumping to an adjacent coverage swath.
- **Dual-RTK + IMU EKF localization**: Gazebo poses drive simulated front and rear RTK antennas that output latitude/longitude, fused with the IMU by an EKF into the full `x/y/yaw/vx/vy` state, complete with engineering details such as baseline-length gating, RTK accuracy tiers, and stationary detection.
- **Multi-level localization fallback**: the MPC prefers `/ackermann_car/ekf_state`, falls back to `/ackermann_car/odom` on timeout, and then to `/odom`.
- **cmd_vel → tricycle control conversion**: `cmd_vel_to_tricycle.py` converts Twist to Gazebo tricycle-plugin control values via `steering_angle = atan(wheelbase * angular_z / linear_x)` and clamps them.
- **RViz + GUI dual visualization**: the Gazebo scene, RViz (including the sonar wedge marker), and the PyQt5 GUI display the vehicle position, planned path, and tracking status in sync.
- **One-click install/launch scripts**: `install.sh` takes care of dependency installation and the catkin build, and `run.sh` cleans up leftover Gazebo processes and starts the main entry in a new terminal.
- **gtest unit tests**: `test/test_coverage_path_planner.cpp` provides unit tests for the planning algorithm, run via `catkin run_tests`.

## Project Structure

```text
CovNav-Ackermann/
├── .gitignore                     # Git ignore rules (excludes catkin build artifacts)
├── README.md                      # Chinese README
├── README_en.md                   # English README
├── docs/                          # The two pipeline illustrations (pipeline.png, coverage_planner_pipeline.png)
├── install.sh                     # One-click install: apt/ROS deps + rosdep + catkin build
├── run.sh                         # One-click launch: clean leftover Gazebo processes, start main entry in a new terminal
├── PROJECT_HANDOFF.md             # Project handoff doc (theory, interfaces, troubleshooting, dev guide)
└── src/
    └── ackermann_car_gazebo/      # The single ROS package
        ├── launch/                # 6 launch files: GUI main entry / simulation / planner / EKF / MPC / RViz
        ├── config/
        │   └── coverage_navigation_params.yaml   # Unified parameter entry for all nodes
        ├── scripts/               # PyQt5 GUI, cmd_vel conversion, dual-RTK publisher, sonar marker
        ├── src/                   # C++ sources: coverage planner, planner service node, EKF, MPC
        ├── include/
        │   └── ackermann_car_gazebo/
        │       └── coverage_path_planner.hpp     # Coverage path planner header
        ├── msg/                   # RtkImuEkfState.msg custom message
        ├── srv/                   # PlanCoveragePath.srv planning service definition
        ├── test/                  # gtest unit tests
        ├── urdf/                  # Vehicle URDF (xacro)
        ├── worlds/                # Gazebo terrain worlds, autogen scripts, and the GUI background render
        ├── rviz/                  # RViz configuration
        ├── package.xml
        └── CMakeLists.txt
```

> Note: `build/`, `devel/`, `logs/`, and `.catkin_tools/` in the local workspace are catkin build artifacts, and `.catkin_workspace` is a tool-generated workspace marker file; all of them are excluded by `.gitignore` and never committed.

## Installation

### System Requirements

- Ubuntu 20.04
- ROS Noetic desktop edition (**must be installed beforehand** — `install.sh` only checks that `/opt/ros/noetic/setup.bash` exists and does not install ROS from scratch; see the [ROS Noetic on Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu) installation tutorial)
- Gazebo (installing it together with ROS Noetic is sufficient)
- A working `sudo` / `apt-get`

### Get the Code

```bash
git clone <repository-url>   # replace <repository-url> with this repository's actual URL
cd CovNav-Ackermann
```

### One-Click Install (install.sh)

Run from the workspace root:

```bash
./install.sh
```

The script performs the following steps in order:

1. Checks that `/opt/ros/noetic/setup.bash` and the `src/` directory exist and that `sudo`/`apt-get` are usable;
2. Runs `sudo apt-get update` and installs the apt/ROS dependencies (see the Dependencies section below for the list);
3. Initializes rosdep (`sudo rosdep init` if `/etc/ros/rosdep/sources.list.d/20-default.list` does not exist) and runs `rosdep update`;
4. Cleans old `build/ devel/ logs/` build artifacts (`catkin clean --workspace ... -y --force --all-profiles -b -d -L`);
5. Installs package dependencies via `rosdep install --from-paths src --ignore-src --rosdistro noetic -y`;
6. Builds the entire workspace with `catkin build --workspace ... --force-cmake`.

After the build completes, load the environment:

```bash
source devel/setup.bash
```

### Manual Installation

If you would rather not run the script, the equivalent commands are:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  python3-catkin-tools python3-rosdep python3-rospkg python3-pyqt5 \
  ros-noetic-osqp-vendor ros-noetic-osqp \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-hector-gazebo-plugins \
  ros-noetic-rviz ros-noetic-rviz-plugin-tutorials \
  ros-noetic-xacro ros-noetic-robot-state-publisher
# rosdep requires a one-time sudo rosdep init before first use
rosdep update
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin build
source devel/setup.bash
```

## Dependencies

**ROS packages** (declared in `package.xml`):

- **Build tools**: `catkin` (buildtool), `cmake_modules`
- **Build**: `eigen`, `rospy`, `roscpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `gazebo_msgs`, `message_filters`, `message_generation`, `urdf`, `osqp_vendor`
- **Run**: `rospy`, `roscpp`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `gazebo_msgs`, `message_filters`, `message_runtime`, `urdf`, `nav_msgs`, `robot_state_publisher`, `rviz`, `rviz_plugin_tutorials`, `xacro`, `gazebo_ros`, `gazebo_plugins`, `hector_gazebo_plugins`, `tf`, `python3-pyqt5`, `osqp_vendor`

**apt packages** (installed by `install.sh`):

- **System tools**: `build-essential`, `cmake`, `git`, `python3-catkin-tools`, `python3-rosdep`, `python3-rospkg`, `python3-pyqt5`
- **ROS**: `ros-noetic-osqp-vendor`, `ros-noetic-osqp`, `ros-noetic-gazebo-ros`, `ros-noetic-gazebo-plugins`, `ros-noetic-hector-gazebo-plugins`, `ros-noetic-rviz`, `ros-noetic-rviz-plugin-tutorials`, `ros-noetic-xacro`, `ros-noetic-robot-state-publisher`

Key points: OSQP is provided via `osqp_vendor` (the rosdep key, which maps to the apt packages `ros-noetic-osqp-vendor` + `ros-noetic-osqp`); the C++ side relies on Eigen for linear algebra; the GUI depends on the system `python3-pyqt5`.

## Quick Start

### One-Click Mode (run.sh)

```bash
./run.sh
```

`run.sh` first cleans up leftover Gazebo processes, then sources ROS and the workspace, and finally starts the main entry `roslaunch ackermann_car_gazebo coverage_navigation_gui.launch` in a supported new terminal (gnome-terminal / x-terminal-emulator / xfce4-terminal / konsole / xterm).

### Manual roslaunch

When no supported terminal emulator is available:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch
```

### Launching Individual Modules

There are 6 launch files in the `launch/` directory:

| Launch file | Purpose | Common parameters |
| --- | --- | --- |
| `coverage_navigation_gui.launch` | **Main entry**: loads the YAML and starts the planning service, the GUI, and the static map→odom TF | `start_planner` (default `true`), `start_gui` (default `true`), `config` (default `config/coverage_navigation_params.yaml`) |
| `gazebo_ackermann_car.launch` | Gazebo simulation: world, vehicle, sensors, EKF, RViz | `world_name`, `x/y/z/yaw`, `gui`, `rviz`, `start_ekf`, `sonar_max_range_cm`, etc. |
| `coverage_path_planner.launch` | Starts only the coverage planning service node | — |
| `mpc_path_tracker.launch` | Starts only the MPC tracking node | — |
| `dual_rtk_imu_ekf.launch` | Starts only the dual-RTK + IMU EKF node | `robot_namespace` (default `ackermann_car`) |
| `rviz_ackermann_car.launch` | Starts only RViz | `rviz_config`, `use_sim_time` |

```bash
roslaunch ackermann_car_gazebo coverage_navigation_gui.launch start_planner:=true start_gui:=true  # starts the planning service, GUI, and static map→odom TF (Gazebo simulation and MPC are not started)
roslaunch ackermann_car_gazebo gazebo_ackermann_car.launch      # start the simulation alone
roslaunch ackermann_car_gazebo mpc_path_tracker.launch          # start the MPC tracker alone
```

By default, running the main entry **only** brings up: the YAML parameter load, the planning service (`/coverage_path_planner`), the GUI, and the static map→odom TF; the Gazebo simulation and the MPC tracker are not among the main entry's switch arguments — they are started later by GUI buttons according to the `sim_launch` / `tracker_launch` settings in the YAML.

### GUI Workflow

1. Start the GUI.
2. Click the simulation start button in the GUI to launch Gazebo, the vehicle, RViz, and the localization nodes.
3. Draw the coverage region on the map.
4. Adjust planning parameters such as swath width, coverage ratio, and turn radius.
5. Click the plan button; the GUI calls `/coverage_path_planner/plan_coverage_path`.
6. After confirming the path is displayed correctly, click navigate.
7. The GUI publishes `/plan_path` and launches `mpc_path_tracker.launch` per the configuration.
8. `/mpc_controller` outputs `/ackermann_car/cmd_vel`, and the car starts tracking.
9. Watch `/tracking_status` in the status bar: normally it goes through `tracking` and changes to `arrived` once the goal is reached.

### Common Debugging Commands

```bash
rosnode list
rostopic info /plan_path
rostopic echo /tracking_status
rosservice info /coverage_path_planner/plan_coverage_path
rqt_graph
```

Run the unit tests:

```bash
catkin run_tests ackermann_car_gazebo
catkin_test_results
```

### Demo Screenshots

*The following are placeholder images — real screenshots are welcome after you run the project.*

![GUI demo](docs/gui.png)

![RViz demo](docs/rviz.png)

## System Architecture

### Control Chain (GUI region drawing → vehicle motion)

1. The PyQt5 GUI (base node name `coverage_navigation_gui`; the script initializes with `anonymous=True`, so the runtime node name gets a process-ID suffix appended, such as `/coverage_navigation_gui_12345`) loads the terrain map, where the user draws a polygonal coverage region and sets the planning parameters.
2. The GUI calls the service `/coverage_path_planner/plan_coverage_path` (type `ackermann_car_gazebo/PlanCoveragePath`).
3. The `/coverage_path_planner` node performs polygon decomposition, OBB sweep direction, Dubins turns, and DP ordering, and returns a `nav_msgs/Path` together with statistics.
4. The GUI publishes the path latched on `/plan_path` and starts `mpc_path_tracker.launch` per the YAML config (the node actually started is `/mpc_controller`).
5. `/mpc_controller` subscribes to `/plan_path` and the localization state, solves the OSQP MPC once per `control_rate` cycle (20 Hz), outputs `/ackermann_car/cmd_vel` (`geometry_msgs/Twist`), and publishes `/tracking_status` at the same time.
6. `/ackermann_car/cmd_vel_to_tricycle` converts linear/angular velocity into a front-wheel steering angle (`atan(wheelbase * angular_z / linear_x)`) and publishes `/ackermann_car/tricycle_cmd`.
7. The Gazebo tricycle plugin executes the command, the vehicle moves, and RViz and the GUI display the state.

### Coverage Path Generation Pipeline (inside the planner)

<div align="center">
  <img src="docs/coverage_planner_pipeline.png" alt="Coverage planner pipeline" width="1150">
</div>

<p align="center"><b>Figure 2</b> · Cartoon eight-station assembly line of the coverage planner: the input polygon is cleaned and validated, decomposed, oriented by OBB, turned into swaths, joined by Dubins turns, ordered by DP, and output as the final coverage path.</p>

### Localization Chain (simulation ground truth → EKF state)

1. `dual_rtk_publisher.py` subscribes to `/gazebo/model_states` and computes latitude/longitude/altitude for the two antennas from the front/rear antenna offsets (±0.30 m).
2. It publishes `/ackermann_car/rtk_front/fix` and `/ackermann_car/rtk_rear/fix` (`sensor_msgs/NavSatFix`, ENU origin defaults to Shanghai 31.230416, 121.473701).
3. `dual_rtk_imu_ekf_node` fuses the dual RTK with `/ackermann_car/imu/data` (`sensor_msgs/Imu`).
4. The EKF outputs `/ackermann_car/ekf_state` (custom type `ackermann_car_gazebo/RtkImuEkfState`, containing `x/y/yaw/vx/vy`).
5. The GUI and `/mpc_controller` consume this state; the MPC prefers `ekf_state`, falling back on timeout to `/ackermann_car/odom` and then `/odom`.

### Core Nodes

| Node | Source | Purpose |
| --- | --- | --- |
| `/coverage_navigation_gui_*` | `coverage_navigation_gui.py` | GUI, planning requests, path publishing, status display |
| `/coverage_path_planner` | `coverage_path_planner_node` | Coverage path planning service |
| `/mpc_controller` | `mpc_controller_node` | OSQP MPC path tracking |
| `/ackermann_car/cmd_vel_to_tricycle` | `cmd_vel_to_tricycle.py` | Twist to tricycle control conversion |
| `/ackermann_car/dual_rtk_publisher` | `dual_rtk_publisher.py` | Gazebo poses to dual RTK |
| `/ackermann_car/dual_rtk_imu_ekf` | `dual_rtk_imu_ekf_node` | Dual-RTK + IMU localization fusion |
| `/ackermann_car/sonar_range_marker` | `sonar_range_marker.py` | Sonar RViz Marker |

> Note: the GUI node is initialized with `anonymous=True` (`scripts/coverage_navigation_gui.py`), so its actual runtime node name gets a process-ID suffix appended (e.g. `/coverage_navigation_gui_12345`); `/coverage_navigation_gui_*` in the table refers to that base name, and the other node names are the actual names shown by `rosnode list`.

### Main Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/plan_path` | `nav_msgs/Path` | GUI → MPC, latched |
| `/ackermann_car/cmd_vel` | `geometry_msgs/Twist` | MPC → conversion node; the GUI stop button also publishes zero velocity |
| `/ackermann_car/tricycle_cmd` | `geometry_msgs/Twist` | Conversion node → Gazebo plugin |
| `/tracking_status` | `std_msgs/String` | MPC → GUI tracking status |
| `/ackermann_car/ekf_state` | `ackermann_car_gazebo/RtkImuEkfState` | EKF → GUI / MPC |
| `/ackermann_car/odom`, `/odom` | `nav_msgs/Odometry` | Fallback odometry |
| `/ackermann_car/rtk_front/fix`, `/ackermann_car/rtk_rear/fix` | `sensor_msgs/NavSatFix` | Simulated dual-RTK data |
| `/ackermann_car/imu/data` | `sensor_msgs/Imu` | IMU |
| `/ackermann_car/sonar/range` | `sensor_msgs/Range` | Sonar ranging (visualization only for now, not involved in control) |
| `/ackermann_car/sonar/marker` | `visualization_msgs/Marker` | Sonar wedge visualization |

### Services

`/coverage_path_planner/plan_coverage_path`, type `ackermann_car_gazebo/PlanCoveragePath`:

- **Request** core fields: `polygon` (`geometry_msgs/Point[]`, vertices of the coverage-region polygon), `use_robot_start` / `robot_start` / `use_robot_start_heading` (injecting the start pose), `scan_width`, `coverage_ratio`, `turn_radius`, `turn_step`, `start_connection_weight`;
- **Response** core fields: `success`, `message`, `path` (`nav_msgs/Path`), `num_lines`, `decomposition_count`, `line_spacing`, `total_path_cost`, `internal_path_cost`, `smooth_connection_cost`, `start_connection_cost`, `selected_variant_indices`, `connection_order`.

**Custom message** `ackermann_car_gazebo/RtkImuEkfState`: `header` + `float64 x/y/yaw/vx/vy` (2D pose in the map frame + planar velocity).

### tracking_status Values

| Status | Meaning |
| --- | --- |
| `idle` | No valid path, or an empty path was received |
| `tracking` | Currently tracking the path |
| `arrived` | Goal reached; the node publishes zero velocity |
| `error:no_state` | EKF/odom/fallback odom all timed out or are invalid |
| `error:off_path` | The vehicle's lateral deviation from the current ordered path exceeded the threshold |
| `error:optimization` | The OSQP solve did not succeed |

## Parameter Configuration

All node parameters are configured in one place, `src/ackermann_car_gazebo/config/coverage_navigation_params.yaml` (loaded only once, by `coverage_navigation_gui.launch`, and grouped by node name). The map and planning consistently use ENU coordinates (x=East, y=North, z=Up).

### Planning Parameters (coverage_path_planner)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `scan_width` | `1.0` | Swath width (m); multiplied by `coverage_ratio` to obtain the line spacing; a `scan_width > 0` in the GUI request overrides this default |
| `coverage_ratio` | `0.8` | Coverage ratio (0~1); the larger it is, the smaller the line spacing and the denser the scan lines |
| `turn_radius` | `2.0` | Dubins U-turn radius (m) |
| `turn_step` | `0.1` | Discretization step of the Dubins curves (m) |
| `straight_step` | `0.80` | Discretization step of the straight coverage segments (m) |
| `start_connection_weight` | `1.0` | Weight of the start-connection cost; 0 means it is not considered |
| `exact_connection_limit` | `16` | Maximum number of sub-regions for the exact bitmask DP; beyond this it degrades to greedy |
| `nonadjacent_penalty` | `10.0` | Penalty for connections between non-adjacent sub-regions |
| `frame_id` | `"map"` | Frame of the planned path |

> How the two work together: the line spacing is `line_spacing = scan_width × coverage_ratio` (default `1.0 × 0.8 = 0.80` m) — `scan_width` sets the base width, and `coverage_ratio` tightens the spacing proportionally.

### Tracking Parameters (mpc_controller)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `control_rate` | `20.0` | Control loop frequency (Hz) |
| `wheelbase` | `0.60` | Wheelbase (m); must match the URDF and the conversion node |
| `target_speed` | `0.80` | Target cruise speed (m/s) |
| `min_linear_velocity` / `max_linear_velocity` | `0.0` / `1.20` | Lower/upper linear velocity limits (m/s) |
| `goal_tolerance` | `0.35` | Distance threshold for considering the goal reached (m) |
| `state_timeout` | `0.75` | State message timeout (s); exceeding it publishes `error:no_state` |
| `max_steering_angle` | `0.60` | Maximum front-wheel steering angle (rad) |
| `max_angular_velocity` | `1.4` | Maximum angular velocity (rad/s) |
| `slowdown_distance` | `1.5` | Slow-down distance before the goal (m) |
| `off_path_stop_distance` | `1.80` | Lateral-deviation stop threshold (m); exceeding it publishes `error:off_path` |
| `lookahead_time` / `min_lookahead_dist` / `max_lookahead_dist` | `1.0` / `0.50` / `1.40` | Lookahead distance = clamp(\|v\|·lookahead_time, min, max) |
| `control_dt` | `0.10` | Discrete step of the prediction model (s) |
| `predict_time_domain` / `control_time_domain` | `10` / `4` | Prediction / control horizon step counts |
| `q_matrix_x` / `q_matrix_y` / `q_matrix_theta` | `12.0` / `12.0` / `3.0` | State error weights for [x, y, yaw] |
| `r_matrix_v` / `r_matrix_w` | `0.8` / `0.8` | Control weights for [v, yaw_rate] |
| `delta_linear_velocity_min` / `max` | `-0.20` / `0.12` | Per-step linear velocity increment constraints |
| `delta_angular_velocity_min` / `max` | `-0.30` / `0.30` | Per-step angular velocity increment constraints |

### Other Key Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `true` | In simulation, all nodes use the Gazebo `/clock`; on a real vehicle this is usually changed to `false` |
| `coverage_navigation_gui/render_path` | `package://ackermann_car_gazebo/worlds/render.png` | Terrain render image for the GUI |
| `coverage_navigation_gui/map_world_size_m` | `129.0` | Real-world side length covered by the terrain map (m) |
| `ackermann_car/cmd_vel_to_tricycle/wheelbase` | `0.60` | Must stay consistent with `wheelbase` on the MPC side |
| `ackermann_car/cmd_vel_to_tricycle/max_steering_angle` | `0.60` | Must stay consistent with the MPC side |
| `ackermann_car/dual_rtk_imu_ekf/rtk_position_std` | `0.03` | RTK position measurement standard deviation (m) |
| `ackermann_car/dual_rtk_imu_ekf/dual_rtk_yaw_std` | `0.03` | Dual-RTK heading measurement standard deviation (rad) |
| `ackermann_car/dual_rtk_imu_ekf/expected_rtk_baseline` | `0.60` | Expected front/rear RTK baseline length (m) |

## Notes

- **Simulation time**: `use_sim_time: true` globally, so all nodes use the Gazebo `/clock`; for a real vehicle this is usually changed to `false`.
- **wheelbase and max_steering_angle consistency**: the `wheelbase: 0.60` and `max_steering_angle: 0.60` values in both `mpc_controller` and `ackermann_car/cmd_vel_to_tricycle` must be kept consistent with each other and with the vehicle URDF, otherwise the car "does not turn or turns with the wrong radius".
- **Launch file name compatibility**: `mpc_path_tracker.launch` keeps this file name for compatibility with the older GUI entry; the node it actually starts is `mpc_controller_node` (node name `/mpc_controller`).
- **Leftover Gazebo processes**: after an abnormal exit, Gazebo may leave processes behind; you can clean them up manually (`run.sh` already does this built-in):

  ```bash
  pkill -TERM -x gzclient || true
  pkill -TERM -x gzserver || true
  pkill -TERM -x gazebo || true
  ```

- **Parameters are loaded in exactly one place**: in normal use, only `coverage_navigation_gui.launch` loads the YAML; the other launch files neither load it separately nor hard-code node parameters again. Therefore, when you start modules such as `gazebo_ackermann_car.launch` or `mpc_path_tracker.launch` **on their own**, the nodes fall back to their in-code defaults, which may differ from the YAML (for example, the planner's `turn_radius` defaults to `0.5` in code but is `2.0` in the YAML), so the behavior will not be exactly the same as in GUI mode; when debugging a single module, you can load the parameters manually before starting it:

  ```bash
  rosparam load src/ackermann_car_gazebo/config/coverage_navigation_params.yaml
  ```

- **Coordinate convention**: the map and planning consistently use ENU Cartesian coordinates (x=East, y=North, z=Up).
- **Sonar is visualization-only**: sonar data is currently only converted by `sonar_range_marker.py` into an RViz Marker for display; it is not involved in control or obstacle avoidance (obstacle avoidance is roadmap item 4).
- **Troubleshooting**: for common problems (`roslaunch` cannot find the package, the GUI map does not display, `error:no_state` / `error:off_path` / `error:optimization`, `install.sh` failures, etc.) see Chapter 9 of [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md).

## Roadmap

1. **Real-vehicle bag recording, replay & run videos**: record bags and run videos on the real car, build the record → replay → tune → retest loop, and add demo videos.
2. **Unified timestamps & ENU origin**: align time synchronization across sensors and chassis, and fix a single ENU origin to remove sim-vs-real time and coordinate discrepancies.
3. **Upload the host↔chassis Mbus communication code**: release the Mbus chassis-communication node, with timeout safe-stop and the `mbus/tx` / `mbus/rx` debug topics.
4. **Upload the camera & ultrasonic perception/avoidance code**: release the YOLO vision + ultrasonic blind-spot obstacle-avoidance module, with a traversable-class whitelist and safe-stop policy.

## Contributing

Improvements via Issues and Pull Requests are welcome: bug reports, documentation fixes, test cases, and parameter-tuning experience are all appreciated. Before submitting, it is recommended to first run `catkin build` and `catkin run_tests ackermann_car_gazebo` successfully.

## Acknowledgements

- The algorithmic core of the MPC path-tracking node is a rewrite based on the QP (quadratic programming) approach of the open-source **MPC local planner**, with the original project's `pluginlib`, `nav_core`, and `costmap_2d` dependencies removed and the code reworked into a standalone ROS node (the solver was switched to OSQP, and the model is `[x, y, yaw]` / `[v, yaw_rate]`).
- Thanks to [ROS](https://www.ros.org/), [Gazebo](https://gazebosim.org/), [OSQP](https://osqp.org/), [Eigen](https://eigen.tuxfamily.org/), [PyQt5](https://www.riverbankcomputing.com/software/pyqt/), and other open-source projects.

## License

This project uses the [MIT License](https://opensource.org/licenses/MIT) (see the `<license>` declaration in `src/ackermann_car_gazebo/package.xml`). The repository root does not yet include a full LICENSE file; it is recommended to add one before the official release.

## Support

- For usage questions and troubleshooting, please first consult [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md) (especially Chapter 9, "Common Problems and Troubleshooting") and the Notes section of this README;
- If your problem remains unsolved, feel free to open an Issue; attaching an `rqt_graph` screenshot and the `/tracking_status` output makes the problem much easier to locate;
- If this project helps you, please consider giving it a ⭐ Star!
