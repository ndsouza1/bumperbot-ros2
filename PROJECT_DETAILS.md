# BumperBot — Full Technical Reference

This document contains the complete technical breakdown of the BumperBot ROS 2 project. For the concise project overview, see [README.md](README.md).

---

## Table of Contents

1. [Project Phases](#1-project-phases)
2. [System Architecture](#2-system-architecture)
3. [Phase 1 — SLAM Mapping](#3-phase-1--slam-mapping)
   - [3.1 Robot Model](#31-robot-model)
   - [3.2 Odometry Estimation](#32-odometry-estimation)
   - [3.3 Custom Occupancy-Grid Mapper](#33-custom-occupancy-grid-mapper)
   - [3.4 SLAM Session (slam_toolbox)](#34-slam-session-slam_toolbox)
4. [Phase 2 — Autonomous Navigation](#4-phase-2--autonomous-navigation)
   - [4.1 Global Planning — SmacPlanner2D](#41-global-planning--smacplanner2d-nav2)
   - [4.2 Local Control — Regulated Pure Pursuit](#42-local-control--regulated-pure-pursuit-controller-nav2)
   - [4.3 Recovery Behaviours](#43-recovery-behaviours)
   - [4.4 BT Navigator](#44-bt-navigator)
5. [Custom Plugin Implementations](#5-custom-plugin-implementations)
   - [5.1 A* Planner Plugin](#51-a-planner-plugin)
   - [5.2 Dijkstra Planner Plugin](#52-dijkstra-planner-plugin)
   - [5.3 PD Motion Planner Plugin](#53-pd-motion-planner-plugin)
   - [5.4 Pure Pursuit Plugin](#54-pure-pursuit-plugin)
6. [Node Architecture & Communication](#6-node-architecture--communication)
   - [6.1 Topic Map](#61-topic-map)
   - [6.2 TF Tree](#62-tf-tree)
   - [6.3 Lifecycle Management](#63-lifecycle-management)
   - [6.4 Twist Mux — Velocity Arbitration](#64-twist-mux--velocity-arbitration)
7. [Algorithms Used](#7-algorithms-used)
8. [Costmap Configuration](#8-costmap-configuration)
9. [Package Structure](#9-package-structure)
10. [Dependencies](#10-dependencies)
11. [How to Build](#11-how-to-build)
12. [How to Run](#12-how-to-run)
    - [12.1 Mapping Session](#121-mapping-session)
    - [12.2 Autonomous Navigation](#122-autonomous-navigation)
    - [12.3 SLAM + Navigation Concurrently](#123-slam--navigation-concurrently)
    - [12.4 Custom Mapper Standalone](#124-custom-mapper-standalone)
    - [12.5 Switching Planner / Controller Plugins](#125-switching-planner--controller-plugins)
13. [Future Improvements](#13-future-improvements)

---

## 1. Project Phases

| Phase | Goal | Key Output |
|---|---|---|
| **Phase 1 – Mapping** | Build an accurate occupancy-grid map of an unknown environment | Saved `.pgm` + `.yaml` map files |
| **Phase 2 – Navigation** | Autonomously navigate to goal poses using the Nav2 stack | Velocity commands on `/bumperbot_controller/cmd_vel` |

Navigation is built as a **direct algorithmic extension** of the mapping system. The sensor pipeline, odometry model, TF tree, and costmap infrastructure established in Phase 1 form the foundation on which the Phase 2 navigation stack operates. Navigation adds goal management, path planning, and closed-loop trajectory tracking on top of the same low-level architecture.

The entire stack is written in **C++**, organised as modular ROS 2 packages, and designed for straightforward extension — new planners or controllers can be registered as `pluginlib` plugins without modifying the core navigation stack.

---

## 2. System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                         │
│   ┌───────────────┐    /scan     ┌──────────────────────────┐   │
│   │  bumperbot    │ ──────────► │   bumperbot_mapping /    │   │
│   │  (URDF/Xacro) │             │   slam_toolbox           │   │
│   │               │ ◄────────── └──────────────────────────┘   │
│   │  /joint_states│  /cmd_vel           │ /map                   │
│   └──────┬────────┘                     ▼                        │
│          │                  ┌──────────────────────────────┐     │
│          │ /odom            │      bumperbot_navigation     │     │
│          ▼                  │         (Nav2 stack)          │     │
│   ┌──────────────┐          │  ┌────────────────────────┐  │     │
│   │ SimpleCtrl   │ ◄──────  │  │    Planner Server      │  │     │
│   │ (odometry +  │ /cmd_vel │  │   (SmacPlanner2D)      │  │     │
│   │  TF broadcast│          │  └──────────┬─────────────┘  │     │
│   └──────────────┘          │             │ /plan           │     │
│                             │  ┌──────────▼─────────────┐  │     │
│                             │  │   Controller Server     │  │     │
│                             │  │ (RegulatedPurePursuit)  │  │     │
│                             │  └──────────┬─────────────┘  │     │
│                             │             │ /cmd_vel        │     │
│                             │  ┌──────────▼─────────────┐  │     │
│                             │  │  BT Navigator /         │  │     │
│                             │  │  Behavior Server        │  │     │
│                             └──┴─────────────────────────┴──┘     │
└──────────────────────────────────────────────────────────────────┘
```

> **Architecture diagram placeholder** — replace with `docs/architecture.png` once exported from draw.io or equivalent.

---

## 3. Phase 1 — SLAM Mapping

### 3.1 Robot Model

The robot is described as a **modular URDF/Xacro** model (`bumperbot_description`). The kinematic chain:

| Link | Joint Type | Notes |
|---|---|---|
| `base_footprint` | Virtual root | Ground contact reference frame |
| `base_link` | Fixed to `base_footprint` | Main chassis, mass ≈ 0.826 kg |
| `wheel_right_link` / `wheel_left_link` | Continuous | Radius 33 mm, effective separation 170 mm (controller config) |
| `caster_front_link` / `caster_rear_link` | Fixed (sphere collision) | Passive front/rear casters |
| `laser_link` | Fixed sensor frame | LiDAR mounted at +120.8 mm Z, rotated 180° |
| `imu_link` | Fixed sensor frame | IMU mounted at +69.9 mm Z on chassis top |

> **Note on wheel separation:** The URDF places wheel joints at ±70 mm from the chassis centreline (140 mm geometric separation). The `diff_drive_controller` config uses `wheel_separation: 0.17` (170 mm) as the effective value — this compensates for real-world slip and physical tolerances.

Differential-drive kinematics are handled by `ros2_control` with a `JointGroupVelocityController` (`simple_velocity_controller`), accepting per-wheel angular velocity commands on `/simple_velocity_controller/commands`.

### 3.2 Odometry Estimation

`bumperbot_controller/simple_controller` implements incremental odometry from joint encoder data (`/joint_states`):

```
d_s     = (r · Δθ_right + r · Δθ_left) / 2          # linear displacement
d_theta = (r · Δθ_right - r · Δθ_left) / wheelbase   # angular displacement

x     += d_s · cos(θ)
y     += d_s · sin(θ)
theta += d_theta
```

The current pose and corresponding `odom → base_footprint` TF transform are broadcast on every joint-state update via `tf2_ros::TransformBroadcaster`. Odometry is published on `/bumperbot_controller/odom`.

A probabilistic **odometry motion model** (`bumperbot_localization/odometry_motion_model`) propagates a configurable particle cloud (default: 300 samples) using zero-mean Gaussian noise parameterised by `(α1..α4)`. This provides an observable measure of dead-reckoning uncertainty over time.

### 3.3 Custom Occupancy-Grid Mapper

`bumperbot_mapping/mapping_with_known_poses` implements a **log-odds occupancy-grid mapper** from scratch in C++. The update cycle per LiDAR scan:

**1. TF lookup:**
```
odom → laser_link   (via tf2_ros::Buffer::lookupTransform)
```

**2. Polar to Cartesian conversion** — each beam is projected to world coordinates:
```
px = range · cos(angle + yaw) + robot_x
py = range · sin(angle + yaw) + robot_y
```

**3. Bresenham ray tracing** — the full Bresenham line algorithm traces from the robot's grid cell to each beam endpoint. Intermediate cells are marked `FREE`, the terminal cell `OCCUPIED`.

**4. Log-odds update** — per-cell occupancy belief updated in log-odds space:
```
L(cell) += log-odds(p_measurement) − log-odds(p_prior)
```
Where `p_prior = 0.5`, `p_free < 0.5`, `p_occupied > 0.5`.

**5. Publication** — every 1 second, the log-odds map is converted back to integer occupancy values `[0, 100]` and published as `nav_msgs/OccupancyGrid` on `/map`.

Configurable parameters: `width` (m), `height` (m), `resolution` (m/cell).

### 3.4 SLAM Session (slam_toolbox)

For the full SLAM workflow with loop closure and pose-graph optimisation, `bumperbot_mapping/slam.launch.py` brings up:

- **`slam_toolbox`** (synchronous mode) — consumes `/scan` + TF tree, outputs a continuously refined `/map` with loop-closure corrections.
- **`nav2_map_server/map_saver_server`** — persists the completed map to `.pgm` / `.yaml` on service call.
- **`nav2_lifecycle_manager`** — manages configure → activate lifecycle transitions for both nodes.

```
lifecycle_nodes: ["slam_toolbox", "map_saver_server"]
```

---

## 4. Phase 2 — Autonomous Navigation

The Nav2 stack is launched via `bumperbot_navigation/navigation.launch.py` with five managed server nodes, all coordinated by a lifecycle manager.

### 4.1 Global Planning — SmacPlanner2D (Nav2)

**Active plugin:** `nav2_smac_planner::SmacPlanner2D` (configured in `planner_server.yaml`).

This runs an optimised 2D A\*-based search over the global costmap. Key active parameters:

| Parameter | Value | Effect |
|---|---|---|
| `tolerance` | 0.125 m | Goal proximity threshold |
| `downsample_costmap` | false | Full-resolution planning |
| `allow_unknown` | true | Traversal through unexplored space |
| `max_planning_time` | 2.0 s | Planning timeout |
| `cost_travel_multiplier` | 2.0 | Steers paths toward aisle centres, away from inflation zones |
| `smoother.w_smooth` | 0.3 | Post-planning path smoothing weight |
| `smoother.w_data` | 0.2 | Data fidelity weight for smoother |

**Custom planner plugins (implemented, swappable):** `bumperbot_planning::AStarPlanner` and `bumperbot_planning::DijkstraPlanner` — see [Section 5](#5-custom-plugin-implementations).

### 4.2 Local Control — Regulated Pure Pursuit Controller (Nav2)

**Active plugin:** `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` (configured in `controller_server.yaml`).

| Parameter | Value | Effect |
|---|---|---|
| `desired_linear_vel` | 0.5 m/s | Nominal forward speed |
| `lookahead_dist` | 0.6 m | Base look-ahead distance |
| `min_lookahead_dist` | 0.3 m | Minimum at low speeds |
| `max_lookahead_dist` | 0.9 m | Maximum at high speeds |
| `use_rotate_to_heading` | true | In-place rotation to align with goal direction before driving |
| `rotate_to_heading_min_angle` | 0.785 rad (~45°) | Threshold to trigger heading rotation |
| `max_angular_accel` | 5.0 rad/s² | Angular acceleration cap |
| `use_regulated_linear_velocity_scaling` | true | Reduces speed on tight curves |
| `use_collision_detection` | true | Stops if forward path would collide within time horizon |

Output: `geometry_msgs/TwistStamped` remapped to `/bumperbot_controller/cmd_vel`.

**Custom controller plugins (implemented, swappable):** `bumperbot_motion::PDMotionPlanner` and `bumperbot_motion::PurePursuit` — see [Section 5](#5-custom-plugin-implementations).

### 4.3 Recovery Behaviours

When `bt_navigator` detects that the robot has made insufficient progress (`required_movement_radius: 0.1 m` within `movement_time_allowance: 20.0 s`), `behavior_server` triggers:

- **Spin** — in-place rotation to escape local minima or re-localise.
- **Back-Up** — short reverse translation to clear narrow passages.

Recovery velocity commands are remapped to `/bumperbot_controller/cmd_vel` alongside normal navigation.

### 4.4 BT Navigator

`nav2_bt_navigator` executes a behaviour tree that sequences:

```
compute path → follow path → [recover if stuck] → succeed / fail
```

The BT architecture decouples high-level mission management from low-level planner/controller decisions, and allows the recovery sequence to be customised by swapping BT XML files without code changes.

---

## 5. Custom Plugin Implementations

All four custom plugins are registered via `pluginlib` and can be activated at runtime through YAML configuration changes — no recompilation required.

### 5.1 A\* Planner Plugin

**File:** `bumperbot_planning/src/a_star_planner.cpp`  
**Interface:** `nav2_core::GlobalPlanner`  
**Registration:** `global_planner_plugins.xml`

Implementation details:
- **Graph representation:** 4-connected grid on the `nav2_costmap_2d::Costmap2D`.
- **Heuristic:** Manhattan distance — `h(n) = |Δx| + |Δy|`.
- **Priority queue:** `std::priority_queue` ordered by `f(n) = g(n) + h(n)`.
- **Edge cost:** `g(neighbor) = g(current) + 1 + costmap_cost(neighbor)`. Integrates real obstacle proximity cost into path cost.
- **Obstacle rejection:** Cells with `costmap_cost ≥ 99` (lethal obstacle + full inflation zone) are pruned.
- **Coordinate mapping:** `worldToGrid()` and `gridToWorld()` convert between world metres and costmap cells using origin and resolution.
- **Path smoothing:** The raw grid path is submitted as a `nav2_msgs::action::SmoothPath` goal to `smoother_server` (3 s timeout). The smoothed path is returned if the action succeeds, otherwise the raw path is used.

### 5.2 Dijkstra Planner Plugin

**File:** `bumperbot_planning/src/dijkstra_planner.cpp`  
**Interface:** `nav2_core::GlobalPlanner`

Uniform-cost search variant of the above — same grid representation and cost model, without the heuristic component (`h(n) = 0`). Useful as a reference baseline for comparing plan quality and planning time against A\*.

### 5.3 PD Motion Planner Plugin

**File:** `bumperbot_motion/src/pd_motion_planner.cpp`  
**Interface:** `nav2_core::Controller`  
**Registration:** `motion_planner_plugin.xml`

Control loop per cycle:

1. **Plan transformation** — the global path (in `map` frame) is transformed into the robot's current frame via `tf2::Buffer::lookupTransform`.
2. **Carrot selection** (`getNextPose()`) — iterates the path in reverse to find the farthest waypoint still beyond `step_size` (0.2 m) from the robot. Provides smooth look-ahead without oscillation near the goal.
3. **Error decomposition** — in the robot body frame, `x`-axis error drives linear velocity; `y`-axis error drives angular velocity.
4. **PD control law:**

```
v_linear  = clamp(kp · e_x  + kd · ė_x,  ±v_max)
v_angular = clamp(kp · e_y  + kd · ė_y,  ±ω_max)

Default: kp=2.0, kd=0.1, v_max=0.3 m/s, ω_max=1.0 rad/s
```

5. **Output:** `geometry_msgs/TwistStamped` on `/bumperbot_controller/cmd_vel`.

The derivative term `kd · ė` uses the elapsed time between consecutive control cycles (`clock_->now() - last_cycle_time_`) for proper temporal scaling.

### 5.4 Pure Pursuit Plugin

**File:** `bumperbot_motion/src/pure_pursuit.cpp`  
**Interface:** `nav2_core::Controller`

Geometric controller that steers toward a look-ahead point on the path. Alternative to the PD planner — useful for comparing tracking behaviour on curved paths.

---

## 6. Node Architecture & Communication

### 6.1 Topic Map

| Topic | Message Type | Producer | Consumer(s) |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo sim | `slam_toolbox`, `mapping_with_known_poses`, `obstacle_layer` |
| `/joint_states` | `sensor_msgs/JointState` | `ros2_control` | `simple_controller`, `robot_state_publisher` |
| `/bumperbot_controller/odom` | `nav_msgs/Odometry` | `simple_controller` | `odometry_motion_model`, Nav2 stack |
| `/tf` | `tf2_msgs/TFMessage` | `simple_controller`, `robot_state_publisher` | All nodes consuming transforms |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` / `map_server` | `global_costmap`, `rviz2` |
| `/plan` | `nav_msgs/Path` | `planner_server` | `controller_server`, `rviz2` |
| `/bumperbot_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `controller_server`, `behavior_server` | `simple_controller` |
| `/simple_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `simple_controller` | `ros2_control` hardware interface |
| `/odometry_motion_model/samples` | `geometry_msgs/PoseArray` | `odometry_motion_model` | `rviz2` (visualisation only) |
| `joy_vel` | `geometry_msgs/TwistStamped` | Joystick node | `twist_mux` |
| `key_vel` | `geometry_msgs/TwistStamped` | Keyboard teleop | `twist_mux` |

### 6.2 TF Tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── wheel_right_link
                ├── wheel_left_link
                ├── caster_front_link
                ├── caster_rear_link
                ├── imu_link
                └── laser_link
```

| Transform | Publisher | When |
|---|---|---|
| `map → odom` | `slam_toolbox` | During mapping (SLAM mode) |
| `map → odom` | `amcl` | During navigation (localisation mode) |
| `odom → base_footprint` | `simple_controller` | Always (encoder integration) |
| `base_footprint → *` | `robot_state_publisher` | Always (static URDF joints) |

### 6.3 Lifecycle Management

All Nav2 server nodes follow the ROS 2 managed node lifecycle:

```
Unconfigured → Inactive → Active → [Deactivating] → Inactive → Finalized
```

`nav2_lifecycle_manager` handles configure and activate transitions for:

```
["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
```

`autostart: true` means all nodes reach the Active state without any manual trigger.

### 6.4 Twist Mux — Velocity Arbitration

`twist_mux` arbitrates between velocity command sources by priority. Higher-priority sources preempt lower ones for as long as input is received. Topic selection lapses after a configurable timeout with no input.

| Source | Topic | Priority | Timeout |
|---|---|---|---|
| Joystick | `joy_vel` | 99 (highest) | 0.5 s |
| Keyboard | `key_vel` | 90 | 0.5 s |
| Nav2 navigation | `cmd_vel` | 80 (lowest) | 0.5 s |

This allows manual operator override during autonomous navigation — when joystick input stops, control returns to Nav2 automatically within 0.5 s.

---

## 7. Algorithms Used

| Algorithm | Source File | Status | Purpose |
|---|---|---|---|
| **SmacPlanner2D** (A\*-based) | `nav2_smac_planner` | **Active** | Global path planning on costmap |
| **Regulated Pure Pursuit** | `nav2_regulated_pure_pursuit_controller` | **Active** | Local path tracking with collision detection |
| **A\*** (4-connected grid) | `bumperbot_planning/a_star_planner.cpp` | Implemented, swappable | Custom global planner plugin |
| **Dijkstra** | `bumperbot_planning/dijkstra_planner.cpp` | Implemented, swappable | Custom global planner plugin |
| **PD Control** | `bumperbot_motion/pd_motion_planner.cpp` | Implemented, swappable | Custom local controller plugin |
| **Geometric Pure Pursuit** | `bumperbot_motion/pure_pursuit.cpp` | Implemented, swappable | Custom local controller plugin |
| **Bresenham Line Algorithm** | `bumperbot_mapping/mapping_with_known_poses.cpp` | Active | LiDAR ray tracing through occupancy grid |
| **Log-Odds Bayesian Update** | `bumperbot_mapping/mapping_with_known_poses.cpp` | Active | Probabilistic occupancy belief update |
| **Odometry Motion Model** | `bumperbot_localization/odometry_motion_model.cpp` | Active | Particle-based dead-reckoning uncertainty |
| **Differential Drive Kinematics** | `bumperbot_controller/simple_controller.cpp` | Active | Wheel ↔ body velocity conversion via Eigen matrix |
| **Kalman Filter** | `bumperbot_localization/kalman_filter.cpp` | Active | IMU / odometry sensor fusion |

---

## 8. Costmap Configuration

### Global Costmap (`planner_server.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `global_frame` | `map` | Planning in map coordinates |
| `resolution` | 0.05 m | 5 cm grid cells |
| `robot_radius` | 0.1 m | Inflation reference footprint |
| `track_unknown_space` | true | Plans through unknown regions |
| **Layers** | `static_layer`, `obstacle_layer`, `inflation_layer` | |
| `obstacle_layer.scan.topic` | `/scan` | Live LiDAR updates |
| `inflation_layer.inflation_radius` | 0.55 m | Safe clearance around obstacles |
| `inflation_layer.cost_scaling_factor` | 3.0 | Cost decay rate from obstacle boundary |

### Local Costmap (`controller_server.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `global_frame` | `odom` | Tracking in odometry coordinates |
| `rolling_window` | true | Window follows the robot |
| `width` / `height` | 3 m × 3 m | Local planning window size |
| `resolution` | 0.05 m | |
| **Layers** | `obstacle_layer`, `inflation_layer` | No static layer in local |

---

## 9. Package Structure

```
bumperbo_Ws/
├── snapshot_map_navigation/
│   ├── map_gz_rviz.png               # Mapping result snapshot
│   └── autonomous_navigation.mp4     # Navigation demo video
└── src/
    ├── bumperbot_bringup/            # Top-level launch files
    │   └── launch/
    │       ├── simulated_robot.launch.py   # Main simulation entry point
    │       └── real_robot.launch.py        # Hardware deployment entry point
    ├── bumperbot_description/        # Robot model
    │   ├── urdf/
    │   │   ├── bumperbot.urdf.xacro        # Main robot description
    │   │   ├── bumperbot_gazebo.xacro      # Gazebo sensor plugins
    │   │   └── bumperbot_ros2_control.xacro  # ros2_control hardware interface
    │   ├── meshes/                         # STL mesh files
    │   └── worlds/                         # Gazebo world files
    ├── bumperbot_controller/         # Low-level drive controller
    │   ├── config/
    │   │   ├── bumperbot_controllers.yaml  # ros2_control configuration
    │   │   └── twist_mux_topics.yaml       # Velocity source priorities
    │   └── src/
    │       ├── simple_controller.cpp       # Encoder odometry + TF + cmd_vel relay
    │       └── noisy_controller.cpp        # Odometry with configurable noise
    ├── bumperbot_localization/       # Localisation utilities
    │   └── src/
    │       ├── odometry_motion_model.cpp   # Probabilistic particle propagation
    │       ├── kalman_filter.cpp           # IMU/odometry fusion
    │       └── imu_republisher.cpp
    ├── bumperbot_mapping/            # Occupancy-grid mapping
    │   ├── config/slam_toolbox.yaml        # slam_toolbox parameters
    │   ├── maps/                           # Saved map files (.pgm + .yaml)
    │   └── src/
    │       └── mapping_with_known_poses.cpp  # Custom log-odds Bayesian mapper
    ├── bumperbot_planning/           # Global planner plugins
    │   ├── global_planner_plugins.xml      # pluginlib registration
    │   └── src/
    │       ├── a_star_planner.cpp          # A* (Manhattan heuristic, 4-connected)
    │       └── dijkstra_planner.cpp        # Dijkstra (uniform-cost)
    ├── bumperbot_motion/             # Local controller plugins
    │   ├── motion_planner_plugin.xml       # pluginlib registration
    │   └── src/
    │       ├── pd_motion_planner.cpp       # PD error-based path tracker
    │       └── pure_pursuit.cpp            # Geometric look-ahead controller
    ├── bumperbot_navigation/         # Nav2 configuration and launch
    │   ├── config/
    │   │   ├── planner_server.yaml         # Global planner + global costmap
    │   │   ├── controller_server.yaml      # Local controller + local costmap
    │   │   ├── behavior_server.yaml        # Recovery behaviour parameters
    │   │   ├── bt_navigator.yaml           # Behaviour tree configuration
    │   │   └── smoother_server.yaml        # Path smoother parameters
    │   └── launch/
    │       └── navigation.launch.py        # Nav2 stack bringup
    ├── bumperbot_msgs/               # Custom message/service definitions
    ├── bumperbot_utils/              # Utilities (safety stop, etc.)
    ├── bumperbot_firmware/           # Embedded firmware (real hardware)
    ├── bumperbot_cpp_examples/       # Standalone ROS 2 C++ examples
    ├── bumperbot_py_examples/        # Standalone ROS 2 Python examples
    └── twist_relay/                  # Twist topic relay utility
```

---

## 10. Dependencies

| Dependency | Version | Purpose |
|---|---|---|
| ROS 2 | Jazzy | Core middleware |
| Gazebo | Jazzy default | Physics simulation |
| `slam_toolbox` | ROS 2 Jazzy | Graph-based SLAM with loop closure |
| `nav2_*` | ROS 2 Jazzy | Full navigation stack |
| `ros2_control` | ROS 2 Jazzy | Hardware abstraction / controller manager |
| `tf2` / `tf2_ros` | ROS 2 Jazzy | Transform tree management |
| `Eigen3` | ≥ 3.3 | Matrix math for differential drive kinematics |
| `pluginlib` | ROS 2 Jazzy | Runtime plugin registration |

All ROS 2 dependencies are declared in each package's `package.xml` and resolved via `rosdep`.

---

## 11. How to Build

```bash
# 1. Clone the workspace
git clone <repo-url> bumperbo_Ws
cd bumperbo_Ws

# 2. Install system-level ROS 2 dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build all packages
colcon build --symlink-install

# 4. Source the workspace overlay
source install/setup.bash
```

> **Tip:** Add `source ~/bumperbo_Ws/install/setup.bash` to your `~/.bashrc` to avoid sourcing on every terminal session.

---

## 12. How to Run

### 12.1 Mapping Session

**Step 1** — Launch Gazebo, the robot, SLAM, and teleop:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true use_teleop:=true
```

This brings up: Gazebo → `ros2_control` → `simple_controller` (odometry + TF) → `slam_toolbox` → joystick teleop → RViz2 (pre-configured with `nav2_default_view.rviz`).

**Step 2** — Drive the robot. The `/map` topic updates in real time in RViz2.

**Step 3** — Save the map once coverage is satisfactory:

```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: /map, map_url: src/bumperbot_mapping/maps/my_map, \
    image_format: pgm, map_mode: trinary, free_thresh: 0.196, occupied_thresh: 0.65}"
```

---

### 12.2 Autonomous Navigation

The Nav2 stack is embedded inside the bringup launch file. A **single command** starts everything — Gazebo, the robot, localisation, Nav2, and RViz2:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

To select a specific world (default is `empty.world`):

```bash
# Available worlds: empty.world | small_house.world | small_warehouse.world
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house.world
```

> **RViz2 launches pre-configured** from `/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz`. All Nav2 panels (Global Costmap, Local Costmap, Path, Pose) are ready — no manual topic configuration required.

**Step 2** — Set the robot's initial pose using the **2D Pose Estimate** tool in RViz2.

**Step 3** — Place a **Nav2 Goal** in RViz2, or send one programmatically:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, \
    orientation: {w: 1.0}}}"
```

**Manual override during navigation:**

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_teleop:=true
```

Joystick input (priority 99) immediately overrides Nav2 commands (priority 80) via `twist_mux`. Control returns to Nav2 automatically after 0.5 s of inactivity.

---

### 12.3 SLAM + Navigation Concurrently

Run SLAM and Nav2 together in a single command — the robot builds the map while navigating to goals:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

---

### 12.4 Custom Mapper Standalone

Run the from-scratch log-odds mapper directly (without slam_toolbox):

```bash
ros2 run bumperbot_mapping mapping_with_known_poses \
  --ros-args -p width:=50.0 -p height:=50.0 -p resolution:=0.1
```

---

### 12.5 Switching Planner / Controller Plugins

**Activate the custom A\* planner** — edit `bumperbot_navigation/config/planner_server.yaml`:

```yaml
# Replace the SmacPlanner2D plugin:
GridBased:
  plugin: "bumperbot_planning::AStarPlanner"

# Or use Dijkstra:
# plugin: "bumperbot_planning::DijkstraPlanner"
```

**Activate the custom PD motion planner** — edit `bumperbot_navigation/config/controller_server.yaml`:

```yaml
# Replace RegulatedPurePursuitController:
FollowPath:
  plugin: "bumperbot_motion::PDMotionPlanner"
  kp: 2.0
  kd: 0.1
  max_linear_velocity: 0.3
  max_angular_velocity: 1.0
  step_size: 0.2

# Or the custom geometric Pure Pursuit:
# plugin: "bumperbot_motion::PurePursuit"
```

No recompilation required — `pluginlib` loads the selected implementation at runtime. All four custom plugins are pre-built and registered.

---

## 13. Future Improvements

- **Continuous SLAM localisation** — replace static-map AMCL with ongoing slam_toolbox for long-duration operation in dynamic environments.
- **Dynamic obstacle avoidance** — integrate the `nav2_costmap_2d` dynamic obstacle layer with real-time LiDAR updates into the local planner window.
- **3D perception** — add a depth camera to the robot model and fuse point-cloud data with the existing LiDAR-based costmap layers.
- **Multi-goal mission planning** — implement a waypoint sequencer that chains navigation goals with per-waypoint actions (stop, rotate, record).
- **Real hardware deployment** — validate the full software stack on physical hardware using `bumperbot_firmware` and `real_robot.launch.py`.
- **Kinematic path smoothing** — replace the `simple_smoother` post-processing step with a spline-based planner that satisfies curvature constraints natively.
- **Adaptive controller tuning** — replace fixed `kp`/`kd` gains in `PDMotionPlanner` with an auto-tuning mechanism that adapts to surface conditions estimated from wheel slip.
- **Custom BT mission trees** — author BT XML files beyond the Nav2 defaults to support complex mission profiles (conditional docking, room-by-room coverage).

---

*Built with ROS 2 Jazzy · Simulated in Gazebo · Implemented in C++*
