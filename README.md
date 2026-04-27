# BIN-X (binx)

Complete simulatin of a mecanum-wheel robot that autonomously collects color-coded trash and deposits it in the correct bin. Built with ROS2 Jazzy and Gazebo Harmonic.

## Demo

The robot spawns at the origin of a 15m × 10m room. A random colored object (red, blue, or green) is spawned in front of it. The robot:

1. Navigates to object using Nav2
2. Identifies color with its onboard camera (OpenCV/HSV detection)
3. Navigates through zigzag maze to matching bin
4. Deposits trash in the right bin area

The world has three zones: a scatter of obstacles near the start, a three-wall zigzag maze in the middle, and a line of colored bins at the far end.
note: the spawned trash is placeholder location of where its position would be. Simulation of the robot arm is not included in here.

## Hardware model

| Property | Value |
|---|---|
| Drive | Mecanum (4-wheel omnidirectional) |
| Chassis | 1.0m × 0.75m × 0.3m |
| Wheel radius | 0.15m |
| LiDAR | 720-beam, ±180°, range 0.1–12m |
| Camera | 640×480, 80° FOV, 20Hz |

## Prerequisites

- ROS2 Jazzy
- Gazebo Harmonic
- Python 3

ROS2 packages:

```
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-image
ros-jazzy-robot-state-publisher
ros-jazzy-robot-localization
ros-jazzy-slam-toolbox
ros-jazzy-nav2-bringup
ros-jazzy-nav2-simple-commander
ros-jazzy-cv-bridge
python3-opencv
```

Install all at once with rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

note: because `config/` files are copied (not symlinked) by CMake's `install(DIRECTORY)`, you have to rebuild after editing any YAML in `src/pkg_binx/config/`.

## Running the trash collection demo

**Terminal 1 — launch the sim:**

```bash
source install/setup.bash
ros2 launch pkg_binx gz_sim.launch.py
```

Wait 15s for Gazebo to load and Nav2 to fully activate.

**Terminal 2 — run the collector:**

```bash
source install/setup.bash
ros2 run pkg_binx trash_collector.py
```

The robot will navigate autonomously from start to finish. Mission complete is logged when it reaches the bin. Each step executed in the mission is in the terminal.

## What the launch file starts automatically

| Delay | What starts |
|---|---|
| Immediately | Gazebo, robot, EKF, ROS-Gazebo bridges |
| 8 s | `spawn_trash.py` — drops a random colored box at `(2.0, 0.0)` |
| 10 s | Nav2 lifecycle manager — activates map server, AMCL, planner, controller, BT navigator |

AMCL is configured with `set_initial_pose: true` at `(0, 0, 0)`, so no manual pose estimate is needed in RViz.

## Building a new map

If you modify the world and need a new map:

```bash
ros2 launch pkg_binx slam_mapping.launch.py
```

Drive the robot manually (e.g. with `teleop_twist_keyboard`) to cover the environment. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/pkg_binx/maps/maze_map
```

Then rebuild so the new map is installed

## Architecture

```
Gazebo Harmonic
  ├── MecanumDrive plugin  ──►  /odom, /tf
  ├── JointStatePublisher  ──►  /joint_states
  ├── GPU LiDAR            ──►  /scan
  └── Camera               ──►  /camera/image

robot_localization (EKF)
  └── fuses /odom  ──►  smoothed odometry + odom→base_footprint TF

Nav2 stack
  ├── map_server     loads maze_map.pgm
  ├── amcl           localizes against map using /scan
  ├── planner        NavFn / A* global path
  ├── controller     Regulated Pure Pursuit local controller
  └── bt_navigator   orchestrates the above

trash_collector.py
  ├── ColorDetector node    /camera/image → HSV → red/blue/green
  └── BasicNavigator        sends goals to Nav2 → /cmd_vel → robot
```

## Package structure

```
src/pkg_binx/
├── config/
│   ├── ekf_params.yaml           # EKF sensor fusion config
│   ├── nav2_params.yaml          # Nav2 stack (AMCL, costmaps, controller)
│   └── slam_toolbox_params.yaml  # SLAM mapping config
├── description/
│   ├── binx.urdf.xacro           # Top level URDF assembler combining everything
│   ├── binx_core.xacro           # Chassis + 4 mecanum wheels
│   ├── camera.xacro              # Camera link and optical frame
│   ├── lidar.xacro               # LiDAR link
│   └── robot.gazebo              # Gazebo plugins (drive, sensors)
├── launch/
│   ├── gz_sim.launch.py          # Main launch (sim + Nav2 + EKF + trash)
│   ├── slam_mapping.launch.py    # Mapping-only launch
│   └── launch_binx.py            # Simple launch (no Nav2/old file for testing)
├── maps/
│   ├── maze_map.pgm              # Occupancy grid image
│   └── maze_map.yaml             # Map metadata (resolution, origin)
├── models/
│   ├── mecanum_wheels/           # STL meshes for wheel visuals
│   └── line_path_ground_plane/   # Textured ground for line-following worlds
├── parameters/
│   └── bridge_parameters.yaml   # ROS ↔ Gazebo topic bridges
├── scripts/
│   ├── trash_collector.py        # Autonomous mission node
│   ├── spawn_trash.py            # Spawns colored trash into Gazebo
│   └── lineFollower.py           # Camera-based line follower (earlier feature)
└── world/
    └── slam_world.sdf            # 3 zone world (obstacles, maze, bins)
```

## Known issues

- The installed `config/` YAML files are copies, not symlinks. Edit → rebuild to apply changes.
- The LiDAR visual mesh has no inertial properties (it is fixed to the chassis and does not need them, but URDF validators may warn).
