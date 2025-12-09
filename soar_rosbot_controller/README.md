# soar_rosbot_controller

Soar-based controller for ROSbot XL maze solving using cognitive architecture.

## Overview

This package implements a cognitive controller using the Soar cognitive architecture to solve mazes. It integrates with the perception nodes from `soar_rosbot_perception` to receive wall detection and ArUco target detection data, then uses Soar rules to make navigation decisions.

## Architecture

```
┌─────────────────────────┐
│  Perception Layer       │
│  (Python Nodes)         │
├─────────────────────────┤
│  - wall_detector        │  → /wall/front, /wall/left, /wall/right, /wall/back
│  - aruco_detector       │  → /aruco/detected, /aruco/distance
└─────────────────────────┘
           ↓
┌─────────────────────────┐
│  Soar Controller (C++)  │
│  soar_maze_controller   │
├─────────────────────────┤
│  Input-link:            │
│  - walls (front/left/   │
│    right/back)          │
│  - aruco (detected/     │
│    distance)            │
│                         │
│  Output-link:           │
│  - cmd_vel (linear-x/   │
│    angular-z)           │
└─────────────────────────┘
           ↓
┌─────────────────────────┐
│  Motor Control          │
│  /cmd_vel               │  → ROSbot XL
└─────────────────────────┘
```

## Soar Input-Link Structure

```
io
└── input-link
    ├── walls
    │   ├── front: 0/1
    │   ├── left: 0/1
    │   ├── right: 0/1
    │   └── back: 0/1
    └── aruco
        ├── detected: 0/1
        └── distance: float (meters)
```

## Soar Output-Link Structure

```
io
└── output-link
    └── cmd_vel
        ├── linear-x: float
        └── angular-z: float
```

## Maze Solving Algorithm

The implemented algorithm uses a **right-hand wall following** strategy:

1. **Stop at goal**: If ArUco marker is detected and within 0.5m, stop
2. **Approach goal**: If ArUco marker is detected but far, move forward slowly
3. **Turn right**: If no wall on right (prefer this to maintain right-hand rule)
4. **Move forward**: If no wall in front
5. **Turn left**: If wall in front and no wall on left
6. **Turn around**: If blocked on three sides

## Building

```bash
cd /Users/alvaro/soar_ws
colcon build --packages-select soar_rosbot_controller
source install/setup.bash
```

## Running

The controller is launched automatically with the main maze launch file:

```bash
ros2 launch soar_rosbot_perception soar_maze.launch.py
```

Or run standalone (requires perception nodes to be running):

```bash
ros2 run soar_rosbot_controller soar_maze_controller
```

## Soar Rules

The Soar rules are located in `soar_rules/`:
- `main.soar`: Entry point
- `maze_solver.soar`: Maze solving operators and rules

To modify the behavior, edit `maze_solver.soar` and rebuild the package.

## Dependencies

- `soar_ros`: ROS 2 interface for Soar cognitive architecture
- `rclcpp`: ROS 2 C++ client library
- `std_msgs`: Standard ROS messages
- `geometry_msgs`: Geometry messages (Twist for velocity commands)

## Topics

### Subscribed
- `/wall/front` (std_msgs/Bool): Front wall detection
- `/wall/left` (std_msgs/Bool): Left wall detection
- `/wall/right` (std_msgs/Bool): Right wall detection
- `/wall/back` (std_msgs/Bool): Back wall detection
- `/aruco/detected` (std_msgs/Bool): ArUco marker detected
- `/aruco/distance` (std_msgs/Float32): Distance to ArUco marker

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to robot

## Future Improvements

- Implement mapping to avoid revisiting areas
- Add more sophisticated path planning
- Tune velocity parameters for better performance
- Add emergency stop behaviors
