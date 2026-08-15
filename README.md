# Custom Global & Local Planner Integration

This repository contains a hybrid path planning system for ROS 1:
*   **Global Planner:** Multi-Layer Particle Swarm Optimization (MLPSO)
*   **Local Planner:** Artificial Potential Field (APF)

## Core Code Location
All C++ source files (`.cpp`) for both planners are located in:
`src/ros_sim/src/`

---

## How to Port and Enable the Planners

To use these planners in a new ROS package, you must copy and configure the following files:

### 1. Copy Source Files & XML Plugin
*   Copy all `.cpp` and `.h` files from `src/ros_sim/src/` into your new package.
*   Copy the plugin `.xml` registration file from the package root directory. This file is required so `move_base` can detect and load the custom planners.

### 2. Update `package.xml`
Add the standard ROS navigation dependencies along with the specific multi-robot dependency required by the global planner:
```xml
<build_depend>nav_core</build_depend>
<build_depend>costmap_2d</build_depend>
<build_depend>tuw_multi_robot</build_depend>
<exec_depend>tuw_multi_robot</exec_depend>
```

### Update `CMakeLists.txt`
Include `tuw_multi_robot` in your catkin components list to link the libraries properly during build:
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  geometry_msgs
  tuw_multi_robot
)
```
