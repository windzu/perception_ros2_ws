# perception_ros2

## Overview

This repository contains ROS2 packages for perception tasks, including object detection, segmentation, and tracking. It leverages state-of-the-art models and is designed to work seamlessly with ROS2.

本仓库包含用于感知任务的 ROS2 软件包，包括一些基本的目标检测、分割和跟踪
其基于 autoware 进行构建，系统版本与主体的软件版本与 Jetpack 6.0 兼容

### Prerequisites

- Docker
- Nvidia Container Toolkit

## Installation

1. Clone the repository
2. Use the DevContainer feature to install the required dependencies and build the workspace(Use VSCode with DevContainer extension)
3. Import Some Autoware Packages
   ```bash
   cd /workspace/src
   vcs import < ../autoware.repos
   ```
4. Install dependencies
   ```bash
   cd /workspace/ansible
   ansible-playbook playbooks/install_spconv.yml
   ```
5. Build the workspace
   ```bash
   colcon build --symlink-install
   ```
6. Source the workspace
   ```bash
   source install/setup.bash
   ```

## Quick Start

1. Launch the perception node
   ```bash
   ros2 launch perception_ros2 perception_launch.py
   ```
2. Use RViz2 to visualize the results
   ```bash
   rviz2 -d `ros2 pkg prefix perception_ros2`/share/perception_ros2/rviz/perception.rviz
   ```

# Known Issues

- Unknown CMake command "add_launch_test".
  - Known Issues Nodes:
    - core/autoware_core/api/autoware_default_adapi
    - core/autoware_core/localization/autoware_ekf_localizer
    - core/autoware_core/localization/autoware_ndt_scan_matcher
    - core/autoware_core/map/autoware_lanelet2_map_visualizer
    - core/autoware_core/map/autoware_map_loader
    - core/autoware_core/map/autoware_map_projection_loader
  - Solution:
    - package.xml: add below lines to 'test_depend' section
      ```xml
      <test_depend>ament_cmake_test</test_depend>
      <test_depend>launch_testing_ament_cmake</test_depend>
      ```
    - CMakeLists.txt: add below lines to `BUILD_TESTING` section
      ```cmake
      find_package(ament_cmake_test REQUIRED)
      find_package(launch_testing_ament_cmake REQUIRED)
      ```
    - Reference:
      - https://github.com/orgs/autowarefoundation/discussions/6417
- /opt/hpcx is broken
  - Solution:
    ```bash
    sudo mv /opt/hpcx /opt/hpcx_BROKEN
    ```
