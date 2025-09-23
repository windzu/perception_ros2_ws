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

1. Download Test Data :
   - [split_0.db3](https://drive.google.com/file/d/15-4uEJsEA4ktmRTRPixH8p4PPnRrDxLO/view?usp=drive_link)
   - Place it in `/workspace/autoware_data/test_20240930_134039` directory
   - Test Data Introduction: [Link](https://autowarefoundation.github.io/autoware-documentation/main/datasets/),raw bag size is about 60GB, i split it into 2GB files for easy download
2. Launch the perception node
   ```bash
   ros2 launch autoware_ground_segmentation scan_ground_filter.launch.xml
   ros2 launch autoware_euclidean_cluster euclidean_cluster.launch.xml
   ```
3. Play the test data
   ```bash
   ros2 bag play /workspace/autoware_data/test_20240930_134039/split_0.db3 --remap /pandar_points:=/sensing/lidar/concatenated/pointcloud --clock
   ```
4. Use RViz2 to visualize the results
   ```bash
   ros2 run rviz2 rviz2 -d /workspace/rviz/perception_demo.rviz
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
