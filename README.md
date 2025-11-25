# Unitree Go2 Simulation (go2_sim)

This repository contains the ROS 2 simulation packages for the Unitree Go2 quadruped robot. It provides a complete simulation environment with Gazebo, including mapping and navigation capabilities.

## Features
- **Gazebo Simulation**: Realistic physics simulation of the Unitree Go2.
- **Navigation Stack (Nav2)**: Fully configured navigation stack for autonomous movement.
- **SLAM**: Mapping capabilities using `slam_toolbox` or `cartographer`.
- **LiDAR Support**: Simulation of the Unitree 4D LiDAR.

## Installation

1.  Navigate to your `unitree-sdk` directory (assuming it is in your workspace `src`):
    ```bash
    cd ~/ros2_ws/src/unitree-sdk
    ```

2.  Clone this repository as a submodule:
    ```bash
    git clone https://github.com/OpenMind/go2_sim.git
    ```

3.  Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  Build the workspace:
    ```bash
    colcon build --symlink-install
    ```

5.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Usage

### 1. Launch SLAM (Mapping)
To launch the simulation and start mapping the environment:
```bash
ros2 launch unitree_go2_sim unitree_go2_slam.launch.py
```
Drive the robot around using the teleop terminal to cover the entire area.

### 2. Save the Map
Once you are satisfied with the map, save it using the map saver CLI:
```bash
cd ~
ros2 run nav2_map_server map_saver_cli -f my_map
```
This will create `my_map.yaml` and `my_map.pgm` in your home directory.

### 3. Launch Navigation
To launch the simulation with the navigation stack using your saved map:
```bash
ros2 launch unitree_go2_sim unitree_go2_nav.launch.py map:=${HOME}/my_map.yaml
```
You can now set 2D Pose Estimates and Navigation Goals in RViz.

## Packages
- `unitree_go2_sim`: Main simulation launch files and configurations.
- `unitree_go2_description`: URDF and mesh files for the robot.
- `champ`: Quadruped controller framework.

## Acknowledgements
Special thanks to [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2) and its contributors. This repository is built upon their excellent work.
