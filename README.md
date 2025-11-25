# Unitree Go2 Simulation (go2_sim)

This repository contains the ROS 2 simulation packages for the Unitree Go2 quadruped robot. It provides a complete simulation environment with Gazebo, including mapping and navigation capabilities.

## Features
- **Gazebo Simulation**: Realistic physics simulation of the Unitree Go2.
- **Navigation Stack (Nav2)**: Fully configured navigation stack for autonomous movement.
- **SLAM**: Mapping capabilities using `slam_toolbox` or `cartographer`.
- **LiDAR Support**: Simulation of the Unitree 4D LiDAR.

## Installation

1.  Clone this repository into your ROS 2 workspace `src` folder:
    ```bash
    cd ~/ros2_ws/src
    git clone <repository_url> go2_sim
    ```

2.  Install dependencies:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  Build the workspace:
    ```bash
    colcon build --symlink-install
    ```

4.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Usage

### Launch Simulation & Navigation
To launch the simulation with the navigation stack:
```bash
ros2 launch unitree_go2_sim unitree_go2_nav.launch.py map:=/path/to/your/map.yaml
```

### Launch SLAM (Mapping)
To launch the simulation and start mapping:
```bash
ros2 launch unitree_go2_sim unitree_go2_slam.launch.py
```

## Packages
- `unitree_go2_sim`: Main simulation launch files and configurations.
- `unitree_go2_description`: URDF and mesh files for the robot.
- `champ`: Quadruped controller framework.
