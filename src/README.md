# Dual Lidar Robot Simulation

ROS2 robot simulation with dual lidar setup in Gazebo.

## Features
- Dual lidar sensors (front and rear) positioned diagonally on chassis
- Automatic lidar scan merging
- Pre-configured RViz visualization

## Requirements
- ROS2 Humble
- Gazebo
- Python 3

## Installation
```bash
# Create workspace
mkdir -p ~/duallidar_ws/src
cd ~/duallidar_ws/src

# Clone this repository
git clone https://github.com/beyondabhay21/sim_duallidar.git .

# Install dependencies
cd ~/duallidar_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Usage

### Launch Everything (Gazebo + Merger + RViz)
```bash
ros2 launch dual_lidar_complete.launch.py
```

### Launch Components Separately

**Gazebo only:**
```bash
ros2 launch articubot_one launch_sim.launch.py world:=src/articubot_one/worlds/world.world 
```

**Laser merger:**
```bash
ros2 launch articubot_one dual_lidar_merger.launch.py
```

**static_tf**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.175 0 0 0 base_link laser_merged
```

### Control the Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped
```

## Topics

- `/scan_front` - Front lidar data
- `/scan_rear` - Rear lidar data
- `/scan` - Merged lidar data
- `/diff_cont/cmd_vel_unstamped` - Differential drive controller

## Configuration

RViz configuration is saved in `articubot_one/config/dual_lidar.rviz` with pre-configured colors for front, rear, and merged scans.

## Packages

- **articubot_one**: Main robot description and launch files
- **dual_lidar_merger**: Custom node for merging dual lidar scans
