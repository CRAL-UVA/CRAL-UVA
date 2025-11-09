# Demo 1: Single Robot Setup

This guide covers the setup and operation of a single Clearpath Jackal robot for autonomous navigation, obstacle avoidance, and path planning. Each robot is run individually and sequentially, not simultaneously.

## Overview

**Goal:** Configure and run a single Jackal robot to demonstrate autonomous navigation capabilities including:
- Simultaneous Localization and Mapping (SLAM)
- Obstacle avoidance
- Path planning
- Localization

**Note:** This setup is designed for individual robot operation. Each robot should be tested and validated separately before moving to the next robot.

## Prerequisites

Before beginning, ensure the following:

- ROS 2 Humble is installed and configured
- Clearpath Jackal robot is properly configured
- All required sensors (ZED camera, Ouster LiDAR) are connected and functional
- Network connectivity is established
- Robot has appropriate permissions and access

## Preparation & Setup

### 1. System Preparation

1. **Update ROS 2 Stack:**
   - Ensure the robot has the latest ROS 2 stack installed
   - Verify all required packages are available

2. **Configuration Files:**
   - Verify configuration files are present and properly configured
   - Ensure map files are available and accessible

3. **Network Verification:**
   - Confirm network connectivity between the robot and control station
   - Verify ROS 2 discovery is working correctly

### 2. Robot-Specific Configuration

Each robot should have its own configuration based on its serial number:

- **GitHub Repository:** `https://github.com/CRAL-UVA/CRALAutonomyStack/tree/main/jackal{robot_serial_number}`
- Replace `{robot_serial_number}` with the specific robot's serial number (e.g., `jackal93` or `jackal96`)

## Launch Commands

### Navigation Stack

Launch the Nav2 navigation stack:

```bash
ros2 launch nav2_bringup bringup_launch.py \
    map:=/home/administrator/clearpath_ws/lab_map.yaml \
    params_file:=/home/administrator/clearpath_ws/config/multi_nav2.yaml \
    use_sim_time:=false
```

### Point Cloud to Laser Scan Conversion

Launch the point cloud to laser scan converter:

```bash
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
```

### SLAM

Launch the SLAM node:

```bash
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml \
    use_sim_time:=false
```

## System Architecture

The autonomy stack consists of several interconnected components:

### 1. Localization

**Overview:** Provides filtered odometry data for the robot's position estimation.

**Implementation:**
- Uses `platform/odom/filtered` topic from the Clearpath platform
- Applies filtering to IMU data for improved accuracy

**Note:** The built-in odometry from the Clearpath Jackal has limitations. For improved accuracy, the system uses odometry from the ZED camera (`/zed/odom`).

### 2. SLAM (Simultaneous Localization and Mapping)

**Overview:** Creates a 2D occupancy map of the environment using LiDAR data and odometry.

**Functionality:**
- Combines 2D LiDAR scan data with odometry information
- Generates a real-time occupancy grid map
- Performs simultaneous localization and mapping

**Inputs:**
- **Laser Scan**
  - Type: `sensor_msgs/msg/LaserScan`
  - Topic: `/scanner/scan`
- **Odometry**
  - Type: `nav_msgs/msg/Odometry`
  - Topic: `/zed/odom`

**Outputs:**
- **Map**
  - Type: `nav_msgs/msg/OccupancyGrid`
  - Topic: `/map`

**Known Issues and Solutions:**

1. **Ouster LiDAR Channel Issue:**
   - **Problem:** The `/ouster/scan` topic from the Ouster sensor only reflects one channel out of 128 available channels, resulting in circular map artifacts
   - **Solution:** Use the `pointcloud_to_laserscan` package to convert point cloud data to laser scan format
   - **Package Location:** `/home/administrator/CRALAutonomyStack/src`
   - **Output Topic:** `/scanner/scan`

2. **Odometry Quality:**
   - **Problem:** Built-in odometry from Clearpath Jackal has poor accuracy
   - **Solution:** Use ZED camera odometry (`/zed/odom`) for improved localization accuracy

### 3. Perception

**Overview:** Processes LiDAR point cloud data to create a filtered 2D occupancy grid for navigation.

**Functionality:**
- Filters floor points and points above the robot height
- Converts filtered point cloud to 2D occupancy grid
- Provides obstacle information for path planning

**Inputs:**
- **LiDAR Point Cloud**
  - Type: `sensor_msgs.msg.PointCloud2`
  - Topic: `/ouster/points`

**Outputs:**
- **Occupancy Grid**
  - Type: `nav_msgs.msg.OccupancyGrid`
  - Topic: `/voxel_grid/occupancy`

### 4. Planner

**Overview:** Computes a safe path from the current position to a goal position using the A* algorithm.

**Functionality:**
- Receives goal position and current ego position
- Applies safety buffer to the occupancy grid
- Executes A* pathfinding algorithm
- Generates target trajectory

**Inputs:**
- **Goal Position**
  - Type: `geometry_msgs.msg.Point`
  - Topic: `/set_goal`
- **Occupancy Grid**
  - Type: `nav_msgs.msg.OccupancyGrid`
  - Topic: `/voxel_grid/occupancy`
- **Ego Odometry**
  - Type: `nav_msgs.msg.Odometry`
  - Topic: `/platform/odom/filtered`

**Outputs:**
- **Target Trajectory**
  - Type: `sensor_msgs.msg.PointCloud2`
  - Topic: `/target_trajectory`

### 5. Controller

**Overview:** Generates velocity commands to follow the planned trajectory using pure pursuit control.

**Functionality:**
- Receives target trajectory and current ego pose
- Implements pure pursuit control algorithm
- Computes command velocities to track the trajectory

**Inputs:**
- **Target Trajectory**
  - Type: `sensor_msgs.msg.PointCloud2`
  - Topic: `/target_trajectory`
- **Ego Odometry**
  - Type: `nav_msgs.msg.Odometry`
  - Topic: `/platform/odom/filtered`

**Outputs:**
- **Command Velocity**
  - Type: `geometry_msgs.msg.Twist`
  - Topic: `/controller/cmd_vel`

### 6. Autonomy (Command Velocity Mux)

**Overview:** Manages the selection between teleoperation and autonomous control modes, with emergency stop functionality.

**Functionality:**
- Multiplexes between joystick (teleop) and controller (autonomous) command velocities
- Provides emergency stop capability
- Controlled via joystick button inputs

**Joystick Button Mapping:**

| Function | Button | Description |
|----------|--------|-------------|
| Teleop ON | Square | Enable joystick command velocity |
| Teleop OFF | Triangle | Enable controller command velocity (autonomous mode) |
| Emergency Stop ON | X | Activate emergency stop |
| Emergency Stop OFF | O | Deactivate emergency stop |
| Faster Acceleration | R1 (hold) | Increase acceleration in teleop mode |
| Slower Acceleration | L1 (hold) | Decrease acceleration in teleop mode |

**Inputs:**
- **Joystick Inputs**
  - Type: `sensor_msgs.msg.Joy`
  - Topic: `/joy_teleop/joy`
- **Command Velocity from Joystick**
  - Type: `geometry_msgs.msg.Twist`
  - Topic: `/joy_teleop/cmd_vel`
- **Command Velocity from Controller**
  - Type: `geometry_msgs.msg.Twist`
  - Topic: `/controller/cmd_vel`

**Outputs:**
- **Command Velocity**
  - Type: `geometry_msgs.msg.Twist`
  - Topic: `/cmd_vel`

## Validation Steps

### Individual Robot Validation

For each robot, perform the following validation steps:

1. **SLAM/Localization Verification:**
   - Launch SLAM and localization nodes
   - Verify map generation is working correctly
   - Confirm robot position is being tracked accurately

2. **Navigation Stack Verification:**
   - Launch Nav2 navigation stack
   - Verify obstacle avoidance works with static obstacles
   - Test path planning functionality

3. **Sensor Verification:**
   - Confirm LiDAR data is being published correctly
   - Verify camera odometry is available
   - Check all sensor topics are active

4. **Control Verification:**
   - Test teleoperation mode
   - Test autonomous navigation mode
   - Verify emergency stop functionality

### Testing Procedure

1. **Start the robot system:**
   - Power on the robot
   - Launch all required nodes
   - Verify all topics are publishing

2. **Test obstacle avoidance:**
   - Place static obstacles in the environment
   - Command the robot to navigate around obstacles
   - Verify the robot successfully avoids obstacles

3. **Test path planning:**
   - Set a goal position
   - Verify the planner generates a valid path
   - Confirm the robot follows the planned path

4. **Shutdown and prepare for next robot:**
   - Stop all running nodes
   - Power down the robot
   - Prepare configuration for the next robot

## Troubleshooting

### Common Issues

1. **Map appears circular or distorted:**
   - Verify point cloud to laser scan conversion is running
   - Check that `/scanner/scan` topic is publishing correctly
   - Ensure Ouster LiDAR is properly configured

2. **Poor localization accuracy:**
   - Verify ZED camera odometry (`/zed/odom`) is available
   - Check that odometry topic is being used instead of platform odometry
   - Confirm camera calibration is correct

3. **Navigation stack not responding:**
   - Verify all required topics are publishing
   - Check that map file is accessible
   - Ensure configuration files are correct

4. **Emergency stop not working:**
   - Verify joystick is connected and publishing
   - Check that autonomy node is running
   - Confirm button mappings are correct

## Additional Resources

- **Code Repository:** `https://github.com/CRAL-UVA/CRALAutonomyStack/tree/main/jackal{robot_serial_number}`
- **Point Cloud to Laser Scan Package:** `/home/administrator/CRALAutonomyStack/src`
- **Configuration Files:** `/home/administrator/clearpath_ws/config/`
- **Map Files:** `/home/administrator/clearpath_ws/lab_map.yaml`
