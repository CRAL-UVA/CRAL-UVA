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

For each robot, perform the following validation steps to ensure proper operation before proceeding to the next robot.

### 1. SLAM/Localization Verification

1. Launch SLAM and localization nodes:
   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
       slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml \
       use_sim_time:=false
   ```

2. Verify map generation is working correctly:
   ```bash
   ros2 topic echo /map --once
   ```

3. Confirm robot position is being tracked accurately:
   ```bash
   ros2 run tf2_ros tf2_echo /map /base_link
   ```

### 2. Navigation Stack Verification

1. Launch Nav2 navigation stack:
   ```bash
   ros2 launch nav2_bringup bringup_launch.py \
       map:=/home/administrator/clearpath_ws/lab_map.yaml \
       params_file:=/home/administrator/clearpath_ws/config/multi_nav2.yaml \
       use_sim_time:=false
   ```

2. Verify obstacle avoidance works with static obstacles:
   - Place obstacles in the environment
   - Command the robot to navigate around them
   - Observe avoidance behavior

3. Test path planning functionality:
   - Set goal positions using:
     ```bash
     ros2 topic pub /set_goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0}"
     ```
   - Verify path is generated and followed

### 3. Sensor Verification

1. Confirm LiDAR data is being published correctly:
   ```bash
   ros2 topic echo /ouster/points --once
   ros2 topic echo /scanner/scan --once
   ```

2. Verify camera odometry is available:
   ```bash
   ros2 topic echo /zed/odom --once
   ```

3. Check all sensor topics are active:
   ```bash
   ros2 topic list | grep -E "(ouster|zed|scanner)"
   ```

### 4. Control Verification

1. Test teleoperation mode:
   - Connect joystick and press Square button to enable teleop
   - Verify robot responds to joystick commands
   - Check that `/joy_teleop/cmd_vel` topic is publishing

2. Test autonomous navigation mode:
   - Press Triangle button to disable teleop (enable autonomous mode)
   - Set a goal position and verify robot navigates autonomously
   - Monitor `/controller/cmd_vel` topic

3. Verify emergency stop functionality:
   - Press X button to activate emergency stop
   - Confirm robot stops immediately
   - Press O button to deactivate emergency stop
   - Verify robot resumes operation

### Testing Procedure

#### Phase 1: Start the Robot System

1. Power on the robot following the procedure in the User Manual

2. Launch all required nodes:
   ```bash
   # Terminal 1: Navigation Stack
   ros2 launch nav2_bringup bringup_launch.py \
       map:=/home/administrator/clearpath_ws/lab_map.yaml \
       params_file:=/home/administrator/clearpath_ws/config/multi_nav2.yaml \
       use_sim_time:=false

   # Terminal 2: Point Cloud to Laser Scan
   ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py

   # Terminal 3: SLAM
   ros2 launch slam_toolbox online_async_launch.py \
       slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml \
       use_sim_time:=false
   ```

3. Verify all topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic hz /map
   ros2 topic hz /scanner/scan
   ros2 topic hz /zed/odom
   ```

#### Phase 2: Test Obstacle Avoidance

1. Place static obstacles in the environment at various locations

2. Command the robot to navigate around obstacles:
   ```bash
   ros2 topic pub /set_goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0}"
   ```

3. Verify the robot successfully avoids obstacles:
   - Monitor robot movement in real-time
   - Confirm path planning adjusts around obstacles
   - Verify robot reaches goal without collisions

4. Observe the robot's path planning behavior:
   - Visualize in Foxglove or RViz
   - Check that `/target_trajectory` shows appropriate paths

#### Phase 3: Test Path Planning

1. Set a goal position:
   ```bash
   ros2 topic pub /set_goal geometry_msgs/msg/Point "{x: 3.0, y: 1.0}"
   ```

2. Verify the planner generates a valid path:
   - Check that `/target_trajectory` topic is publishing:
     ```bash
     ros2 topic echo /target_trajectory --once
     ```
   - Visualize the path in Foxglove or RViz
   - Confirm path avoids known obstacles

3. Confirm the robot follows the planned path:
   - Monitor the robot's movement
   - Verify it reaches the goal position
   - Check that controller commands are being generated:
     ```bash
     ros2 topic echo /controller/cmd_vel
     ```

#### Phase 4: Shutdown and Prepare for Next Robot

1. Stop all running nodes:
   - Press `Ctrl+C` in each terminal running launch files
   - Or use: `pkill -f ros2`

2. Power down the robot:
   - Follow the shutdown procedure in the User Manual
   - Ensure proper battery disconnection

3. Prepare configuration for the next robot:
   - Update namespace if needed
   - Verify configuration files are correct
   - Check that robot-specific settings are applied

## Troubleshooting

This section covers common issues and their solutions. If you encounter problems not listed here, refer to the main [Troubleshooting Guide](../Main_files/Troubleshooting.md).

### Issue 1: Map Appears Circular or Distorted

**Symptoms:**
- Map visualization shows circular artifacts
- Map doesn't accurately represent the environment
- SLAM produces incorrect map geometry

**Diagnosis:**
1. Check if point cloud to laser scan conversion is running:
   ```bash
   ros2 topic list | grep scanner
   ros2 topic hz /scanner/scan
   ```

2. Verify `/scanner/scan` topic is publishing correctly:
   ```bash
   ros2 topic echo /scanner/scan --once
   ```

3. Check Ouster LiDAR status:
   ```bash
   ros2 topic echo /ouster/points --once
   ```

**Solutions:**
1. Ensure point cloud to laser scan converter is launched:
   ```bash
   ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
   ```

2. Verify Ouster LiDAR is properly configured and connected

3. Check that the package is in the correct location:
   ```bash
   ls -la /home/administrator/CRALAutonomyStack/src/pointcloud_to_laserscan
   ```

4. Restart the conversion node if it's not publishing

### Issue 2: Poor Localization Accuracy

**Symptoms:**
- Robot position drifts significantly
- Localization is inconsistent
- Map alignment is incorrect

**Diagnosis:**
1. Verify ZED camera odometry is available:
   ```bash
   ros2 topic echo /zed/odom --once
   ros2 topic hz /zed/odom
   ```

2. Check which odometry topic is being used:
   ```bash
   ros2 topic echo /platform/odom/filtered --once
   ```

3. Verify camera is properly calibrated and functioning

**Solutions:**
1. Ensure ZED camera driver is running:
   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py \
       camera_model:=zed2 \
       namespace:=<robot_namespace>
   ```

2. Verify `/zed/odom` topic is publishing at expected rate (should be > 10 Hz)

3. Check camera calibration if odometry seems incorrect

4. Restart the ZED camera node if odometry stops publishing

### Issue 3: Navigation Stack Not Responding

**Symptoms:**
- Robot doesn't respond to goal commands
- Path planning doesn't generate paths
- Navigation stack appears frozen

**Diagnosis:**
1. Verify all required topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic hz /map
   ros2 topic hz /voxel_grid/occupancy
   ros2 topic hz /platform/odom/filtered
   ```

2. Check that map file is accessible:
   ```bash
   ls -la /home/administrator/clearpath_ws/lab_map.yaml
   ```

3. Verify configuration files exist and are correct:
   ```bash
   cat /home/administrator/clearpath_ws/config/multi_nav2.yaml
   ```

**Solutions:**
1. Restart all navigation-related nodes:
   - Stop all launch files
   - Relaunch in the correct order (SLAM → Navigation → Perception)

2. Verify map file path is correct in launch command

3. Check configuration file syntax:
   ```bash
   ros2 param validate /nav2_bringup
   ```

4. Review node logs for errors:
   ```bash
   ros2 topic echo /rosout
   ```

### Issue 4: Emergency Stop Not Working

**Symptoms:**
- Emergency stop button doesn't stop the robot
- Robot continues moving after pressing X button
- No response from joystick emergency stop

**Diagnosis:**
1. Verify joystick is connected and publishing:
   ```bash
   ros2 topic echo /joy_teleop/joy --once
   ros2 topic hz /joy_teleop/joy
   ```

2. Check that autonomy node is running:
   ```bash
   ros2 node list | grep autonomy
   ```

3. Verify button mappings are correct:
   - X button should activate emergency stop
   - O button should deactivate emergency stop

**Solutions:**
1. Reconnect joystick:
   - Disconnect and reconnect USB cable
   - Press PS4 button to reconnect wirelessly
   - Wait for solid light indicator

2. Restart autonomy node if it's not running:
   ```bash
   # Find and restart the autonomy launch file
   ```

3. Verify joystick button mappings in the autonomy node configuration

4. Test joystick input directly:
   ```bash
   ros2 topic echo /joy_teleop/joy
   # Press buttons and verify messages are received
   ```

### Issue 5: Topics Not Publishing

**Symptoms:**
- Expected topics are missing from `ros2 topic list`
- Nodes appear to be running but topics are empty
- System seems unresponsive

**Diagnosis:**
1. Check if nodes are actually running:
   ```bash
   ros2 node list
   ```

2. Verify topics are being published:
   ```bash
   ros2 topic list
   ros2 topic hz <topic_name>
   ```

3. Check for node errors:
   ```bash
   ros2 topic echo /rosout
   ```

**Solutions:**
1. Restart the affected service:
   ```bash
   sudo systemctl restart clearpath-platform.service
   sudo systemctl restart clearpath-sensors.service
   sudo systemctl restart clearpath-custom.service
   ```

2. Reboot the robot if services fail to start:
   ```bash
   sudo reboot
   ```

3. Verify workspace is properly sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   source ~/CRALAutonomyStack/install/setup.bash
   ```

### Issue 6: SLAM Not Generating Map

**Symptoms:**
- `/map` topic is empty or not publishing
- Map doesn't update as robot moves
- SLAM node appears to be running but no output

**Diagnosis:**
1. Check SLAM node is running:
   ```bash
   ros2 node list | grep slam
   ```

2. Verify required input topics are available:
   ```bash
   ros2 topic hz /scanner/scan
   ros2 topic hz /zed/odom
   ```

3. Check SLAM configuration file:
   ```bash
   cat /home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml
   ```

**Solutions:**
1. Ensure all input topics are publishing:
   - Verify point cloud to laser scan is running
   - Check ZED camera odometry is available

2. Restart SLAM node with correct parameters:
   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
       slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml \
       use_sim_time:=false
   ```

3. Check SLAM logs for specific errors:
   ```bash
   ros2 topic echo /rosout | grep slam
   ```

## Additional Resources

- **Code Repository:** `https://github.com/CRAL-UVA/CRALAutonomyStack/tree/main/jackal{robot_serial_number}`
- **Point Cloud to Laser Scan Package:** `/home/administrator/CRALAutonomyStack/src`
- **Configuration Files:** `/home/administrator/clearpath_ws/config/`
- **Map Files:** `/home/administrator/clearpath_ws/lab_map.yaml`
