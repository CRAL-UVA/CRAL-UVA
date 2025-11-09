# Sensor Drivers Setup

This guide covers the installation and configuration of ZED camera and Ouster LiDAR drivers for Clearpath robots.

## Prerequisites

- ROS 2 Humble installed
- Clearpath robot system configured
- Appropriate permissions to edit systemd services

## 1. ZED Camera ROS Driver

### 1.1 Installation

1. Create or navigate to your ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone the ZED ROS 2 wrapper repository:
   ```bash
   git clone https://github.com/stereolabs/zed-ros2-wrapper.git
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select zed_wrapper zed_components zed_interfaces
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 1.2 Testing

Launch the ZED camera with the following command:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2 \
    namespace:=<robot_namespace>
```

**Note:** Replace `<robot_namespace>` with your robot's namespace (e.g., `j100_0893`).

### 1.3 Additional Resources

- Official ZED camera ROS 2 documentation: https://www.stereolabs.com/docs/ros2

## 2. Ouster ROS Driver

### 2.1 Installation

1. Navigate to your ROS 2 workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Clone the Ouster ROS repository (Humble branch):
   ```bash
   git clone -b humble-devel https://github.com/ouster-lidar/ouster-ros.git
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ouster_ros
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 2.2 Testing

Launch the Ouster LiDAR driver:

```bash
ros2 launch ouster_ros driver.launch.py \
    sensor_hostname:=<sensor_ip> \
    lidar_port:=7502 \
    imu_port:=7503 \
    namespace:=<robot_namespace>
```

**Note:** 
- Replace `<sensor_ip>` with your LiDAR's IP address
- Replace `<robot_namespace>` with your robot's namespace

### 2.3 Additional Resources

- Ouster ROS repository: https://github.com/ouster-lidar/ouster-ros/tree/ros2

## 3. Integrating with Clearpath Services

To ensure the sensor drivers are properly sourced when Clearpath services start, you need to override the systemd service configurations.

### 3.1 Override Robot Service

1. Edit the robot service:
   ```bash
   sudo systemctl edit clearpath-robot.service
   ```

2. Add the following configuration:
   ```ini
   [Service]
   Environment="CLEARPATH_ROBOT_NAMESPACE=<robot_namespace>"
   ExecStart=
   ExecStart=/bin/bash -lc ' \
       source /opt/ros/humble/setup.bash && \
       source /home/administrator/clearpath_ws/install/setup.bash && \
       source ~/ros2_ws/install/setup.bash && \
       exec /usr/sbin/clearpath-robot-start \
   '
   ```

   **Note:** Replace `<robot_namespace>` with your robot's namespace.

### 3.2 Override Platform Service

1. Edit the platform service:
   ```bash
   sudo systemctl edit clearpath-platform.service
   ```

2. Add the following configuration:
   ```ini
   [Service]
   ExecStart=
   ExecStart=/bin/bash -lc ' \
       source /opt/ros/humble/setup.bash && \
       source /home/administrator/clearpath_ws/install/setup.bash && \
       source ~/ros2_ws/install/setup.bash && \
       exec /usr/sbin/clearpath-platform-start \
   '
   ```

### 3.3 Override Sensors Service

1. Edit the sensors service:
   ```bash
   sudo systemctl edit clearpath-sensors.service
   ```

2. Add the following configuration:
   ```ini
   [Service]
   ExecStart=
   ExecStart=/bin/bash -lc ' \
       source /opt/ros/humble/setup.bash && \
       source /home/administrator/clearpath_ws/install/setup.bash && \
       source ~/ros2_ws/install/setup.bash && \
       exec /usr/sbin/clearpath-sensors-start \
   '
   ```

### 3.4 Restart Services

After making these changes, restart the services:

```bash
sudo systemctl daemon-reload
sudo systemctl restart clearpath-robot.service
sudo systemctl restart clearpath-platform.service
sudo systemctl restart clearpath-sensors.service
```

## 4. Verification

1. Check that services are running:
   ```bash
   sudo systemctl status clearpath-robot.service
   sudo systemctl status clearpath-platform.service
   sudo systemctl status clearpath-sensors.service
   ```

2. Verify sensor topics are published:
   ```bash
   ros2 topic list | grep -E "(zed|ouster)"
   ```

3. Check TF frames:
   ```bash
   ros2 run tf2_ros tf2_echo /map /<robot_namespace>/base_link
   ```

## 5. Troubleshooting

### Services fail to start
- Verify all workspace paths are correct
- Check that all packages built successfully
- Review service logs: `sudo journalctl -u clearpath-robot.service -n 50`

### Sensor topics not appearing
- Ensure sensors are physically connected
- Verify sensor IP addresses and ports
- Check that launch files are being called correctly

### Namespace issues
- Confirm namespace is consistent across all configurations
- Verify environment variables are set correctly