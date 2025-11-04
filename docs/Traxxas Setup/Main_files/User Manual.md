# Traxxas Robot User Manual

## 1. Introduction

The Traxxas Unlimited Desert Racer (UDR) is a high-performance RC platform modified for robotics research at CRAL. This vehicle serves as a cost-effective, high-speed testbed for autonomous navigation, motion planning, and control systems research. The platform features a Jetson Xavier NX compute unit, LiDAR, camera, and IMU sensors integrated for full autonomy capabilities.

## 2. Platform Overview

### 2.1 Hardware Specifications

* **Base Vehicle:** Traxxas Unlimited Desert Racer (UDR)
* **Compute Unit:** NVIDIA Jetson Xavier NX
* **Battery Power:** 11.1V 3S LiPo powering all peripherals
* **Motor Controller:** VESC (version with built-in IMU)

### 2.2 Sensor Suite

* **Hokuyo 10LX LiDAR:** 2D scanning LiDAR for obstacle detection and SLAM
* **Intel RealSense Camera (D435i):** RGB-D camera for visual perception and depth sensing
* **IMU:** Integrated Motion Unit from VESC for odometry and orientation estimation

## 3. Powering the Vehicle

### 3.1 Turning On

1. **Connect Battery:**
   - Ensure the LiPo battery is fully charged
   - Connect the battery to the vehicle's power system
   - Verify all peripheral connections are secure

2. **Power Sequence:**
   - The Jetson Xavier NX should automatically boot when power is connected
   - Wait for the system to fully initialize (typically 30-60 seconds)
   - Check for LED indicators showing system status

3. **Safety Check:**
   - **IMPORTANT:** Ensure the vehicle is elevated or in a safe area before powering on motors
   - Verify that the vehicle cannot accidentally drive off a surface
   - Keep hands and objects clear of wheels and moving parts

### 3.2 Turning Off

1. **Shutdown Procedure:**
   - Stop any running ROS 2 nodes or launch files (Ctrl+C in terminal)
   - Safely stop the vehicle if motors are active
   - Properly shut down the Jetson: `sudo shutdown now` or via SSH

2. **Power Disconnection:**
   - Wait for the Jetson to fully shut down before disconnecting power
   - Disconnect the battery carefully
   - Store the battery in a safe location (fire-resistant bag recommended)

## 4. Connecting to the Vehicle

### 4.1 SSH Connection

#### Wireless Connection
1. **Power on the vehicle** and wait for network initialization
2. **Connect to the same network** as the Traxxas vehicle (check network settings)
3. **SSH into the Jetson:**
   ```bash
   ssh cral-traxxas@192.168.1.4
   ```
4. **Enter password:** `CRALRObOtics`

**Note:** For Traxxas Vehicle 1, use IP `192.168.1.101` instead.

#### Ethernet Connection
1. Connect an Ethernet cable directly from your computer to the Jetson
2. Configure your network adapter with a static IP on the same subnet
3. SSH using the same command as above

### 4.2 Direct Monitor Connection

1. **Connect peripherals:**
   - Connect a monitor via HDMI or DisplayPort
   - Connect USB keyboard and mouse
2. **Login credentials:**
   - Username: `cral-traxxas`
   - Password: `CRALRObOtics`





