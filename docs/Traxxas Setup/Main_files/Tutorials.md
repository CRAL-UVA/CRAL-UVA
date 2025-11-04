
## 1. Initial Setup and Hardware Integration

##Power and Wiring
* **LiDAR Power:** LiPo battery powers LiDAR via 9-36V DC converter
* **RealSense Camera:** Connected via USB3 port on Jetson
* **Hokuyo UST-10LX:** Connected via 12V output and Ethernet to Jetson
* **VESC Controller:** Connected via USB serial port (`/dev/ttyACM1`)

**Important Wiring Notes:**
- Always verify polarity before connecting power
- Use appropriate voltage converters to protect sensitive components
- Ensure all connections are secure to prevent disconnection during operation


### 1.1 Lower Level Chassis
1. Disassembly - Removing Traxxas stack components
2. Setting up the Batteries 
3. 
### 1.2 Autonomy Elements
1. [NVIDIA Jetson setup](../Autonomy/jetson.md)
2. WiFi Antenna Setup 
3. [Joystick Setup](../Autonomy/Joystick.md)

### 1.3 Upper Level Stack
1. [VESC Soldering](../Upper_stack/vesc_wire_soldering.md)
2. [VESC Setup](../Upper_stack/vesc.md)
3. [LIDAR Setup](../sensors/lidar.md)
4. [REALSENSE Camera Setup](../sensors/realsense_camera.md)

<!-- ### 1.4 Putting it all together
1. Trial Run
   a. Launch components -->


<!-- ## 3. Launching the System

### 3.1 Basic Launch

1. **Navigate to workspace:**
   ```bash
   cd ~/ros2_ws  # or your workspace directory
   ```

2. **Source the workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

3. **Launch the base code:**
   ```bash
   ros2 launch base_code base_code.launch.py
   ```
   Or if using the f1tenth stack:
   ```bash
   cd f1tenth_system
   ros2 launch f1tenth_stack bringup_launch.py
   ```

### 3.2 Launch File Components

The launch file typically starts:
* **VESC Driver Node:** Handles motor control and IMU data
* **VESC to Odometry Node:** Converts VESC data to ROS odometry messages
* **Robot State Publisher:** Publishes robot transforms using URDF model

**Launch File Location:**
`/home/cral-traxxas/ros2_ws/src/base_code/launch`
Launch File Setup

* Custom launch file created to start: /home/cral-traxxas/ros2_ws/src/base_code/launch
    * VESC Driver
    * VESC to Odom Node
    * Robot State Publisher with URDF
* Before launching the file give permission to access the port - ttyACM1 is the port thru which the vesc is connected to the jetson. sudo chmod 777 /dev/ttyACM1
* or you will face this error 

[vesc_driver_node-1] [FATAL] [1757516613.565518522] [vesc_driver_node]: Failed to connect to the VESC, SerialException Failed to open the serial port /dev/ttyACM1 to the VESC. open: Permission denied failed..
[INFO] [vesc_driver_node-1]: process has finished cleanly [pid 5975]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[vesc_to_odom_node-2] [INFO] [1757516632.393750407] [rclcpp]: signal_handler(signal_value=2)

Testing:
* to test the motors run - ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0" 
* this is because the vescdrivernode is the one that publishes the /commands/motor/topic and the message is std_msgs/msg/Float64

Node name: vesc_driver_node
Node namespace: /
Topic type: std_msgs/msg/Float64
Endpoint type: SUBSCRIPTION
GID: 01.0f.aa.bc.86.17.08.97.01.00.00.00.00.00.19.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Joystick setup 
* the bluetoothctl is not working, have to debug for now the ds4drv driver is running to keep the joystick and the car connected  -->
