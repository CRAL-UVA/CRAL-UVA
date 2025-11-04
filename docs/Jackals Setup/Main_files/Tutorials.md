# Jackal Tutorials 

## 1. Jackal Overview 
### 1.1 Introduction 
Jackal is a rugged, lightweight, fast and easy-to-use unmanned ground robot for rapid prototyping and research applications. These tutorials will assist you with setting up and operating your Jackal.

### 1.2 Topics 
You can view all topics that are active using `ros2 topic list`.

### 1.3 Software Setup 
[//]: # (TODO: Add PS4 controller setup link)
- [Install ROS 2](../Setup files/Ros_installation.md)
- [Sensor drivers](../Setup files/sensor_drivers.md)
- PS4 Joystick Controller

### 1.4 Using the Robot 
### 1.4.1 Running the Autonomy Package

1. Go to ros2 workspace folder
    1. Run cd ~/CRALAutonomyStack
2. Source workspace
    1. Run source /opt/ros/humble/setup.bash && source install/setup.bash
3. Launch package
    1. Run ros2 launch base_code base_code.launch.py
4. Set goal position
    1. In another terminal, run ros2 topic pub /set_goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0}"
5. Connect joystick and turn off teleop mode
    1. Press PS4 button on the joystick, and wait for the light on the back of the joystick to turn solid (usually turns red, but could turn white/light blue)
    2. Press the triangle button

### 1.4.2 Visualising - Foxglove
* Run foxbridge in a terminal window on the Jackal
* Open foxglove and connect to the foxglove bridge
    * If you're visualizing directly from the jackal, connect to ws://localhost:8765
    * If you're visualizing from another computer, connect to ws://192.168.1.2:8765 for the J100-0893 jackal or ws://192.168.1.3:8765 for the J100-0896 jackal.
* You can load jackal.json into foxglove for basic visualization

### 1.5 Multirobot Setup

### 1.5 Support 