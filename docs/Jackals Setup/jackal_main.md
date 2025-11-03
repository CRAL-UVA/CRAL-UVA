# Turning the Jackal on and off

## Turning on procedure

1. Unplug the battery from the wall
    1. Unplug the battery from the charger first, and the charger from the wall second
2. Open the Jackal lid
    1. You need lower the lever at the front of the Jackal
3. Plug the battery into the Jackal
    1. IMPORTANT: Make sure you only plug in the red and back connector and NOT the white connector
        1. We've fried a connector doing this
4. Press the power button on the back of the Jackal
5. Turn off the motor
    1. It's good practice to turn off the motor unless you explicitly need it on, because we don't want to accidentally crash the Jackal (they're expensive)
    2. You can do this by pressing the "M" button close to the power button
        1. Light on = motor on
        2. Light off  = motor off

## Turning off procedure

1. Turn Jackal off
    1. Press the power button on the back of the Jackal
    2. Wait for the light to turn off
2. Unplug the battery from the Jackal
    1. You need to pull on the connector a bit hard, just be careful not to break anything
3. Plug the battery into the wall
    1. It used to spark when you did this, and the best way I've found to do that is as follows:
        1. Plug the charger into the wall
        2. Unplug the charger from the wall
        3. Wait for the light on the charger to turn off
        4. Plug the battery into the charger
        5. Plug the charger into the wall

## Connecting to the Jackal

### Wireless SSH

1. Turn the Jackal on and wait for it to connect to WiFi (you'll see the WiFi light turn on)
2. Connect laptop to "NETGEAR52"
    1. SSID: "NETGEAR52"
    2. Password: "oddbird088"
3. Run ssh administrator@192.168.1.2
    1. Password: "clearpath"
    2.  192.168.1.2 is the static IP of the J100-0893 Jackal. 
    3. 192.168.1.3 is the static IP of the J100-0896 Jackal

### Ethernet SSH

Directions for this can be found in the user manual, which should be in the lab

Connecting to a monitor

1. Turn the Jackal on
2. Open up the tray with the electronics inside the Jackal
    1. You should see the motherboard and the Nvidia GPU
3. Connect the monitor's DisplayPort (or HDMI) cable, as well as the USBs for the mouse and keyboard
    1. Be careful doing this. The inner tray is not meant to be accessed often, and you could break something if you don't treat it with proper care and attention
4. You should see the login screen on the monitor
    1. User: "administrator"
    2. Password: "clearpath"



### Base Code Overview

GitHub repo: https://github.com/CRAL-UVA/CRALAutonomyStack

## Localization

We use platform/odom/filtered that comes with the robot. This just runs filtering on IMU data.


## SLAM 

### Overview 

Takes the 2D lidar points and the odometry data and creates a 2D occupancy map

### Inputs 

* Laserscan 
    * Type: sensor_msgs/msg/LaserScan 
    * Topic: /scanner/scan
* Odometry
    * Type: nav_msgs/msg/Odometry 
    * Topic: /zed/odom

### Output

* Map
    * Type: nav_msgs/msg/OccupancyGrid 
    * Topic: /map

To run the SLAM node - ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml use_sim_time:=false


## Issue faced - 

* The /ouster/scan topic from the Ouster sensor is only reflecting the wrong channel out of the 128 channel present in the sensor, so we had been seeing maps in circles 
* Used the pointcloudtolaserscan package and converted the point cloud to laserscan and this publishes the 

/scanner/scan  topic. 
Location of the package - /home/administrator/CRALAutonomyStack/src

* And also the inbuilt odom from the clearpath jackal is very bad, so we are using the odom from the zed camera 

/zed/odom

## Perception

### Overview

Takes lidar point cloud, filters floor points and points above the jackal, and creates an 2D occupancy grid

### Inputs

* Lidar point cloud
    * Type: sensor_msgs.msg.PointCloud2 
    * Topic: /ouster/points 

### Outputs

* Occupancy grid
    * Type: nav_msgs.msg.OccupancyGrid
    * Topic: /voxel_grid/occupancy



## Planner

### Overview

Given a goal position, ego position, and an occupancy grid, the planner adds a safety buffer to the occupancy grid, and runs A* to find a path to the goal position.

### Inputs

* Goal position
    * Type: geometry_msgs.msg.Point
    * Topic: /set_goal
* Occupancy grid
    * Type: nav_msgs.msg.OccupancyGrid
    * Topic: /voxel_grid/occupancy
* Ego odometry
    * Type: nav_msgs.msg.Odometry
    * Topic: /platform/odom/filtered

### Outputs

* Target trajectory
    * Type: sensor_msgs.msg.PointCloud2
    * Topic: /target_trajectory

## Controller

### Overview

Given ego pose and a target trajectory to follow, the controller uses pure pursuit to determine the command velocity to follow.

### Inputs

* Target trajectory
    * Type: sensor_msgs.msg.PointCloud2
    * Topic: /target_trajectory
* Ego odometry
    * Type: nav_msgs.msg.Odometry
    * Topic: /platform/odom/filtered

### Outputs

* Command velocity
    * Type: geometry_msgs.msg.Twist
    * Topic: /controller/cmd_vel

## Autonomy

### Overview

The autonomy node is essentially a mux between the joystick and controller command velocity. Which one it forwards is controller by buttons on the joystick. The autonomy node also has an emergency stop functionality that can be triggered with joystick buttons.

The joystick button mapping is as follows:

* Teleop ON (use joystick command velocity)
    * Square button
* Teleop OFF (use controller command velocity)
    * Triangle button
* Emergency Stop ON
    * X button
* Emergency Stop OFF
    * O button
* Latest test: To move the robot in teleop mode, hold on to just R1 for faster acceleration and L1 for slower acceleration.

### Inputs

* Joystick inputs
    * Type: sensor_msgs.msg.Joy
    * Topic: /joy_teleop/joy
* Command velocity from joystick
    * Type: geometry_msgs.msg.Twist
    * Topic: /joy_teleop/cmd_vel
* Command velocity from controller
    * Type: geometry_msgs.msg.Twist
    * Topic: /controller/cmd_vel

### Outputs

* Command velocity
    * Type: geometry_msgs.msg.Twist
    * Topic: /cmd_vel

## Known Bugs

* Occupancy grid transformation
    * In map_conversion_callback(), we transform the occupancy grid from ego frame to map frame. This transformation does not seem to be working properly.
    * To see this issue, run the base_code package, connect to foxglove, and visualize the occupancy grid from the /voxel_grid/occupancy/test topic. If you rotate the robot, the map orientation compared to ego orientation should get messed up.



## Running the Jackal

### Running the package

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

## Visualizing

### Foxglove

* Run foxbridge in a terminal window on the Jackal
* Open foxglove and connect to the foxglove bridge
    * If you're visualizing directly from the jackal, connect to ws://localhost:8765
    * If you're visualizing from another computer, connect to ws://192.168.1.2:8765 for the J100-0893 jackal or ws://192.168.1.3:8765 for the J100-0896 jackal.
* You can load jackal.json into foxglove for basic visualization

## Autonomy modes

Checking teleop mode:

* Run ros2 topic echo /teleop_enabled
    * true means teleop ON
    * false means teleop OFF

Checking emergency stop mode

* Run ros2 topic echo /e_stop
    * true means emergency stop is active
    * false means emergency stop is not active



## Common Issues

No odometry or no map_frame transform

* Try restarting the platform linux service
    * Run sudo systemctl restart clearpath-platform.service
    * Sudo password: "clearpath"
* Alternatively, you can also try rebooting the jackal
    * Run sudo reboot
    * Sudo password: "clearpath"
* This can be a bit finicky, and sometimes it takes a few tries for everything to start up properly

Sensors not publishing anything

* Try restarting the custom linux service
    * Run sudo systemctl restart clearpath-custom.service
    * Sudo password: "clearpath"
* Alternatively, you can also try rebooting the jackal
    * Run sudo reboot
    * Sudo password: "clearpath"
* This can be a bit finicky, and sometimes it takes a few tries for everything to start up properly

* If the screen turns yellow, follow the instructions mentioned here for enabling Wayland: https://askubuntu.com/questions/1404516/screen-turns-yellow-even-using-the-live-option-ubuntu-22-04

## Flashing Ubuntu 22 with ROS2 Humble


Follow the instructions here to Flash Ubuntu 22 and install ROS2: https://github.com/r-shima/jackal_ros2_humble?tab=readme-ov-file

* For the firmware flashing, follow the instructions here: https://docs.clearpathrobotics.com/docs/ros/installation/robot/


sudo apt install ros-humble-desktop-full

wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/clearpath_computer_installer.sh && bash -e clearpath_computer_installer.sh`


Connect to eduroam by following the instructions under automatic here: http://galileo.phys.virginia.edu/compfac/faq/linux-eduroam.html

Install openssh and setup the ssh client: https://www.linode.com/docs/guides/enable-ssh-ubuntu/

Connect the ps4 controller: https://docs.clearpathrobotics.com/docs/ros/installation/controller/

* You may need to restart your computer after connecting the controller


Install CUDA: https://www.cherryservers.com/blog/install-cuda-ubuntu


## Usage


Launching zed camera: ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 namespace:=j100_0893 
ZED camera setup info: https://www.stereolabs.com/docs/ros2

Launching Ouster LiDAR: ros2 launch ouster_ros driver.launch.py
https://github.com/ouster-lidar/ouster-ros/tree/ros2


## URDF Files

















