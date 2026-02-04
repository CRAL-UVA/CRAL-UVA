How to create a map using Slam?

1. Connect to the robot via ssh 

2. Open a new terminal, source the following 
``` bash 
source /opt/ros/humble/setup.bash
```
3. Check all the topics and see its working 
``` bash 
ros2 topic list 
```
4. Check for /scan topic and see if it publishing messages 
``` bash 
ros2 topic echo /scan
```
5. Install the Nav2 packages using your operating system’s package manager:
``` bash 
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```
>[!NOTE]
> If you are gonna use turtlebot3 package for simulation
> Install the demo robot (Turtlebot) for gazebo:

> For Jazzy and newer, install the Turtlebot 3 & 4 packages for Gazebo Modern. It should be automatically installed with nav2_bringup:
``` bash
sudo apt install ros-<ros2-distro>-nav2-minimal-tb*
```
6. Terminal 2 – Launch SLAM Toolbox
``` bash 
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=/home/administrator/CRALAutonomyStack/config/mapper_params_online_async.yaml \
use_sim_time:=false
```
>[!NOTE]
> SLAM Toolbox will publish the map → odom transform.
7. Terminal 3 – View the map in RViz2
```bash
In RViz:
    * Add → Map
    * Set the topic to /map
    * You should now see the map being generated in real time.
```
> [!NOTE]
> When running RViz on your own computer, you do not need to SSH into the robot — you can connect 
> directly over the ROS 2 network as long as both machines are on the same network and have the correct ROS_DOMAIN_ID and environment variables configured.

8. Drive the robot to scan the area using the joystick, continue until a complete map is generated.

9. Save the map
``` bash 
ros2 run nav2_map_server map_saver_cli -f my_map
```
10. Replace my_map with your desired filename.

``` bash 
* If you run the command inside ~/CRALAutonomyStack, the following files will be created there:
    * my_map.pgm (map image file)
    * my_map.yaml (map metadata file)
```