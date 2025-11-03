## 1. ROS
### 1.1 Occupancy grid transformation
    * In map_conversion_callback(), we transform the occupancy grid from ego frame to map frame. This transformation does not seem to be working properly.
    * To see this issue, run the base_code package, connect to foxglove, and visualize the occupancy grid from the /voxel_grid/occupancy/test topic. If you rotate the robot, the map orientation compared to ego orientation should get messed up.

### 1.2 No odometry or no map_frame transform
* Try restarting the platform linux service
    * Run sudo systemctl restart clearpath-platform.service
    * Sudo password: "clearpath"
* Alternatively, you can also try rebooting the jackal
    * Run sudo reboot
    * Sudo password: "clearpath"
* This can be a bit finicky, and sometimes it takes a few tries for everything to start up properly

### 1.3 Sensors not publishing anything
* Try restarting the custom linux service
    * Run sudo systemctl restart clearpath-custom.service
    * Sudo password: "clearpath"
* Alternatively, you can also try rebooting the jackal
    * Run sudo reboot
    * Sudo password: "clearpath"
* This can be a bit finicky, and sometimes it takes a few tries for everything to start up properly

## 2. Ubuntu 
### 2.1 Screen turns yellow
* If the screen turns yellow, follow the instructions mentioned here for enabling Wayland: https://askubuntu.com/questions/1404516/screen-turns-yellow-even-using-the-live-option-ubuntu-22-04