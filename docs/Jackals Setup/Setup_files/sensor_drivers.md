## 1. ZED camera ROS Driver 
### 1.1 Installation 
- Create a `ros2_ws` and clone the reposistory under the src folder `git clone https://github.com/stereolabs/zed-ros2-wrapper.git `
- Build the Package & source the workspace.

### 1.2 Testing 
Launching zed camera: ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 namespace:=j100_0893 
ZED camera setup info: https://www.stereolabs.com/docs/ros2

## 2. Ouster ROS Driver
### 2.1 Installation 
- Inside the same `ros2_ws/src`, clone the following reposistory ` git clone -b humble-devel https://github.com/ouster-lidar/ouster-ros.git `
- Build the Package & source the workspace.

### 2.2 Testing 
Launching Ouster LiDAR: ros2 launch ouster_ros driver.launch.py
https://github.com/ouster-lidar/ouster-ros/tree/ros2

[//]: # (Add the documentation of the launch commands to the clearpath-robot services)