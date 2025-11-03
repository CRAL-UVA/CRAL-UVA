Lidar setup - 

Connections :

1. Open the Network setting and add a new connection 
2. In the Ipv4 settings add the following:
    1. IP address - 192.168.0.15
    2. subnet mask is 255.255.255.0
3. call the connection Hokuyo  and ping the either 192.168.0.10 or 192.168.0.15
4. If that pings, bravo the lidar connection is succesfull

Installation of Ros2 Driver for the Hokuyo UST 10LX

1. The urg_node2 is the driver for the hokuyo lidar sensor 
2. Create a workspace and clone this repo :

git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git

1. Update the ros dependencies 

rosdep update
rosdep install -i --from-paths urg_node2

1. Build the package - colcon build
2. Run the command to start the drive and get the /scan topic running 

ros2 launch urg_node2 urg_node2.launch.py

Setting up Tf tree
