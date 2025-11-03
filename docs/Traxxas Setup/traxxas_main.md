Robot Manual - Traxxas
Platform Overview

* Base Vehicle: Traxxas Unlimited Desert Racer (UDR)
* Compute Unit: NVIDIA Jetson Xavier NX
* Battery Power: 11.1V 3S LiPo powering all peripherals
* Sensors:
    * Hokuyo 10LX LiDAR
    * Intel RealSense Camera (D435i)
    * IMU from VESC
* Motor Controller: VESC (version with built-in IMU)

To Connect to the Car

ssh cral-traxxas@192.168.1.4
password - CRALRObOtics

Initial Setup and Hardware Integration

1. Jetson Xavier Flash and Setup

* JetPack flashed using SDK Manager (Linux Docker or Windows host)
* GUI login stuck issue resolved by completing setup via serial/debug terminal
* GUI login credentials recovered/reset

2. Power and Wiring

* LiPo battery directly powers LiDAR and Jetson via proper converters (9-36v DC convertor)
* RealSense camera connected via USB3
* Hokuyo UST-10LX connected via a 12v out and a ethernet to the jetson






1. Jetson Xavier setup :


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

* the bluetoothctl is not working, have to debug for now the ds4drv driver is running to keep the joystick and the car connected 

