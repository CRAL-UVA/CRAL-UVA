## 1. To connect to the car
**Make sure you are connected to the NETGEAR52 WiFi**
ssh cral-traxxas@192.168.1.101
password - CRALRObOtics

### Launch the bringup to get all the sensors running 
- cd f1tenth_system
`source /opt/ros/foxy/setup.bash`
`source install/setup.bash`
`ros2 launch f1tenth_stack bringup_launch.py`

## 2. Manual Motor testing
The topic responsible to send signals to the motor is `/commands/servo/position'

```
Type: std_msgs/msg/Float64

Publisher count: 1

Node name: ackermann_to_vesc_node
Node namespace: /
Topic type: std_msgs/msg/Float64
Endpoint type: PUBLISHER
GID: 01.0f.26.bc.82.1a.7e.95.01.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 1

Node name: vesc_driver_node
Node namespace: /
Topic type: std_msgs/msg/Float64
Endpoint type: SUBSCRIPTION
GID: 01.0f.26.bc.86.1a.8b.80.01.00.00.00.00.00.19.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

```
Run `ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 3000.0" `
 - You should hear or see the wheels start spinning.

 To stop it immediately run this command 
 `ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 0.0" `

 - But it should get to zero when you stop publishing any spped to the motor

## 2. Joystick Testing:
### 2.1 To test the joystick, follow these steps:
1. To see the kernel recognizes the joystick run 
`ls /dev/input/js*`

### 2.2 To test it using jstest:
`sudo jstest /dev/input/js0`

### 2.3 To identify the Deadman switch, echo the /joy topic and see 

