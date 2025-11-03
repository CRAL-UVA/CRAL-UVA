To connect to the car
ssh cral-traxxas@192.168.1.101
password - CRALRObOtics

## Launch the bringup to get all the sensors running 
cd f1tenth_system
ros2 launch f1tenth_stack bringup_launch.py





## To test the joystick, follow these steps:
1. To see the kernel recognizes the joystick run 
ls /dev/input/js*

2. To test it using jstest:
sudo jstest /dev/input/js0
3. To identify the Deadman switch, echo the /joy topic and see 