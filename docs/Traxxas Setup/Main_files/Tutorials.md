# Tutorials

## 1. Power On & Connect

1. Connect the drive LiPo (6S) and the compute/peripheral LiPo (3S, 11.1V) — verify polarity before connecting.
2. **Safety check:** elevate the vehicle or place it in a clear area before powering on motors. Keep hands/objects clear of wheels.
3. Wait 30–60 seconds for the Jetson Xavier NX to boot.
4. Connect your laptop to the robot's WiFi (see the robot's own page: [Traxxas UDR 01](../robots/traxxas_1.md) / [Traxxas UDR 02](../robots/traxxas_2.md)) and SSH in.

## 2. Bring Up the Full Stack

```bash
sudo chmod 777 /dev/ttyACM1   # required once per boot, or VESC connection fails with Permission denied
cd f1tenth_system
source /opt/ros/<ros2-distro>/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```
This starts the VESC driver, VESC-to-odometry node, and robot state publisher.

## 3. Manual Motor / Servo Test

```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0"
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 0.0"     # stop
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.85"
```
You should hear/see the wheels spin at the first command, and stop at the second.

## 4. Sensor Sanity Checks

```bash
ros2 topic echo /sensors/imu/raw | grep -A 3 "orientation:"
ros2 topic echo /scan
```

## 5. Joystick Test

```bash
ls /dev/input/js*          # confirm the kernel sees the joystick
sudo jstest /dev/input/js0 # test raw input
```
To identify the deadman switch, echo the `/joy` topic while pressing buttons. Note: Bluetooth pairing via `bluetoothctl` is currently unreliable — `ds4drv` is used as a workaround to keep the controller connected. See [Joystick Setup](../Autonomy/Joystick.md).

## 6. SLAM Mapping

See [Mapping](../Ros2_setup/Mapping.md) for the full walkthrough (launching `slam_toolbox`, viewing in RViz, saving the map). Quick reference:
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<path> use_sim_time:=false
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 7. Time Sync

If the system clock has drifted (common after being powered off for a while):
```bash
sudo timedatectl set-ntp true
```

## Command Cheat Sheet

| Task | Command |
|---|---|
| Fix VESC permission error | `sudo chmod 777 /dev/ttyACM1` |
| Bring up full stack | `ros2 launch f1tenth_stack bringup_launch.py` |
| Motor test | `ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0"` |
| Stop motor | `ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 0.0"` |
| Servo test | `ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.85"` |
| IMU check | `ros2 topic echo /sensors/imu/raw \| grep -A 3 "orientation:"` |
| LiDAR check | `ros2 topic echo /scan` |
| Joystick device check | `ls /dev/input/js*` |
| Joystick raw test | `sudo jstest /dev/input/js0` |
| Start SLAM | `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<path> use_sim_time:=false` |
| Save map | `ros2 run nav2_map_server map_saver_cli -f my_map` |
| Fix clock drift | `sudo timedatectl set-ntp true` |
| Restart ROS2 daemon (missing nodes) | `ros2 daemon stop && ros2 daemon start` |
