## Traxxas
   - **GUI login stuck:** Complete setup via serial/debug terminal (`Ctrl+Alt+F2`) — see [User Manual](User%20Manual.md)
   - **Login credentials:** May need recovery/reset if forgotten
   - Reference troubleshooting guides if issues persist

## 1. Bring Up the Full Stack, if the service is not working or you are not able to restart the service

```bash
sudo chmod 777 /dev/ttyACM1   # required once per boot, or VESC connection fails with Permission denied
cd f1tenth_system
source /opt/ros/<ros2-distro>/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```
This starts the VESC driver, VESC-to-odometry node, and robot state publisher.
## 2. Manual Motor / Servo Test

```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0"
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 0.0"     # stop
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.85"
```
You should hear/see the wheels spin at the first command, and stop at the second.

## 3. Sensor Sanity Checks

```bash
ros2 topic echo /sensors/imu/raw | grep -A 3 "orientation:"
ros2 topic echo /scan
```

## 4. Joystick Test

```bash
ls /dev/input/js*          # confirm the kernel sees the joystick
sudo jstest /dev/input/js0 # test raw input
```
To identify the deadman switch, echo the `/joy` topic while pressing buttons. Note: Bluetooth pairing via `bluetoothctl` is currently unreliable — `ds4drv` is used as a workaround to keep the controller connected. See [Joystick Setup](../Autonomy/Joystick.md).

## 5. SLAM Mapping

See [Mapping](../Ros2_setup/Mapping.md) for the full walkthrough (launching `slam_toolbox`, viewing in RViz, saving the map). Quick reference:
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<path> use_sim_time:=false
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 6. Time Sync issue (especially in Traxxas UDR 01)

If the system clock has drifted (common after being powered off for a while):
```bash
sudo timedatectl set-ntp true
```
```bash
sudo timedatectl set-time "YYYY-MM-DD HH:MM:SS"
```
and verify it before moving to next step, 
```bash
timedatect
```

## VESC will not connect / firmware read fails
- Confirm the selected port actually belongs to the VESC.
- If using UART, confirm the port is enabled and wired correctly (RX→TX, TX→RX) at the correct baud rate.
- Confirm firmware version shown in VESC Tool matches the actual hardware version (Mk5 vs Mk6) — a mismatch causes hard-to-diagnose faults.

## Permission denied opening /dev/ttyACM1
```bash
sudo chmod 777 /dev/ttyACM1
```

## Hokuyo LiDAR not detected
- Check `~/f1tenth_system/f1tenth_stack/config/sensors.yaml` for a port typo.
- Confirm IPv4 config: local IP `192.168.0.15`, subnet `255.255.255.0`, and ping `192.168.0.10` or `192.168.0.15`.

## Wheel speed does not match VESC Tool reading
- Expected — VESC Tool reports ERPM, not wheel RPM. Apply the ~55.9 RPM/1000 ERPM factor and the 17.89:1 gear ratio (see [VESC Setup](../Upper_stack/vesc.md)).

## Map / RViz frames look wrong or map does not appear
- Publish a static transform: `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser`.
- Inspect the TF tree: `ros2 run tf2_tools view_frames.py`.

## `ros2 node list` shows nothing
- Retry with `ros2 node list --no-daemon`.
- If that reveals nodes, restart the daemon: `ros2 daemon stop` then `ros2 daemon start`.

## Joystick appears unresponsive
- Confirm axis mapping in `joy_teleop.yaml` (`~/f1tenth_system/f1tenth_stack/config`) matches the axes shown by `ros2 topic echo /joy`.
- Bluetooth pairing via `bluetoothctl` is currently unreliable; `ds4drv` is used as a workaround.


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