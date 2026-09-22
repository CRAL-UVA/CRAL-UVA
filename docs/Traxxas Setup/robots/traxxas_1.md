# Traxxas UDR 01 — Robot Reference

## Network & Login

| | |
|---|---|
| **WiFi SSID** | takeda |
| **SSH command** | `ssh cral-traxxas@192.168.4.3` |
| **Password** | cralrobots |
| **systemd service** | `traxxas_robot.service` |

## Initial Commands — run this set right after booting it up

```bash
ssh cral-traxxas@192.168.4.3
source /opt/ros/foxy/setup.bash
source install/setup.bash
sudo systemctl restart traxxas_robot.service
```

## Autostart Service — 

The teleop/autonomy stack runs on Jetson boot via `traxxas_robot.service`.

```bash
sudo systemctl status traxxas_robot.service
sudo systemctl enable traxxas_robot.service
sudo systemctl start traxxas_robot.service
sudo systemctl disable traxxas_robot.service   # to stop autostart
```

## Manual Bring-Up (if not using the service)

```bash
cd f1tenth_system
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

## Manual Motor Testing

The topic responsible for sending signals to the motor is `/commands/motor/speed`.

```
Type: std_msgs/msg/Float64
Node name: vesc_driver_node
Node namespace: /
Topic type: std_msgs/msg/Float64
Endpoint type: SUBSCRIPTION
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
```

```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 3000.0"
```
You should hear or see the wheels start spinning.

To stop it immediately:
```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 0.0"
```
It should also return to zero once you stop publishing any speed to the motor.

## Joystick Testing

1. Confirm the kernel recognizes the joystick:
   ```bash
   ls /dev/input/js*
   ```
2. Test it using jstest:
   ```bash
   sudo jstest /dev/input/js0
   ```
3. To identify the deadman switch, echo the `/joy` topic and watch for changes while pressing buttons.

## Known Issues for This Vehicle

- RealSense motion module errors / poor image quality — kernel patch applied via compatible L4T fork, issue not fully resolved. See [Issues](../Issues/Issues.md).
- `urg_node_driver` "could not open serial device" for the Hokuyo LiDAR — check `~/f1tenth_system/f1tenth_stack/config/sensors.yaml` for a port/typo mismatch.
- VESC serial port can drift — if `/dev/ttyACM*` does not match `/ros2ws/src/vesc/vescdriver/params/vesc_config.yaml`, update the yaml to match.

> Older lab notes also list `ssh cral-traxxas@192.168.1.101` (or `192.168.1.4`), password `CRALRObOtics`, on WiFi `NETGEAR52`. The `192.168.4.x` / `takeda` credentials above are the current/confirmed set — the `192.168.1.x` set is superseded and kept here only for traceability.
