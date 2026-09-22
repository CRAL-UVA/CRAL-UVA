# Traxxas UDR 02 — Robot Reference

## Network & Login

| | |
|---|---|
| **WiFi SSID** | takeda |
| **SSH command** | `ssh cral_traxxas2@192.168.4.4` |
| **Password** | cralrobots |
| **systemd service** | `traxxas_robot2.service` |

## Initial Commands — run this set right after booting it up

```bash
ssh cral_traxxas2@192.168.4.4
source /opt/ros/foxy/setup.bash
source install/setup.bash
sudo systemctl restart traxxas_robot2.service
```

## Autostart Service — always restart the service before working

The teleop/autonomy stack runs on Jetson boot via `traxxas_robot2.service`.

```bash
sudo systemctl status traxxas_robot2.service
sudo systemctl enable traxxas_robot2.service
sudo systemctl start traxxas_robot2.service
sudo systemctl disable traxxas_robot2.service   # to stop autostart
```

## Manual Bring-Up (if not using the service)

```bash
cd f1tenth_system
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

## Known Issues for This Vehicle

- Same VESC port-drift and LiDAR serial-connection checks as UDR 01 apply (see [Traxxas UDR 01](traxxas_1.md)) — verify `/dev/ttyACM*` and `sensors.yaml` if either device fails to connect.
- Joystick Bluetooth pairing unreliable via `bluetoothctl`; `ds4drv` driver used as the current workaround.
