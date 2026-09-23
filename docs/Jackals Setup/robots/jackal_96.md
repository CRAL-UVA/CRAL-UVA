# Jackal 96 — Robot Reference

## Network & Login

| | |
|---|---|
| **Robot ID** | J100-0896 |
| **WiFi SSID** | cralrobotics | sometimes it also switched to CRAL
| **SSH command** | `ssh administrator@cpr-j100-0896.local` |
| **Password** | clearpath |
| **ROS namespace** | `j100_0896` |
| **Foxglove bridge (remote)** | `ws://192.168.1.3:8765` ⚠️ **[VERIFY]** — documented separately from the SSH static IP above; confirm before relying on it |
| **Autonomy stack repo** | [github.com/CRAL-UVA/CRALAutonomyStack/tree/main/jackal96](https://github.com/CRAL-UVA/CRALAutonomyStack/tree/main/jackal96) |

## Initial steps 
Once you have been sshed into the robot, always restart the robot service once ,
``` bash 
sudo systemctl restart clearpath-robot.service
```

## Services

The autonomy/platform/sensor stack runs via Clearpath's systemd services. Sudo password: `clearpath`.

```bash
sudo systemctl status clearpath-platform.service
sudo systemctl status clearpath-sensors.service
sudo systemctl status clearpath-robot.service
```

If odometry, the map frame transform, or sensor topics stop working, restart the relevant service (see [Troubleshooting](../Main_files/Troubleshooting.md)):

```bash
sudo systemctl restart clearpath-platform.service   # no odometry / no map_frame transform
sudo systemctl restart clearpath-sensors.service    # sensors not publishing
```

This can be finicky — it sometimes takes a few tries, or a full `sudo reboot`, before everything starts up properly.

## Known Issues for This Vehicle

- Same Ouster LiDAR channel issue and ZED odometry dependency as documented in [Demo 1: Single Robot Setup](../demos/single_robot_setup.md) apply.
- Occupancy grid transform from ego frame to map frame does not appear to work correctly — see [Troubleshooting](../Main_files/Troubleshooting.md#1-ros).
