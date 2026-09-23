# Hardware Datasheet — CRAL Autonomy Stack

This page documents what CRAL adds to the stock Jackal platform described in the [Hardware Datasheet](Hardware_Datasheet.md). This configuration is common to both Jackal 93 and Jackal 96 unless a [robot-specific page](../robots/jackal_93.md) says otherwise.

## Compute

| | |
|---|---|
| **ROS 2 distro** | Humble |
| **Autonomy stack** | [CRALAutonomyStack](https://github.com/CRAL-UVA/CRALAutonomyStack) (GitHub), per-robot branch/folder: `jackal93` / `jackal96` |
| **Additional onboard GPU** | NVIDIA GPU installed in the electronics tray, in addition to the stock onboard PC |

## Sensors

| | |
|---|---|
| **RGB-D / stereo camera** | ZED (2-series); see [Sensor Drivers](../Setup_files/sensor_drivers.md). Also used for odometry (`/zed/odom`) since built-in Jackal odometry has limited accuracy |
| **3D LiDAR** | Ouster OS1-series; raw `/ouster/points` converted to a 2D scan via `pointcloud_to_laserscan` — the native `/ouster/scan` topic only reflects 1 of 128 channels. See [Demo 1: Single Robot Setup](../demos/single_robot_setup.md) for the known-issue writeup. |

## Control

| | |
|---|---|
| **Joystick** | PS4 controller — Square: teleop ON, Triangle: autonomous mode, X: e-stop ON, O: e-stop OFF |

## Networking

| | |
|---|---|
| **WiFi SSID** | NETGEAR52 |
| **Per-robot static IP / credentials** | See [Jackal 93](../robots/jackal_93.md) and [Jackal 96](../robots/jackal_96.md) |

## Handling & Wiring Safety Notes

- **Battery connector:** plug in ONLY the red and black connector, NOT the white connector — a connector has been fried doing this before.
- When unplugging the battery, pull firmly but carefully to avoid damaging the connector.
- To avoid sparking when reconnecting the charger: plug the charger into the wall, unplug it, wait for the charger light to turn off, plug the battery into the charger, then plug the charger back into the wall.
- The internal electronics tray (motherboard + GPU) is not meant to be accessed often — handle it with care, e.g. when connecting a monitor.
