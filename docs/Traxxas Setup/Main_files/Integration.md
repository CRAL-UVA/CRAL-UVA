# Hardware Datasheet — CRAL Autonomy Stack

This page documents what CRAL adds to or replaces on the stock vehicle described in the [Hardware Datasheet](Hardware%20Datasheet.md). This configuration is common to both Traxxas UDR 01 and UDR 02 unless a [robot-specific page](../robots/traxxas_1.md) says otherwise.

## Compute

| | |
|---|---|
| **Compute unit** | NVIDIA Jetson Xavier NX |
| **OS / software** | JetPack (flashed via NVIDIA SDK Manager), ROS 2 |
| **ROS 2 distro** | Foxy |

## Motor Controller

| | |
|---|---|
| **Unit** | VESC 6 MkVI (replaces stock Velineon VXL-6s ESC) |
| **Connection to Jetson** | USB serial, typically `/dev/ttyACM1` |
| **Onboard IMU** | VESC-internal IMU, used for odometry/orientation estimation |
| **Wiring method** | VESC soldered directly to the stock VXL-6s motor pins (A-white, B-black, C-blue) rather than using a connector, to reduce points of failure — see [VESC Wire Soldering](../Upper_stack/vesc_wire_soldering.md) |
| **Full configuration values** | See [VESC Setup](../Upper_stack/vesc.md) |

## Sensors

| | |
|---|---|
| **2D LiDAR** | Hokuyo UST-10LX; 12V power input + Ethernet data to Jetson; ROS 2 driver: `urg_node2` — see [LiDAR Setup](../sensors/lidar.md) |
| **RGB-D camera** | Intel RealSense D435i; USB3 connection to Jetson; RGB + depth + IMU — see [RealSense Camera Setup](../sensors/realsense_camera.md) |
| **Orientation / IMU** | VESC-internal IMU (see Motor Controller, above) |

## Power

| | |
|---|---|
| **Drive battery** | 6S LiPo (powers motor via VESC). Configured cutoff: 3.4 V/cell start (20.4 V), 3.2 V/cell end (19.2 V) |
| **Compute / peripheral battery** | 11.1 V 3S LiPo, independent of drive battery — powers Jetson, LiDAR, camera |
| **LiDAR power** | Regulated from LiPo via 9–36 V DC converter |

## Networking

| | |
|---|---|
| **WiFi antennas** | Stock antennas swapped for larger antennas for wider coverage — see [Jetson Setup](../Autonomy/jetson.md) |
| **Per-robot SSID / IP / credentials** | See [Traxxas UDR 01](../robots/traxxas_1.md) and [Traxxas UDR 02](../robots/traxxas_2.md) |

## Wiring Safety Notes

- Always verify polarity before connecting power.
- Use appropriate voltage converters to protect sensitive components (e.g. LiDAR 9–36V DC converter).
- Ensure all connections are secure to prevent disconnection during operation.
- Add heat shrink **before** soldering VESC-to-motor connections, not after.
