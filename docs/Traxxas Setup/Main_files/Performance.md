# Performance & Capability

## Speed

| | |
|---|---|
| **Stock rated top speed** | 30+ mph on 4S LiPo, 50+ mph on 6S LiPo (manufacturer rating, stock ESC/motor) |
| **CRAL configured ERPM limit** | ±100,000 ERPM (see [VESC Setup](../Upper_stack/vesc.md)) |
| **Actual top speed with VESC + current config** | ⚠️ **[NOT BENCHMARKED]** — not yet measured on either vehicle; the configuration raises the ceiling relative to stock but no field measurement is on record |

## Sensing & Autonomy Capability

- 2D obstacle detection and SLAM mapping via Hokuyo UST-10LX + `slam_toolbox` — see [Mapping](../Ros2_setup/Mapping.md)
- RGB-D perception (color + depth) via Intel RealSense D435i for close-range perception tasks
- Odometry / orientation estimation via VESC-internal IMU
- Autonomous path planning / navigation via Nav2 on top of a generated SLAM map
- Manual/teleop control via RC transmitter (PPM through VESC) or PS4/DS4-style joystick over ROS 2 (deadman switch on LB, right stick drive)

## Control Interfaces

| | |
|---|---|
| **Motor speed (direct)** | `/commands/motor/speed` (`std_msgs/msg/Float64`) |
| **Servo / steering (direct)** | `/commands/servo/position` (`std_msgs/msg/Float64`) |
| **Joystick input** | `/joy` topic; `ds4drv` driver used as a Bluetooth pairing workaround |

## Known Capability Limitations

- RealSense motion module errors and poor image quality reported; a kernel patch (compatible L4T fork) was applied but the issue is not fully resolved as of the latest notes — see [Issues](../Issues/Issues.md).
- Bluetooth joystick pairing is unreliable via `bluetoothctl`; `ds4drv` is used as a workaround to keep the controller connected.
- ROS 2 distro consistency between vehicles is unverified — see the Compute note in [Integration](Integration.md).
