# Traxxas Robot Documentation

Welcome to the **Traxxas UDR Robot** documentation for CRAL-UVA!

## Introduction

The Traxxas Unlimited Desert Racer (UDR) is a high-performance RC platform repurposed as a research testbed for autonomous navigation, motion planning, and control at CRAL. Two units are maintained in the lab — **Traxxas UDR 01** and **Traxxas UDR 02**. Both share the same base vehicle and the same autonomy stack architecture; differences between them are limited to network identity, service names, and minor as-built variance, which are documented in each robot's own page.

This site covers the physical platform, its as-built hardware/sensor configuration, and the steps to power on, connect to, and bring up each robot. For a single printable reference covering the same material, see `Traxxas_UDR_Reference_Datasheet.docx` in the repo root.

## What the autonomy conversion adds to the stock vehicle

- Replaces the stock Velineon VXL-6s ESC with a **VESC 6 MkVI** motor controller, enabling closed-loop speed control, telemetry, and an onboard IMU for odometry.
- Adds an **NVIDIA Jetson Xavier NX** as the onboard compute unit running ROS 2.
- Adds a 2D LiDAR (**Hokuyo UST-10LX**) and an RGB-D camera (**Intel RealSense D435i**) for perception, mapping, and obstacle detection.
- Adds a dedicated 11.1V 3S LiPo to power the compute and sensor stack, independent of the main drive battery.
- Runs a full SLAM / navigation software stack (`slam_toolbox`, `Nav2`) on top of the f1tenth-derived driver stack.

**Data path:**
```
RC Transmitter / Joystick -> Jetson Xavier NX (ROS 2) -> VESC 6 MkVI -> Motor + Servo

Hokuyo UST-10LX (scan)   --\
Intel RealSense D435i    ---+--> Jetson Xavier NX (ROS 2) --> SLAM / Nav2 / Odometry
VESC internal IMU        --/
```

## Quick Links

- [Hardware Datasheet](Main_files/Hardware%20Datasheet.md) - Base vehicle specs (dimensions, gearing, suspension, stock electronics)
- [Integration](Main_files/Integration.md) - CRAL autonomy stack: compute, motor controller, sensors, power, wiring
- [Performance & Capability](Main_files/Performance.md) - Speed, sensing capability, control interfaces
- [User Manual](Main_files/User%20Manual.md) - Complete guide to operating the Traxxas vehicle
- [Tutorials](Main_files/Tutorials.md) - Step-by-step tutorials for common tasks
- [Troubleshooting](Main_files/Troubleshooting.md) - Common issues and solutions
- [Maintenance](Main_files/Maintanence.md)

## Platform Specifications

- **Base Vehicle:** Traxxas Unlimited Desert Racer (UDR)
- **Compute Unit:** NVIDIA Jetson Xavier NX
- **Sensors:** Hokuyo UST-10LX LiDAR, Intel RealSense D435i
- **Motor Controller:** VESC 6 MkVI

---

*For general lab information, visit the [CRAL main page](https://github.com/CRAL-UVA/CRAL-UVA)*
