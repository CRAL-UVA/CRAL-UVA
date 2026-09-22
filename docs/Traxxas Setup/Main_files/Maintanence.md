# Maintenance

> This page is a starting point — expand it as more maintenance procedures are established.

## Battery Care

- Store the drive LiPo (6S) in a fire-resistant bag when not in use.
- Do not disconnect the battery until the Jetson has fully shut down (see [User Manual](User%20Manual.md#6-shutting-down)).
- Configured low-voltage cutoffs: 3.4 V/cell start (20.4 V on 6S), 3.2 V/cell end (19.2 V on 6S) — see [VESC Setup](../Upper_stack/vesc.md).

## Wiring Checks

- Periodically verify VESC-to-motor solder joints and heat shrink coverage — see [VESC Wire Soldering](../Upper_stack/vesc_wire_soldering.md).
- Confirm all sensor connections (LiDAR Ethernet/power, RealSense USB3) are secure before each run.

## Pre-Run Checklist

- Confirm firmware/hardware version match in VESC Tool (mismatches are a known cause of hard-to-diagnose motor faults).
- Confirm `/dev/ttyACM*` matches the port set in `vesc_config.yaml`.
- Elevate the vehicle and keep hands/objects clear of wheels before powering on motors.
