## Traxxas
   - **GUI login stuck:** Complete setup via serial/debug terminal (`Ctrl+Alt+F2`) — see [User Manual](User%20Manual.md)
   - **Login credentials:** May need recovery/reset if forgotten
   - Reference troubleshooting guides if issues persist

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