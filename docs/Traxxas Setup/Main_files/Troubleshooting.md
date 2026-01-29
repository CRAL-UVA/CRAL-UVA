## Hokuyo LiDAR Connection Error

```text
[urg_node_driver-6] [ERROR] [1764961112.876242810] [urg_node]: Error connecting to Hokuyo
[urg_node_driver-6] /dev/cu.usbmodem141101 @ 115200
[urg_node_driver-6] could not open serial device.
```
This means that the LiDAR is not connecting or being detected by ROS2, check in ~/f1tenth_system/f1tenth_stack/config/sensors.yaml for any typos.

# How to SSH into a Jetson Device

## Devices
- **Target Device:** Jetson
- **Base Device:** Laptop

---

## Steps

### 1. Access Terminal on Target Device
- At the login screen, press **CTRL + ALT + F2** to access the boot-up terminal.
- Log in with your username and password.

> Only required if the GUI is not working.

---

### 2. List Available Wi-Fi Networks
```bash
nmcli device wifi list
```

### 3. Connect to Wi-Fi
- Make sure to connect to the same WiFi as the Base Device, usually cralrobotics
```bash
nmcli dev wifi connect <wifi-name> password <password>
```
### 4. Find the Jetson IP Address
```bash
ip addr show
```

- Locate the wlan0 interface.
- Use the inet address (e.g., 192.168.x.x).

### 5. SSH from Base Device
```bash
ssh <username>@<target-ip>
```

- Example:
```bash
ssh jetson@192.168.1.42
```
