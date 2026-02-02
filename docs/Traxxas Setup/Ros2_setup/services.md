How to create a service:
1. Create a service file - **`/etc/systemd/system/my_ros2.service`**

Example file for ROS2 

`[Unit]
Description=My ROS 2 Launch Service
After=network-online.target
Wants=network-online.target`

`[Service]
Type=simple
User=cral-traxaas
WorkingDirectory=/home/robot
ExecStart=/bin/bash -c “source /opt/ros/humble/setup.bash && source /home/robot/ros2_ws/install/setup.bash &&
ros2 launch your_package your_launch_file.launch.py
”
Restart=on-failure
RestartSec=5
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

`[Install]
WantedBy=multi-user.target`

### EXPLANATION-

### `[Unit]`

```
After=network-online.target
Wants=network-online.target

```

Ensures:

- Network interfaces are up
- DDS discovery works reliably

---

### `[Service]`

### `User=robot`

- ROS nodes **should not run as root**
- Must be the same user that owns the workspace

---

### `ExecStart=/bin/bash -c '...'`

Why?

- systemd does **not load `.bashrc`**
- You must manually source:
    - ROS installation
    - Your workspace

The `&&` ensures:

- If sourcing fails → node does not start silently

---

### `Restart=on-failure`

Very important for robots:

- Node crashes → service restarts
- Better than silent failure

---

### `RMW_IMPLEMENTATION`

Optional but recommended:

- Makes DDS explicit
- Avoids discovery issues

Common values:

```
rmw_fastrtps_cpp
rmw_cyclonedds_cpp

```

---

### Enable and test the service

```jsx
sudo systemctl daemon-reload
sudo systemctl enable my_ros2.service
sudo systemctl start my_ros2.service
```

To check logs:

```jsx
journalctl -u my_ros2.service -f

```

## Debug checklist if it doesn’t start

1️⃣ Can you run this manually as the same user?

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch your_package your_launch_file.launch.py

```

2️⃣ Check environment inside systemd:

```bash
systemctl show my_ros2.service | grep Environment

```

3️⃣ Check network:

```bash
ip route
```