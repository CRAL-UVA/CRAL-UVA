# 1. Introduction 
Jackal is a rugged, lightweight, fast and easy-to-use unmanned ground robot for rapid prototyping and research applications. Jackal includes a computer, as well as basic IMU and GPS. Standard perception modules are available, including URDF, simulator integration, and demonstration applications.

# 2. Turning the Jackal on and off

## 2.1 Turning on procedure
- Unplug the battery from the charger first, and the charger from the wall second
- Open the Jackal lid(You need lower the lever at the front of the Jackal)
- Plug the battery into the Jackal
    - IMPORTANT: Make sure you only plug in the red and back connector and NOT the white connector
    - We've fried a connector doing this
- Press the power button on the back of the Jackal
- Turn off the motor
    - It's good practice to turn off the motor unless you explicitly need it on, because we don't want to accidentally crash the Jackal (they're expensive)
    - You can do this by pressing the "M" button close to the power button
        a. Light on = motor on
        b. Light off  = motor off

## 2.2 Turning off procedure

- Turn Jackal off
    - Press the power button on the back of the Jackal
    - Wait for the light to turn off
- Unplug the battery from the Jackal
    - You need to pull on the connector a bit hard, just be careful not to break anything
- Plug the battery into the wall
    - It used to spark when you did this, and the best way I've found to do that is as follows:
        a. Plug the charger into the wall
        b. Unplug the charger from the wall
        c. Wait for the light on the charger to turn off
        d. Plug the battery into the charger
        e. Plug the charger into the wall

# 3. Connecting to the Jackal

## 3.1 Wireless SSH

- Turn the Jackal on and wait for it to connect to WiFi (you'll see the WiFi light turn on)
-  Connect laptop to "NETGEAR52"
    SSID: "NETGEAR52"
    Password: "oddbird088"

[//]:# (Add more documentation of each Jackal)
- Run ssh administrator@192.168.1.2
     Password: "clearpath"
     10.10.10.53 is the static IP of the J100-0893 Jackal. 
    192.168.4.27 is the static IP of the J100-0896 Jackal

## 4.Ethernet SSH

Directions for this can be found in the user manual, which should be in the lab

## 5. Connecting to a monitor

-  Turn the Jackal on
-  Open up the tray with the electronics inside the Jackal
    - You should see the motherboard and the Nvidia GPU
-  Connect the monitor's DisplayPort (or HDMI) cable, as well as the USBs for the mouse and keyboard
    - Be careful doing this. The inner tray is not meant to be accessed often, and you could break something if you don't treat it with proper care and attention
- You should see the login screen on the monitor
     User: "administrator"
     Password: "clearpath"





























