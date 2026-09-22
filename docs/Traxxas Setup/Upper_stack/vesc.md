## 1. VESC Setup
Equipment Required:
• VESC tool
• Box or Car stand to put the vehicle on
• Laptop/computer (does not need to be running Linux)

### 1.1 Connect the servo motor to the vesc via the transmitter 
- Step1 - Connect the PPM cable to the transmitter CH2 
ppm cable - 
[alt text](../images/vesc_setup/image1.png)
connect the ppm cable to the CH2 
[alt text](../images/vesc_setup/image2.png)

- Step2  - connect the other end of the cable to the vesc 
[alt text](../images/vesc_setup/image3.png)

- Step3 - Start with the installation process

## 2. Installing the VESC Tool

We need to configure the VESC so that it works with our motor and vehicle transmission. Before you start, you’ll need to install the [VESC Tool](https://vesc-project.com/vesc_tool). You’ll have to register for an account to download. Add the free tier tool to the cart (you don’t have to fill in any information other than your email.) After checkout, a download link will be sent to your email address. There should be versions of the software for Linux, Windows, and macOS.

### 2.1 Powering the VESC

First, we need to power the VESC. Plug the battery in, and make sure the polarity is correct. Note that you don’t need to turn on the power board for configuring the VESC.
[alt text](../images/vesc_setup/image4.png)
Next, unplug the USB cable of the VESC from the Jetson NX and plug the USB into your laptop that’s running the VESC Tool. You may want to use a longer cable.
[alt text](../images/vesc_setup/image5.png)

### 2.2 Connecting the VESC to Your Laptop

Launch the VESC Tool. On the Welcome page, press the **AutoConnect** button on bottom left of the page. After the VESC is connected, you should see an updated status on the bottom right of the screen.
[alt text](../images/vesc_setup/image6.png)

### 2.3 Updating the Firmware on the VESC
The first thing you’ll need to do is to update the firmware onboard the VESC. Depending on the version of the VESC tool you’re using, you’ll need to go through different steps to enable servo out from the ppm port on the VESC.

With VESC Tool versions released after Mar. 31 2021, you can use the latest default firmware. And to enable servo out, go to **App Settings** > **General** > **Enable Servo Output** in the VESC Tool to enable servo out.
[alt text](../images/vesc_setup/image7.png)
Before VESC Tool version 2.05, you can enable servo out by using a non-default firmware. On the left side of the screen, click on the Firmware tab. On bottom left of the page, check the Show non-default firmwares check box. On the right, you should see extra firmware options show up. Select the VESC_servoout.bin option. Afterwards, on the bottom right of the page, press the button with the down arrow to update the firmware on the connected VESC. A status bar at the bottom of the page will show firmware update status. After it’s finished, follow the prompt on screen.
[alt text](../images/vesc_setup/image8.png)

### 2.4 Uploading the Motor Configuration XML
After firmware update, select Load Motor Configuration XML from the drop-down menu and select the provided XML file from here. After the XML is uploaded, click on the Write Motor Configuration button (the button with a down arrow and the letter M) on the right side of the screen to apply the motor configuration. Note that in the future, you’ll have to press this button whenever you make a change in motor configuration.
[alt text](../images/vesc_setup/image9.png)

### 2.5 Detecting and Calculating Motor Parameters
To detect and calculate the FOC motor parameters, navigate to the FOC tab under Motor Settings on the left. At the bottom of the screen, follow the direction of the arrows and clck on the four buttons one by one, and follow the on-screen prompt. Note that during the measuring process, the motor will make noise and spin; make sure the wheels of your vehicle are clear.
[alt text](../images/vesc_setup/image10.png)

After the motor parameters are measured, the fields at the bottom of the screen should turn green. Click on the Apply button, and click the Write Motor Configuration button.
[alt text](../images/vesc_setup/image11.png)

### 2.6 Changing the Open Loop Hysteresis and Open Loop Time
Navigate to the Sensorless tab at the top of the screen. Change the Openloop Hysteresis and Openloop Time to 0.01, and click the Write Motor Configuration button.
[alt text](../images/vesc_setup/image12.png)

### 2.7 Tuning the PID controller
Now you can start tuning the speed PID controller. To see the RPM response from the motor, navigate to the Realtime Data tab under Data Analysis on the left. Click Stream Realtime Data button on the right (the button with letters RT), and navigate to the RPM tab on the top of the screen. You should see RPM data streaming now.
[alt text](../images/vesc_setup/image13.png)

To create a step response for the motor, you can set a target RPM at the bottom of the screen (values between 2000 and 10000 RPM). Click the play button next to the text box to start the motor. Note that the motor will spin, so make sure the wheels of your vehicle are clear from objects. Click the Anchor or STOP button to stop the motor.
[alt text](../images/vesc_setup/image14.png)

You want to look for a clean step response that has a quick rise time and zero to very little steady-state error. Adjust the gains accordingly by navigating to the PID Controllers tab under Motor Settings on the left and changing the Speed Controller gains. General rules of tuning PID gains apply. If you’re seeing a lot of oscillations, try changing the Speed PID Kd Filter.
[alt text](../images/vesc_setup/image15.png)

### 2.8 Adjusting Top Speed of the car
By default, the motor configuration sets a safe top motor RPM. If you wish to change the hard limit set by the VESC firmware, you can go to Motor Settings > General and change the max ERPM for forward and backwards rotations. You’ll also have to change the configuration file mentioned in the Odometry Tuning section in the software stack setup to change the software limit for your motor ERPM.
[alt text](../images/vesc_setup/image16.png)

**Once this is done, lets do a sanity test on the working of the motor - Motor test**s

## 3. UDR-Specific Configuration Values (Reference)

> ⚠️ **Use these values, not any older ones.** An earlier VESC configuration circulated in lab notes was built for a Traxxas Slash 4x4, not the UDR, and used a smaller motor. It is superseded and should not be used on the UDR platform. The values below are UDR-specific and current.

### 3.1 Prerequisites
- VESC Tool installed (register at [vesc-project.com/vesc_tool](https://vesc-project.com/vesc_tool) for the free-tier download)
- Vehicle on a stand with wheels clear — motor will spin during detection
- Confirm firmware reads **VESC 6 MkVI** in VESC Tool before proceeding; update firmware if a mismatch is offered. A firmware/hardware version mismatch (e.g. Mk5 firmware on Mk6 hardware) is a known cause of hard-to-diagnose motor faults.

### 3.2 Detection parameters
| Parameter | Value |
|---|---|
| Detection current | 12 A |
| Detection duty | 0.06 |
| Motor size | Large / 75 mm |
| HFI | Disabled |

Click **Run Detection** — hold the car steady, wheels will spin briefly. Note the auto-set **Observer Gain** value from the wizard before continuing (write it down), then **Apply** and **Write Motor Configuration**.

> **Direction check — do not skip:** in Realtime Data, ramp duty to 0.10 and confirm the car moves forward. If reversed, swap any two motor phase wires and re-run detection before continuing.

### 3.3 FOC / Advanced tuning
| Parameter | Value |
|---|---|
| Observer Gain | Exactly half the value noted during detection (e.g. wizard gives 3.14 → enter 1.57) |
| Openloop ERPM | 600 |
| Sensorless ERPM | 1500 |

### 3.4 Current limits (Motor Settings → General → Current)
| Parameter | Value |
|---|---|
| Motor Max | 80 A |
| Motor Max (Brake) | -60 A (unchanged from default) |
| Battery Max | 60 A |
| Battery Max (Regen) | -20 A (unchanged from default) |

### 3.5 Voltage limits (Motor Settings → General → Voltage)
| Parameter | Value |
|---|---|
| Battery cutoff start | 3.4 V/cell (20.4 V on 6S) |
| Battery cutoff end | 3.2 V/cell (19.2 V on 6S) |

### 3.6 RPM limits (Motor Settings → General → RPM)
| Parameter | Value |
|---|---|
| ERPM limit (forward) | 100,000 |
| ERPM limit (reverse) | -100,000 |

### 3.7 Speed PID (App Settings → PID Controllers → Speed)
- Kp: halve the existing value (e.g. 0.004 → 0.002)
- Ki and Kd: leave unchanged
- Write App Configuration after changing

### 3.8 Input calibration (App Settings → PPM → Setup Input)
- Click **Start**; hold full forward throttle 1s, full brake 1s, return to neutral 1s
- Click **Apply**, then **Write App Configuration**
- Unplug USB. Test-drive and verify: smooth acceleration from standstill, no jitter, higher top speed than the unconfigured baseline

### 3.9 ERPM ↔ RPM conversion
VESC Tool reports ERPM (electrical RPM), the motor's revolutions before gear reduction — not wheel speed. Approximate conversion: motor spins at ~55.9 RPM per 1000 ERPM; apply the 17.89:1 internal gear ratio on top of that to get wheel RPM. If tuned speed appears to mismatch observed wheel speed, this is the expected explanation, not a fault.