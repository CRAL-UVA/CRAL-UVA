# Demo 2 -Multi-Robot Setup Guide

This guide documents the process of configuring Clearpath robots for multi-robot operations, including namespace configuration, URDF generation, and component modifications.

## Overview

When setting up multiple Clearpath robots, each robot requires a unique namespace to avoid topic and frame name conflicts. This document outlines the modifications needed to the Clearpath generator source code and configuration files to support proper namespace isolation.

## Initial Setup and Problem Identification

### Modified Clearpath Generator Source Code

The following files were modified to add namespace support to frame names during URDF generation:

- `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/links.py`
- `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/platform.py`

These files now add the namespace (from the environment variable) to frame names during URDF generation.

### Regenerating Robot Description

1. Regenerate the robot description with the namespace environment variable:

```bash
sudo bash -c "source /opt/ros/humble/setup.bash && CLEARPATH_ROBOT_NAMESPACE=j100_0893 python3 /opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generate_description"
```

**Result:** The generated `/etc/clearpath/robot.urdf.xacro` will include namespaced frames:

```xml
<xacro:j100 wheel="default" odom_frame="j100_0893/odom" base_link_frame="j100_0893/base_link"/>
```

2. Add the environment variable to the systemd service:

```bash
sudo systemctl edit clearpath-robot.service
```

Add the following configuration:

```ini
[Service]
Environment="CLEARPATH_ROBOT_NAMESPACE=j100_0893"
```

3. Restart the service:

```bash
sudo systemctl restart clearpath-robot.service
```

### Observed Behavior

The following behavior was observed during testing:

- **With environment variable enabled:**
  - `/etc/clearpath/robot.urdf.xacro` includes namespace frames
  - ❌ `robot_state_publisher` does not start, resulting in "no TF data received"
  
- **Without environment variable:**
  - The URDF reverts to plain frames (`odom`, `base_link`)
  - ✅ `robot_state_publisher` launches correctly and TFs appear

## Current Issue

The system is stuck between two inconsistent states:

| Scenario | URDF Frame Names | robot_state_publisher | TF Visible? |
|----------|------------------|----------------------|-------------|
| `CLEARPATH_ROBOT_NAMESPACE` enabled | ✅ Prefixed (`j100_0893/odom`) | ❌ Missing | ❌ No TF data |
| Environment variable removed | ❌ Plain (`odom`) | ✅ Running | ✅ TF visible |

**Summary:** Namespace propagation to URDF works, but enabling it breaks `robot_state_publisher` startup.

## Proposed Solution

Instead of injecting namespace via environment variables, introduce a prefix configuration in `/etc/clearpath/robot.yaml`:

```yaml
system:
  ros2:
    namespace: j100_0893
    prefix: robot1
```

Modify the generator to read the prefix instead of namespace for TF frame names:

- Topics remain under `/j100_0893/...`
- Frames become `robot1/odom`, `robot1/base_link`, etc.

## Configuration File

Edit `/etc/clearpath/robot.yaml`:

```yaml
system:
  ros2:
    namespace: j100_0893
```

## Code Modifications

### 1. Generator Core (`generator.py`)

**File:** `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py`

**Modifications:**
1. Read and export namespace/prefix from configuration
2. Implement unified generate sequence

**Updated `generate()` method:**

```python
def generate(self) -> None:
    self.generate_common()
    self.generate_platform(prefix=self.prefix)
    self.generate_attachments(prefix=self.prefix)
    self.generate_links(prefix=self.prefix)
    self.generate_mounts(prefix=self.prefix)
    self.generate_sensors(prefix=self.prefix)
    self.generate_manipulators(prefix=self.prefix)
    self.generate_extras()
```

### 2. Platform Base Frames (`platform.py`)

**File:** `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/platform.py`

Modify to construct namespaced base frames using the prefix parameter.

## Sensor Configuration

### Ouster OS1 LiDAR

To automatically pass the namespace parameter when calling `<xacro:ouster_os1>`, follow these steps:

#### Step 1: Modify Generator (`generator.py`)

**File:** `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py`

1. Locate the function:
   ```python
   def generate_sensors(self, prefix="") -> None:
   ```

2. Inside the loop, find the following code:
   ```python
   self.xacro_writer.write_macro(
       macro='{0}'.format(sensor_description.model),
       parameters=sensor_description.parameters,
       blocks=XacroWriter.add_origin(
           sensor_description.xyz, sensor_description.rpy)
   )
   ```

3. **Before** that block, insert this logic:
   ```python
   if hasattr(self, "namespace") and self.namespace:
       if "namespace" not in sensor_description.parameters:
           sensor_description.parameters["namespace"] = self.namespace
   ```

This ensures that when the generator calls `<xacro:ouster_os1>`, it automatically passes the namespace:
```xml
<xacro:ouster_os1 ... namespace="j100_0893" />
```

#### Step 2: Modify Ouster OS1 Xacro File

**File:** `/opt/ros/humble/share/clearpath_sensors_description/urdf/ouster_os1.urdf.xacro`

1. Add namespace parameter to the macro definition:
   ```xml
   <xacro:macro name="ouster_os1" params="name parent_link namespace:='' ...">
   ```
   (Ensure `namespace:=''` is included in the parameter list)

2. Add namespace prefix handling right after the macro definition:
   ```xml
   <xacro:if value="${namespace != ''}">
     <xacro:property name="ns" value="${namespace + '/'}" />
   </xacro:if>
   <xacro:unless value="${namespace != ''}">
     <xacro:property name="ns" value="" />
   </xacro:unless>
   ```

3. Update all link and joint names to include `${ns}` prefix:
   - Change: `<link name="${name}_link">` → `<link name="${ns}${name}_link">`
   - Change: `<joint name="${name}_joint" type="fixed">` → `<joint name="${ns}${name}_joint" type="fixed">`

   **Important:** Do not add `${ns}` to the parent link. Keep it as:
   ```xml
   <parent link="${parent_link}" />
   ```

4. Example macro call:
   ```xml
   <xacro:ouster_os1 name="lidar3d_0" parent_link="base_link" namespace="j100_0893"/>
   ```

This ensures the generator dynamically injects the robot namespace into all link and joint names when building the URDF.

## Attachment Configuration

### Fenders

To automatically support namespace when calling `<xacro:fender>`:

#### Step 1: Modify Generator (`generator.py`)

**File:** `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py`

1. Locate the function:
   ```python
   def generate_attachments(self, prefix="") -> None:
   ```

2. Add namespace parameter passing logic similar to sensors.

#### Step 2: Modify Fender Xacro File

**File:** `/opt/ros/humble/share/clearpath_platform_description/urdf/j100/attachments/fender.urdf.xacro`

1. Add namespace parameter to the macro definition:
   ```xml
   <xacro:macro name="fender" params="name model parent_link:=base_link namespace:='' *origin">
   ```

2. Add namespace prefix handling:
   ```xml
   <xacro:if value="${namespace != ''}">
     <xacro:property name="ns" value="${namespace + '/'}" />
   </xacro:if>
   <xacro:unless value="${namespace != ''}">
     <xacro:property name="ns" value="" />
   </xacro:unless>
   ```

3. Update all link and joint names to include `${ns}` prefix.
   **Note:** Keep the parent link unchanged (do not prefix it).

## IMU Configuration

### IMU Filter Configuration

Edit the IMU filter configuration file:

```bash
sudo nano /opt/ros/humble/share/imu_filter_madgwick/config/imu_filter.yaml
```

### IMU Parameter Generation

**File:** `/opt/ros/humble/lib/python3.10/site-packages/clearpath_generator_common/param/platform.py`

Modify the `ImuFilterParam` class:

```python
class ImuFilterParam(BaseParam):
    def generate_parameters(self, use_sim_time: bool = False):
        # Add namespace-aware parameter generation
        ...
```

## Odometry Configuration

### Control Configuration Template

**Template file:** `/opt/ros/humble/share/clearpath_control/config/j100/control.yaml`

**Parameter generation file:** `/opt/ros/humble/lib/python3.10/site-packages/clearpath_generator_common/param/platform.py`

After regeneration, verify the control configuration:

```bash
sudo clearpath-robot-generate
cat /etc/clearpath/platform/config/control.yaml
```

**Expected output:**
```yaml
left_wheel_names: ['j100_0893/front_left_wheel_joint', 'j100_0893/rear_left_wheel_joint']
right_wheel_names: ['j100_0893/front_right_wheel_joint', 'j100_0893/rear_right_wheel_joint']
```

## LiDAR Configuration

### Sensor Parameter Generation

**File:** `/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_robot/param/sensors.py`

Modify the `generate_config` method to include namespace-aware configuration.

After modification, regenerate the sensor configuration:

```bash
sudo clearpath-robot-generate
```

Verify the generated configuration:

```bash
cat /etc/clearpath/sensors/config/lidar3d_0.yaml
```

## Verification Steps

After completing all modifications:

1. Regenerate robot description:
   ```bash
   sudo clearpath-robot-generate
   ```

2. Verify URDF includes namespaced frames:
   ```bash
   cat /etc/clearpath/robot.urdf.xacro
   ```

3. Check that `robot_state_publisher` starts correctly:
   ```bash
   sudo systemctl status clearpath-robot.service
   ```

4. Verify TF data is published:
   ```bash
   ros2 run tf2_ros tf2_echo /map /j100_0893/base_link
   ```

## Troubleshooting

- **If `robot_state_publisher` fails to start:** Check that all frame names in the URDF are consistently namespaced and match the expected format.

- **If TF data is missing:** Verify that the namespace parameter is correctly passed to all xacro macros and that the prefix handling logic is properly implemented.

- **If topics are not namespaced:** Ensure the ROS 2 namespace configuration in `/etc/clearpath/robot.yaml` is correct.
