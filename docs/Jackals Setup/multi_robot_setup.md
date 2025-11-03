Multi-robotModified Clearpath generator source code:

    * /opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/links.py
    * /opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/platform.py
         These now add the namespace (from the environment variable) to frame names during URDF generation.

1. Regenerated the robot description:

sudo bash -c "source /opt/ros/humble/setup.bash && CLEARPATH_ROBOT_NAMESPACE=j100_0893 python3 /opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generate_description"
Result:

/etc/clearpath/robot.urdf.xacro
  <xacro:j100 wheel="default" odom_frame="j100_0893/odom" base_link_frame="j100_0893/base_link"/>

2.Added environment variable in systemd service:

sudo systemctl edit clearpath-robot.service

Environment="CLEARPATH_ROBOT_NAMESPACE=j100_0893"


and restarted:

sudo systemctl restart clearpath-robot.service



3.Observed the behavior:

    * With the environment variable enabled:
        /etc/clearpath/robot.urdf.xacro includes namespace frames
        BUT → robot_state_publisher does not start, resulting in “no TF data received.”
    * Without the environment variable:
         The URDF reverts to plain frames (odom, base_link),
        BUT robot_state_publisher launches correctly and TFs appear.


❗ Current Issue

stuck between two inconsistent states:

Scenario
	URDF Frame Names
	robot_state_publisher
	TF Visible?

CLEARPATH_ROBOT_NAMESPACE enabled
	✅ Prefixed (j100_0893/odom)
	❌ Missing
	❌ No TF data

Environment variable removed
	❌ Plain (odom)
	✅ Running
	✅ TF visible


Essentially, the namespace propagation to URDF works —
 but enabling it breaks robot_state_publisher startup



Next Step Proposal？？

Instead of injecting namespace via environment variables,
 introduce prefix in /etc/clearpath/robot.yaml:

ros2:
    namespace: j100_0893
    prefix: robot1




and modify the generator to read prefix instead of namespace for TF frame names:

* Topics remain under /j100_0893/...
* Frames become robot1/odom, robot1/base_link, etc.
* 
* 





Configuration file
/etc/clearpath/robot.yaml:
system:
  ros2:
    namespace: j100_0893


Code Modifications (Step by Step)：
1.generator.py
File:
/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py

(1) Read and export namespace/prefix
(2) Unified generate sequence

def generate(self) -> None:
    self.generate_common()
    self.generate_platform(prefix=self.prefix)
    self.generate_attachments(prefix=self.prefix)
    self.generate_links(prefix=self.prefix)
    self.generate_mounts(prefix=self.prefix)
    self.generate_sensors(prefix=self.prefix)
    self.generate_manipulators(prefix=self.prefix)
    self.generate_extras()

2.platform.py — Construct Namespaced Base Frames
File:
/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/platform.py



Sensors:
Make the generator.py automatically pass namespace="j100_0893" when calling <xacro:ouster_os1>.
Follow these steps:
1.Open  generator.py file.  
/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py

2.Find the function:
def generate_sensors(self, prefix="") → None:

3.Inside its loop, locate the following code:

self.xacro_writer.write_macro(
    macro='{0}'.format(sensor_description.model),
    parameters=sensor_description.parameters,
    blocks=XacroWriter.add_origin(
        sensor_description.xyz, sensor_description.rpy)
)

4.Right before that block, insert this logic 

if hasattr(self, "namespace") and self.namespace:
    if "namespace" not in sensor_description.parameters:
        sensor_description.parameters["namespace"] = self.namespace

5.Now, when the generator calls <xacro:ouster_os1>,  it will automatically pass the namespace like this:
<xacro:ouster_os1 ... namespace="j100_0893" />
This ensures every sensor macro (like ouster_os1.urdf.xacro) automatically receives the correct namespace from the generator — no need to manually add it in each call.

We need to pass the namespace parameter from the generator, and also define this parameter inside the ouster_os1.urdf.xacro file.
 Below are the steps to define this parameter:

1.Open the file/opt/ros/humble/share/clearpath_sensors_description/urdf/ouster_os1.urdf.xacro

2.At the top of the file, inside the <xacro:macro ...> definition,  add a parameter for the namespace, like this:
<xacro:macro name="ouster_os1" params="name parent_link namespace:='' ...">
(Make sure to include namespace:='' in the parameter list.)
3.Right below the <xacro:macro ...> line,  add the following lines to handle the namespace prefix:
3.Update all link and joint names inside the file to include ${ns} as a prefix:

<link name="${name}_link">

should be changed to

<link name="${ns}${name}_link">

and

<joint name="${name}_joint" type="fixed">

should be changed to

<joint name="${ns}${name}_joint" type="fixed">

Note:
 Do not add ${ns} to the parent link — keep it as:

<parent link="${parent_link}" />


5.In the generator code, make sure when calling the macro, the namespace parameter is passed properly, e.g.:
<xacro:ouster_os1 name="lidar3d_0" parent_link="base_link" namespace="j100_0893"/>
This way, the generator dynamically injects the robot namespace into all link and joint names when building the URDF.


Fenders(attachment):
Make the fender.urdf.xacro and generator.py automatically support, namespace="j100_0893" when calling <xacro:fender>.
1.Open the generator.py file
/opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_common/description/generator.py
2.Find the function:
def generate_attachments(self, prefix="") → None:
3.Now modify the fender.urdf.xacro file to properly handle the namespace parameter.
Open:
/opt/ros/humble/share/clearpath_platform_description/urdf/j100/attachments/fender.urdf.xacro
4.At the top of the file, inside the <xacro:macro ...> definition, add a parameter for namespace, like this:
<xacro:macro name="fender" params="name model parent_link:=base_link namespace:='' *origin">
5.Right below the <xacro:macro ...> line, add these lines to handle the namespace prefix:
<xacro:if value="${namespace != ''}">
  <xacro:property name="ns" value="${namespace + '/'}" />
</xacro:if>
<xacro:unless value="${namespace != ''}">
  <xacro:property name="ns" value="" />
</xacro:unless>
6.Update all link and joint names inside the file to include ${ns} as a prefix and Keep the parent link unchanged (do not prefix it).


sudo nano /opt/ros/humble/share/imu_filter_madgwick/config/imu_filter.yaml

ODOM:
/opt/ros/humble/lib/python3.10/site-packages/clearpath_generator_common/param/platform.py
template：
/opt/ros/humble/share/clearpath_control/config/j100/control.yaml


sudo clearpath-robot-generate

cat /etc/clearpath/platform/config/control.yaml

left_wheel_names: ['j100_0893/front_left_wheel_joint', 'j100_0893/rear_left_wheel_joint']
right_wheel_names: ['j100_0893/front_right_wheel_joint', 'j100_0893/rear_right_wheel_joint']


IMU:
/opt/ros/humble/lib/python3.10/site-packages/clearpath_generator_common/param/platform.py

Modify：
class ImuFilterParam(BaseParam)
def generate_parameters(self, use_sim_time: bool = False)


lidar，config：
sudo nano /opt/ros/humble/local/lib/python3.10/dist-packages/clearpath_generator_robot/param/sensors.py

Modified：
generate_config :
then,
etc/clearpath/sensors/config/lidar3d_0.yaml:
