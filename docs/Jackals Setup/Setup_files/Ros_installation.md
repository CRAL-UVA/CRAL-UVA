## Flashing Ubuntu 22 with ROS2 Humble


Follow the instructions here to Flash Ubuntu 22 and install ROS2: https://github.com/r-shima/jackal_ros2_humble?tab=readme-ov-file

* For the firmware flashing, follow the instructions here: https://docs.clearpathrobotics.com/docs/ros/installation/robot/


sudo apt install ros-humble-desktop-full

wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/clearpath_computer_installer.sh && bash -e clearpath_computer_installer.sh`


Connect to eduroam by following the instructions under automatic here: http://galileo.phys.virginia.edu/compfac/faq/linux-eduroam.html

Install openssh and setup the ssh client: https://www.linode.com/docs/guides/enable-ssh-ubuntu/

Connect the ps4 controller: https://docs.clearpathrobotics.com/docs/ros/installation/controller/

* You may need to restart your computer after connecting the controller


Install CUDA: https://www.cherryservers.com/blog/install-cuda-ubuntu
