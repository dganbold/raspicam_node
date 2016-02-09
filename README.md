
Raspicam_ros
=======
#### ROS Indigo node for mipi camera module of Raspberry Pi

###1. Prerequisites (dependencies)

#####1.1 ARM side libraries for interfacing to Raspberry Pi GPU

	git clone https://github.com/raspberrypi/userland.git /home/pi/userland

Then use buildme to build. It requires cmake to be installed and an arm cross compiler.

#####1.2 ROS
If you do not have already installed ROS in your rapsberry pi/pi2, we recommend you to install the ROS Indigo on the Rapsberry Pi which installed Raspbian Jessie or Ubuntu OS.

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

###2. Installation

1. Clone the repository:

		cd /home/pi/catkin_ws/src
		git clone https://github.com/dganbold/raspicam_ros.git

2. Build

		source /opt/ros/indigo/setup.bash
		cd /home/pi/catkin_ws/
		catkin_make

###3. Usage
	source devel/setup.bash
	roslaunch raspicam_ros auto_shutter.launch

###4. License
raspicam_ros is released with a BSD license.
