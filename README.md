
raspicam_node
=======
ROS node for mipi camera module of the Raspberry Pi.

## 1. Prerequisites
First of all, you will need to connect the Camera Module to the Raspberry Pi's camera port and enable it.

https://www.raspberrypi.org/documentation/usage/camera/

This node is primarily supported ROS Kinetic. If you do not have already installed ROS in your Rapsberry Pi, we recommend you to install the ROS Indigo or Kinetic version.

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

The existing user is should be in the video group in other to access to command interface between the running Linux kernel and peripherals (camera, audio subsystems etc) of the VideoCore. 

1. Make sure that by execute below: 
	
		groups | grep video
	
	If it's not existing then use the following commands for adding an user into video group:

		sudo -s
		usermod -a -G video `your user`
	
2. And also need to make a rule for /dev/vchiq is accessible to users in video group:
 	
		echo 'SUBSYSTEM=="vchiq",GROUP="video",MODE="0660"' > /etc/udev/rules.d/10-vchiq-permissions.rules
		reboot

## 2. Installation

Go to your workspace `cd ~/catkin_ws/src`.

1. Clone the repository:

		git clone https://github.com/dganbold/raspicam_node.git
   
2. Build

		cd ~/catkin_ws/
		catkin_make --pkg raspicam_node

## 3. Usage
Once camera node building is successfully done, you can run it using a launch file.

		source devel/setup.bash
		roslaunch raspicam_node camera_module_v2_640x480_30fps.launch
		
Start captune and image publish

		rosservice call /raspicam_node/start_capture

If you want to change or tuning the parameters of camera, you can use the following tools.
		
		rosrun rqt_reconfigure rqt_reconfigure 
		
		rosrun dynamic_reconfigure dynparam <load or dump> /raspicam_node <your parameter file>.yaml
		
![Alt text](rqt_reconfigure.bmp?raw=true "Title")

## 4. Debug
You can easily check that published image from camera node by using image_viewer.

		rosrun image_view image_view image:=/raspicam_node/image_raw
		
![Screenshot](image_view.gif?raw=true "Title")

## License
raspicam_node is released with a BSD license.
