
raspicam_node
=======
ROS node for mipi camera module of the Raspberry Pi.

## 1. Prerequisites
First of all, you will need to connect the Camera Module to the Raspberry Pi's camera port.

https://www.raspberrypi.org/documentation/usage/camera/

This node is primarily supported ROS Kinetic. If you do not have already installed ROS in your Rapsberry Pi, we recommend you to install the ROS Indigo or Kinetic version.

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

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
		
Change parameters
		
	rosrun rqt_reconfigure rqt_reconfigure 

## 4. Debug
You can easily check that published image from camera node by using image_viewer.

	rosrun image_view image_view image:=/raspicam_node/image_raw

## License
raspicam_node is released with a BSD license.
