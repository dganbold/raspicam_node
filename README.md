
raspicam_node
=======
ROS node for mipi camera module of the Raspberry Pi.

## 1. Prerequisites
This node is primarily supported ROS Kinetic.
If you do not have already installed ROS in your Rapsberry Pi, we recommend you to install the ROS Indigo or Kinetic.

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

## 2. Installation

Go to your workspace `cd ~/catkin_ws/src`.

1. Clone the repository:

		git clone https://github.com/dganbold/raspicam_node.git
   
2. Build

		cd ~/catkin_ws/
		catkin_make --pkg raspicam_node

## 3. Usage
Once you have built the node, you can execute it using a launch file.

	  source devel/setup.bash
	  roslaunch raspicam_node camera_module_v2_640x480_30fps.launch
    rosservice call /raspicam_node/start_capture

## 4. License
raspicam_ros is released with a BSD license.
