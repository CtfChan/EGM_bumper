**Author(s): Christopher Chan**

## Overview
EGM stands for ephemeral grid mapping (I will try to come up with better names next time). 
EGM_class is the name of the package, and the name of the class used as the nodelet.
EGM_ns is the name of the C++ namespace used for the project. 
This nodelet subscribes to a lidar point cloud, stereo point cloud and a sonar polygon message and outputs the a set of polygons that will be processed by the virtual bumper node. Full report describing code and findings is available at:

https://docs.google.com/document/d/1T3X65S8JkSBpmvoSFQGFBKfZ1-tISEBzRHocDmxHMqA/edit?usp=sharing.

Demo of this code is available at:

https://www.youtube.com/watch?v=K7GaomP2tZw

#### Subscribed Topics

* **`/lidar`** (sensor_msgs::PointCloud2)

	Lidar pointcloud from Velodyne.

* **`/stereo`** (sensor_msgs::PointCloud2)

	Realsesne pointcloud. 

* **`/sonar`** (zio_obstacle_msgs/ObstaclesStamped)

	Obstacle polygons from sonar array. 


#### Published Topics

* **`/obstacles`** (zio_obstacle_msgs/ObstaclesStamped)

	A set of obstacle polygons to be processed by the virtual bumper node. 


* **`/grid_map_fused`** (grid_map_msgs/GridMap)

	For visualizing grid cells that were determined to be obstacles.


#### Parameters

The full list of parameters that can be tuned is listed in **EGM_bumper/config/param.default.yaml**.


## Install dependencies
	$ sudo apt-get install ros-kinetic-grid-map

	$ cd your_workspace/src
	$ git clone https://gitlab.acfr.usyd.edu.au/zio/zio_obstacle.git
	$ catkin build virtual_bumper
	$ catkin build zio_obstacle_msgs

## To Build
	$ cd your_workspace/src
	$ git clone https://gitlab.acfr.usyd.edu.au/c.chan/egm_bumper.git
	$ catkin build EGM_class --cmake-args -DCMAKE_BUILD_TYPE=Release

## To Run (on local machine)
Download a bag listed in egm.launch from RCOS and change the default path in egm.launch. 

Terminal 1:

	$ export PLATFORM_NAME=zio-003
	$ roslaunch zio_description zio_description.launch 

Terminal 2:

	$ roslaunch EGM_class egm.launch 


## To Run Testing with Gamepad (on EV)
Connect gamepad to EV and go into gamepad mode on EV.

	$ sudo systemctl start zio-gamepad-teleop.service

Connect ethernet from laptop to EV. Change ROS master and launch EGM bumper on your laptop (don't forget to source your workspace).

	$ export ROS_MASTER_URI=http://zio-nuc:11311  
	$ roslaunch EGM_class egm.launch bag_play:=false

Run the following commands in another terminal on your laptop (don't forget to source your workspace). Make sure you change virtual_bumper.yaml according to your needs.

	$ export ROS_MASTER_URI=http://zio-nuc:11311 
	$ cd (Path to EGM_bumper)/scripts
	$ rosparam delete zio/virtual_bumper
	$ rosparam load virtual_bumper.yaml zio/virtual_bumper
	$ rostopic pub -1 zio/virtual_bumper/reload_parameters std_msgs/Empty '{}'

## Bugs and Other Probelms

Feel free to email me at christophertzechan@gmail.com if there's anything that was unclear or if there are some bugs that need to be changed. 
