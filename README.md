## Overview

The hero controller by william ps:DynamicX's final work

**Keywords:** RoboMaster, ROS, ros_control,PID,Forward and Inverse kinematics 


### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: William<br />
Affiliation: [Personal]()<br />
Maintainer: WilliamWu, tboyv22shit@163.com**

The hero_chassis_controller package has been tested under [ROS]  Noetic on 20.04.



#### Dependencies


- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- Mecanum_controller
- hardware_interface
- pluginlib
- control_toolbox
- realtime_tools
- control_msgs

Install dependencies:

    sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	
	git clone git@github.com/Williamgot1t/hero_chassis_controller.git
  
    # git clone https://github.com/Williamgot1t/hero_chassis_controller.git
    
	cd ../
	
	catkin build 


## Usage


Run the controller with:

	roslaunch hero.launch
