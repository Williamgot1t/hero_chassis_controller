# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;control_msgs;control_toolbox;controller_interface;forward_command_controller;hardware_interface;realtime_tools;std_msgs;urdf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhero_chassis_controller".split(';') if "-lhero_chassis_controller" != "" else []
PROJECT_NAME = "hero_chassis_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.0"
