cmake_minimum_required(VERSION 2.8.3)
project(raspicam_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  compressed_image_transport 
  roscpp 
  std_msgs 
  camera_info_manager
  dynamic_reconfigure
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Camera.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES raspicam
#  CATKIN_DEPENDS compressed_image_transport roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
##  /home/erle/userland
  /opt/vc/include
  /opt/vc/include/interface/vcos/pthreads
  /opt/vc/include/interface/vmcs_host/linux
)

## Declare a cpp library
add_library(raspicli STATIC
  src/RaspiCLI.c 
)

add_library(raspicamcontrol STATIC
  src/RaspiCamControl.c 
)

## Declare a cpp executable
add_executable(raspicam_node src/raspicam_node.cpp)
 
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(raspicam_node raspicam_node_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(raspicam_node 
    ${catkin_LIBRARIES}
    raspicamcontrol raspicli
    /opt/vc/lib/libbcm_host.so
    /opt/vc/lib/libvcos.so
    /opt/vc/lib/libmmal.so
    /opt/vc/lib/libmmal_core.so
    /opt/vc/lib/libmmal_util.so
    /opt/vc/lib/libmmal_vc_client.so
    /opt/vc/lib/libvchostif.a
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS raspicam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_raspicam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
