cmake_minimum_required(VERSION 2.8.3)
project(colibri_crabnav)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5) 
if(NEW_YAMLCPP_FOUND) 
add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND) 


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_crabnav
#  CATKIN_DEPENDS nav_msgs rosconsole roscpp rospy sensor_msgs std_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories( ${catkin_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(/usr/local/include) 
link_directories(/usr/local/lib)
INCLUDE_DIRECTORIES(include/colibri_crabnav/)

 add_executable(nav_rvizsinglegoal src/nav_to_point.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp)
 target_link_libraries(nav_rvizsinglegoal
   ${catkin_LIBRARIES}
 )

 add_executable(crab_safe_node src/safe_node.cpp src/protect.cpp )
 target_link_libraries(crab_safe_node
   ${catkin_LIBRARIES}
 )

 add_executable(nav_mult_rvizgoal src/nav_to_multipoint.cpp src/task_mgr.cpp src/global_planner.cpp src/colibri_action.cpp src/colibri_local_nav.cpp 
    src/colibri_ca.cpp src/PID_controller.cpp src/nav_node_proc.cpp)
 target_link_libraries(nav_mult_rvizgoal yaml-cpp
   ${catkin_LIBRARIES}
 )


