cmake_minimum_required(VERSION 2.8.3)
project(colibri_pathproc)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ") 
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(PkgConfig) 
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND) 
	add_definitions(-DHAVE_NEW_YAMLCPP) 
endif(NEW_YAMLCPP_FOUND)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colibri_pathproc
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/usr/local/include)
link_directories(/usr/local/lib)

INCLUDE_DIRECTORIES(include/colibri_pathproc)

add_executable(pathproc_node src/path_proc_test.cpp src/path_proc.cpp)
target_link_libraries(pathproc_node yaml-cpp ${catkin_LIBRARIES})


add_executable(path_handle_node src/path_handle.cpp src/path_proc.cpp)
target_link_libraries(path_handle_node yaml-cpp ${catkin_LIBRARIES})

