cmake_minimum_required(VERSION 3.0.2)
project(tiago_tactile_ta11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  message_generation
  rospy
  roscpp
  std_msgs
  )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TA11.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##

###################################
catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ta11_pub src/nodes/ta11_pub.cpp src/ta11.cpp src/u6.cpp)
target_link_libraries(ta11_pub ${catkin_LIBRARIES} labjackusb)

install(PROGRAMS
  scripts/measure.py
  scripts/load_cell_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)