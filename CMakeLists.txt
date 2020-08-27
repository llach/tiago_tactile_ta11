cmake_minimum_required(VERSION 3.0.2)
project(tiago_tactile_ta11)

find_package(catkin REQUIRED
  rospy
  roscpp
  )

###################################
## catkin specific configuration ##
###################################
catkin_package()

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