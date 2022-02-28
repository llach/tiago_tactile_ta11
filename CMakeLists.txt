cmake_minimum_required(VERSION 3.0.2)
project(tiago_tactile_ta11)

find_package(catkin REQUIRED
  rospy
  roscpp
  tiago_tactile_msgs
  )

#disable_pal_flags()

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

install(PROGRAMS
  scripts/measure.py
  scripts/load_cell_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
