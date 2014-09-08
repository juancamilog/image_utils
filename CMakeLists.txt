cmake_minimum_required(VERSION 2.8.3)
project(image_utils)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  camera_calibration_parsers
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES illumination_invariant_filter_nodelet
  CATKIN_DEPENDS roscpp nodelet pluginlib camera_calibration_parsers sensor_msgs image_transport
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(illumination_invariant_filter_nodelet src/illumination_invariant_filter.cpp)
target_link_libraries(illumination_invariant_filter_nodelet ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS illumination_invariant_filter_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY plugins
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

