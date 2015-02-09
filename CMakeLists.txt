cmake_minimum_required(VERSION 2.8.3)
project(image_preprocessing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS image_transport cv_bridge roscpp sensor_msgs
  DEPENDS OpenCV
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(dehazer_node src/dehazer_node.cpp src/mono_dehazer.cpp src/dehazer.cpp)
add_executable(stereo_dehazer_node src/stereo_dehazer_node.cpp src/stereo_dehazer.cpp src/dehazer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dehazer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(stereo_dehazer_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS dehazer_node stereo_dehazer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
