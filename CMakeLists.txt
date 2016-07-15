cmake_minimum_required(VERSION 2.8.3)
project(image_preprocessing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  cv_bridge
  stereo_msgs
  image_geometry
  stereo_image_proc
  pcl_ros
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Dependencies - PCL:
find_package(PCL REQUIRED)

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
  ${PCL_INCLUDE_DIRS}
)

## Executables
add_executable(dehazer_node
  src/nodes/dehazer_node.cpp
  src/mono_dehazer.cpp
  src/dehazer.cpp)
target_link_libraries(dehazer_node
  ${catkin_LIBRARIES}
)

add_executable(stereo_dehazer_node
  src/nodes/stereo_dehazer_node.cpp
  src/stereo_dehazer.cpp
  src/dehazer.cpp)
target_link_libraries(stereo_dehazer_node
  ${catkin_LIBRARIES}
  ${Boost_Thread_LIBRARIES}
)

add_executable(vignetting_extractor
  src/nodes/vignetting_extractor_node.cpp)
target_link_libraries(vignetting_extractor
  ${catkin_LIBRARIES}
)

add_executable(vignetting_remover
  src/nodes/vignetting_remover_node.cpp)
target_link_libraries(vignetting_remover
  ${catkin_LIBRARIES}
)

add_executable(clahs_node
  src/nodes/clahs_node.cpp
  src/mono_clahs.cpp
  src/clahs.cpp)
target_link_libraries(clahs_node
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS dehazer_node stereo_dehazer_node vignetting_extractor vignetting_remover
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

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)
