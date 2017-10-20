cmake_minimum_required(VERSION 2.8.3)
project(image_preprocessing)

# C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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

add_executable(stereo_clahs_node
  src/nodes/stereo_clahs_node.cpp
  src/stereo_clahs.cpp
  src/clahs.cpp)
target_link_libraries(stereo_clahs_node
  ${catkin_LIBRARIES}
  ${Boost_Thread_LIBRARIES}
)

add_executable(color_correction_image
  src/nodes/color_correction_image_node.cpp)
target_link_libraries(color_correction_image
  ${catkin_LIBRARIES}
)

# add_executable(flicker_remover_node
#   src/nodes/flicker_remover_node.cpp
#   src/flicker_remover.cpp)
# target_link_libraries(flicker_remover_node
#   ${catkin_LIBRARIES}
# )