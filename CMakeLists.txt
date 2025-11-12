cmake_minimum_required(VERSION 3.0.2)
project(image_preprocessing)


##############
## Compiler ##
##############

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++20, C++17 or C++14 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    add_definitions(-DCOMPILEDWITHC20)
    message(" Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(" Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(" Using flag -std=c++14.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20, C++17 or C++14 support. image_preprocessing needs at least C++14 support. Please use a different C++ compiler.")
endif()


#######################
## Required packages ##
#######################

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

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)

# Dependencies - Boost:
find_package(Boost REQUIRED COMPONENTS thread)

# Dependencies - OpenMP:
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Dependencies - PCL:
find_package(PCL REQUIRED)

###################################
## Catkin specific configuration ##
###################################

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

add_executable(color_correction_node
  src/nodes/color_correction_node.cpp
  src/color_correction.cpp)
target_link_libraries(color_correction_node
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

add_executable(clahe_node
  src/nodes/clahe_node.cpp)
target_link_libraries(clahe_node
  ${catkin_LIBRARIES}
)

# add_executable(flicker_remover_node
#   src/nodes/flicker_remover_node.cpp
#   src/flicker_remover.cpp)
# target_link_libraries(flicker_remover_node
#   ${catkin_LIBRARIES}
# )