cmake_minimum_required(VERSION 3.0.2)
project(image_preprocessing)


##############
## Compiler ##
##############

# C++17 Standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


#######################
## Required packages ##
#######################

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    cv_bridge
    sensor_msgs
    stereo_msgs
    image_geometry
    image_transport
    message_filters
    stereo_image_proc
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


####################
## Create library ##
####################

add_library(${PROJECT_NAME}_core
  src/clahe.cpp
  src/clahs.cpp
  src/dehazer.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)


###################################
## Catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs image_transport
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


#################
## EXECUTABLES ##
#################

# --- Dehazer ---
add_executable(mono_dehazer_node
  src/nodes/mono_dehazer_node.cpp
  src/mono_dehazer.cpp
)
target_link_libraries(mono_dehazer_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_executable(stereo_dehazer_node
  src/nodes/stereo_dehazer_node.cpp
  src/stereo_dehazer.cpp
)
target_link_libraries(stereo_dehazer_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_Thread_LIBRARIES}
)

# --- CLAHE ---
add_executable(mono_clahe_node
  src/nodes/mono_clahe_node.cpp
  src/mono_clahe.cpp
)
target_link_libraries(mono_clahe_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_executable(stereo_clahe_node
  src/nodes/stereo_clahe_node.cpp
  src/stereo_clahe.cpp
)
target_link_libraries(stereo_clahe_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

# --- CLAHS ---
add_executable(mono_clahs_node
  src/nodes/mono_clahs_node.cpp
  src/mono_clahs.cpp
)
target_link_libraries(mono_clahs_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_executable(stereo_clahs_node
  src/nodes/stereo_clahs_node.cpp
  src/stereo_clahs.cpp
)
target_link_libraries(stereo_clahs_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_Thread_LIBRARIES}
)

# --- COLOR CORRECTION ---
add_executable(color_correction_node
  src/nodes/color_correction_node.cpp
  src/color_correction.cpp
)
target_link_libraries(color_correction_node
  ${catkin_LIBRARIES}
)

# --- VIGNETTING ---
add_executable(vignetting_extractor_node
  src/nodes/vignetting_extractor_node.cpp
)
target_link_libraries(vignetting_extractor_node
  ${catkin_LIBRARIES}
)

add_executable(vignetting_remover_node
  src/nodes/vignetting_remover_node.cpp
)
target_link_libraries(vignetting_remover_node
  ${catkin_LIBRARIES}
)


# add_executable(flicker_remover_node
#   src/nodes/flicker_remover_node.cpp
#   src/flicker_remover.cpp)
# target_link_libraries(flicker_remover_node
#   ${catkin_LIBRARIES}
# )


#############
## INSTALL ##
#############

install(TARGETS 
    ${PROJECT_NAME}_core
    mono_dehazer_node
    stereo_dehazer_node
    mono_clahe_node
    stereo_clahe_node
    mono_clahs_node
    stereo_clahs_node
    color_correction_node
    vignetting_extractor_node
    vignetting_remover_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)