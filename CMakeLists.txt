cmake_minimum_required(VERSION 3.0.2)
project(video_manipulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package( OpenCV 4 REQUIRED )
find_package( Eigen3 REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
)

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/node.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/node.hpp
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS EIGEN3
)

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${video_manipulator_INCLUDE_DIRECTORIES})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")


###########
## Build ##
###########

include_directories(
  include
  ${video_manipulator_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(video_manipulator ${video_manipulator_SRCS} ${video_manipulator_RCC})
add_dependencies(video_manipulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_manipulator ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############


