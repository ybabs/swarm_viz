cmake_minimum_required(VERSION 2.8.3)
project(m100_odom_adapter)

## Compile as C++11, supported in ROS Kinetic and newer
 #add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(DJIOSDK REQUIRED)





catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dji_odom_adapter
#  CATKIN_DEPENDS dji_sdk nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include ${EIGEN3_INCLUDE_DIR}
)


add_executable(dji_odom_publisher src/odom_msg_adapter.cpp )


target_link_libraries(dji_odom_publisher
		${catkin_LIBRARIES}
		${EIGEN_LIBRARIES}
                ${DJIOSDK_LIBRARIES}
		)

add_dependencies(dji_odom_publisher dji_sdk_generate_messages_cpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)





