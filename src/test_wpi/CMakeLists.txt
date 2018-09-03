cmake_minimum_required(VERSION 2.8.3)
project(test_wpi)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

set(wiringPi_include "/usr/local/include")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

LINK_DIRECTORIES("/usr/local/lib")
add_executable(${PROJECT_NAME}_node src/test_wpi_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} wiringPi
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_wpi
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

