cmake_minimum_required(VERSION 2.8.3)
project(emergency_stopper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
  geometry_msgs
  tf
p2os_driver
sensor_msgs
autonomy_leds_msgs
nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf p2os_driver sensor_msgs autonomy_leds_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(emergency_stopper src/emergency_stopper.cpp src/main.cpp)

target_link_libraries(emergency_stopper
   ${catkin_LIBRARIES}
)

install(TARGETS emergency_stopper
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main)
