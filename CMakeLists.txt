cmake_minimum_required(VERSION 2.8.3)
project(pro_pilot_park)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
  eigen_conversions
	sensor_msgs
	std_msgs
	)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES pro_pilot_park
	# CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
	# DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)


add_library(pro_pilot_park
	src/pro_pilot_park.cpp
	)
target_link_libraries(pro_pilot_park
	ypspur
	${catkin_LIBRARIES}
	)

add_executable(pro_pilot_park_node
	main/pro_pilot_park_node.cpp)
target_link_libraries(pro_pilot_park_node
	pro_pilot_park
	ypspur
	${catkin_LIBRARIES}
	)
