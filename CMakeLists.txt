cmake_minimum_required(VERSION 2.8.3)
project(a02yyuw_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
	sensor_msgs
	std_msgs
	geometry_msgs
    message_generation
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/sonar.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)