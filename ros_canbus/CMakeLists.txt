cmake_minimum_required(VERSION 3.0.2)
project(ros_canbus)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

add_message_files(
	DIRECTORY msg
	FILES
  Status_conveyor.msg
  Control_conveyors.msg
  MCU_info.msg
  HCU_info.msg
  PSU_info.msg
  HCU_ledsound_request.msg
  HCU_io_request.msg
  HCU_fieldSick_request.msg
  MCU_io_request.msg
  MCU_safetyRelay_request.msg
  PSU_request.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs )