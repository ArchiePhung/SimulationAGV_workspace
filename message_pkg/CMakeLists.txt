cmake_minimum_required(VERSION 3.0.2)
project(message_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

add_message_files(
  DIRECTORY msg
  FILES
  Status_port.msg
  Status_launch.msg
  Velocities.msg
  MC_status.msg
  Driver_query.msg
  Driver_respond.msg
  Reconnect_sts.msg
  Status_reconnect.msg
  Safety_zone.msg
  Imu_version1.msg
  Status_navi.msg
  Nav350_data.msg
  Parking_request.msg
  Parking_respond.msg
  App_button.msg
  App_button12.msg
  App_color.msg
  App_lbv.msg
  App_launch.msg
  Loadcell_info.msg

  Server_cmdRequest.msg
  Server_cmdRespond.msg
  Server_infoRequest.msg
  Server_infoRespond.msg

  Loadcell_query.msg
  Loadcell_respond.msg

  Reflector_data.msg
  Reflector_array.msg
  
  Control_conveyors.msg
  RTC_received.msg
  CPD_write.msg
  CPD_read.msg
  CAN_received.msg
  CAN_send.msg
  CAN_status.msg
  RTC_send.msg
  RTC_status.msg
  Signal_conveyor.msg
  
  Navigation_query.msg
  Navigation_respond.msg

  Signal_AGVToyo.msg
  Signal_CYMToyo.msg

  CY_respond.msg
  
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES message_pkg
  CATKIN_DEPENDS message_runtime roscpp rospy
  # DEPENDS system_lib
)
