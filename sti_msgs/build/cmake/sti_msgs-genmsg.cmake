# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sti_msgs: 70 messages, 0 services")

set(MSG_I_FLAGS "-Isti_msgs:/home/hoang/Documents/shiv/src/sti_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sti_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" "std_msgs/Int16"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" "std_msgs/Int16"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" "sti_msgs/Reconnect_stt"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" "std_msgs/Int16"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" "std_msgs/Bool"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" "geometry_msgs/Quaternion:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" ""
)

get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_custom_target(_sti_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sti_msgs" "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
  "${MSG_I_FLAGS}"
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)
_generate_msg_cpp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(sti_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sti_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sti_msgs_generate_messages sti_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_cpp _sti_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sti_msgs_gencpp)
add_dependencies(sti_msgs_gencpp sti_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sti_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
  "${MSG_I_FLAGS}"
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)
_generate_msg_eus(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(sti_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sti_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sti_msgs_generate_messages sti_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_eus _sti_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sti_msgs_geneus)
add_dependencies(sti_msgs_geneus sti_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sti_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
  "${MSG_I_FLAGS}"
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)
_generate_msg_lisp(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(sti_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sti_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sti_msgs_generate_messages sti_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_lisp _sti_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sti_msgs_genlisp)
add_dependencies(sti_msgs_genlisp sti_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sti_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
  "${MSG_I_FLAGS}"
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)
_generate_msg_nodejs(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(sti_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sti_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sti_msgs_generate_messages sti_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_nodejs _sti_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sti_msgs_gennodejs)
add_dependencies(sti_msgs_gennodejs sti_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sti_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
  "${MSG_I_FLAGS}"
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)
_generate_msg_py(sti_msgs
  "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(sti_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sti_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sti_msgs_generate_messages sti_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg" NAME_WE)
add_dependencies(sti_msgs_generate_messages_py _sti_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sti_msgs_genpy)
add_dependencies(sti_msgs_genpy sti_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sti_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sti_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sti_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(sti_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sti_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sti_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(sti_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sti_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sti_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(sti_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sti_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sti_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(sti_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sti_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sti_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(sti_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
