cmake_minimum_required(VERSION 3.0.2 ) #2.8.3)
project(sti_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

add_message_files(
	DIRECTORY msg
	FILES
	Velocities.msg
	EMC.msg
	Modbus_0x.msg
	Modbus_4x_read.msg
	Modbus_4x_write.msg

	Map_config.msg
	Request_mapInfo.msg
	List_MapName.msg
	Parameter_mergerMap.msg
	webConsole_receive.msg
	Goal_control.msg
	Check_shelves.msg
	Status_goal_control.msg
	Status_goalControl.msg
	Parking_status.msg
	Parking_control.msg
	Setpose_control.msg
	Setpose_status.msg
	Safety_zone.msg
	Mission_status.msg
	Mission_control.msg 
	#new
	Client_request.msg
	Client_respond.msg
	Move_request.msg
	Move_respond.msg 
	Ps2_msgs.msg
	Read_prepheral.msg
	Write_prepheral.msg
	Status_base.msg
	Motor_parameter.msg
	Motor_rpm.msg


	FL_infoRespond.msg
	FL_infoRequest.msg
	FL_cmdRespond.msg
	FL_cmdRequest.msg  
	NN_cmdRequest.msg 
	NN_infoRespond.msg
	NN_infoRequest.msg

	HMI_papeAuto.msg
	HMI_papeByHand.msg
	HMI_papeLaunch.msg
	HMI_papeDetailFL.msg
	HMI_papeDetailNN.msg
	HMI_papeMessage.msg
	HMI_allButton.msg
	HMI_infoGeneral.msg
	HMI_buttonPassword.msg
	HMI_papePassword.msg

	PID.msg
	Main_missionControl.msg
	Main_status.msg
	Main_switchMode.msg
	Zone_lidar_2head.msg
	
	ManageLaunch.msg
	Port_status.msg
	Reconnect_status.msg
	Reconnect_stt.msg

	Magnetic_line.msg
	RFID.msg

	#hc
	HC_info.msg
	HC_request.msg
	#main
	POWER_info.msg
	POWER_request.msg
	Lift_control.msg	
	Lift_status.msg
	#oc
	Lift_control.msg
	Lift_status.msg
	Imu_version1.msg
	
	#hoang
	APathParking.msg
	PointOfPath.msg

	#new navi
	PointRequestMove.msg
	ListPointRequestMove.msg
	LineRequestMove.msg
	PathInfo.msg
	
	#OC_test_status.msg
)	

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs )
