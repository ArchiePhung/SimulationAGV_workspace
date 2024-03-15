#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phung Quy Duong(Archie)
Company: STI Viet Nam
Date  : 25/12/2023
					
"""

import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8, Bool
from message_pkg.msg import *
from ros_canbus.msg import *
from sensor_msgs.msg import Imu
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

import json

class Velocities:
	def __init__(self, vel_x = 0.0, vel_y = 0.0, vel_r = 0.0):
		self.x = vel_x # - m/s
		self.y = vel_y # - m/s
		self.r = vel_r # - rad/s

class GoalFollow:
	def __init__(self):
		self.id = 0
		self.pose = Pose()			# - Pose()
		
		self.angleLine = 0 			# - rad (5 | not care)
		self.directionTravel = 0 	# 
		self.roadWidth = 0 			# - m  (-1 | not care)
		
		self.angleFinal = 0 		# - rad (5 | not care)
		self.deltaAngle = 0 		# - rad.
		self.speed = 0     			# - (0.2 - 0.8 m/s) (not care - 0).

#--------------------------------------------------------------------------------- ROS
class ros_control():
	def __init__(self):
		rospy.init_node('stiControl', anonymous=False)
		self.rate = rospy.Rate(60)
		# -------------- Parameter -------------- #
		self.name_card = "wlp3s0"
		self.address = "172.21.16.224" # "172.21.15.224"

		# -- App
		rospy.Subscriber("/app_button", App_button12, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
		self.app_button = App_button12()

		# -- Cancel mission
		rospy.Subscriber("/cancelMission_control", Int16, self.callback_cancelMission)
		self.cancelMission_control = Int16()		
		self.flag_cancelMission = 0
		self.status_cancel = 0
		# -
		self.pub_cancelMission = rospy.Publisher("/cancelMission_status", Int16, queue_size= 20)	
		self.cancelMission_status = Int16()

		# -------------- Cac ket noi ngoai vi -------------- #
		# -- Reconnect
		rospy.Subscriber("/status_reconnect", Status_reconnect, self.callback_reconnect)
		self.status_reconnect = Status_reconnect()

		# -- Board RTC
		rospy.Subscriber("/CAN_received", CAN_received, self.callback_RTC) 
		self.timeStampe_RTC = rospy.Time.now()

		# -- Board MAIN - POWER
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.main_info = POWER_info()
		self.timeStampe_main = rospy.Time.now()
		self.voltage = 24.5
		# -
		self.pub_requestMain = rospy.Publisher("/POWER_request", POWER_request, queue_size= 20)	
		self.power_request = POWER_request()

		# -- BATTERY INFO 
		rospy.Subscriber("/Battery_info", Battery_info, self.callback_Battery)
		self.battery_info = Battery_info()
		self.timeStampe_battery = rospy.Time.now()

		self.pub_pin = rospy.Publisher("/Pin_info", Pin_info, queue_size= 20)	
		self.pin_info = Pin_info()			

		# -- Board HC 82
		rospy.Subscriber("/HC_info", HC_info, self.callback_HC, queue_size= 30) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.HC_info = HC_info()
		self.timeStampe_HC = rospy.Time.now()
		# -
		self.pub_controlHC = rospy.Publisher("/HC_request", HC_request, queue_size= 20)	# dieu khien den bao va den ho tro camera
		self.HC_request = HC_request()

		# -- Board OC 12 | conveyor11
		rospy.Subscriber("/signal_conveyor11", Signal_conveyor, self.callback_conveyor11) # 
		self.signal_conveyor11 = Signal_conveyor()
		self.timeStampe_conveyor2 = rospy.Time.now()

		# -- Board OC 12 | conveyor12
		rospy.Subscriber("/signal_conveyor12", Signal_conveyor, self.callback_conveyor12) # 
		self.signal_conveyor12 = Signal_conveyor()

		# -- Board OC 34 | conveyor21
		rospy.Subscriber("/signal_conveyor21", Signal_conveyor, self.callback_conveyor21) # 
		self.signal_conveyor21 = Signal_conveyor()
		self.timeStampe_conveyor1 = rospy.Time.now()

		# -- Board OC 34 | conveyor22
		rospy.Subscriber("/signal_conveyor22", Signal_conveyor, self.callback_conveyor22) # 
		self.signal_conveyor22 = Signal_conveyor()

		self.listMission_conveyor = [0, 0, 0, 0, 0, 0]
		self.listMission_completed = [0, 0, 0, 0, 0, 0]

		# - Board OC | Control Conveyors
		self.pub_controlConveyors = rospy.Publisher("/control_conveyors", Control_conveyors, queue_size= 20)	# Dieu khien ban nang.
		self.control_conveyors = Control_conveyors()

		# -- Board CPD | Toyo
		rospy.Subscriber("/signal_CYMToyo", Signal_CYMToyo, self.callback_CYMToyo) # 
		self.signal_CYMToyo = Signal_CYMToyo()
		self.timeStampe_CYMToyo = rospy.Time.now()
		# -
		self.pub_AGVToyoSignal = rospy.Publisher("/AGVToyo_signal", Signal_AGVToyo, queue_size= 20)	# Dieu khien Toyo.
		self.AGVToyo_signal = Signal_AGVToyo()
		# -------------------

		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.callback_nav350) 
		self.nav350_data = Nav350_data()

		# -- Data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.callback_safety_NAV) 
		self.safety_NAV = Int8()

		# -- robotPose_nav - POSE
		rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_getPoseRobot, queue_size = 20)
		self.robotPose_nav = PoseStamped()

		# -- Port physical
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.status_port = Status_port()

		# -------------- Cac node thuat toan dieu khien --------------
		# -- Communicate with Server
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_Traffic_infoRequest)
		self.Traffic_infoRequest = NN_infoRequest()
		self.timeStampe_TrafficReceived = rospy.Time.now()
		# -
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.callback_Traffic_cmdRequest)
		self.Traffic_cmdRequest = NN_cmdRequest()

		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()
		self.serverCmd_now = Server_cmdRequest()

		# -- add 18/01/2022 : Sua loi di lai cac diem cu khi mat ket noi server.
		self.listID_old = [0, 0, 0, 0, 0]
		self.flag_listPoint_ok = 0
		# -
		self.pub_infoRespond = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size= 120)
		self.Traffic_infoRespond = NN_infoRespond()

		# - Navigation
		rospy.Subscriber("/navigation_respond", Navigation_respond, self.callback_navigation, queue_size = 120)
		self.navigation_respond = Navigation_respond() # sub from move_base
		self.timeStampe_navigationRespond = rospy.Time.now()
		# -
		self.pub_navigationQuery = rospy.Publisher("/navigation_query", Navigation_query, queue_size= 60)
		self.navigation_query = Navigation_query()
		self.enable_moving = 0 # cho phep navi di chuyen

		# -- Parking
		rospy.Subscriber("/parking_respond", Parking_respond, self.callback_parking)
		self.parking_status = Parking_respond()
		self.parking_offset = 0.0
		self.parking_poseBefore = Pose()
		self.parking_poseTarget = Pose()
		# -
		self.pub_parking = rospy.Publisher("/parking_request", Parking_request, queue_size= 20)
		self.enable_parking = 0
		self.parking_poseBefore = Pose()
		self.parking_poseAfter = Pose()
		self.parking_offset = 0.0

		# -- Cmd_vel
		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 20. 

		# -- Driver Motor
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1)
		self.driver1_respond = Driver_respond()
		# - 
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2)
		self.driver2_respond = Driver_respond()
		self.timeStampe_driver = rospy.Time.now()
		# -
		self.pub_taskDriver = rospy.Publisher("/task_driver", Int16, queue_size= 20)
		self.task_driver = Int16()
		# -
		self.pub_disableBrake = rospy.Publisher("/disable_brake", Bool, queue_size= 20)
		self.disable_brake = Bool()
		# - Task driver
		self.taskDriver_nothing = 0
		self.taskDriver_resetRead = 1
		self.taskDriver_Read = 2
		self.task_driver.data = self.taskDriver_Read

		# --------------------- Parameter p --------------------- #
		# -- Hz
		self.FrequencePubBoard = 10.
		self.pre_timeBoard = rospy.get_time()
		# --
		self.FrequencePub = 15.
		self.pre_timePub = rospy.get_time()
		# -- Mode operate
		self.MODE_MANUAL = 1
		self.MODE_AUTO = 2
		self.mode_operate = self.MODE_MANUAL    # Lưu chế độ hoạt động.

		# -- Target
		self.target_id = 0
		self.target_x = 0.		 # lưu tọa độ điểm đích hiện tại.			
		self.target_y = 0.
		self.target_z = 0.
		self.target_tag = 0.
		# - 
		self.process = 0         # - Tiến trình đang xử lý
		self.mission_before = 0  # - Nhiệm vụ trước khi di chuyển.
		self.mission_after  = 0  # - Nhiệm vụ sau khi di chuyển.

		self.enable_mission = 0
		# -
		self.completed_before_mission = 0	        # Báo nhiệm vụ trước đã hoàn thành.
		self.completed_after_mission = 0
		self.completed_after_mission1 = 0	        # Báo nhiệm vụ sau đã hoàn thành.
		self.completed_move = 0			 	        # Báo di chuyển đã hoàn thành.
		self.completed_moveOut = 0      	        # Sau khi đi vào sạc -> Phải đi thẳng ra.
		self.completed_moveSimple = 0      	        # bao da den dich.
		self.completed_moveSpecial = 0     	        # bao da den aruco.
		self.completed_reset = 0                    # hoan thanh reset.
		self.completed_checkConveyors_before = 0 	# kiem tra ke co hay ko sau khi nang. 
		self.completed_checkConveyors_after = 0 	# kiem tra ke co hay ko sau khi nang. 
		self.completed_checkRack = 0
		# -- Flag
		self.flag_afterChager = 0
		self.flag_Byhand_to_Auto = 0
		self.flag_Auto_to_Byhand = 0
		self.flag_read_client = 0
		self.flag_error = 0
		self.flag_warning = 0
		# -
		self.flagWarning_rack11 = 0
		self.flagWarning_rack12 = 0
		self.flagWarning_rack21 = 0
		self.flagWarning_rack22 = 0
		# -
		self.flagWarning_receivedRack11 = 0
		self.flagWarning_receivedRack12 = 0
		self.flagWarning_receivedRack21 = 0
		self.flagWarning_receivedRack22 = 0
		# -
		self.saveTime_warningRack11 = rospy.get_time()
		self.saveTime_warningRack12 = rospy.get_time()
		self.saveTime_warningRack21 = rospy.get_time()
		self.saveTime_warningRack22 = rospy.get_time()

		# --
		self.pre_mess = ""                                       # lưu tin nhắn hiện tại.
		# -- Status to server
		self.statusAGV = 0
		self.STTAGV_ALLRIGHT = 0
		self.STTAGV_WARNING = 1
		self.STTAGV_ERROR = 2	
		self.STTAGV_CANCELMISSION = 5	
		# -- Status to detail to follow:
		self.stf = 0
		self.stf_wakeup = 0
		self.stf_running_simple = 1
		self.stf_stop_obstacle = 2
		self.stf_running_speial = 3
		self.stf_running_backward = 4
		self.stf_performUp = 5
		self.stf_performDown = 6
		# -- EMC reset
		self.EMC_RESETON = 1
		self.EMC_RESETOFF = 0
		self.EMC_reset = self.EMC_RESETOFF
		# -- EMC write
		self.EMC_WRITEON = 1
		self.EMC_WRITEOFF = 0
		self.EMC_write = self.EMC_WRITEOFF
		# -- Led
		self.led_effect = 0
		self.LED_ERR = 1 			# 1
		self.LED_SIMPLERUN = 2 		# 2
		self.LED_SPECIALRUN = 3 	# 3
		self.LED_PERFORM = 4 		# 4
		self.LED_COMPLETED = 5  	# 5	
		self.LED_STOPBARRIER = 6  	# 6
		# -- Mission server
		self.STTTASK_LIFTERR = 64 # trang thái nâng kệ nhueng ko có kệ.
		self.SERVERMISSION_LIFTUP = 65
		self.SERVERMISSION_LIFTDOWN = 66
		self.SERVERMISSION_CHARGER = 5
		self.SERVERMISSION_UNK = 0
		self.SERVERMISSION_LIFTDOWN_CHARGER = 6
		# -- Conveyor task.
		self.CYTASK_RECEIVE = 1
		self.CYTASK_TRANSMIT = 2
		self.CYTASK_STOP = 0
		self.conveyor11_taskByHand = self.CYTASK_STOP
		self.conveyor12_taskByHand = self.CYTASK_STOP
		self.conveyor21_taskByHand = self.CYTASK_STOP
		self.conveyor22_taskByHand = self.CYTASK_STOP
		# - Conveyor Status.
		self.CYTASKRCV_DONE = 3               # 3 nhận xong, 4 trả xong                             # Archie: update scripts of OC board
		self.CYTASKTRSM_DONE = 4
		self.CYTASK_UNDONE = 0
		self.CYTASK_RUNNING = 1
		self.CYTASKRCV_TIMERR = -1
		self.CYTASKRCV_UNKERR = -3

		self.CYTASKTRSM_TIMERR = -2
		self.CYTASKTRSM_UNKERR = -4
		# -
		self.allowTestNextCY = 1
		# -- Speaker
		self.speaker_effect = 0
		self.speaker_requir = 0 # luu trang thai cua loa
		self.SPK_ERR = 3
		self.SPK_MOVE = 1
		self.SPK_NOT = 4		
		self.SPK_WARN = 2
		self.SPK_OFF = 0
		self.enable_speaker = 1
		# -- Charger
		self.CHARGER_ON = 1
		self.CHARGER_OFF = 0		
		self.charger_requir = self.CHARGER_OFF
		self.charger_write = self.charger_requir
		self.charger_valueOrigin = 10.0           #modify 31/5/2023 0-> 10
		# -- Voltage
		self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
		self.timeCheckVoltage_normal = 10     # s
		self.pre_timeVoltage = rospy.get_time()   # s
		self.valueVoltage = 0
		self.step_readVoltage = 0
		# -- Error Type
		self.error_move = 0
		self.error_perform = 0
		self.error_device = 0  # camera(1) - MC(2) - Main(3) - SC(4)

		self.numberError = 0
		self.lastTime_checkLift = 0.0
		# -- add new
		self.enb_debug = 0
		# --
		self.listError = []
		self.job_doing = 0
		# -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
		# -- Pose tai vi tri Ke, sac
		self.poseCharger = Pose()
		self.distance_resetMission = 0.15
		self.flag_resetFramework = 0
		# --
		self.flag_stopMove_byHand = 0
		# --
		self.timeStampe_reflectors = rospy.Time.now()
		self.pose_parkingRuning = Pose()
		# -- 
		self.getOutCharge_pose = Pose() # - Lưu vị trí khi đi ra khỏi sạc.
		self.flag_requirOutCharge = 0
		# -- add 19/01/2022 : Check error lost server.
		self.saveTime_checkServer = rospy.Time.now()
		self.saveStatus_server = 0
		# -- add 22/01/2022
		self.flag_notCharger = 0
		# -- add 15/04/2022
		self.flag_listPointEmpty = 0
		# --
		self.timeStampe_move = rospy.Time.now()
		self.flag_Auto = 0
		self.poseRobot_now = Pose()
		self.step_moveSimple = 0
		self.linear_min = 0.01
		self.linear_max = 0.24
		self.deceleration_distance = 1.2
		self.step_moveSimple = 0
		self.step_moveSpecial = 0
		self.goalFollow_now = GoalFollow()
		# - Di chuyển bằng đầu hay đuôi.
		self.moveBy = 0
		self.MOVEBY_AHEAD = 0
		self.MOVEBY_TAIL = 1
		print (" --- Launch ---")
		# -- Driver Motor
		rospy.Subscriber("/auto_now", Bool, self.callback_autoNow)
		self.goalTarget = GoalFollow() # - Đích của các vị trí thao tác: Sạc Pin, Nhận/Trả hàng.
		self.goal_moveOut = GoalFollow()
		# - 
		self.flag_blsockCollide = 0
		self.saveTime_blsockCollide = rospy.Time.now()
		# -
		self.saveTime_EMG = rospy.Time.now()
		self.saveTime_resetFrameWork = rospy.Time.now()

		# - Flag announce Conveyor machine error 
		self.btValue_linkCY = 0
		self.flagWarning_posError = 0                              # cờ báo lỗi
		self.flag_CYmachine_readyError = 0                         # không cần thiết, nó sẽ quy về hết thằng bit8_error
		self.flag_CYmachine_generalError = 0

		# - Flag announce Conveyor AGV error
		self.flag_CY11_transmitError_Blank = 0
		self.flag_CY11_receiveError_Full = 0
		self.flag_CY12_transmitError_Blank = 0
		self.flag_CY12_receiveError_Full = 0
		self.flag_CY21_transmitError_Blank = 0
		self.flag_CY21_receiveError_Full = 0
		self.flag_CY22_transmitError_Blank = 0
		self.flag_CY22_receiveError_Full = 0

		self.flag_transmitError_Blank = 0		

		# - Mission case
		self.CASE_MISSION_SIMPLE = 0
		self.CASE_MISSION_SPECIAL = 1
		self.case_mission_type = self.CASE_MISSION_SIMPLE
		# self.case_mission1 = 1
		# self.case_mission2 = 1
		self.case_step1 = 1
		self.case_step2 = 1	
		self.mission_val = 0

		self.flag_runAgain = 0
		
	# ------------------------ Call Back ------------------------ #
	def callback_autoNow(self, data):
		self.flag_Auto = data.data

	def callback_appButton(self, data):
		self.app_button = data

	def callback_cancelMission(self, data):
		self.cancelMission_control = data

	def callback_reconnect(self, data):
		self.status_reconnect = data

	def callback_RTC(self, data):
		self.timeStampe_RTC = rospy.Time.now()

	def callback_Main(self, data):
		self.main_info = data
		self.voltage = round(self.main_info.voltages, 2)
		self.timeStampe_main = rospy.Time.now()

	def callback_Battery(self, data):
		self.battery_info = data
		self.timeStampe_battery = rospy.Time.now()
		# self.pin_info.pinState = self.battery_info.pinState
		# self.pin_info.pinVolt = self.battery_info.pinVolt
		# self.pin_info.pinCurr = self.battery_info.pinCurr
		# self.pin_info.pinPercent = self.battery_info.pinPercent
		# self.pin_info.timeCharge = self.convert_intTotime(self.battery_info.timeCharge)
		# self.pin_info.timeChargePropose = self.convert_intTotime(self.battery_info.timeChargePropose)
		# self.pub_pin.publish(self.pin_info)

	def callback_HC(self, data):
		# print ("HC read")
		self.HC_info = data
		self.timeStampe_HC = rospy.Time.now()
		if self.HC_info.vacham == 0:
			self.saveTime_blsockCollide = rospy.Time.now()
		self.detect_blsockCollide()

	def callback_conveyor12(self, data):
		self.signal_conveyor12 = data
		self.timeStampe_conveyor2 = rospy.Time.now()

	def callback_conveyor22(self, data):
		self.signal_conveyor22 = data
		self.timeStampe_conveyor2 = rospy.Time.now()

	def callback_conveyor11(self, data):
		self.signal_conveyor11 = data
		self.timeStampe_conveyor1 = rospy.Time.now()

	def callback_conveyor21(self, data):
		self.signal_conveyor21 = data
		self.timeStampe_conveyor1 = rospy.Time.now()

	def callback_CYMToyo(self, data):
		self.signal_CYMToyo = data
		# self.timeStampe_CYMToyo = rospy.Time.now()

	def callback_nav350(self, data):
		self.nav350_data = data

	def callback_safety_NAV(self, data):
		self.safety_NAV = data

	def callback_getPoseRobot(self, data):
		self.robotPose_nav = data
		# doi quaternion -> rad    
		quaternion1 = (data.pose.orientation.x, data.pose.orientation.y,\
					data.pose.orientation.z, data.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion1)

		self.Traffic_infoRespond.x = round(data.pose.position.x, 3)
		self.Traffic_infoRespond.y = round(data.pose.position.y, 3)
		self.Traffic_infoRespond.z = round(euler[2], 3)

	def callback_port(self, data):
		self.status_port = data

	def callback_Traffic_infoRequest(self, data):
		self.Traffic_infoRequest = data
		self.timeStampe_TrafficReceived = rospy.Time.now()

	def callback_Traffic_cmdRequest(self, dat):
		self.Traffic_cmdRequest = dat
		self.timeStampe_TrafficReceived = rospy.Time.now()
		# self.flag_listPoint_ok = 0
		
	def callback_server_cmdRequest(self, data):
		# - add 20/06/2023
		if data.target_id > 0:
			self.server_cmdRequest = data
		self.timeStampe_TrafficReceived = rospy.Time.now()
		# -
		if self.listID_old != self.server_cmdRequest.list_id:
			self.flag_listPoint_ok = 0

	def callback_navigation(self, data):
		self.navigation_respond = data
		self.timeStampe_navigationRespond = rospy.Time.now()

	def callback_parking(self, data):
		self.parking_status = data

	def callback_driver1(self, data):
		self.driver1_respond = data
		self.timeStampe_driver = rospy.Time.now()

	def callback_driver2(self, data):
		self.driver2_respond = data
		self.timeStampe_driver = rospy.Time.now()

	# ------------------------   ------------------------ #
	def pub_park(self, modeRun, poseBefore, poseTarget, offset):
		park = Parking_request()
		park.modeRun = modeRun
		park.poseBefore = poseBefore
		park.poseTarget = poseTarget
		park.offset = offset

		self.pub_parking.publish(park)

	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
			self.pub_vel.publish(twist)
		# else :
			# rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

	def pub_Main(self, charge, sound, EMC_write, EMC_reset):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		if sound == 0:
			mai.sound_on = 0	
		else:
			mai.sound_on = 1
			mai.sound_type = sound
		mai.EMC_write = EMC_write
		mai.EMC_reset = EMC_reset
		
		self.pub_requestMain.publish(mai)

	def getPose_from_offset_old(self, pose_in, offset):
		pose_out = Pose()
		angle = self.quaternion_to_euler(pose_in.orientation)

		if (angle >= 0):
			angle_target = angle - pi
		else:
			angle_target = pi + angle

		pose_out.position.x = pose_in.position.x + cos(angle_target)*offset
		pose_out.position.y = pose_in.position.y + sin(angle_target)*offset

		pose_out.orientation = self.euler_to_quaternion(angle_target)
		return pose_out

	def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		d_x = pointB.x - pointA.x
		d_y = pointB.y - pointA.y
		ang = 0
		if (d_x == 0):
			if (d_y >= 0):
				ang = pi/2.
			else:
				ang = -pi/2.
		else:
			if (d_y == 0):
				if (d_x > 0):
					ang = 0
				else:
					ang = pi
			else:
				ang = atan2(d_y, d_x)
				if (ang > pi):
					ang = ang - 2*pi
				if (ang < -pi):
					ang = 2*pi + ang
		return ang 

	# -- add 15/04/2022
	def check_listPoints(self, list_id):
		count = 0
		try: 
			for i in range(5):
				if (list_id[i] != 0):
					count += 1

			if count != 0:
				return 1
			else: 
				return 0
		except:
			return 0

	def log_mess(self, typ, mess, val):
		# -- add new
		if (self.enb_debug):
			if self.pre_mess != mess:
				if typ == "info":
					rospy.loginfo (mess + ": %s", val)
				elif typ == "warn":
					rospy.logwarn (mess + ": %s", val)
				else:
					rospy.logerr (mess + ": %s", val)
			self.pre_mess = mess

	def troubleshoot_mess(self, typ, mess, val):
		if typ == "info":
			rospy.loginfo (mess + ": %s", val)
		elif typ == "warn":
			rospy.logwarn (mess + ": %s", val)
		else:
			rospy.logerr (mess + ": %s", val)

# 	def do_cancelMisssion(self):
# 		if self.flag_cancelMission == 0:
# 			self.status_cancel = 0
# 		else:
# 			self.status_cancel = 1

	# -- add 19/01/2020
	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			# print ("address: ", address)
			return 1
		except Exception:
			return 0

	def pingServer(self, address):
			# - ping server
			try:
				ping = subprocess.check_output("ping -{} 1 {}".format('c',address), shell=True)
				# print(ping)
				vitri = str(ping).find("time")
				time_ping = str(ping)[(vitri+5):(vitri+9)]
				return float(time_ping)
			except Exception:
				# print("no ping")			
				return -1

	def check_server(self):
		is_ip = self.get_ipAuto(self.name_card)
		# is_ip = 1
		# time_ping = self.pingServer(self.address)
		time_ping = 0
		if (is_ip == 1):
			if (time_ping == -1):
				return 1 # khong Ping dc server
			else:
				return 0 # oki
		else:
			return 2 # khong lay dc IP

	def point_same_point(self, x1, y1, z1, x2, y2, z2):
		# tọa độ
		x = x2 - x1
		y = y2 - y1
		d = math.sqrt(x*x + y*y)
		# góc 
		if z2*z1 >= 0:
			z = z2 - z1
		else:
			z = z2 + z1
		if d > 0.2 or abs(z) > 0.14:  # 20 cm - ~ 20*C
			return 1
		else:
			return 0

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def constrain(self, value_in, value_min, value_max):
		value_out = 0.0
		if (value_in < value_min):
			value_out = value_min
		elif (value_in > value_max):
			value_out = value_max
		else:
			value_out = value_in
		return value_out

	def readbatteryVoltage(self): # 
		time_curr = rospy.get_time()
		delta_time = (time_curr - self.pre_timeVoltage)
		if self.charger_requir == self.CHARGER_ON:
			self.flag_afterChager = 1
			if self.step_readVoltage == 0:  # bat sac.
				self.charger_write = self.CHARGER_ON
				if (delta_time > self.timeCheckVoltage_charger):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.CHARGER_OFF
				if (delta_time > self.timeCheckVoltage_normal*15):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.main_info.voltages, 1)*10
				# print "charger --"
				if  bat > 255:
					self.valueVoltage = 255
				elif bat < 0:
					self.valueVoltage = 0
				else:
					self.valueVoltage = bat

				self.pre_timeVoltage = time_curr
				self.step_readVoltage = 3

			elif self.step_readVoltage == 3: # doi.
				if (delta_time > 2):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 0

		elif self.charger_requir == self.CHARGER_OFF:
			if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
				self.pre_timeVoltage = time_curr
				self.flag_afterChager = 0
				self.charger_write = self.CHARGER_OFF
				self.step_readVoltage = 0
			else:
				if (delta_time > self.timeCheckVoltage_normal):
					self.pre_timeVoltage = time_curr
					bat = round(self.main_info.voltages, 1)*10
					# print "normal --"
					if  bat > 255:
						self.valueVoltage = 255
					elif bat < 0:
						self.valueVoltage = 0
					else:
						self.valueVoltage = int(bat)

	def calculate_angle(self, qua1, qua2): # geometry_msgs/Orientation
		euler1 = self.quaternion_to_euler(qua1)
		euler2 = self.quaternion_to_euler(qua2)

		delta_angle = euler2 - euler1
		if (abs(delta_angle) >= pi):
			if (delta_angle >= 0):
				delta_angle = (pi*2 - abs(delta_angle))*(-1)
			else:
				delta_angle = pi*2 - abs(delta_angle)
		return delta_angle

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def getPose_from_offset(self, goal_x, goal_y, goal_z, offset):
		# print ("First -X: " + str(goal_x) + " |Y: " + str(goal_y) + " |R: " + str(goal_z) + " |Of: "  + str(offset))
		pose_out = Pose()
		# print ("Fir: ", goal_z)
		angle_target = goal_z + pi
		angle_target = self.limitAngle(angle_target)
		# print ("Fir2: ", angle_target)
		# -
		pose_out.position.x = goal_x + cos(angle_target)*offset
		pose_out.position.y = goal_y + sin(angle_target)*offset
		pose_out.orientation = self.euler_to_quaternion(angle_target)
		# print ("After -X: " + str(pose_out.position.x) + " |Y: " + str(pose_out.position.y) + " |R: " + str(angle_target) + " |Of: "  + str(offset))
		return pose_out

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0

	def getBit_missionConveyor(self, byte, pos):
		return 0

	# --
	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	# --
	def getBit_fromInt16(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(16):
			bit_out = value_now%2
			value_now = int(value_now/2)
			if i == pos:
				return bit_out

			if value_now < 1:
				return 0		
		return 0

	# -- 
	def convert_intTotime(self, time):
		str_time = ""
		time_hour = int(time/3600)
		time = time - time_hour*3600
		time_minute = int(time/60)
		time_second = time - time_minute*60

		if time_hour == 0:
			if time_minute == 0:
				str_time = str(time_second) + "s"
			else:
				str_time = str(time_minute) + "m" + str(time_second) + "s"
		else:
			str_time = str(time_hour) + "h" + str(time_minute) + "m" + str(time_second) + "s"
			
		return str_time
	# ------------------------ Detect Lost ------------------------ #
	def detectLost_navigation(self):
		delta_t = rospy.Time.now() - self.timeStampe_navigationRespond
		if (delta_t.to_sec() > 1.0):
			return 1
		return 0

	def detectLost_driver(self):
		delta_t = rospy.Time.now() - self.timeStampe_driver
		if (delta_t.to_sec() > 0.6):
			return 1
		return 0

	def detectLost_RTC(self):
		delta_t = rospy.Time.now() - self.timeStampe_RTC
		if delta_t.to_sec() > 0.6:
			return 1
		return 0

	def detectLost_HC(self):
		delta_t = rospy.Time.now() - self.timeStampe_HC
		if delta_t.to_sec() > 0.6:
			return 1
		return 0

	def detectLost_OC12(self):
		delta_t = rospy.Time.now() - self.timeStampe_conveyor1
		if delta_t.to_sec() > 4.0:
			return 1
		return 0

	def detectLost_OC34(self):
		delta_t = rospy.Time.now() - self.timeStampe_conveyor2
		if delta_t.to_sec() > 4.0:
			return 1
		return 0

	def detectLost_Main(self):
		delta_t = rospy.Time.now() - self.timeStampe_main
		if delta_t.to_sec() > 4.0:
			return 1
		return 0

	def detectLost_Battery(self):
		delta_t = rospy.Time.now() - self.timeStampe_battery
		if delta_t.to_sec() > 4.0:
			return 1
		return 0

	def detectLost_nav(self):
		delta_t = rospy.Time.now() - self.nav350_data.header.stamp
		if delta_t.to_sec() > 0.6:
			return 1
		return 0

	def detectLost_poseRobot(self):
		delta_t = rospy.Time.now() - self.robotPose_nav.header.stamp
		if delta_t.to_sec() > 0.6:
			return 1
		return 0

	def detect_blsockCollide(self):
		delta_t = rospy.Time.now() - self.saveTime_blsockCollide
		if delta_t.to_sec() > 0.6:
			self.flag_blsockCollide = 1

	def detect_EMG(self):
		if self.main_info.EMC_status == 0:
			self.saveTime_EMG = rospy.Time.now()
		# -
		delta_t = rospy.Time.now() - self.saveTime_EMG
		if delta_t.to_sec() > 0.6:
			return 1
		else:
			return 0

	# -- add 19/01/2022
	def detectLost_server(self):
		delta_t = rospy.Time.now() - self.timeStampe_TrafficReceived
		if delta_t.to_sec() > 15:
			delta_s = rospy.Time.now() - self.saveTime_checkServer
			if delta_s.to_sec() > 5:
				self.saveTime_checkServer = rospy.Time.now()
				self.saveStatus_server = self.check_server()

			if self.saveStatus_server == 1:
				return 2
			elif self.saveStatus_server == 2:
				return 3
			return 1
		return 0

	def detectLost_reflectors(self):
		# -- so luong guong
		if self.nav350_data.number_reflectors >= 3: # loi mat guong
			self.timeStampe_reflectors = rospy.Time.now()

		delta_t = rospy.Time.now() - self.timeStampe_reflectors
		if delta_t.to_sec() > 1.2:
			return 1
		return 0

	def find_stopAvoiding(self, textIn):
		length = len(textIn)
		vitri = textIn.find("Dung tranh :")
		# print (textIn)
		# print ("vitri: ", vitri)
		if vitri == -1:
			return 0
		return 1

	# ------------------------ ------------------------ #
	def synthetic_error(self):
		listError_now = []
		# -- EMG          # Archie add 10/01/2023
		# if self.main_info.EMC_status == 1:
		# if self.detect_EMG() == 1:
		# 	listError_now.append(121)
		# else:
		# 	# -- Lost Driver 1
		# 	if self.detectLost_driver() == 1: 
		# 		listError_now.append(251)

		# 	# -- Error Driver 1
		# 	summation1 = self.driver1_respond.alarm_all + self.driver1_respond.alarm_overload + self.driver1_respond.warning
		# 	if (summation1 != 0):
		# 		listError_now.append(252)

		# 	# -- Lost Driver 2
		# 	if self.detectLost_driver() == 1: 
		# 		listError_now.append(261)

		# 	# -- Error Driver 2
		# 	summation2 = self.driver2_respond.alarm_all + self.driver2_respond.alarm_overload + self.driver2_respond.warning
		# 	if (summation2 != 0):
		# 		listError_now.append(262)

		# -- Va cham      # Archie add 10/01/2023
		# self.detect_blsockCollide()
		# if self.flag_blsockCollide == 1:
		# 	listError_now.append(122)
  
		# -- Lost RTC Board.   
		# if (self.detectLost_RTC() == 1):       # Archie add 10/01/2023
		# 	listError_now.append(311)
		# else:
		# 	# -- Error RTC Board: CAN not Send.
		# 	# if (self.() == 1):
		# 	# 	listError_now.append(312)

		# 	# -- Lost Main Board
		# 	if (self.detectLost_Main() == 1):
		# 		listError_now.append(321)

		# 	# -- Lost HC Board.
		# 	if self.detectLost_HC() == 1:
		# 		listError_now.append(351)

		# 	# -- Lost OC Board No.12
		# 	if (self.detectLost_OC12() == 1):
		# 		listError_now.append(344)

		# 	# -- Lost OC Board No.34
		# 	if self.detectLost_OC34() == 1:
		# 		listError_now.append(345)

		# -- Battery info
		# if self.detectLost_Battery() == 1:
		# 	listError_now.append(283)

		# -- Goal Control
		if self.detectLost_navigation() == 1:
			listError_now.append(282)
			self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

		# -- Lost Nav350
		# if (self.detectLost_nav() == 1):           # Archie add 10/01/2023
		# 	listError_now.append(221)

		# -- Lost pose robot
		# if (self.detectLost_poseRobot() == 1):        # Archie add 10/01/2023
		# 	listError_now.append(222)

		# -- Lỗi không thể định vị tọa độ.
		# if self.mode_operate == self.MODE_AUTO and self.detectLost_reflectors() == 1: #              # Archie add 10/01/2023
		# 	listError_now.append(272)

		# -- Loi Code Parking:
		if (self.parking_status.warning == 2):
			listError_now.append(281)

		# -- 19/01/2022 - Mat giao tiep voi Server                                # Archie add 10/01/2023
		# sts_sr = self.detectLost_server()
		# if sts_sr == 1: # lost server
		# 	listError_now.append(431)
		# elif sts_sr == 2: # lost server: Ping
		# 	listError_now.append(432)
		# elif sts_sr == 3: # lost server: IP
		# 	listError_now.append(433)

		# -- Cảnh báo: Low battery
		if self.voltage < 23:
			listError_now.append(451)

		# -- Cảnh báo: Co vat can
		if (self.navigation_respond.stop == 1):
			listError_now.append(411)

		# -- Cảnh báo: Co vat can khi di chuyen parking
		if (self.parking_status.warning == 1):
			listError_now.append(412)
			
		# -- Cảnh báo: AGV dung do da di het danh sach diem.
		if self.navigation_respond.error == 2 and self.navigation_respond.modeMove == 4:
			self.flag_listPoint_ok = 1
			self.listID_old = self.server_cmdRequest.list_id
		if self.flag_listPoint_ok == 1:
			listError_now.append(441)

		# -- Cảnh báo: AGV Dừng Tránh AGV Khác.
		if self.navigation_respond.error == 4 and self.navigation_respond.modeMove == 4:
			if self.find_stopAvoiding(self.server_cmdRequest.command) == 1:
				listError_now.append(442)

		# -- Cảnh báo: Lỗi lệnh Traffic. update: 28/09/2023.
		if self.navigation_respond.error == -5 and self.navigation_respond.modeMove == 5:
			listError_now.append(443)

		# -- Cảnh báo: Không sạc được Pin.
		if self.flag_notCharger == 1:
			listError_now.append(452)

		# -- Cảnh Báo: không thể định vị tọa độ.
		# if self.mode_operate == self.MODE_MANUAL and self.detectLost_reflectors() == 1: #                     # Archie add 10/01/2023
		# 	listError_now.append(453)

		# -- Cảnh báo: Trả hàng - Vị trí trả hàng đã có thùng.                                 # Archie remove
		# - Rack 11
		# if self.flagWarning_rack11 == 1:
		# 	listError_now.append(491)
		# # - Rack 12
		# if self.flagWarning_rack12 == 1:
		# 	listError_now.append(492)
		# # - Rack 21
		# if self.flagWarning_rack21 == 1:
		# 	listError_now.append(494)
		# # - Rack 22
		# if self.flagWarning_rack22 == 1:
		# 	listError_now.append(495)

		# -- Cảnh báo: Băng tải không nhận được thùng hàng.
		# - Conveyor 11
		if self.signal_conveyor11.status == 255 or self.flagWarning_receivedRack11 == 1:
			listError_now.append(481)
		# - Conveyor 12
		if self.signal_conveyor12.status == 255 or self.flagWarning_receivedRack12 == 1:
			listError_now.append(482)
		# - Conveyor 21
		if self.signal_conveyor21.status == 255 or self.flagWarning_receivedRack21 == 1:
			listError_now.append(484)
		# - Conveyor 22
		if self.signal_conveyor22.status == 255 or self.flagWarning_receivedRack22 == 1:
			listError_now.append(485)

		# # -- Cảnh báo: Băng tải không trả được thùng hàng.
		# # - Conveyor 11
		if self.signal_conveyor11.status == 254:                                                   # Archie: Làm gì có trong code của mach nhỉ? 
			listError_now.append(471)
		# - Conveyor 12
		if self.signal_conveyor12.status == 254:
			listError_now.append(472)
		# - Conveyor 21
		if self.signal_conveyor21.status == 254:
			listError_now.append(474)
		# - Conveyor 22
		if self.signal_conveyor22.status == 254:
			listError_now.append(475)

		# - error of Conveyor machine
		if self.flagWarning_posError == 1:
			listError_now.append(301)
		if self.flag_CYmachine_readyError == 1:                     # không cần thiết
			listError_now.append(502)
		if self.flag_CYmachine_generalError == 1 or self.signal_CYMToyo.bit8_err == 1:
			listError_now.append(503)

		# - error of Conveyor AGV:
		# - Cảnh báo trên băng tải AGV đã có thùng nên không thể nhận
		if self.flag_CY11_receiveError_Full == 1:
			listError_now.append(511)
		if self.flag_CY12_receiveError_Full == 1:
			listError_now.append(512)
		if self.flag_CY21_receiveError_Full == 1:
			listError_now.append(513)
		if self.flag_CY22_receiveError_Full == 1:
			listError_now.append(514)

		# - Cảnh báo trên băng tải AGV không có thùng nên ko thể trả
		if self.flag_CY11_transmitError_Blank == 1:
			listError_now.append(515)
		if self.flag_CY12_transmitError_Blank == 1:
			listError_now.append(516)
		if self.flag_CY21_transmitError_Blank == 1:
			listError_now.append(517)
		if self.flag_CY22_transmitError_Blank == 1:
			listError_now.append(518)	

		# - Cảnh báo AGV trả tự động nhưng không có hàng
		if self.flag_transmitError_Blank == 1:
			listError_now.append(519)			

		return listError_now

	def resetAll_variable(self):
		self.enable_moving  = 0
		self.enable_parking = 0
		self.enable_mission = 0
		# -
		self.completed_before_mission = 0
		self.completed_after_mission = 0
		self.completed_after_mission1 = 0
		self.completed_moveSimple = 0
		self.completed_moveSpecial = 0
		self.completed_moveOut = 0
		self.completed_checkConveyors = 0
		self.completed_checkRack = 0
		# -
		self.mission = 0
		self.flag_listPoint_ok = 0

		# rospy.logwarn("Update new target from: X= %s | Y= %s to X= %s| Y= %s", self.target_x, self.target_y, self.Traffic_cmdRequest.target_x, self.Traffic_cmdRequest.target_y)
		self.log_mess("info", "Update new target: X_new = ", self.server_cmdRequest.target_x)
		self.target_id = self.server_cmdRequest.target_id
		self.target_x  = self.server_cmdRequest.target_x
		self.target_y  = self.server_cmdRequest.target_y
		self.target_z  = self.server_cmdRequest.target_z
		self.target_tag = self.server_cmdRequest.tag
		self.mission_before = self.server_cmdRequest.before_mission
		self.mission_after = self.server_cmdRequest.after_mission
		self.serverCmd_now = self.server_cmdRequest
		# -
		self.flag_resetFramework = 0
		self.flag_Byhand_to_Auto = 0
		# -
		self.flag_notCharger = 0
		# -
		self.flagWarning_rack11 = 0
		self.flagWarning_rack12 = 0
		self.flagWarning_rack21 = 0
		self.flagWarning_rack22 = 0
		# -
		self.flagWarning_receivedRack11 = 0
		self.flagWarning_receivedRack12 = 0
		self.flagWarning_receivedRack21 = 0
		self.flagWarning_receivedRack22 = 0

		# -
		self.flag_transmitError_Blank = 0

		# -
		self.listMission_completed = [0, 0, 0, 0, 0, 0]
		# -- add 17/03/2023
		self.control_CPD = CPD_write()
		self.control_conveyors = Control_conveyors()
		self.AGVToyo_signal = Signal_AGVToyo()

		# - 
		self.case_step1 = 1
		self.case_step2 = 1

	def run_maunal(self):
		cmd_vel = Twist()
		velMax_lx = 0.32
		velMax_lr = 0.24
		# - Tiến.
		if self.app_button.bt_forwards == True:
			cmd_vel.angular.z = 0.0
			if self.HC_info.zone_sick_ahead == 1:
				cmd_vel.linear.x = 0.0
			else:
				if self.HC_info.zone_sick_ahead == 2:
					cmd_vel.linear.x = velMax_lx*0.5*(self.app_button.vs_speed/100.)
				else:
					cmd_vel.linear.x = velMax_lx*(self.app_button.vs_speed/100.)

				if cmd_vel.linear.x < 0.02:
					cmd_vel.linear.x = 0.02

		# - Lùi.
		if self.app_button.bt_backwards == True:
			cmd_vel.angular.z = 0.0
			if self.HC_info.zone_sick_behind == 1:
				cmd_vel.linear.x = 0.0
			else:
				if self.HC_info.zone_sick_behind == 2:
					cmd_vel.linear.x = velMax_lx*0.5*(self.app_button.vs_speed/100.)*-1
				else:
					cmd_vel.linear.x = velMax_lx*(self.app_button.vs_speed/100.)*-1
				if cmd_vel.linear.x > -0.02:
					cmd_vel.linear.x = -0.02

		# - Xoay Trái.
		if self.app_button.bt_rotation_left == True:
			cmd_vel.linear.x = 0.0
			if self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1:
				cmd_vel.angular.z = 0.0				
			else:
				if self.HC_info.zone_sick_ahead == 2 or self.HC_info.zone_sick_behind == 2:
					cmd_vel.angular.z = velMax_lr*0.5*(self.app_button.vs_speed/100.)
				else:
					cmd_vel.angular.z = velMax_lr*(self.app_button.vs_speed/100.)
				if cmd_vel.angular.z < 0.06:
					cmd_vel.angular.z = 0.06
					
		# - Xoay Phải.
		if self.app_button.bt_rotation_right == True:
			cmd_vel.linear.x = 0.0
			if self.HC_info.zone_sick_ahead == 1 or self.HC_info.zone_sick_behind == 1:
				cmd_vel.angular.z = 0.0				
			else:
				if self.HC_info.zone_sick_ahead == 2 or self.HC_info.zone_sick_behind == 2:
					cmd_vel.angular.z = velMax_lr*0.5*(self.app_button.vs_speed/100.)*-1
				else:
					cmd_vel.angular.z = velMax_lr*(self.app_button.vs_speed/100.)*-1
				if cmd_vel.angular.z > -0.02:
					cmd_vel.angular.z = -0.02

		# - Dừng.
		if self.app_button.bt_stop == True:
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0

		if self.safety_NAV.data == 1:
			cmd_vel = Twist()

		return cmd_vel

	def convertCY_bt_flag(self):
		# -- 11
		if self.app_button.bt_cy11_receive == True:             
			self.conveyor11_taskByHand = self.CYTASK_RECEIVE
			self.conveyor12_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy11_transmit == True:
			self.conveyor11_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor12_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy11_stop == True:
			self.conveyor11_taskByHand = self.CYTASK_STOP
			self.conveyor12_taskByHand = self.CYTASK_STOP
		# - 12
		if self.app_button.bt_cy12_receive == True:             
			self.conveyor12_taskByHand = self.CYTASK_RECEIVE
			self.conveyor11_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy12_transmit == True:
			self.conveyor12_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor11_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy12_stop == True:
			self.conveyor12_taskByHand = self.CYTASK_STOP
			self.conveyor11_taskByHand = self.CYTASK_STOP
		# - 21
		if self.app_button.bt_cy21_receive == True:             
			self.conveyor21_taskByHand = self.CYTASK_RECEIVE
			self.conveyor22_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy21_transmit == True:
			self.conveyor21_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor22_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy21_stop == True:
			self.conveyor21_taskByHand = self.CYTASK_STOP
			self.conveyor22_taskByHand = self.CYTASK_STOP
		# - 22
		if self.app_button.bt_cy22_receive == True:             
			self.conveyor22_taskByHand = self.CYTASK_RECEIVE
			self.conveyor21_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy22_transmit == True:
			self.conveyor22_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor21_taskByHand = self.CYTASK_STOP
		
		if self.app_button.bt_cy22_stop == True:
			self.conveyor22_taskByHand = self.CYTASK_STOP
			self.conveyor21_taskByHand = self.CYTASK_STOP
		# -- 1st floor
		if self.app_button.bt_cy1stfloor_receive == True:             
			self.conveyor11_taskByHand = self.CYTASK_RECEIVE
			self.conveyor12_taskByHand = self.CYTASK_RECEIVE
		
		if self.app_button.bt_cy1stfloor_transmit == True:
			self.conveyor11_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor12_taskByHand = self.CYTASK_TRANSMIT
		
		if self.app_button.bt_cy1stfloor_stop == True:
			self.conveyor11_taskByHand = self.CYTASK_STOP
			self.conveyor12_taskByHand = self.CYTASK_STOP
		# -- 2nd floor
		if self.app_button.bt_cy2ndfloor_receive == True:             
			self.conveyor21_taskByHand = self.CYTASK_RECEIVE
			self.conveyor22_taskByHand = self.CYTASK_RECEIVE
		
		if self.app_button.bt_cy2ndfloor_transmit == True:
			self.conveyor21_taskByHand = self.CYTASK_TRANSMIT
			self.conveyor22_taskByHand = self.CYTASK_TRANSMIT
		
		if self.app_button.bt_cy2ndfloor_stop == True:
			self.conveyor21_taskByHand = self.CYTASK_STOP
			self.conveyor22_taskByHand = self.CYTASK_STOP
	
	def resetCY_signal(self):
		# - reset bit toyo 
		self.AGVToyo_signal = Signal_AGVToyo()
		# - reset cy mission 
		self.control_conveyors = Control_conveyors()
		# -
		self.conveyor11_taskByHand = self.CYTASK_STOP
		self.conveyor12_taskByHand = self.CYTASK_STOP
		self.conveyor21_taskByHand = self.CYTASK_STOP
		self.conveyor22_taskByHand = self.CYTASK_STOP

	def ManualCY_link(self, bit_task, bit1, bit2, bit3, bit4, cy_mission1, cy_mission2, cy_mission3, cy_mission4):
		no_cy = 0
		self.AGVToyo_signal.bit1 = bit1             #  send bit 1 
		self.AGVToyo_signal.bit2 = bit2
		self.AGVToyo_signal.bit3 = bit3
		self.AGVToyo_signal.bit4 = bit4

		self.AGVToyo_signal.bit7_rdy = bit_task

		if bit1 == 1 and bit2 == 0:
			no_cy = 11
		elif bit1 == 0 and bit2 == 1:
			no_cy = 12
		elif bit1 == 1 and bit2 == 1:
			no_cy = 1112

		if bit3 == 1 and bit4 == 0:
			no_cy = 21
		elif bit3 == 0 and bit4 == 1:
			no_cy = 22
		elif bit3 == 1 and bit4 == 1:
			no_cy = 2122

		if self.signal_CYMToyo.bit7_rdy == 1:   #  receive respond bit1 from Conveyor
			self.troubleshoot_mess("info", "Liên kết băng tải, Gửi lệnh chạy băng tải", str(no_cy))
			self.control_conveyors.No1_mission = cy_mission1
			self.control_conveyors.No2_mission = cy_mission2
			self.control_conveyors.No3_mission = cy_mission3
			self.control_conveyors.No4_mission = cy_mission4
			self.AGVToyo_signal.bit6_done = 0

	def ManualCY_unlink(self, bit1, bit2, bit3, bit4):
		no_cy = 0
		if (bit1 == 1 or bit1 ==2) and bit2 == 0:
			no_cy = 11
		elif bit1 == 0 and (bit2 == 1 or bit2 == 2):
			no_cy = 12
		elif (bit1 == 1 and bit2 == 1) or (bit1 == 2 and bit2 == 2):
			no_cy = 1112

		if (bit3 == 1 or bit3 == 2) and bit4 == 0:
			no_cy = 21
		elif bit3 == 0 and (bit4 == 1 or bit4 == 2):
			no_cy = 22
		elif (bit3 == 1 and bit4 == 1) or (bit3 == 2 and bit4 == 2):
			no_cy = 2122

		if no_cy == 0:
			self.troubleshoot_mess("info", "Ko kết nối băng tải - Gửi lệnh các băng tải dừng hết", 0)
		else:
			self.troubleshoot_mess("info", "Ko kết nối băng tải - Đang gửi lệnh cho băng tải", str(no_cy))

		self.control_conveyors.No1_mission = bit1
		self.control_conveyors.No2_mission = bit2
		self.control_conveyors.No3_mission = bit3
		self.control_conveyors.No4_mission = bit4
		
	def check_AGVrun(self):
		# - 1, Đang nhận/trả hàng.
		condition_No1 = 0
		condition_No2 = 0
		condition_No3 = 0
		condition_No4 = 0
		# -
		condition_No1112 = 0
		condition_No2122 = 0
		
		# -
		if (self.signal_conveyor11.status == 1 or self.signal_conveyor11.status == 2 or self.signal_conveyor12.status == 1 or self.signal_conveyor12.status == 2):
			condition_No1112 = 1
		# -
		if (self.signal_conveyor21.status == 1 or self.signal_conveyor21.status == 2 or self.signal_conveyor22.status == 1 or self.signal_conveyor22.status == 2):
			condition_No2122 = 1

		# -- add 17/03/2023: Lỗi AGV di chuyển khi thùng chưa sang hẳn (quy trình nhận thùng)
		if self.signal_conveyor11.status == 254 or self.signal_conveyor11.status == 255 or self.signal_conveyor12.status == 254 or self.signal_conveyor12.status == 255:
			condition_No1112 = 1
		# -
		if self.signal_conveyor21.status == 254 or self.signal_conveyor21.status == 255 or self.signal_conveyor22.status == 254 or self.signal_conveyor22.status == 255:
			condition_No2122 = 1

		# -- add 19/06/2023: Lỗi AGV di chuyển khi thùng chưa sang hẳn (Do cảm biến bị nhiễu -> AGV nhận lỗi)
		if self.flagWarning_receivedRack11 == 1 or self.flagWarning_receivedRack12 == 1:
			condition_No1112 = 1

		if self.flagWarning_receivedRack21 == 1 or self.flagWarning_receivedRack22 == 1:
			condition_No2122 = 1

		condition_No1 = condition_No1112 + condition_No2122

		# - 2, Đang Parking: Vào sạc.
		if self.navigation_respond.modeMove == 2 and self.navigation_respond.status == 1 and self.navigation_respond.completed == 0:
			condition_No2 = 1

		# - 3, Đang đi ra khỏi sạc.
		if self.navigation_respond.modeMove == 3 and self.navigation_respond.status == 1: # and self.navigation_respond.completed == 0:
			if self.completed_moveSpecial == 0:
				condition_No3 = 1
		
		return (condition_No1, condition_No2, condition_No3)
				
	def run(self):
		while not rospy.is_shutdown():
			# ------ 
			self.listError =  self.synthetic_error()
			self.numberError = len(self.listError)
			lenght = len(self.listError)

			count_error = 0
			count_warning = 0
			for i in range(lenght):
				if (self.listError[i] < 400):
					count_error += 1
				else:
					count_warning += 1
			# --
			if count_error == 0 and count_warning == 0:
				self.flag_error = 0
				self.flag_warning = 0

			elif count_error == 0 and count_warning > 0:
				self.flag_error = 0
				self.flag_warning = 1                    # = 1: Archie add 5/1/2023

			else:
				self.flag_error = 1                      # = 1: Archie add 5/1/2023
				self.flag_warning = 0

			# -- 
			if self.flag_error == 1:
				self.statusAGV = self.STTAGV_ERROR
			else:
				if self.flag_warning == 1:
					self.statusAGV = self.STTAGV_WARNING
				else:
					self.statusAGV = self.STTAGV_ALLRIGHT

			# -- ERROR
			if self.app_button.bt_clearError == 1: # or self.main_info.stsButton_reset == 1:
				# -- Send clear error
				self.flag_error = 0
				self.error_device = 0
				self.error_move = 0
				self.error_perform = 0

				self.flag_blsockCollide = 0
				# -
				if self.parking_status.warning == 2: # - Nếu Parking lỗi -> Cho phép Rest.
					self.enable_parking = 0
				# -
				self.EMC_write = self.EMC_WRITEOFF
				self.EMC_reset = self.EMC_RESETON
				self.task_driver.data = self.taskDriver_resetRead
				# print ("Run Reset: " + str(self.app_button.bt_clearError) + " | " + str(self.main_info.stsButton_reset))
				
				# -- Xóa lỗi băng tải.
				if self.signal_conveyor11.status > 125:
					self.control_conveyors.No5_mission = 0

				if self.signal_conveyor21.status > 125:
					self.control_conveyors.No6_mission = 0

				if self.signal_conveyor12.status > 125:
					self.control_conveyors.No3_mission = 0

				if self.signal_conveyor22.status > 125:
					self.control_conveyors.No4_mission = 0

				# -- add 22/01/2022
				self.flag_notCharger = 0
				# -
				self.flagWarning_rack11 = 0
				self.flagWarning_rack12 = 0
				self.flagWarning_rack21 = 0
				self.flagWarning_rack22 = 0
				# -
				self.flagWarning_receivedRack11 = 0
				self.flagWarning_receivedRack12 = 0
				self.flagWarning_receivedRack21 = 0
				self.flagWarning_receivedRack22 = 0
				# - 
				self.flag_CYmachine_generalError = 0
				self.flagWarning_posError = 0
				self.flag_CYmachine_readyError = 0

				# - 
				self.flag_CY11_transmitError_Blank = 0
				self.flag_CY11_receiveError_Full = 0
				self.flag_CY12_transmitError_Blank = 0
				self.flag_CY12_receiveError_Full = 0
				self.flag_CY21_transmitError_Blank = 0
				self.flag_CY21_receiveError_Full = 0
				self.flag_CY22_transmitError_Blank = 0
				self.flag_CY22_receiveError_Full = 0

				# - 
				if self.signal_conveyor11.sensor_checkItem == 1 or self.signal_conveyor12.sensor_checkItem == 1 or self.signal_conveyor21.sensor_checkItem == 1 or self.signal_conveyor22.sensor_checkItem == 1:
					self.flag_transmitError_Blank = 0
					self.flag_runAgain = 1

			else:
				self.task_driver.data = self.taskDriver_Read
				self.EMC_reset = self.EMC_RESETOFF	

			if self.process == 0: # khi moi khoi dong len
				self.enable_moving  = 0
				self.enable_parking = 0
				self.enable_mission = 0

				self.mode_operate = self.MODE_MANUAL
				self.control_conveyors = Control_conveyors()

				self.led_effect = 0
				self.speaker_requir = self.SPK_WARN
				# -
				self.flag_error = 0
				self.error_device = 0
				self.error_move = 0
				self.error_perform = 0
				# -
				self.completed_before_mission = 0
				self.completed_after_mission = 0
				self.completed_after_mission1 = 0
				self.completed_checkConveyors = 0
				
				self.process = 1

			elif self.process == 1:	# chờ cac node khoi dong xong.
				ct = 8
				if ct == 8:
					self.process = 2

			elif self.process == 2: # - Read app
				if self.app_button.bt_passHand == 1:
					if self.mode_operate == self.MODE_AUTO: # - Kéo cờ báo kiểm tra trạng thái Lệnh tự động sau khi chuyển chế độ. 
						self.flag_Byhand_to_Auto = 1				
					self.mode_operate = self.MODE_MANUAL
					
				if self.app_button.bt_passAuto == 1 or self.flag_Auto == 1:
					self.mode_operate = self.MODE_AUTO
					self.flag_Auto = 0

				if self.mode_operate == self.MODE_MANUAL:
					self.process = 30

				elif self.mode_operate == self.MODE_AUTO:
					self.process = 40
		# ------------------------------------------------------------------------------------
		# -- BY HAND:
			elif self.process == 30:
				self.job_doing = 20
				
				# ------------ Reset FrameWork------------ #
				if self.app_button.bt_resetFrameWork == 1:
					delta_t = rospy.Time.now() - self.saveTime_resetFrameWork
					if delta_t.to_sec() > 0.8:
						self.resetAll_variable()
				else:
					self.saveTime_resetFrameWork = rospy.Time.now()

				# ------------ Stop Auto ------------ #
				self.enable_moving = 0
				self.enable_parking = 0
				self.enable_mission = 1
				# ------------ Navigation ------------ #
				if self.flag_error == 0:
				  	# -- Send vel
					if self.signal_conveyor12.status == 0 or self.signal_conveyor12.status >= 3:  
						stsRun_cy3 = 0
					else:
						stsRun_cy3 = 1

					if self.signal_conveyor22.status == 0 or self.signal_conveyor22.status >= 3:  
						stsRun_cy4 = 0
					else:
						stsRun_cy4 = 1

					if self.signal_conveyor11.status == 0 or self.signal_conveyor11.status >= 3:  
						stsRun_cy5 = 0
					else:
						stsRun_cy5 = 1

					if self.signal_conveyor21.status == 0 or self.signal_conveyor21.status >= 3:  
						stsRun_cy6 = 0
					else:
						stsRun_cy6 = 1

					# - >> Băng tải đang vận hành -> ko cho phép di chuyển <<
					if stsRun_cy3 == 0 and stsRun_cy4 == 0 and stsRun_cy5 == 0 and stsRun_cy6 == 0:  
						# -- Move
						vel_manual = self.run_maunal()
						self.pub_cmdVel(vel_manual, self.rate_cmdvel, rospy.get_time())
					else:
						self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

				else: # -- Has error
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
					self.control_CPD = CPD_write()
					# -
					self.control_conveyors = Control_conveyors()

			# ------------ Conveyor ------------ #
				if self.flag_Auto_to_Byhand == 1:
					self.control_conveyors = Control_conveyors()
					self.flag_Auto_to_Byhand = 0

				# - convert CY bt event to flag
				self.convertCY_bt_flag()

				# -- reset variable when chuyển chế độ test
				if self.btValue_linkCY != self.app_button.bt_linkConveyor:
					self.resetCY_signal()
					self.btValue_linkCY = self.app_button.bt_linkConveyor					

				# -- control conveyor by manual
				if self.app_button.bt_linkConveyor == 1:                      
					self.AGVToyo_signal.bit5_cnt = 1
					if self.signal_CYMToyo.bit5_cnt == 1:
						if self.signal_CYMToyo.bit8_err == 0:
							# --
							if self.signal_conveyor11.status != self.CYTASKRCV_DONE and self.signal_conveyor11.status != self.CYTASKTRSM_DONE and self.signal_conveyor12.status != self.CYTASKRCV_DONE and self.signal_conveyor12.status != self.CYTASKTRSM_DONE and self.signal_conveyor21.status != self.CYTASKRCV_DONE and self.signal_conveyor21.status != self.CYTASKTRSM_DONE and self.signal_conveyor22.status != self.CYTASKRCV_DONE and self.signal_conveyor22.status != self.CYTASKTRSM_DONE:
								# -- CY RECEIVE 11
								if self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
										self.ManualCY_link(0, 1,0,0,0, 1,0,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 đã có thùng hàng", 0)
										self.flag_CY11_receiveError_Full = 1
								# - 12
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
										self.ManualCY_link(0, 0,1,0,0, 0,1,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
										self.flag_CY12_receiveError_Full = 1
								# - 1st floor 11 - 12
								elif self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0 and self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
										self.ManualCY_link(0, 1,1,0,0, 1,1,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên tầng 1 đã có thùng hàng", 0)
										self.flag_CY11_receiveError_Full = 1
										self.flag_CY12_receiveError_Full = 1
								# - 21
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
										self.ManualCY_link(0, 0,0,1,0, 0,0,1,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng 21 đã có thùng hàng", 0)
										self.flag_CY21_receiveError_Full = 1
								# - 22
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
									if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
										self.ManualCY_link(0, 0,0,0,1, 0,0,0,1)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 đã có thùng hàng", 0)
										self.flag_CY12_receiveError_Full = 1
								# - 2nd floor - 21 - 22
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
									if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0 and self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:								
										self.ManualCY_link(0, 0,0,1,1, 0,0,1,1)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên tầng 2 đã có thùng hàng", 0)
										self.flag_CY21_receiveError_Full = 1
										self.flag_CY22_receiveError_Full = 1

								# -- CY transmit 11
								if self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 1:								
										self.ManualCY_link(1, 1,0,0,0, 2,0,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 không có thùng hàng", 0)
										self.flag_CY11_transmitError_Blank = 1
								# - 12
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 1:								
										self.ManualCY_link(1, 0,1,0,0, 0,2,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 không có thùng hàng", 0)
										self.flag_CY12_transmitError_Blank = 1
								# - 1st floor 11 - 12
								elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 1 and self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 1:
										self.ManualCY_link(1, 1,1,0,0, 2,2,0,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên tầng 1 không có thùng hàng", 0)
										self.flag_CY11_transmitError_Blank = 1
										self.flag_CY12_transmitError_Blank = 1
								# - 21
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_STOP:
									if self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 1:								
										self.ManualCY_link(1, 0,0,1,0, 0,0,2,0)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 không có thùng hàng", 0)
										self.flag_CY21_transmitError_Blank = 1
								# - 22
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
									if self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 1:								
										self.ManualCY_link(1, 0,0,0,1, 0,0,0,2)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 không có thùng hàng", 0)
										self.flag_CY22_transmitError_Blank = 1
								# - 2nd floor - 21 - 22
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
									if self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 1 and self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 1:								
										self.ManualCY_link(1, 0,0,1,1, 0,0,2,2)
									else:
										self.troubleshoot_mess("warn", "Liên kết băng tải, Trên tầng 2 không có thùng hàng", 0)
										self.flag_CY21_transmitError_Blank = 1
										self.flag_CY22_transmitError_Blank = 1

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									# - reset toyo signal -- 
									self.troubleshoot_mess("info", "Liên kết băng tải, Gửi lệnh dừng các băng tải", 0)
									self.AGVToyo_signal.bit1 = 0                
									self.AGVToyo_signal.bit2 = 0
									self.AGVToyo_signal.bit3 = 0
									self.AGVToyo_signal.bit4 = 0
									self.AGVToyo_signal.bit5_cnt = 0
									self.AGVToyo_signal.bit6_done = 0       # need chỉnh lại thông số này
									self.AGVToyo_signal.bit7_rdy = 0

									self.control_conveyors.No1_mission = self.CYTASK_STOP
									self.control_conveyors.No2_mission = self.CYTASK_STOP
									self.control_conveyors.No3_mission = self.CYTASK_STOP
									self.control_conveyors.No4_mission = self.CYTASK_STOP								

							else:
								# -
								if (self.conveyor11_taskByHand == self.CYTASK_RECEIVE or self.conveyor11_taskByHand == self.CYTASK_TRANSMIT) and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải 11 chạy xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No1_mission = self.CYTASK_STOP
									self.conveyor11_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and (self.conveyor12_taskByHand == self.CYTASK_RECEIVE or self.conveyor11_taskByHand == self.CYTASK_TRANSMIT) and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải 12 chạy xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No2_mission = self.CYTASK_STOP
									self.conveyor12_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải tầng 1 nhận xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.control_conveyors.No1_mission = self.CYTASK_STOP
									self.control_conveyors.No2_mission = self.CYTASK_STOP
									self.conveyor11_taskByHand = self.CYTASK_STOP
									self.conveyor12_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải tầng 1 trả xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No1_mission = self.CYTASK_STOP
									self.control_conveyors.No2_mission = self.CYTASK_STOP
									self.conveyor11_taskByHand = self.CYTASK_STOP
									self.conveyor12_taskByHand = self.CYTASK_STOP
								# -
								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and (self.conveyor21_taskByHand == self.CYTASK_RECEIVE or self.conveyor21_taskByHand == self.CYTASK_TRANSMIT) and self.conveyor22_taskByHand == self.CYTASK_STOP:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải 21 chạy xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No3_mission = self.CYTASK_STOP
									self.conveyor21_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and (self.conveyor22_taskByHand == self.CYTASK_RECEIVE or self.conveyor21_taskByHand == self.CYTASK_TRANSMIT):						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải 22 chạy xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No4_mission = self.CYTASK_STOP
									self.conveyor22_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải tầng 2 nhận xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.control_conveyors.No3_mission = self.CYTASK_STOP
									self.control_conveyors.No4_mission = self.CYTASK_STOP
									self.conveyor21_taskByHand = self.CYTASK_STOP
									self.conveyor22_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:						
									self.troubleshoot_mess("info", "Liên kết băng tải, Băng tải tầng 2 trả xong >> Dừng", 0)
									self.AGVToyo_signal.bit6_done = 1
									self.AGVToyo_signal.bit7_rdy = 0
									self.control_conveyors.No3_mission = self.CYTASK_STOP
									self.control_conveyors.No4_mission = self.CYTASK_STOP
									self.conveyor21_taskByHand = self.CYTASK_STOP
									self.conveyor22_taskByHand = self.CYTASK_STOP

								elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
									self.troubleshoot_mess("info", "Liên kết băng tải, Các băng tải đã chạy xong >> Dừng", 0)
									self.AGVToyo_signal.bit1 = 0                
									self.AGVToyo_signal.bit2 = 0
									self.AGVToyo_signal.bit3 = 0
									self.AGVToyo_signal.bit4 = 0
									self.AGVToyo_signal.bit5_cnt = 0
									self.AGVToyo_signal.bit6_done = 0              # need chỉnh lại thông số này
									self.AGVToyo_signal.bit7_rdy = 0

									self.control_conveyors.No1_mission = self.CYTASK_STOP
									self.control_conveyors.No2_mission = self.CYTASK_STOP
									self.control_conveyors.No3_mission = self.CYTASK_STOP
									self.control_conveyors.No4_mission = self.CYTASK_STOP
									self.conveyor11_taskByHand = self.CYTASK_STOP
									self.conveyor12_taskByHand = self.CYTASK_STOP
									self.conveyor21_taskByHand = self.CYTASK_STOP
									self.conveyor22_taskByHand = self.CYTASK_STOP
						
						else:
							self.troubleshoot_mess("warn", "Băng tải máy gặp lỗi >> ko cho phép AGV thực hiện nhiệm vụ", 0)
							self.flag_CYmachine_generalError = 1					
					else:
						self.troubleshoot_mess("warn", "Không phát hiện vị trị cụm băng tải máy", 0)
						self.flagWarning_posError = 1								
				else:
					# --
					self.AGVToyo_signal.bit5_cnt = 0
					self.AGVToyo_signal.bit6_done = 0 
					self.AGVToyo_signal.bit7_rdy = 0

					if self.signal_conveyor11.status != self.CYTASKRCV_DONE and self.signal_conveyor11.status != self.CYTASKTRSM_DONE and self.signal_conveyor12.status != self.CYTASKRCV_DONE and self.signal_conveyor12.status != self.CYTASKTRSM_DONE and self.signal_conveyor21.status != self.CYTASKRCV_DONE and self.signal_conveyor21.status != self.CYTASKTRSM_DONE and self.signal_conveyor22.status != self.CYTASKRCV_DONE and self.signal_conveyor22.status != self.CYTASKTRSM_DONE:
						# -- 
						if self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
								self.ManualCY_unlink(1,0,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 11 đã có thùng hàng", 0)
								self.flag_CY11_receiveError_Full = 1

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
								self.ManualCY_unlink(0,1,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 12 đã có thùng hàng", 0)
								self.flag_CY12_receiveError_Full = 1

						elif self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0 and self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
								self.ManualCY_unlink(1,1,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên tầng 1 đã có thùng hàng", 0)
								self.flag_CY11_receiveError_Full = 1
								self.flag_CY12_receiveError_Full = 1							
						# -
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
								self.ManualCY_unlink(0,0,1,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 21 đã có thùng hàng", 0)
								self.flag_CY21_receiveError_Full = 1							

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
							if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
								self.ManualCY_unlink(0,0,0,1)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 22 đã có thùng hàng", 0)
								self.flag_CY22_receiveError_Full = 1							

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
							if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0 and self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:								
								self.ManualCY_unlink(0,0,1,1)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên tầng 2 đã có thùng hàng", 0)
								self.flag_CY21_receiveError_Full = 1
								self.flag_CY22_receiveError_Full = 1							
						# --
						if self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 1:								
								self.ManualCY_unlink(2,0,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 11 không có thùng hàng", 0)
								self.flag_CY11_transmitError_Blank = 1							
							
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 1:								
								self.ManualCY_unlink(0,2,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 12 không có thùng hàng", 0)
								self.flag_CY12_transmitError_Blank = 1							

						elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 1 and self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 1: 
								self.ManualCY_unlink(2,2,0,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên tầng 1 không có thùng hàng", 0)
								self.flag_CY11_transmitError_Blank = 1
								self.flag_CY12_transmitError_Blank = 1							
						# -
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 1:								
								self.ManualCY_unlink(0,0,2,0)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 21 không có thùng hàng", 0)
								self.flag_CY21_transmitError_Blank = 1							

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
							if self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 1:								
								self.ManualCY_unlink(0,0,0,2)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên băng tải 22 không có thùng hàng", 0)
								self.flag_CY22_transmitError_Blank = 1						

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
							if self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 1 and self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 1:								
								self.ManualCY_unlink(0,0,2,2)
							else:
								self.troubleshoot_mess("warn", "Ko kết nối băng tải, Trên tầng 2 không có thùng hàng", 0)
								self.flag_CY21_transmitError_Blank = 1	
								self.flag_CY22_transmitError_Blank = 1								
						# -- 
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							self.ManualCY_unlink(0,0,0,0)
							self.flag_CY11_receiveError_Full = 0
							self.flag_CY12_receiveError_Full = 0
							self.flag_CY21_receiveError_Full = 0
							self.flag_CY22_receiveError_Full = 0

							self.flag_CY11_transmitError_Blank = 0
							self.flag_CY12_transmitError_Blank = 0
							self.flag_CY21_transmitError_Blank = 0 
							self.flag_CY22_transmitError_Blank = 0											

					else:
						self.ManualCY_unlink(0,0,0,0)
						self.conveyor11_taskByHand = self.CYTASK_STOP
						self.conveyor12_taskByHand = self.CYTASK_STOP
						self.conveyor21_taskByHand = self.CYTASK_STOP
						self.conveyor22_taskByHand = self.CYTASK_STOP

			# ------------ Speaker ------------ #
				# -- Speaker
				if self.app_button.bt_speaker == True:
					self.enable_speaker = 1
				else:
					self.enable_speaker = 0
				# ------------ Charger ------------ #
				if self.app_button.bt_charger == True:
					self.charger_requir = self.CHARGER_ON
				else:
					self.charger_requir = self.CHARGER_OFF
				# ------------ Brake ------------ #
				if self.app_button.bt_brake == True:
					self.disable_brake.data = 1
				else:
					self.disable_brake.data = 0

				self.process = 2

		# -- RUN AUTO:
			elif self.process == 40: # -- kiem tra loi
				self.flag_Auto_to_Byhand = 1
				if self.flag_error == 1: #
					self.enable_moving = 0
					self.enable_parking = 0
					self.enable_mission = 0
					# -
					self.control_conveyors = Control_conveyors()
					# -
					self.process = 2
				else:
					self.process = 41 # 41

			elif self.process == 41: # - Kiểm tra lộ trình thay đổi -> Cập nhật lệnh.
				if self.target_id != self.server_cmdRequest.target_id:
					# - Không cho phép đổi lệnh khi đang thực hiện thao tác đặc biệt:
					# - 1, Đang nhận/trả hàng.
					# -------------------
					(condition_No1, condition_No2, condition_No3) = self.check_AGVrun()

					if condition_No1 != 0 or condition_No2 != 0 or condition_No3 != 0:
						self.log_mess("warn", "Have new target but must Waiting perform done ....", 0)
						self.process = 2
						self.job_doing = 8
					else:
						self.resetAll_variable()
						print ("--- ResetAll Variable ---")
						self.process = 42
				else:
					self.process = 42

			elif self.process == 42: # - Kiểm tra trạng thái sau khi điều khiển bằng tay.
				if self.flag_Byhand_to_Auto == 1:
					# - add 18/07/2023.
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
					
					# -- add 15/08/2023 - Kiểm tra vị trí có bị thay đổi không.
					if self.completed_moveSimple == 1 and self.completed_moveSpecial == 1:
						if self.completed_after_mission1 == 0 or self.completed_after_mission == 0:
							point1 = Point(self.Traffic_infoRespond.x, self.Traffic_infoRespond.y, 0)
							point2 = Point(self.navigation_query.GoalX, self.navigation_query.GoalY, 0)
							err_distance = self.calculate_distance(point1, point2)
							err_angle = self.Traffic_infoRespond.z - self.navigation_query.GoalAngle
							err_angle = self.limitAngle(err_angle)
							# -
							if (err_distance > 0.03) or (abs(err_angle) > radians(8)):
								self.completed_moveSimple = 0
								self.completed_moveSpecial = 0
								self.completed_checkRack = 0
								self.completed_after_mission1 = 0
								self.completed_after_mission = 0

					# -- add 15/08/2023 - Kiểm tra vị trí mode = 5 đang chạy -> reset.
					if self.completed_moveSimple == 1 and self.completed_moveSpecial == 0:
						point11 = Point(self.Traffic_infoRespond.x, self.Traffic_infoRespond.y, 0)
						point12 = Point(self.serverCmd_now.target_x, self.serverCmd_now.target_y, 0)
						err_distance1 = self.calculate_distance(point11, point12)
						if err_distance1 > 2.0:
							self.completed_moveSimple = 0

					# -- add 12/11/2021 : Chay lai quy trinh Vao Sac khi Chuuyen che do.
					if self.completed_after_mission == 1 and self.mission_after == 10:
						delta_distance = self.calculate_distance(self.robotPose_nav.pose.position, self.poseCharger.position)
						if delta_distance > self.distance_resetMission:
							self.resetAll_variable()
							self.completed_moveOut = 1
							print ("---- Reset - flag_Byhand_to_Auto ----")
						else:
							print ("-------- Charger ----------")
							self.charger_requir = self.CHARGER_ON

					self.job_doing = 1
					self.flag_Byhand_to_Auto = 0
					self.disable_brake.data = 0
					self.process = 2		
				else:
					self.process = 43		

				# --
				if self.flag_runAgain == 1:
					self.flag_runAgain = 0
					self.resetAll_variable()
					print ("---- Reset - Run process again ----")
					self.process = 43

			elif self.process == 43: # - Thực hiện nhiệm vụ trước.
				if self.completed_checkConveyors_before == 0: # - Chưa thực hiện.
					self.process = 2
					self.completed_checkConveyors_before = 1
					self.charger_requir = self.CHARGER_OFF
					# - add 18/07/2023.
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
				else:
					self.process = 44

			elif self.process == 44: # - Thực hiện di chuyển: tiến ra khỏi sạc.
				if self.completed_moveOut == 0:
					self.job_doing = 2
					if self.flag_requirOutCharge == 1:
						self.navigation_query.modeMove = 3
						self.navigation_query.GoalID = 0
						self.navigation_query.GoalX = self.goal_moveOut.pose.position.x
						self.navigation_query.GoalY = self.goal_moveOut.pose.position.y
						self.navigation_query.GoalAngle = 0
						self.enable_moving = 1
						# --
						if self.navigation_respond.modeMove == 3 and self.navigation_respond.completed == 1:
							self.completed_moveOut = 1
							self.flag_requirOutCharge = 0
							self.enable_moving = 0
					else:
						self.enable_moving = 0
						self.completed_moveOut = 1

					self.process = 2
				else:
					self.process = 45

			elif self.process == 45: # - Thực hiện di chuyển: đi giữa các điểm.
				if self.completed_moveSimple == 0:
					self.job_doing = 3
					self.enable_moving = 1

					# - Khi có cờ báo di chuyển hết điểm -> Không cho chạy lại tuyến đường cũ.
					if self.flag_listPoint_ok == 1:
						self.enable_moving = 0

					self.navigation_query.modeMove = 4
					# --
					self.serverCmd_now.list_id = self.server_cmdRequest.list_id
					self.serverCmd_now.list_x  = self.server_cmdRequest.list_x
					self.serverCmd_now.list_y  = self.server_cmdRequest.list_y
					self.serverCmd_now.list_speed = self.server_cmdRequest.list_speed
					self.serverCmd_now.list_directionTravel = self.server_cmdRequest.list_directionTravel
					self.serverCmd_now.list_angleLine  = self.server_cmdRequest.list_angleLine
					self.serverCmd_now.list_roadWidth  = self.server_cmdRequest.list_roadWidth
					self.serverCmd_now.list_angleFinal = self.server_cmdRequest.list_angleFinal
					# -
					self.navigation_query.GoalID = self.serverCmd_now.target_id
					self.navigation_query.GoalX  = self.serverCmd_now.target_x
					self.navigation_query.GoalY  = self.serverCmd_now.target_y
					self.navigation_query.GoalAngle = self.serverCmd_now.target_z
					# --
					self.navigation_query.listID = self.serverCmd_now.list_id
					self.navigation_query.listX  = self.serverCmd_now.list_x
					self.navigation_query.listY  = self.serverCmd_now.list_y
					self.navigation_query.listSpeed = self.serverCmd_now.list_speed
					self.navigation_query.listDirectionTravel = self.serverCmd_now.list_directionTravel
					self.navigation_query.listAngleLine  = self.serverCmd_now.list_angleLine
					self.navigation_query.listRoadWidth  = self.serverCmd_now.list_roadWidth
					self.navigation_query.listAngleFinal = self.serverCmd_now.list_angleFinal
					# - 
					if self.navigation_respond.modeMove == 4 and self.navigation_respond.completed == 1:
						self.completed_moveSimple = 1
						self.enable_moving = 0
					self.process = 2
				else:
					self.process = 46

			elif self.process == 46: # - Thực hiện di chuyển: 1, Đi đến đích Nhận/Trả rack | 2, Đi vào sạc.
				if self.completed_moveSpecial == 0:
					self.job_doing = 4
					if self.mission_after == 0: # - Không có lệnh.
						self.completed_moveSpecial = 1

					elif self.mission_before == 66 and self.mission_after == 10: # - Lệnh Sạc.
						self.enable_moving = 1

						self.navigation_query.modeMove = 2
						self.goalTarget.id = self.serverCmd_now.target_id
						self.goalTarget.pose = self.getPose_from_offset(self.serverCmd_now.target_x, self.serverCmd_now.target_y, self.serverCmd_now.target_z, self.serverCmd_now.offset)
						self.goalTarget.angleFinal = self.serverCmd_now.target_z
						# - 
						self.navigation_query.GoalID = self.goalTarget.id
						self.navigation_query.GoalX  = self.goalTarget.pose.position.x
						self.navigation_query.GoalY  = self.goalTarget.pose.position.y
						self.navigation_query.GoalAngle = self.goalTarget.angleFinal
						# - 
						if self.navigation_respond.modeMove == 2 and self.navigation_respond.completed == 1:
							self.completed_moveSpecial = 1
							self.enable_moving = 0
							# - 
							# print ("I Heaaaaa")
							self.flag_requirOutCharge = 1
							self.goal_moveOut.id = self.goalTarget.id
							self.goal_moveOut.pose.position.x = self.serverCmd_now.target_x
							self.goal_moveOut.pose.position.y = self.serverCmd_now.target_y
							# -
							self.poseCharger.position.x = self.robotPose_nav.pose.position.x
							self.poseCharger.position.y = self.robotPose_nav.pose.position.y
					
					else:
						self.enable_moving = 1

						self.navigation_query.modeMove = 5
						# --
						self.serverCmd_now = self.server_cmdRequest
						# -
						self.navigation_query.GoalID = self.serverCmd_now.target_id
						self.navigation_query.GoalX  = self.serverCmd_now.target_x
						self.navigation_query.GoalY  = self.serverCmd_now.target_y
						self.navigation_query.GoalAngle = self.serverCmd_now.target_z
						# --
						self.navigation_query.listID = self.serverCmd_now.list_id
						self.navigation_query.listX  = self.serverCmd_now.list_x
						self.navigation_query.listY  = self.serverCmd_now.list_y
						self.navigation_query.listSpeed = self.serverCmd_now.list_speed
						self.navigation_query.listDirectionTravel = self.serverCmd_now.list_directionTravel
						self.navigation_query.listAngleLine  = self.serverCmd_now.list_angleLine
						self.navigation_query.listRoadWidth  = self.serverCmd_now.list_roadWidth
						self.navigation_query.listAngleFinal = self.serverCmd_now.list_angleFinal
						# - 
						if self.navigation_respond.modeMove == 5 and self.navigation_respond.completed == 1:
							self.completed_moveSpecial = 1
							self.enable_moving = 0

					self.process = 2
				else:
					self.process = 48

			elif self.process == 48: # Thực hiện nhiệm vụ nhận/trả hàng
				if self.completed_after_mission == 0:
					self.job_doing = 6
					# - 
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

					if self.mission_after == 0:
						self.charger_requir = self.CHARGER_OFF
						self.completed_after_mission = 1
						self.control_CPD = CPD_write()
						self.AGVToyo_signal = Signal_AGVToyo()
						# self.flagWarning_rack11 = 0
						# self.flagWarning_rack12 = 0
						# self.flagWarning_rack13 = 0
						# self.flagWarning_rack21 = 0
						# self.flagWarning_rack22 = 0
						# self.flagWarning_rack23 = 0
						# -
						self.flagWarning_receivedRack11 = 0
						self.flagWarning_receivedRack12 = 0
						self.flagWarning_receivedRack13 = 0
						self.flagWarning_receivedRack21 = 0
						self.flagWarning_receivedRack22 = 0
						self.flagWarning_receivedRack23 = 0
						# -
						self.flagWarning_transfer11 = 0
						self.flagWarning_transfer12 = 0
						self.flagWarning_transfer13 = 0
						self.flagWarning_transfer21 = 0
						self.flagWarning_transfer22 = 0
						self.flagWarning_transfer23 = 0

						self.flag_transmitError_Blank = 0

					elif self.mission_before == 66 and self.mission_after == 10: # - Nhiệm vụ sạc.
						self.completed_after_mission = 1
						print ("-------- Charger ----------")
						self.charger_requir = self.CHARGER_ON
					else:
						if self.mission_before == 0:    
							if self.mission_after == 1: # Trường hợp nhận khay ở kho Modula
								if self.case_step1 == 1:                     # Check AGV đã đến băng tải chưa 
									self.AGVToyo_signal.bit5_cnt = 1
									if self.signal_CYMToyo.bit5_cnt == 1:    # Conveyor xác nhận vị trí với AGV
										self.case_step1 = 2
									else:
										self.flagWarning_posError = 1
										self.process = 2
										self.troubleshoot_mess("info", "AutoMode -> Ko phát hiện vị trí của băng tải máy", 0)

								elif self.case_step1 == 2:                   # Kiểm tra vị trí băng tải cần nhận
									if self.signal_CYMToyo.bit8_err == 0:
										if self.signal_CYMToyo.bit1 == 0 and self.signal_CYMToyo.bit2 == 0:
											self.completed_after_mission = 1
											self.troubleshoot_mess("info", "AutoMode -> Cụm băng tải không có tray hàng", 0)
											self.case_step1 = 1
											self.mission_val = 0
											self.AGVToyo_signal.bit6_done = 1

										elif self.signal_CYMToyo.bit1 == 1 and self.signal_CYMToyo.bit2 == 0:
											if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
												self.case_step1 = 3
												self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 11", 0)
												self.mission_val = 0b0001
											else:
												self.troubleshoot_mess("info", "AutoMode -> AGV đã có tray 11 -> bỏ qua nhận tray", 0)
												self.mission_val = 0

										elif self.signal_CYMToyo.bit1 == 0 and self.signal_CYMToyo.bit2 == 1:
											if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
												self.case_step1 = 3
												self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 12", 0)
												self.mission_val = 0b0010
											else:
												self.troubleshoot_mess("info", "AutoMode -> AGV đã có tray 12 -> bỏ qua nhận tray", 0)
												self.mission_val = 0																		

										elif self.signal_CYMToyo.bit1 == 1 and self.signal_CYMToyo.bit2 == 1:
											if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0 and self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
												self.case_step1 = 3
												self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 11 && 12", 0)
												self.mission_val = 0b0011
											else:
												self.troubleshoot_mess("info", "AutoMode -> AGV đã có tray 11 && 12 -> bỏ qua nhận tray", 0)
												self.mission_val = 0	

									else:
										self.flag_CYmachine_generalError = 1
										self.mission_val = 0
										self.case_step1 = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step1 == 3:			
									# - Wait CYM1 confirm 
									if self.signal_CYMToyo.bit8_err == 0:      
										self.AGVToyo_signal.bit7_rdy = 0	
										if self.mission_val == 0b0001:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 0
										elif self.mission_val == 0b0010:
											self.AGVToyo_signal.bit1 = 0	
											self.AGVToyo_signal.bit2 = 1
										elif self.mission_val == 0b0011:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 1

										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step1 = 4

									else:
										self.flag_CYmachine_generalError = 1
										self.case_step1 = 1
										self.AGVToyo_signal.bit1 = 0
										self.AGVToyo_signal.bit2 = 0									
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step1 == 4:                                          ## thực hiện nhận hàng
									if self.signal_CYMToyo.bit8_err == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE							
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5
									else:
										self.flag_CYmachine_generalError = 1
										self.control_conveyors.No1_mission = self.CYTASK_STOP							
										self.control_conveyors.No2_mission = self.CYTASK_STOP
										self.AGVToyo_signal.bit6_done = 0
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step1 == 5:             # Finish and reset
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay trống", 0)
									self.case_mission1 = 2
									self.case_step1 = 1
									self.completed_after_mission = 1
									self.AGVToyo_signal = Signal_AGVToyo()

							elif self.mission_after == 3: # - Nhận khay trống ở dây chuyển IE5
								if self.case_step1 == 1:
									self.AGVToyo_signal.bit5_cnt = 1
									if self.signal_CYMToyo.bit5_cnt == 1:                   # Conveyor xác nhận vị trí với AGV
										self.case_step1 = 2
									else:
										self.flagWarning_posError = 1
										self.process = 2
										self.troubleshoot_mess("info", "AutoMode -> Ko phát hiện vị trí của băng tải máy", 0)

								elif self.case_step1 == 2:
									if self.signal_CYMToyo.bit8_err == 0:                         
										self.AGVToyo_signal.bit7_rdy = 0
										# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
										if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.AGVToyo_signal.bit1 = 1
										# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
										elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.AGVToyo_signal.bit2 = 1
										# - 21
										elif self.mission_val == 0b0100:
											self.AGVToyo_signal.bit3 = 1
										# - 22
										elif self.mission_val == 0b1000:
											self.AGVToyo_signal.bit4 = 1
										# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
										elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 1
										# - 21 && 22
										elif self.mission_val == 0b1100:               
											self.AGVToyo_signal.bit3 = 1
											self.AGVToyo_signal.bit4 = 1
										# -- empty
										elif self.mission_val == 0:
											self.case_step2 = 10
											self.troubleshoot_mess("info", "AutoMode -> Bỏ qua nhận hàng IE5", 0)

										# - Wait CYM1 confirm                      
										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step1 = 3
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step1 == 3:
									if self.signal_CYMToyo.bit8_err == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10
										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10
										elif self.mission_val == 0b0100:
											self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10									
										elif self.mission_val == 0b1000:
											self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10
										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10
										elif self.mission_val == 0b1100:
											self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
											self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 10
											
										# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
										elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5
										# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
										elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5														
										# - (11 && 12 && 21) or (11 && 12 && 22)
										elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step1 = 5			

									else:
										self.flag_CYmachine_generalError = 1
										self.control_conveyors.No1_mission = self.CYTASK_STOP							
										self.control_conveyors.No2_mission = self.CYTASK_STOP
										self.control_conveyors.No3_mission = self.CYTASK_STOP
										self.control_conveyors.No4_mission = self.CYTASK_STOP
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)	
								
								elif self.case_step1 == 5:
									if self.signal_CYMToyo.bit8_err == 0: 
										self.AGVToyo_signal.bit7_rdy = 0
										self.AGVToyo_signal.bit6_done = 0
										if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111:
											self.AGVToyo_signal.bit3 = 1
										elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011:
											self.AGVToyo_signal.bit4 = 1
										elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111:
											self.AGVToyo_signal.bit3 = 1
											self.AGVToyo_signal.bit4 = 1								
										
										# - Wait CYM1 confirm                        
										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step1 = 6
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)
								
								elif self.case_step1 == 6:
									if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111:
										self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step1 = 10
									elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011:
										self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step1 = 10																
									elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111:
										self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
										self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step1 = 10

								elif self.case_step1 == 10:
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay hàng", 0)
									# self.case_mission1 = 1
									self.case_step1 = 1
									self.completed_after_mission = 1
									self.AGVToyo_signal = Signal_AGVToyo()
									self.control_conveyors = Control_conveyors()

						elif self.mission_before == 1:  # - Nhiệm vụ trả hàng 
							if self.mission_after == 2:  # - AGV Trả hàng tại truyền IE5
								# print("Case step 2 la: %s", self.case_step2)
								if self.case_step2 == 1:
									self.AGVToyo_signal.bit5_cnt = 1
									if self.signal_CYMToyo.bit5_cnt == 1:                   # Conveyor xác nhận vị trí với AGV
										self.case_step2 = 2
									else:
										self.flagWarning_posError = 1
										self.process = 2
										self.troubleshoot_mess("info", "AutoMode -> Ko phát hiện vị trí của băng tải máy", 0)

								elif self.case_step2 == 2:
									if self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0001
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0010
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0100
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1000
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0011
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0101	
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1001
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0110
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1010						
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1100
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 0:
										self.mission_val = 0b0111						
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 0 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1011
									elif self.signal_conveyor11.sensor_checkItem == 0 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1110
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 0 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1101
									elif self.signal_conveyor11.sensor_checkItem == 1 and self.signal_conveyor12.sensor_checkItem == 1 and self.signal_conveyor21.sensor_checkItem == 1 and self.signal_conveyor22.sensor_checkItem == 1:
										self.mission_val = 0b1111
									else:
										self.mission_val = 0

									self.case_step2 = 3

								elif self.case_step2 == 3:
									if self.signal_CYMToyo.bit8_err == 0:                         
										self.AGVToyo_signal.bit7_rdy = 1
										# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
										if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.AGVToyo_signal.bit1 = 1
										# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
										elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.AGVToyo_signal.bit2 = 1
										# - 21
										elif self.mission_val == 0b0100:
											self.AGVToyo_signal.bit3 = 1
										# - 22
										elif self.mission_val == 0b1000:
											self.AGVToyo_signal.bit4 = 1
										# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
										elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 1
										# - 21 && 22
										elif self.mission_val == 0b1100:               
											self.AGVToyo_signal.bit3 = 1
											self.AGVToyo_signal.bit4 = 1
										# -- empty
										elif self.mission_val == 0:                            ###### thêm cảnh báo lỗi 
											self.troubleshoot_mess("info", "AutoMode -> AGV Bỏ qua trả hàng tại truyền IE5", 0)
											self.flag_transmitError_Blank = 1

										# - Wait CYM1 confirm 
										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step2 = 4
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 1)

								elif self.case_step2 == 4:
									if self.signal_CYMToyo.bit8_err == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10
										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10
										elif self.mission_val == 0b0100:
											self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10									
										elif self.mission_val == 0b1000:
											self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10
										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10
										elif self.mission_val == 0b1100:
											self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
											self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor21.status == self.CYTASKTRSM_DONE and self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 10
											
										# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
										elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5
										# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
										elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5														
										# - (11 && 12 && 21) or (11 && 12 && 22)
										elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5			

									else:
										self.flag_CYmachine_generalError = 1
										self.control_conveyors.No1_mission = self.CYTASK_STOP							
										self.control_conveyors.No2_mission = self.CYTASK_STOP
										self.control_conveyors.No3_mission = self.CYTASK_STOP
										self.control_conveyors.No4_mission = self.CYTASK_STOP
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 2)	
								
								elif self.case_step2 == 5:
									if self.signal_CYMToyo.bit8_err == 0: 
										self.AGVToyo_signal.bit7_rdy = 1
										self.AGVToyo_signal.bit6_done = 0
										if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111:
											self.AGVToyo_signal.bit3 = 1
										elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011:
											self.AGVToyo_signal.bit4 = 1
										elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111:
											self.AGVToyo_signal.bit3 = 1
											self.AGVToyo_signal.bit4 = 1								
										
										# - Wait CYM1 confirm                        
										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step2 = 6
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 3)

								elif self.case_step2 == 6:
									if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111:
										self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step2 = 10
									elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011:
										self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step2 = 10																
									elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111:
										self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
										self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor21.status == self.CYTASKTRSM_DONE and self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
											self.AGVToyo_signal.bit6_done = 1
											self.case_step2 = 10

								elif self.case_step2 == 10:
									self.troubleshoot_mess("info", "AutoMode -> AGV đã trả xong khay hàng", 0)
									# self.case_mission2 = 2
									self.case_step2 = 1
									self.completed_after_mission = 1
									self.AGVToyo_signal = Signal_AGVToyo()
									self.control_conveyors = Control_conveyors()								

							elif self.mission_after == 4:  # - AGV trả tray về tại kho
								if self.case_step2 == 1:                     # Check AGV đã đến băng tải chưa 
									self.AGVToyo_signal.bit5_cnt = 1
									if self.signal_CYMToyo.bit5_cnt == 1:                   # Conveyor xác nhận vị trí với AGV
										self.case_step2 = 2
									else:
										self.flagWarning_posError = 1
										self.process = 2
										self.troubleshoot_mess("info", "AutoMode -> Ko phát hiện vị trí của băng tải máy", 0)

								elif self.case_step2 == 2:
									if self.signal_CYMToyo.bit8_err == 0:
										# - trên cụm băng tải đã có tray
										if self.signal_CYMToyo.bit1 == 1 and self.signal_CYMToyo.bit2 == 1:
											self.completed_after_mission = 1
											self.case_step2 = 1
											self.troubleshoot_mess("info", "AutoMode -> Cụm băng tải đã có đủ tray hàng", 0)
											# self.process = 2
											self.mision_val = 0
											self.AGVToyo_signal.bit6_done = 1
										elif self.signal_CYMToyo.bit1 == 0 and self.signal_CYMToyo.bit2 == 1:
											self.case_step2 = 3
											self.mission_val = 0b0001
											self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 11", 0)
										elif self.signal_CYMToyo.bit1 == 1 and self.signal_CYMToyo.bit2 == 0:
											self.case_step2 = 3
											self.mission_val = 0b0010
											self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 12", 0)
										elif self.signal_CYMToyo.bit1 == 0 and self.signal_CYMToyo.bit2 == 0:
											self.case_step2 = 3
											self.mission_val = 0b0011
											self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 11 && 12", 0)								
									
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)
								
								elif self.case_step2 == 3:
									if self.signal_CYMToyo.bit8_err == 0:    
										self.AGVToyo_signal.bit7_rdy = 1	
										if self.mission_val == 0b0001:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 0
										elif self.mission_val == 0b0010:
											self.AGVToyo_signal.bit1 = 0	
											self.AGVToyo_signal.bit2 = 1
										elif self.mission_val == 0b0011:
											self.AGVToyo_signal.bit1 = 1
											self.AGVToyo_signal.bit2 = 1	
										
										# - Wait CYM1 confirm                     
										if self.signal_CYMToyo.bit7_rdy == 1:
											self.case_step2 = 4
									else:
										self.flag_CYmachine_generalError = 1
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)
								
								elif self.case_step2 == 4:                   ## thực hiện trả hàng
									if self.signal_CYMToyo.bit8_err == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT							
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE: 
												self.AGVToyo_signal.bit6_done = 1
												self.case_step2 = 5
									else:
										self.flag_CYmachine_generalError = 1
										self.control_conveyors.No1_mission = self.CYTASK_STOP							
										self.control_conveyors.No2_mission = self.CYTASK_STOP
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step2 == 5:             # Finish and reset
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay trống", 0)
									# self.case_mission2 = 1
									self.case_step2 = 1
									self.completed_after_mission = 1
									self.AGVToyo_signal = Signal_AGVToyo()

						self.process = 2
				else:
					self.process = 100

			elif self.process == 100: # - Chờ lệnh mới.
				self.job_doing = 7
				self.process = 2
				# -
				self.control_conveyors = Control_conveyors()
				# -
				self.listMission_completed = [0, 0, 0, 0, 0, 0]
				# -
				self.enable_moving = 0
				# - add 18/07/2023.
				self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
				# -
				self.control_CPD = CPD_write()
				# - 
				self.AGVToyo_signal = Signal_AGVToyo()
				self.troubleshoot_mess("warn", "Wating new Target ...", 0)
				
		# ------------------------------------------------------------------------------------
			# -- Tag + Offset:
			if self.mode_operate == self.MODE_AUTO:
				if self.completed_moveSpecial == 1:
					self.Traffic_infoRespond.id_target = self.server_cmdRequest.target_id
					self.Traffic_infoRespond.tag = self.server_cmdRequest.tag
					self.Traffic_infoRespond.offset = self.server_cmdRequest.offset
				else:
					self.Traffic_infoRespond.id_target = 0
					self.Traffic_infoRespond.tag = 0
					self.Traffic_infoRespond.offset = 0

			self.Traffic_infoRespond.status = self.statusAGV    # Status: Error
			self.Traffic_infoRespond.error_perform = self.process
			self.Traffic_infoRespond.error_moving  = self.flag_error
			self.Traffic_infoRespond.error_device  = self.numberError
			self.Traffic_infoRespond.listError = self.listError
			self.Traffic_infoRespond.process = self.job_doing
			self.Traffic_infoRespond.task_num = self.mission_val
			# self.Traffic_infoRespond.task_num = self.case_step2
			# self.Traffic_infoRespond.case_mission1 = self.case_mission1
			# self.Traffic_infoRespond.case_mission2 = self.case_mission2

			# -- 
			if self.completed_after_mission == 1:
				self.Traffic_infoRespond.task_status = self.mission_after
			else:
				self.Traffic_infoRespond.task_status = 0

			# -- Battery - ok
			self.readbatteryVoltage()
			self.Traffic_infoRespond.battery = int(self.valueVoltage)

			if self.flag_cancelMission == 0:
				# -- mode respond server
				if self.mode_operate == self.MODE_MANUAL:         # Che do by Hand
					self.Traffic_infoRespond.mode = 1

				elif self.mode_operate == self.MODE_AUTO:          # Che do Auto
					self.Traffic_infoRespond.mode = 2
			else:
				self.Traffic_infoRespond.mode = 5

			# ---------------- Speaker ---------------- #
			if self.flag_error == 1 and self.flag_warning == 1:
				self.speaker_requir = self.SPK_ERR
			elif self.flag_error == 1 and self.flag_warning == 0:
				self.speaker_requir = self.SPK_ERR
			elif self.flag_error == 0 and self.flag_warning == 1:
				self.speaker_requir = self.SPK_WARN			
			else:
				self.speaker_requir = self.SPK_MOVE

			# ---------------- Board HC - LED ---------------- #
			if self.flag_error == 1:
				self.led_effect = self.LED_ERR
			else:
				if self.navigation_respond.stop == 1:
					self.led_effect = self.LED_STOPBARRIER
				else:
					self.led_effect = self.LED_SIMPLERUN
			
			# ---------------- Board Control - Publish ---------------- #
			time_curr = rospy.get_time()
			d = (time_curr - self.pre_timeBoard)
			if (d > float(1/self.FrequencePubBoard)): # < 20hz 
				self.pre_timeBoard = time_curr

				# -------- Request Board OC - Conveyors -------- #
				self.pub_controlConveyors.publish(self.control_conveyors)

				# -------- Request Board Main -------- #
				if self.flag_error == 0:
					# tat Loa khi sac thanh cong!
					if self.completed_after_mission == 1 and self.mode_operate == self.MODE_AUTO and self.mission_after == 10 and self.mission_before == 66:
						# print ("NOW HERE")
						if self.charger_write == self.CHARGER_ON:
							if self.main_info.charge_current >= self.charger_valueOrigin :
								self.speaker_effect = self.SPK_OFF
								self.flag_notCharger = 0
							else:
								self.flag_notCharger = 1
								if self.enable_speaker == 1:
									self.speaker_effect = self.speaker_requir
								else:
									self.speaker_effect = self.SPK_OFF	
						else:
							self.speaker_effect = self.SPK_OFF
							self.flag_notCharger = 0
					else:
						self.flag_notCharger = 0
						if self.enable_speaker == 1:
							self.speaker_effect = self.speaker_requir
						else:
							self.speaker_effect = self.SPK_OFF	
				else:
					self.flag_notCharger = 0
					if self.enable_speaker == 1:
						self.speaker_effect = self.speaker_requir
					else:
						self.speaker_effect = self.SPK_OFF

				# self.speaker_effect = self.SPK_MOVE
				self.pub_Main(self.charger_write, self.speaker_effect, self.EMC_write, self.EMC_reset)  # MISSION

				# -- Battery info
				self.pin_info.pinState = self.battery_info.pinState
				self.pin_info.pinVolt = self.battery_info.pinVolt
				self.pin_info.pinCurr = self.battery_info.pinCurr
				self.pin_info.pinPercent = self.battery_info.pinPercent
				self.pin_info.timeCharge = self.convert_intTotime(self.battery_info.timeCharge)
				self.pin_info.timeChargePropose = self.convert_intTotime(self.battery_info.timeChargePropose)
				self.pub_pin.publish(self.pin_info)

				# -------- Request Board HC -------- #
				self.HC_request.RBG1 = self.led_effect 
				self.HC_request.RBG2 = self.led_effect 
				self.pub_controlHC.publish(self.HC_request)

				# -------- Request CPD Board -------- #
				# self.pub_controlCPD.publish(self.control_CPD)

				# -------- Request Task Driver Motor -------- #
				self.pub_taskDriver.publish(self.task_driver)

			# -------------------------------------------------------------- #
			time_curr = rospy.get_time()
			d = (time_curr - self.pre_timePub)
			if (d > float(1/self.FrequencePub)): # < 20hz 
				self.pre_timePub = time_curr
				# ---------------- Cancel Mission ---------------- #
				if self.cancelMission_control.data == 1:
					self.flag_cancelMission = 1

				if self.server_cmdRequest.id_command == 0:
					self.flag_cancelMission = 0

				self.cancelMission_status = self.cancelMission_control
				self.pub_cancelMission.publish(self.cancelMission_status)

				# ---------------- Parking Control ---------------- #
				self.pub_park(self.enable_parking, self.parking_poseBefore, self.parking_poseTarget, self.parking_offset)

				# ---------------- Request Navigation ---------------- #
				if self.enable_moving == 0:
					self.navigation_query.modeMove = 0
				self.pub_navigationQuery.publish(self.navigation_query)

				# ---------------- Brake ---------------- #
				self.pub_disableBrake.publish(self.disable_brake)

			# ---------------- Respond Client ---------------- #
			self.pub_infoRespond.publish(self.Traffic_infoRespond)    # Pub Client

			# ---------------- GUI TEST -----------------------# 
			self.pub_AGVToyoSignal.publish(self.AGVToyo_signal)

			self.rate.sleep()


def main():
	# Start the job threads
	class_1 = ros_control()
	# Keep the main thread running, otherwise signals are ignored.
	class_1.run()

if __name__ == '__main__':
	main()

"""
Stt :
0: chờ đủ dữ liệu để parking
1: chờ tín hiệu parking
21:  tính khoảng cách tiến lùi
-21: thực hiện di chuyen tiến lùi
-210: thực hiện quay trước nếu gặp TH AGV bị lệch góc lớn
31: tính góc quay để lùi vào kệ
-31: thực hiện quay
41: Parking
51: completed - Đợi Reset
52: error: bien doi tf loi

"""