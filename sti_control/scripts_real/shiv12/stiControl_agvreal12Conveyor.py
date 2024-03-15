#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phung Quy Duong(Archie)
Company: STI Viet Nam
Date  : 25/12/2023
					
"""
"""
   >> 1. Check mất cảm biến va chạm từ HCU >> MCU  : OK
   >> 2. Main -> MCU info and PSU info  : OK
   >> 3. HC_request -> HC_ledsound_request and HC_io_request : OK
   >> 4. Toyo data                                           : OK 
		- input
         + bit1: conveyor 1 : hcu - input 1
	  	 + bit2: conveyor 2 : hcu - input 2
		 + bit3: conveyor 3 : hcu - input 3
		 + bit4: conveyor 4 : hcu - input 4
		 + bit5: connected  : hcu - input 5
		 + bit6: done       : hcu - input 6
		 + bit7: ready      : mcu - input 1 
		 + bit8: error      : mcu - input 2
		- output
         + bit1: conveyor 1 : hcu - output 1
	  	 + bit2: conveyor 2 : hcu - output 2
		 + bit3: conveyor 3 : hcu - output 3
		 + bit4: conveyor 4 : hcu - output 4
		 + bit5: connected  : hcu - output 5
		 + bit6: done       : hcu - output 6
		 + bit7: ready      : mcu - output 1 
		 + bit8: error      : mcu - output 2

	>> 5. Thiếu Feedback lại AGV gặp lỗi với Conveyor Machine
    >> Kiểm soát tiếp về lỗi mắc thùng cho chế độ Manual/Auto Mode
    >> Trường hợp, AGV chạy local thì cần thêm việc đọc ghi dữ liệu vào file Excel  >> OK
    >> Chương trình Điều hướng AGV bằng bàn phím   >> OK
    >> Hiển thị lộ trình AGV trên màn hình AGV 

*** Các hạng mục cần test với AGV thực tế ***
	+ AGV tự động xóa lỗi với trường user bỏ thùng ra khỏi băng tải
	+ Các trường hợp mắc thùng
	+ 
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
		self.pub_cancelMission = rospy.Publisher("/cancelMission_status", Int16, queue_size = 20)	
		self.cancelMission_status = Int16()

		# -------------- Cac ket noi ngoai vi -------------- #
		# -- Reconnect
		rospy.Subscriber("/status_reconnect", Status_reconnect, self.callback_reconnect)
		self.status_reconnect = Status_reconnect()

		# -- Board RTC
		rospy.Subscriber("/CAN_received", CAN_received, self.callback_RTC) 
		self.timeStampe_RTC = rospy.Time.now()

		# -- Board MCU
		rospy.Subscriber("/MCU_info", MCU_info, self.callback_MCU) 
		self.mcu_info = MCU_info()
		self.timeStampe_mcu = rospy.Time.now()

		# -- Board PSU
		rospy.Subscriber("/PSU_info", PSU_info, self.callback_PSU) 
		self.psu_info = PSU_info()
		self.timeStampe_psu = rospy.Time.now()
		self.voltage = 24.5
		# -
		# self.pub_requestMain = rospy.Publisher("/MCU_io_request", POWER_request, queue_size = 20)	
		# self.power_request = POWER_request()

		self.pub_mcuIorequest = rospy.Publisher("/MCU_io_request", MCU_io_request, queue_size = 20)	
		self.data_mcuIorequest = MCU_io_request()

		self.pub_mcuSTRrequest = rospy.Publisher("/MCU_safetyRelay_request", MCU_safetyRelay_request, queue_size = 20)	
		self.data_mcuSTRrequest = MCU_safetyRelay_request()

		self.pub_PSUrequest = rospy.Publisher("/PSU_request", PSU_request, queue_size = 20)	
		self.data_PSUrequest = PSU_request()

		# -- BATTERY INFO 
		rospy.Subscriber("/Battery_info", Battery_info, self.callback_Battery)
		self.battery_info = Battery_info()
		self.timeStampe_battery = rospy.Time.now()

		self.pub_pin = rospy.Publisher("/Pin_info", Pin_info, queue_size = 20)	
		self.pin_info = Pin_info()			

		# -- Board HCU
		rospy.Subscriber("/HCU_info", HCU_info, self.callback_HCU, queue_size = 30) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.hcu_info = HCU_info()
		self.timeStampe_hcu = rospy.Time.now()
		# -
		self.pub_hcuLedsound = rospy.Publisher("/HCU_ledsound_request", HCU_ledsound_request, queue_size = 20)	# dieu khien den bao va den ho tro camera
		self.data_hcuLedsound = HCU_ledsound_request()
		# -
		self.pub_hcuIorequest = rospy.Publisher("/HCU_io_request", HCU_io_request, queue_size = 20)	# dieu khien den bao va den ho tro camera
		self.data_hcuIorequest = HCU_io_request()
		# -
		rospy.Subscriber("/Safety_Sick", Safety_sick, self.callback_safetySick, queue_size = 30) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.data_safetySick = Safety_sick()

		# -- Board OC 12 | conveyor11
		rospy.Subscriber("/status_conveyor1", Status_conveyor, self.callback_conveyor11) # 
		self.signal_conveyor11 = Status_conveyor()
		self.timeStampe_conveyor1 = rospy.Time.now()

		# -- Board OC 12 | conveyor12
		rospy.Subscriber("/status_conveyor2", Status_conveyor, self.callback_conveyor12) # 
		self.signal_conveyor12 = Status_conveyor()

		# -- Board OC 34 | conveyor21
		rospy.Subscriber("/status_conveyor3", Status_conveyor, self.callback_conveyor21) # 
		self.signal_conveyor21 = Status_conveyor()
		self.timeStampe_conveyor2 = rospy.Time.now()

		# -- Board OC 34 | conveyor22
		rospy.Subscriber("/status_conveyor4", Status_conveyor, self.callback_conveyor22) # 
		self.signal_conveyor22 = Status_conveyor()

		self.listMission_conveyor = [0, 0, 0, 0, 0, 0]
		self.listMission_completed = [0, 0, 0, 0, 0, 0]

		# - Board OC | Control Conveyors
		self.pub_controlConveyors = rospy.Publisher("/control_conveyors", Control_conveyors, queue_size = 20)	# Dieu khien ban nang.
		self.control_conveyors = Control_conveyors()

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
		self.pub_infoRespond = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size = 120)
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

		# -- Driver Motor
		rospy.Subscriber("/auto_now", Bool, self.callback_autoNow)
		self.goalTarget = GoalFollow() # - Đích của các vị trí thao tác: Sạc Pin, Nhận/Trả hàng.
		self.goal_moveOut = GoalFollow()

		# - Task driver
		self.TASKDRIVER_NOTHING = 0
		self.TASKDRIVER_RESETREAD = 1
		self.TASKDRIVER_READ = 2
		self.task_driver.data = self.TASKDRIVER_READ

		# --------------------- Parameter p --------------------- #
		# -- Hz
		self.FREQ_PUBBOARD = 10.
		self.pre_timeBoard = rospy.get_time()
		# --
		self.FREQ_PUB = 15.
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
		# --
		self.pre_mess = ""                                       # lưu tin nhắn hiện tại.
		# -- Status to server
		self.statusAGV = 0
		self.STTAGV_ALLRIGHT = 0
		self.STTAGV_WARNING = 1
		self.STTAGV_ERROR = 2	
		self.STTAGV_CANCELMISSION = 5	

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
		self.TIME_CHECK_VOLTCHARGE = 1800 # s => 30 minutes.
		self.TIME_CHECK_VOLT = 10     # s
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
		# - 
		self.flag_blsockCollide = 0
		self.saveTime_blsockCollide = rospy.Time.now()
		# -
		self.saveTime_EMG = rospy.Time.now()
		self.saveTime_resetFrameWork = rospy.Time.now()

		# - Flag announce Conveyor machine error 
		self.btValue_linkCY = 0
		self.flagWarning_NRpos1 = 0                              # cờ báo lỗi
		self.flagWarning_NRpos2 = 0
		self.flagWarning_NRdone = 0
		self.flagWarning_NRgeneral = 0

		# - Flag announce Conveyor AGV error
		self.flagWarning_RB11transmit_blank = 0
		self.flagWarning_RB11recieve_full = 0
		self.flagWarning_RB12transmit_blank = 0
		self.flagWarning_RB12recieve_full = 0
		self.flagWarning_RB21transmit_blank = 0
		self.flagWarning_RB21recieve_full = 0
		self.flagWarning_RB22transmit_blank = 0           # AGV muốn trả hàng, nhưng trên AGV ko phát hiện hàng
		self.flagWarning_RB22recieve_full = 0             # AGV muốn nhận hàng, nhưng trên AGV đã có hàng

		self.flagWarning_RBempty = 0                      # AGV không phát hiện tray hàng để trả >> có thể do cảm biến lỗi / hoặc ko có tray nào
		self.flagWarning_RBtransmit11_full = 0            # AGV muốn trả hàng, nhưng vị trí nhận hàng đã có 
		self.flagWarning_RBtransmit12_full = 0

		self.flagWarning_RBreceive11_empty = 0            # AGV muốn nhận hàng, nhưng vị trí nhận hàng ko có hàng
		self.flagWarning_RBreceive12_empty = 0

		# -
		# self.flagWarning_RBtransmit11 = 0                  # Hàng bị mắc giữa RB và NARIME CY , OC báo lỗi 255
		# self.flagWarning_RBtransmit12 = 0
		# self.flagWarning_RBtransmit21 = 0
		# self.flagWarning_RBtransmit22 = 0

		# -
		self.flagWarning_RBreceive11 = 0                   # Hàng đã trôi vào AGV nhưng cảm biến phía sau bị chập chờn nên ko bắt được
		self.flagWarning_RBreceive12 = 0
		self.flagWarning_RBreceive21 = 0
		self.flagWarning_RBreceive22 = 0

		self.case_step = 1
		self.mission_val = 0
		self.cy_val = 0

		self.flag_runAgain = 0
		self.is_transfering = 0

		self.TIME_CHECKPOS = 10.0
		self.TIME_LOG = 1
		self.TIME_CHECKFLOOR2 = 10.0
		self.TIME_CHECKDONE = 10.0

		self.TIME_REFRESH_ERR = 5.0		

		self.step_checkpos = 0
		self.step_checkfloor2 = 0
		self.step_checkdone = 0

		self.ct_checkpos = rospy.get_time()
		self.ct_checkfloor2 = rospy.get_time()
		self.ct_checkdone = rospy.get_time()
		self.ct_refresh_transmit11_full = rospy.get_time()
		self.ct_refresh_transmit12_full = rospy.get_time()
		self.ct_refresh_transmitFloor1_full = rospy.get_time()

		self.ct_refresh_receive11_empty = rospy.get_time()
		self.ct_refresh_receive12_empty = rospy.get_time()
		self.ct_refresh_receiveFloor1_empty = rospy.get_time()

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

	def callback_MCU(self, data):
		self.mcu_info = data
		self.timeStampe_mcu = rospy.Time.now()
		if self.mcu_info.status_CS1 == 0 and self.mcu_info.status_CS2 == 0:
			self.saveTime_blsockCollide = rospy.Time.now()
		self.detect_blsockCollide()

	def callback_PSU(self, data):
		self.psu_info = data
		self.voltage = round(self.psu_info.voltages, 2)
		self.timeStampe_psu = rospy.Time.now()

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

	def callback_HCU(self, data):
		# print ("HC read")
		self.hcu_info = data
		self.timeStampe_hcu = rospy.Time.now()

	def callback_safetySick(self, data):
		self.data_safetySick = data

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

	# def callback_CYMToyo(self, data):
	# 	self.signal_CYMToyo = data
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

	def pub_Main(self, charge, EMC_write, EMC_reset):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		# if sound == 0:
		# 	mai.sound_on = 0	
		# else:
		# 	mai.sound_on = 1
		# 	mai.sound_type = sound
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
				if (delta_time > self.TIME_CHECK_VOLTCHARGE):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.CHARGER_OFF
				if (delta_time > self.TIME_CHECK_VOLT*15):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.psu_info.voltages, 1)*10
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
				if (delta_time > self.TIME_CHECK_VOLT):
					self.pre_timeVoltage = time_curr
					bat = round(self.psu_info.voltages, 1)*10
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

	def detectLost_HCU(self):
		delta_t = rospy.Time.now() - self.timeStampe_hcu
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

	def detectLost_MCU(self):
		delta_t = rospy.Time.now() - self.timeStampe_mcu
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
		if self.mcu_info.status_EMG1 == 0 and self.mcu_info.status_EMG2 == 0 and self.mcu_info.status_EMG3 == 0:
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
		# if self.mcu_info.status_EMG1 == 0 and self.mcu_info.status_EMG2 == 0 and self.mcu_info.status_EMG3 == 0:
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
		# 	if (self.detectLost_MCU() == 1):
		# 		listError_now.append(321)

		# 	# -- Lost HC Board.
		# 	if self.detectLost_HCU() == 1:
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
		# if self.signal_conveyor11.status == 255 or self.flagWarning_receivedRack11 == 1:               #   -- Chưa dùng tới
		# 	listError_now.append(481)
		# # - Conveyor 12
		# if self.signal_conveyor12.status == 255 or self.flagWarning_receivedRack12 == 1:
		# 	listError_now.append(482)
		# # - Conveyor 21
		# if self.signal_conveyor21.status == 255 or self.flagWarning_receivedRack21 == 1:
		# 	listError_now.append(484)
		# # - Conveyor 22
		# if self.signal_conveyor22.status == 255 or self.flagWarning_receivedRack22 == 1:
		# 	listError_now.append(485)

		if self.signal_conveyor11.status == 255:               #   -- Chưa dùng tới
			listError_now.append(481)
		# - Conveyor 12
		if self.signal_conveyor12.status == 255:
			listError_now.append(482)
		# - Conveyor 21
		if self.signal_conveyor21.status == 255:
			listError_now.append(484)
		# - Conveyor 22
		if self.signal_conveyor22.status == 255:
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
		if self.flagWarning_RBempty == 1:              # AGV ko phát hiện tray hàng nào để trả khi ở chế độ tự động
			listError_now.append(301)
		if self.flagWarning_NRpos1 == 1:
			listError_now.append(302)
		if self.flagWarning_NRpos2 == 1:                     
			listError_now.append(303)
		if self.flagWarning_NRdone == 1:                     
			listError_now.append(304)
		if self.flagWarning_NRgeneral == 1 or self.mcu_info.status_input2 == 1:
			listError_now.append(305)
			
		if self.flagWarning_RBtransmit11_full == 1:            # AGV muốn trả hàng, nhưng vị trí nhận hàng 11 đã có 
			listError_now.append(506)
		if self.flagWarning_RBtransmit12_full == 1:
			listError_now.append(507)

		if self.flagWarning_RBreceive11_empty == 1:            # AGV muốn nhận hàng, nhưng vị trí nhận hàng ko có hàng
			listError_now.append(508)
		if self.flagWarning_RBreceive12_empty == 1:
			listError_now.append(509)

		# - error of Conveyor AGV:
		# - Cảnh báo trên băng tải AGV đã có thùng nên không thể nhận
		if self.flagWarning_RB11recieve_full == 1:
			listError_now.append(511)
		if self.flagWarning_RB12recieve_full == 1:
			listError_now.append(512)
		if self.flagWarning_RB21recieve_full == 1:
			listError_now.append(513)
		if self.flagWarning_RB22recieve_full == 1:
			listError_now.append(514)

		# - Cảnh báo trên băng tải AGV không có thùng nên ko thể trả
		if self.flagWarning_RB11transmit_blank == 1:
			listError_now.append(515)
		if self.flagWarning_RB12transmit_blank == 1:
			listError_now.append(516)
		if self.flagWarning_RB21transmit_blank == 1:
			listError_now.append(517)
		if self.flagWarning_RB22transmit_blank == 1:
			listError_now.append(518)			

		return listError_now

	def resetAll_variable(self):
		self.enable_moving  = 0
		self.enable_parking = 0
		self.enable_mission = 0
		# -
		self.completed_before_mission = 0
		self.completed_after_mission = 0
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
		self.reset_NRflag()
		self.flagWarning_NRpos1 = 0                              # cờ báo lỗi
		self.flagWarning_NRpos2 = 0

		# -
		self.listMission_completed = [0, 0, 0, 0, 0, 0]
		# -- add 17/03/2023
		self.control_conveyors = Control_conveyors()
		self.data_hcuIorequest = HCU_io_request()
		self.data_mcuIorequest = MCU_io_request()

		# - 
		self.case_step = 1
		self.case_substep = 1
		self.step_checkpos = 0
		self.step_checkfloor2 = 0
		self.step_checkdone = 0

		# -
		# self.is_transfering = 0

	def run_maunal(self):
		cmd_vel = Twist()
		velMax_lx = 0.32
		velMax_lr = 0.24
		# - Tiến.
		if self.app_button.bt_forwards == True:
			cmd_vel.angular.z = 0.0
			if self.data_safetySick.zone_sick_ahead == 1:
				cmd_vel.linear.x = 0.0
			else:
				if self.data_safetySick.zone_sick_ahead == 2:
					cmd_vel.linear.x = velMax_lx*0.5*(self.app_button.vs_speed/100.)
				else:
					cmd_vel.linear.x = velMax_lx*(self.app_button.vs_speed/100.)

				if cmd_vel.linear.x < 0.02:
					cmd_vel.linear.x = 0.02

		# - Lùi.
		if self.app_button.bt_backwards == True:
			cmd_vel.angular.z = 0.0
			if self.data_safetySick.zone_sick_behind == 1:
				cmd_vel.linear.x = 0.0
			else:
				if self.data_safetySick.zone_sick_behind == 2:
					cmd_vel.linear.x = velMax_lx*0.5*(self.app_button.vs_speed/100.)*-1
				else:
					cmd_vel.linear.x = velMax_lx*(self.app_button.vs_speed/100.)*-1
				if cmd_vel.linear.x > -0.02:
					cmd_vel.linear.x = -0.02

		# - Xoay Trái.
		if self.app_button.bt_rotation_left == True:
			cmd_vel.linear.x = 0.0
			if self.data_safetySick.zone_sick_ahead == 1 or self.data_safetySick.zone_sick_behind == 1:
				cmd_vel.angular.z = 0.0				
			else:
				if self.data_safetySick.zone_sick_ahead == 2 or self.data_safetySick.zone_sick_behind == 2:
					cmd_vel.angular.z = velMax_lr*0.5*(self.app_button.vs_speed/100.)
				else:
					cmd_vel.angular.z = velMax_lr*(self.app_button.vs_speed/100.)
				if cmd_vel.angular.z < 0.06:
					cmd_vel.angular.z = 0.06
					
		# - Xoay Phải.
		if self.app_button.bt_rotation_right == True:
			cmd_vel.linear.x = 0.0
			if self.data_safetySick.zone_sick_ahead == 1 or self.data_safetySick.zone_sick_behind == 1:
				cmd_vel.angular.z = 0.0				
			else:
				if self.data_safetySick.zone_sick_ahead == 2 or self.data_safetySick.zone_sick_behind == 2:
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
		# if self.flagWarning_receivedRack11 == 1 or self.flagWarning_receivedRack12 == 1:
		# 	condition_No1112 = 1

		# if self.flagWarning_receivedRack21 == 1 or self.flagWarning_receivedRack22 == 1:
		# 	condition_No2122 = 1

		condition_No1 = condition_No1112 + condition_No2122

		# - 2, Đang Parking: Vào sạc.
		if self.navigation_respond.modeMove == 2 and self.navigation_respond.status == 1 and self.navigation_respond.completed == 0:
			condition_No2 = 1

		# - 3, Đang đi ra khỏi sạc.
		if self.navigation_respond.modeMove == 3 and self.navigation_respond.status == 1: # and self.navigation_respond.completed == 0:
			if self.completed_moveSpecial == 0:
				condition_No3 = 1
		
		return (condition_No1, condition_No2, condition_No3)

	def get_cy_number(self, mss_type):
		mission_val = 0
		cy_val1 = 0
		cy_val2 = 0
		cy_val3 = 0
		cy_val4 = 0
		if mss_type == 2:     # Trả hàng
			if self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0001
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0010
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0100
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1000
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0011
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0101	
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1001
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0110
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1010						
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1100
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				mission_val = 0b0111						
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1011
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1110
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1101
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				mission_val = 0b1111
			else:
				mission_val = 0

		elif mss_type == 1:    # Nhận hàng
			if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 0
			elif self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 1
			elif self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 1:
				cy_val1 = 1
			else:
				cy_val1 = 1

			if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 0
			elif self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 1
			elif self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 1:
				cy_val2 = 1
			else:
				cy_val2 = 1

			if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
				cy_val3 = 0
			elif self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 0:
				cy_val3 = 1
			elif self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 1:
				cy_val3 = 1
			else:
				cy_val3 = 1

			if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
				cy_val4 = 0
			elif self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 0:
				cy_val4 = 1
			elif self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 1:
				cy_val4 = 1
			else:
				cy_val4 = 1

			if cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 1:
				mission_val = 0b0001
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 1:
				mission_val = 0b0010
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 1:
				mission_val = 0b0100
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 0:
				mission_val = 0b1000
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 1:
				mission_val = 0b0011
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 1:
				mission_val = 0b0101
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 0:
				mission_val = 0b1001
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 1:
				mission_val = 0b0110	
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 0:
				mission_val = 0b1010
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 0:
				mission_val = 0b1100
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 1:
				mission_val = 0b0111
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 0:
				mission_val = 0b1011
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 0:
				mission_val = 0b1110
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 0:
				mission_val = 0b1101
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 0:
				mission_val = 0b1111
			else:
				mission_val = 0

		return mission_val

	# Xác nhận nhiệm vụ muốn trả
	def get_cyfloor1_number(self, mss_type):
		mission_val = 0
		cy_val1 = 0
		cy_val2 = 0

		if mss_type == 1:           # for recieve
			if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 0
			elif self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 1
			elif self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 1:
				cy_val1 = 1
			else:
				cy_val1 = 1

			if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 0
			elif self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 1
			elif self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 1:
				cy_val2 = 1
			else:
				cy_val2 = 1

			if cy_val1 == 0 and cy_val2 == 1:
				mission_val = 0b0001
			elif cy_val1 == 1 and cy_val2 == 0:
				mission_val = 0b0010
			elif cy_val1 == 0 and cy_val2 == 0:
				mission_val = 0b0011
			else:
				mission_val = 0

		elif mss_type == 2:         # for transfer
			if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 0
			elif self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 1
			elif self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 1:
				cy_val1 = 1
			else:
				cy_val1 = 1

			if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 0
			elif self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 1
			elif self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 1:
				cy_val2 = 1
			else:
				cy_val2 = 1
			
			if cy_val1 == 1 and cy_val2 == 0:
				mission_val = 0b0001
			elif cy_val1 == 0 and cy_val2 == 1:
				mission_val = 0b0010
			elif cy_val1 == 1 and cy_val2 == 1:
				mission_val = 0b0011
			else:
				mission_val = 0

		return mission_val

	def get_cyfloor2_number(self):
		cy_val = 0
		if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0 and self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
			cy_val = 0
		else:
			cy_val = 1

		return cy_val
	
	def check_NRpos(self):
		if self.step_checkpos == 0:
			self.ct_checkpos = rospy.get_time()
			self.step_checkpos = 1
			self.data_hcuIorequest.output5 = 1
		else:
			if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
				# print("AGV đang chờ xác nhận vị trí với NR")
				if self.hcu_info.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
					print("AGV đã xác nhận vị trí với NARIME_CY")
					self.step_checkpos = 0
					self.case_step = 3
			
			else:
				print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây  >> Cần hủy lệnh và check lại với lỗi này
				self.data_hcuIorequest.output5 = 0
				# self.step_checkpos = 0
				self.flagWarning_NRpos1 = 1
				# self.case_step = 1

	def check_NRfloor2(self):
		# - Wait CYM1 confirm 
		if self.step_checkfloor2 == 0:
			self.ct_checkfloor2 = rospy.get_time()
			self.step_checkfloor2 = 1
		else:
			if rospy.get_time() - self.ct_checkfloor2 < self.TIME_CHECKFLOOR2:
				if self.mcu_info.status_input1 == 1:  
					print("AGV đã xác nhận vị trí tầng 2 với NARIME_CY")                  
					self.case_step = 5
					self.step_checkfloor2 = 0
			else:
				print("Connection Timeout! AGV không xác nhận vị trí tầng 2 với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây >> Cần hủy lệnh và check lại với lỗi này
				# self.step_checkfloor2 = 0
				# self.case_step = 1
				self.flagWarning_NRpos2 = 1

	def check_NRdone(self):
		if self.step_checkdone == 0:
			self.ct_checkdone = rospy.get_time()
			self.step_checkdone = 1
			self.data_hcuIorequest.output6 = 1
		else:
			if rospy.get_time() - self.ct_checkdone < self.TIME_CHECKDONE:
				if self.hcu_info.status_input6 == 1:
					print("Cho phép AGV thực hiện nhiệm vụ tiếp theo!")
					self.case_step = 1
					self.data_hcuIorequest = HCU_io_request()
					self.data_mcuIorequest = MCU_io_request()
					self.control_conveyors = Control_conveyors()		
					self.step_checkdone = 0
					self.completed_after_mission = 1
			else:
				print("Connection Timeout! AGV không xác nhận hoàn thành với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây >> Hủy lệnh cho AGV về sạc
				# self.case_step = 1
				self.data_hcuIorequest = HCU_io_request()
				self.data_mcuIorequest = MCU_io_request()
				self.control_conveyors = Control_conveyors()		
				# self.step_checkdone = 0
				self.flagWarning_NRdone = 1
				# self.is_transfering = 0
	
	def reset_NRflag(self):
		# self.flagWarning_NRpos1 = 0                              # cờ báo lỗi
		# self.flagWarning_NRpos2 = 0
		self.flagWarning_NRdone = 0
		self.flagWarning_NRgeneral = 0

		self.flagWarning_RBempty = 0                      # AGV không phát hiện tray hàng để trả >> có thể do cảm biến lỗi / hoặc ko có tray nào
		self.flagWarning_RBtransmit11_full = 0            # AGV muốn trả hàng, nhưng vị trí nhận hàng đã có 
		self.flagWarning_RBtransmit12_full = 0

		self.flagWarning_RBreceive11_empty = 0
		self.flagWarning_RBreceive12_empty = 0

		# -
		# self.flagWarning_RBtransmit11 = 0                  # Hàng bị mắc giữa RB và NARIME CY , OC báo lỗi 255
		# self.flagWarning_RBtransmit12 = 0
		# self.flagWarning_RBtransmit21 = 0
		# self.flagWarning_RBtransmit22 = 0

		# -
		self.flagWarning_RBreceive11 = 0                   # Hàng đã trôi vào AGV nhưng cảm biến phía sau bị chập chờn nên ko bắt được
		self.flagWarning_RBreceive12 = 0
		self.flagWarning_RBreceive21 = 0
		self.flagWarning_RBreceive22 = 0

	#  --- for Manual Mode -- 
	def Manual_CYFloor1_recv(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 đã có thùng hàng", 0)
					self.flagWarning_RB11recieve_full = 1
					self.Manual_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
					self.flagWarning_RB12recieve_full = 1
					self.Manual_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0 and \
					self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
					self.flagWarning_RB11recieve_full = 1
					self.flagWarning_RB12recieve_full = 1
					self.Manual_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.data_hcuIorequest.output1 = 1
			elif cy_no == 2:
				self.data_hcuIorequest.output2 = 1
			elif cy_no == 3:
				self.data_hcuIorequest.output1 = 1
				self.data_hcuIorequest.output2 = 1

			self.Manual_checkfloor1()

		elif self.case_substep == 3:
			if cy_no == 1:
				self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 11!")
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output1 = 0
					self.case_step = 2
					self.case_substep = 1

			elif cy_no == 2:
				self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 12!")
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output2 = 0
					self.case_step = 2
					self.case_substep = 1				

			elif cy_no == 3:
				self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
				self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 11 && 12!")
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output1 = 0
					self.data_hcuIorequest.output2 = 0
					self.case_step = 2
					self.case_substep = 1

	def Manual_CYFloor2_recv(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 đã có thùng hàng", 0)
					self.flagWarning_RB21recieve_full = 1
					self.Manual_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 đã có thùng hàng", 0)
					self.flagWarning_RB22recieve_full = 1
					self.Manual_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0 and \
					self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
					self.flagWarning_RB21recieve_full = 1
					self.flagWarning_RB22recieve_full = 1
					self.Manual_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.data_hcuIorequest.output3 = 1
			elif cy_no == 2:
				self.data_hcuIorequest.output4 = 1
			elif cy_no == 3:
				self.data_hcuIorequest.output3 = 1
				self.data_hcuIorequest.output4 = 1

			self.Manual_checkfloor2()

		elif self.case_substep == 3:
			if cy_no == 1:
				self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output3 = 0
					self.case_step = 2
					self.case_substep = 1
			elif cy_no == 2:
				self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output4 = 0
					self.case_step = 2
					self.case_substep = 1				

			elif cy_no == 3:
				self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
				self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output3 = 0
					self.data_hcuIorequest.output4 = 0
					self.case_step = 2
					self.case_substep = 1

	def Manual_CYFloor1_trsm(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor11.sensor_limitAhead == 1 or self.signal_conveyor11.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 chưa có thùng hàng", 0)
					self.flagWarning_RB11transmit_blank = 1
					self.Manual_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor12.sensor_limitAhead == 1 or self.signal_conveyor12.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 chưa có thùng hàng", 0)
					self.flagWarning_RB12transmit_blank = 1
					self.Manual_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor11.sensor_limitAhead == 1 or self.signal_conveyor11.sensor_limitBehind == 1 and \
					self.signal_conveyor12.sensor_limitAhead == 1 or self.signal_conveyor12.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 chưa có thùng hàng", 0)
					self.flagWarning_RB11transmit_blank = 1
					self.flagWarning_RB12transmit_blank = 1
					self.Manual_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.data_hcuIorequest.output1 = 1
			elif cy_no == 2:
				self.data_hcuIorequest.output2 = 1
			elif cy_no == 3:
				self.data_hcuIorequest.output1 = 1
				self.data_hcuIorequest.output2 = 1

			self.Manual_checkfloor1()

		elif self.case_substep == 3:
			if cy_no == 1:
				self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output1 = 0
					self.case_step = 2
					self.case_substep = 1
			elif cy_no == 2:
				self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output2 = 0
					self.case_step = 2
					self.case_substep = 1				

			elif cy_no == 3:
				self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
				self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output1 = 0
					self.data_hcuIorequest.output2 = 0
					self.case_step = 2
					self.case_substep = 1

	def Manual_CYFloor2_trsm(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor21.sensor_limitAhead == 1 or self.signal_conveyor21.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 chưa có thùng hàng", 0)
					self.flagWarning_RB21transmit_blank = 1
					self.Manual_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor22.sensor_limitAhead == 1 or self.signal_conveyor22.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 chưa có thùng hàng", 0)
					self.flagWarning_RB22transmit_blank = 1
					self.Manual_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor21.sensor_limitAhead == 1 or self.signal_conveyor21.sensor_limitBehind == 1 and \
					self.signal_conveyor22.sensor_limitAhead == 1 or self.signal_conveyor22.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 chưa có thùng hàng", 0)
					self.flagWarning_RB21transmit_blank = 1
					self.flagWarning_RB22transmit_blank = 1
					self.Manual_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.data_hcuIorequest.output1 = 1
			elif cy_no == 2:
				self.data_hcuIorequest.output2 = 1
			elif cy_no == 3:
				self.data_hcuIorequest.output1 = 1
				self.data_hcuIorequest.output2 = 1

			self.Manual_checkfloor2()

		elif self.case_substep == 3:
			if cy_no == 1:
				self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output3 = 0
					self.case_step = 2
					self.case_substep = 1

			elif cy_no == 2:
				self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output4 = 0
					self.case_step = 2
					self.case_substep = 1				

			elif cy_no == 3:
				self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
				self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.data_hcuIorequest.output3 = 0
					self.data_hcuIorequest.output4 = 0
					self.case_step = 2
					self.case_substep = 1

	def Manual_checkfloor1(self):
		if self.step_checkpos == 0:
			self.ct_checkpos = rospy.get_time()
			self.step_checkpos = 1
			self.data_hcuIorequest.output5 = 1
		else:
			if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
				# print("AGV đang chờ xác nhận vị trí với NR")
				if self.hcu_info.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
					print("AGV đã xác nhận vị trí tầng 1 với NARIME_CY")
					self.step_checkpos = 0
					self.case_substep = 3
			
			else:
				print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
				self.Manual_CYreset()
				self.flagWarning_NRpos1 = 1

	def Manual_checkfloor2(self):
		# - Wait CYM1 confirm 
		if self.step_checkfloor2 == 0:
			self.ct_checkfloor2 = rospy.get_time()
			self.step_checkfloor2 = 1
		else:
			if rospy.get_time() - self.ct_checkfloor2 < self.TIME_CHECKFLOOR2:
				if self.mcu_info.status_input1 == 1:  
					print("AGV đã xác nhận vị trí tầng 2 với NARIME_CY")                  
					self.case_substep = 3
					self.step_checkfloor2 = 0
			else:
				print("Connection Timeout! AGV không xác nhận vị trí tầng 2 với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
				self.Manual_CYreset()
				self.flagWarning_NRpos2 = 1
	
	def Manual_checkdone(self):
		if self.step_checkdone == 0:
			self.ct_checkdone = rospy.get_time()
			self.step_checkdone = 1
			self.data_hcuIorequest.output6 = 1
		else:
			if rospy.get_time() - self.ct_checkdone < self.TIME_CHECKDONE:
				if self.hcu_info.status_input6 == 1:
					print("Cho phép AGV thực hiện nhiệm vụ tiếp theo!")
					self.Manual_CYreset()
			else:
				print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
				self.Manual_CYreset()
				self.flagWarning_NRdone = 1

	def Manual_CYreset(self):
		print("Manual liên kết >> System reseted")
		self.data_hcuIorequest = HCU_io_request()
		self.data_mcuIorequest.output1 = 0
		self.control_conveyors = Control_conveyors()	
		self.conveyor11_taskByHand = self.CYTASK_STOP
		self.conveyor12_taskByHand = self.CYTASK_STOP
		self.conveyor21_taskByHand = self.CYTASK_STOP
		self.conveyor22_taskByHand = self.CYTASK_STOP	
		self.case_step = 1
		self.case_substep = 1
		self.step_checkdone = 0
		self.step_checkpos = 0
		self.step_checkfloor2 = 0

	def Manual_unlink_CYreset(self):
		print("Manual ko liên kết >> System reseted")
		self.case_substep = 1
		self.conveyor11_taskByHand = self.CYTASK_STOP
		self.conveyor12_taskByHand = self.CYTASK_STOP
		self.conveyor21_taskByHand = self.CYTASK_STOP
		self.conveyor22_taskByHand = self.CYTASK_STOP	
		self.control_conveyors = Control_conveyors()

	def Manual_unlink_CYFloor1_recv(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 đã có thùng hàng", 0)
					self.flagWarning_RB11recieve_full = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
					self.flagWarning_RB12recieve_full = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0 and \
					self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 đã có thùng hàng", 0)
					self.flagWarning_RB11recieve_full = 1
					self.flagWarning_RB12recieve_full = 1
					self.Manual_unlink_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 11!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()
			elif cy_no == 2:
				self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 12!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

			elif cy_no == 3:
				self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
				self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 11 && 12!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

	def Manual_unlink_CYFloor2_recv(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 đã có thùng hàng", 0)
					self.flagWarning_RB21recieve_full = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 đã có thùng hàng", 0)
					self.flagWarning_RB22recieve_full = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0 and \
					self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 && 22 đã có thùng hàng", 0)
					self.flagWarning_RB21recieve_full = 1
					self.flagWarning_RB22recieve_full = 1
					self.Manual_unlink_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 21!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

			elif cy_no == 2:
				self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 22!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1				
					self.Manual_unlink_CYreset()

			elif cy_no == 3:
				self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
				self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
				if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
					print("Manual mode >> AGV got tray 21 && 22!")
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

	def Manual_unlink_CYFloor1_trsm(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor11.sensor_limitAhead == 1 or self.signal_conveyor11.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 11 chưa có thùng hàng", 0)
					self.flagWarning_RB11transmit_blank = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor12.sensor_limitAhead == 1 or self.signal_conveyor12.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 chưa có thùng hàng", 0)
					self.flagWarning_RB12transmit_blank = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor11.sensor_limitAhead == 1 or self.signal_conveyor11.sensor_limitBehind == 1 and \
					self.signal_conveyor12.sensor_limitAhead == 1 or self.signal_conveyor12.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 12 chưa có thùng hàng", 0)
					self.flagWarning_RB11transmit_blank = 1
					self.flagWarning_RB12transmit_blank = 1
					self.Manual_unlink_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1				
					self.Manual_unlink_CYreset()

			elif cy_no == 3:
				self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
				self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

	def Manual_unlink_CYFloor2_trsm(self, cy_no):
		if self.case_substep == 1:
			if cy_no == 1:
				if self.signal_conveyor21.sensor_limitAhead == 1 or self.signal_conveyor22.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 chưa có thùng hàng", 0)
					self.flagWarning_RB21transmit_blank = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				if self.signal_conveyor22.sensor_limitAhead == 1 or self.signal_conveyor22.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 22 chưa có thùng hàng", 0)
					self.flagWarning_RB22transmit_blank = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 3:				
				if self.signal_conveyor21.sensor_limitAhead == 1 or self.signal_conveyor21.sensor_limitBehind == 1 and \
					self.signal_conveyor22.sensor_limitAhead == 1 or self.signal_conveyor22.sensor_limitBehind == 1:
					self.case_substep = 2

				else:
					self.troubleshoot_mess("warn", "Liên kết băng tải, Trên băng tải 21 && 22 chưa có thùng hàng", 0)
					self.flagWarning_RB21transmit_blank = 1
					self.flagWarning_RB22transmit_blank = 1
					self.Manual_unlink_CYreset()

		elif self.case_substep == 2:
			if cy_no == 1:
				self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()
			
			elif cy_no == 2:
				self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1				
					self.Manual_unlink_CYreset()

			elif cy_no == 3:
				self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
				self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
				if self.signal_conveyor21.status == self.CYTASKTRSM_DONE and self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
					self.control_conveyors = Control_conveyors()
					self.case_substep = 1
					self.Manual_unlink_CYreset()

	def Manual_reset_flagWarning(self):
		# - Flag announce Conveyor AGV error
		self.flagWarning_RB11transmit_blank = 0
		self.flagWarning_RB11recieve_full = 0
		self.flagWarning_RB12transmit_blank = 0
		self.flagWarning_RB12recieve_full = 0
		self.flagWarning_RB21transmit_blank = 0
		self.flagWarning_RB21recieve_full = 0
		self.flagWarning_RB22transmit_blank = 0
		self.flagWarning_RB22recieve_full = 0

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
				self.data_mcuIorequest.output2 = 1          # báo với conveyor machine, agv đang có lỗi
			else:
				if self.flag_warning == 1:
					self.statusAGV = self.STTAGV_WARNING
					self.data_mcuIorequest.output2 = 1 
				else:
					self.statusAGV = self.STTAGV_ALLRIGHT
					self.data_mcuIorequest.output2 = 0

			# -- ERROR
			if self.app_button.bt_clearError == 1: # or self.mcu_info.status_btnReset == 1:
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
				self.task_driver.data = self.TASKDRIVER_RESETREAD
				# print ("Run Reset: " + str(self.app_button.bt_clearError) + " | " + str(self.mcu_info.stsButton_reset))
				
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
				self.reset_NRflag()

				# - 
				if self.signal_conveyor11.sensor_checkPiece == 1 or self.signal_conveyor12.sensor_checkPiece == 1 or self.signal_conveyor21.sensor_checkPiece == 1 or self.signal_conveyor22.sensor_checkPiece == 1:
					self.flagWarning_RBempty = 0
					self.flag_runAgain = 1

			else:
				self.task_driver.data = self.TASKDRIVER_READ
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
						if self.app_button.bt_remote == 0:
							vel_manual = self.run_maunal()
							self.pub_cmdVel(vel_manual, self.rate_cmdvel, rospy.get_time())
					else:
						self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

				else: # -- Has error
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())
					self.data_hcuIorequest = HCU_io_request()
					self.data_mcuIorequest = MCU_io_request()
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
					self.Manual_CYreset()
					self.flagWarning_NRgeneral = 0
					self.btValue_linkCY = self.app_button.bt_linkConveyor					

				# -- control conveyor by manual
				if self.app_button.bt_linkConveyor == 1:
					if self.mcu_info.status_input2 == 0:
						# -- 11 receive
						if self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_recv(1)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- 12 receive
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_recv(2)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- floor1 receive 
						elif self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_recv(3)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- 21 receive	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor2_recv(1)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- 22 receive	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
							if self.case_step == 1:
								self.Manual_CYFloor2_recv(2)

							elif self.case_step	== 2:
								self.Manual_checkdone()	
						
						# -- floor2 receive	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
							if self.case_step == 1:
								self.Manual_CYFloor2_recv(3)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- 11 transmit
						elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_trsm(1)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						# -- 12 transmit
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_trsm(2)

							elif self.case_step	== 2:
								self.Manual_checkdone()
						
						# -- floor1 transmit 
						elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor1_trsm(3)

							elif self.case_step	== 2:
								self.Manual_checkdone()
						
						# -- 21 transmit	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_STOP:
							if self.case_step == 1:
								self.Manual_CYFloor2_trsm(1)

							elif self.case_step	== 2:
								self.Manual_checkdone()
						
						# -- 22 transmit	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
							if self.case_step == 1:
								self.Manual_CYFloor2_trsm(2)

							elif self.case_step	== 2:
								self.Manual_checkdone()	
						
						# -- floor2 transmit	
						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
							if self.case_step == 1:
								self.Manual_CYFloor2_trsm(3)

							elif self.case_step	== 2:
								self.Manual_checkdone()

						elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
							self.Manual_CYreset()
							self.Manual_reset_flagWarning()

					else:
						self.troubleshoot_mess("warn", "Băng tải máy gặp lỗi >> ko cho phép AGV thực hiện nhiệm vụ", 0)
						self.flagWarning_NRgeneral = 1	
				
				else:
					# -- 
					if self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_recv(1)

					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_recv(2)

					elif self.conveyor11_taskByHand == self.CYTASK_RECEIVE and self.conveyor12_taskByHand == self.CYTASK_RECEIVE and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_recv(3)							
					# -
					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor2_recv(1)						

					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
						self.Manual_unlink_CYFloor2_recv(2)						

					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_RECEIVE and self.conveyor22_taskByHand == self.CYTASK_RECEIVE:
						self.Manual_unlink_CYFloor2_recv(3)					
					# --
					elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_trsm(1)					
						
					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_trsm(2)						

					elif self.conveyor11_taskByHand == self.CYTASK_TRANSMIT and self.conveyor12_taskByHand == self.CYTASK_TRANSMIT and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor1_trsm(3)						
					# -
					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYFloor2_trsm(1)							

					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
						self.Manual_unlink_CYFloor2_trsm(2)			

					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_TRANSMIT and self.conveyor22_taskByHand == self.CYTASK_TRANSMIT:
						self.Manual_unlink_CYFloor2_trsm(3)						
					# -- 
					elif self.conveyor11_taskByHand == self.CYTASK_STOP and self.conveyor12_taskByHand == self.CYTASK_STOP and self.conveyor21_taskByHand == self.CYTASK_STOP and self.conveyor22_taskByHand == self.CYTASK_STOP:
						self.Manual_unlink_CYreset()
						self.Manual_reset_flagWarning()										

			# ------------ Speaker ------------ #
				# -- Speaker
				if self.app_button.bt_speaker == True:
					self.enable_speaker = 1
					if self.app_button.bt_speakerSound_1 == 1:
						self.speaker_effect = self.SPK_MOVE
					if self.app_button.bt_speakerSound_2 == 2:
						self.speaker_effect = self.SPK_WARN
					if self.app_button.bt_speakerSound_3 == 3:
						self.speaker_effect = self.SPK_ERR
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
				
				# -------------LED -------------- #
				if self.app_button.bt_light_green == 1:
					self.led_effect = self.LED_SIMPLERUN
				if self.app_button.bt_light_blue == 1:
					self.led_effect = self.LED_STOPBARRIER
				if self.app_button.bt_light_red == 1:
					self.led_effect = self.LED_ERR
				
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
						if self.completed_after_mission == 0:
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

					# --
					if self.flag_runAgain == 1:
						self.flag_runAgain = 0
						self.resetAll_variable()
						print ("---- Reset - Run process again ----")
						self.process = 43

					self.job_doing = 1
					self.flag_Byhand_to_Auto = 0
					self.disable_brake.data = 0
					self.process = 2	

				else:
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

					if self.mission_after == 0:                                   # AGV chưa cập nhật lệnh / AGV đứng dừng tránh AGV khác.
						self.charger_requir = self.CHARGER_OFF
						self.completed_after_mission = 1
						self.data_hcuIorequest = HCU_io_request()
						self.data_mcuIorequest = MCU_io_request()
						# -
						self.reset_NRflag()

					elif self.mission_before == 66 and self.mission_after == 10: # - Nhiệm vụ sạc.
						self.completed_after_mission = 1
						print ("-------- Charger ----------")
						self.charger_requir = self.CHARGER_ON
					
					else:
						if self.mission_before == 0:     # - Nhiệm vụ nhận hàng
							if self.mission_after == 1:  # - Nhận khay ở kho Modula
								if self.case_step == 1:                     # Check AGV đã đến băng tải chưa 
									if self.step_checkpos == 0:
										self.ct_checkpos = rospy.get_time()
										self.step_checkpos = 1
										self.data_hcuIorequest.output5 = 1
									else:
										if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
											# print("AGV đang chờ xác nhận vị trí với NR")
											if self.hcu_info.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
												print("AGV đã xác nhận vị trí với NARIME_CY")
												self.step_checkpos = 0
												self.case_step = 2
										
										else:
											print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
											self.data_hcuIorequest.output5 = 0
											self.step_checkpos = 0
											self.flagWarning_NRpos1 = 1
											self.case_step = 1

								elif self.case_step == 2:                   # Check tray quatity on AGV 
									self.mission_val = self.get_cyfloor1_number(1)
									if self.mission_val == 0:       # if AGV has enough trays, annouce complete
										self.completed_after_mission = 1            
										self.case_step = 1
									else:
										self.cy_val = self.get_cyfloor2_number()
										self.case_step = 3
									
									print("Vị trí băng tải cần nhận là: ", self.mission_val, " ", self.cy_val)

								elif self.case_step == 3:                   # Quản lý và gửi bit nhận hàng
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:
											if self.case_substep == 1:
												if self.hcu_info.status_input1 == 1:
													self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 11", 0)
													self.data_hcuIorequest.output1 = 1
													self.data_mcuIorequest.output1 = 0
													self.case_step = 4
												else:
													if self.cy_val == 1:      # Floor2 of RB had tray
														self.completed_after_mission = 1
														self.case_step = 1
													else:
														self.troubleshoot_mess("warn", "AutoMode -> AGV KO phát hiện tray 11 trên băng tải cấp", 0)
														self.flagWarning_RBreceive11_empty = 1
														self.case_substep = 2
														self.ct_refresh_receive11_empty = rospy.get_time()

											elif self.case_substep == 2:
												if self.hcu_info.status_input1 == 0:
													self.ct_refresh_receive11_empty = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_receive11_empty >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi nhận thùng 11 đã tự động được xóa", 0)
														self.flagWarning_RBreceive11_empty = 0
														self.case_substep = 1
														self.case_step = 4													

										elif self.mission_val == 0b0010:
											if self.case_substep == 1:
												if self.hcu_info.status_input2 == 1:
													self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 12", 0)
													self.data_hcuIorequest.output2 = 1
													self.data_mcuIorequest.output1 = 0
													self.case_step = 4
												else:
													if self.cy_val == 1:      # Floor2 of RB had enough tray
														self.completed_after_mission = 1
														self.case_step = 1
													else:
														self.troubleshoot_mess("warn", "AutoMode -> AGV KO phát hiện tray 12 trên băng tải cấp", 0)
														self.flagWarning_RBreceive12_empty = 1
														self.case_substep = 2
														self.ct_refresh_receive12_empty = rospy.get_time()

											elif self.case_substep == 2:
												if self.hcu_info.status_input2 == 0:
													self.ct_refresh_receive12_empty = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_receive12_empty >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi nhận thùng 12 đã tự động được xóa", 0)
														self.flagWarning_RBreceive12_empty = 0
														self.case_substep = 1
														self.case_step = 4	
														
										elif self.mission_val == 0b0011:
											if self.case_substep == 1:
												if self.hcu_info.status_input1 == 1 and self.hcu_info.status_input2 == 1:
													self.troubleshoot_mess("info", "AutoMode -> AGV cần nhận tray 11 && 12", 0)
													self.data_hcuIorequest.output1 = 1
													self.data_hcuIorequest.output2 = 1
													self.data_mcuIorequest.output1 = 0
													self.case_step = 4
												else:
													if self.cy_val == 1:      # Floor2 of RB had tray
														self.completed_after_mission = 1
														self.case_step = 1
													else:
														self.troubleshoot_mess("warn", "AutoMode -> AGV KO phát hiện tray 11 && 12 trên băng tải cấp", 0)
														self.flagWarning_RBreceive11_empty = 1
														self.flagWarning_RBreceive12_empty = 1
														self.case_substep = 2
														self.ct_refresh_receiveFloor1_empty = rospy.get_time()

											elif self.case_substep == 2:
												if self.hcu_info.status_input1 == 0 or self.hcu_info.status_input2 == 0:
													self.ct_refresh_receiveFloor1_empty = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_receiveFloor1_empty >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi nhận thùng 11 && 12 đã tự động được xóa", 0)
														self.flagWarning_RBreceive11_empty = 0
														self.flagWarning_RBreceive12_empty = 0
														self.case_substep = 1
														self.case_step = 4		
									
									else:
										self.flagWarning_NRgeneral = 1
										self.mission_val = 0
										self.case_step = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step == 4:                   # Thực hiện nhận hàng
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE: 
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 5

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE: 
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 5

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE							
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE: 
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 5
									else:
										self.flagWarning_NRgeneral = 1
										self.case_step = 1
										self.mission_val = 0
										self.control_conveyors = Control_conveyors()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step == 5:                   # Check tray có hàng trên AGV.
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay trống", 0)
									self.mission_val = self.get_cy_number(2)
									if self.mission_val == 0:
										self.troubleshoot_mess("info", "AutoMode -> Không phát hiện tray cần trả trên AGV", 0)
										self.flagWarning_RBempty = 1
										self.flag_runAgain = 1
										# self.case_step = 1
										# self.case_substep = 1

									else:
										self.completed_after_mission = 1
										self.case_step = 1

							elif self.mission_after == 3: # - Nhận khay trống ở dây chuyển IE5
								if self.case_step == 1:
									self.mission_val = self.get_cy_number(1)
									self.case_step = 2							
									print("Vị trí băng tải cần nhận là: ", self.mission_val)

								elif self.case_step == 2:
									if self.mcu_info.status_input2 == 0:                         
										# self.data_mcuIorequest.output1 = 1
										# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
										if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.data_hcuIorequest.output1 = 1
										
										# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
										elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.data_hcuIorequest.output2 = 1
										
										# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
										elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
											self.data_hcuIorequest.output1 = 1
											self.data_hcuIorequest.output2 = 1

										# - 21 or 22 or (21 && 22)
										elif self.mission_val == 0b0100 or self.mission_val == 0b1000 or self.mission_val == 0b1100: 
											self.case_step = 4

										self.check_NRpos()
									
									else:
										self.flagWarning_NRgeneral = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 1)

								elif self.case_step == 3:
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 10

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 10

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 10
											
										# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
										elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 4

										# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
										elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 4

										# - (11 && 12 && 21) or (11 && 12 && 22)
										elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
											self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
											self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
											if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 4		

									else:
										self.flagWarning_NRgeneral = 1
										self.control_conveyors = Control_conveyors()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 2)
								
								elif self.case_step == 4:
									self.data_hcuIorequest.output1 = 0
									self.data_hcuIorequest.output2 = 0									
									if self.mcu_info.status_input2 == 0: 
										# self.data_mcuIorequest.output1 = 1
										if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
											self.data_hcuIorequest.output3 = 1
										elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
											self.data_hcuIorequest.output4 = 1
										elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
											self.data_hcuIorequest.output3 = 1
											self.data_hcuIorequest.output4 = 1								
										
										# - Wait CYM1 confirm                        
										self.check_NRfloor2()
									else:
										self.flagWarning_NRgeneral = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 5)
								
								elif self.case_step == 5:
									if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
										self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
											self.control_conveyors.No3_mission = self.CYTASK_STOP
											self.case_step = 10
									elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
										self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
											self.control_conveyors.No4_mission = self.CYTASK_STOP
											self.case_step = 10																
									elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
										self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
										self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
										if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
											self.control_conveyors.No3_mission = self.CYTASK_STOP
											self.control_conveyors.No4_mission = self.CYTASK_STOP
											self.case_step = 10

								elif self.case_step == 10:
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay hàng", 0)
									self.check_NRdone()

						elif self.mission_before == 1:    # - Nhiệm vụ trả hàng 
							if self.mission_after == 2:   # - AGV Trả hàng tại truyền IE5
								if self.case_step == 1:
									self.mission_val = self.get_cy_number(2)
									self.case_step = 2

								elif self.case_step == 2:
									if self.mcu_info.status_input2 == 0:                         
										# self.data_mcuIorequest.output1 = 1
										# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
										if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.data_hcuIorequest.output1 = 1
										
										# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
										elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.data_hcuIorequest.output2 = 1
										
										# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
										elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
											self.data_hcuIorequest.output1 = 1
											self.data_hcuIorequest.output2 = 1

										# - 21 or 22 or (21 && 22)
										elif self.mission_val == 0b0100 or self.mission_val == 0b1000 or self.mission_val == 0b1100: 
											self.case_step = 4

										self.check_NRpos()
									else:
										self.flagWarning_NRgeneral = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 1)

								elif self.case_step == 3:
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 10

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 10

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 10
											
										# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
										elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 4

										# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
										elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 4

										# - (11 && 12 && 21) or (11 && 12 && 22)
										elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 4		

									else:
										self.flagWarning_NRgeneral = 1
										self.control_conveyors = Control_conveyors()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 2)	
								
								elif self.case_step == 4:
									self.data_hcuIorequest.output1 = 0
									self.data_hcuIorequest.output2 = 0	
									if self.mcu_info.status_input2 == 0: 
										# self.data_mcuIorequest.output1 = 1
										if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
											self.data_hcuIorequest.output3 = 1
										elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
											self.data_hcuIorequest.output4 = 1
										elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
											self.data_hcuIorequest.output3 = 1
											self.data_hcuIorequest.output4 = 1								
										
										# - Wait CYM1 confirm                        
										self.check_NRfloor2()

									else:
										self.flagWarning_NRgeneral = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 5)

								elif self.case_step == 5:
									if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
										self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
											self.control_conveyors.No3_mission = self.CYTASK_STOP
											self.case_step = 10

									elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
										self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
											self.control_conveyors.No4_mission = self.CYTASK_STOP
											self.case_step = 10
																										
									elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
										self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
										self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
										if self.signal_conveyor21.status == self.CYTASKTRSM_DONE and self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
											self.control_conveyors.No3_mission = self.CYTASK_STOP
											self.control_conveyors.No4_mission = self.CYTASK_STOP
											self.case_step = 10

								elif self.case_step == 10:
									self.troubleshoot_mess("info", "AutoMode -> AGV đã trả xong khay hàng", 0)
									self.check_NRdone()
															
							elif self.mission_after == 4:  # - AGV trả tray về tại kho
								if self.case_step == 1:                     # Check AGV đã đến băng tải chưa 
									if self.step_checkpos == 0:
										self.ct_checkpos = rospy.get_time()
										self.step_checkpos = 1
										self.data_hcuIorequest.output5 = 1
									else:
										if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
											# print("AGV đang chờ xác nhận vị trí với NR")
											if self.hcu_info.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
												print("AGV đã xác nhận vị trí với NARIME_CY")
												self.step_checkpos = 0
												self.case_step = 2
										
										else:
											print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
											self.data_hcuIorequest.output5 = 0
											self.step_checkpos = 0
											self.flagWarning_NRpos1 = 1
											self.case_step = 1

								elif self.case_step == 2:                   # Xác nhận băng tải cần trả
									self.mission_val = self.get_cyfloor1_number(2)
									self.case_step = 3

								elif self.case_step == 3:
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:                     
											if self.case_substep == 1:
												if self.hcu_info.status_input1 == 0:           # Conveyor is available
													self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 11", 0)
													self.data_hcuIorequest.output1 = 1
													self.data_mcuIorequest.output1 = 1
													self.case_step = 4
												else:
													# Because of tray on the storage, AGV cannot transfer and announce warning
													# if remove the tray, After 5s, AGV will continue transfering
													self.flagWarning_RBtransmit11_full = 1
													self.ct_refresh_transmit11_full = rospy.get_time()
													self.case_substep = 2
											
											elif self.case_substep == 2:
												if self.hcu_info.status_input1 == 1:
													self.ct_refresh_transmit11_full = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_transmit11_full >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi trả thùng 11 đã tự động được xóa", 0)
														self.flagWarning_RBtransmit11_full = 0
														self.case_substep = 1
														self.case_step = 4

										elif self.mission_val == 0b0010:
											if self.case_substep == 1:
												if self.hcu_info.status_input2 == 0:           # Conveyor is available
													self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 12", 0)
													self.data_hcuIorequest.output2 = 1
													self.data_mcuIorequest.output1 = 1
													self.case_step = 4
												else:
													# Because of tray on the storage, AGV cannot transfer and announce warning
													# if remove the tray, After 5s, AGV will continue transfering
													self.flagWarning_RBtransmit12_full = 1
													self.ct_refresh_transmit12_full = rospy.get_time()
													self.case_substep = 2
											
											elif self.case_substep == 2:
												if self.hcu_info.status_input2 == 1:
													self.ct_refresh_transmit12_full = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_transmit12_full >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi trả thùng 12 đã tự động được xóa", 0)
														self.flagWarning_RBtransmit12_full = 0
														self.case_substep = 1
														self.case_step = 4

										elif self.mission_val == 0b0011:
											if self.case_substep == 1:
												if self.hcu_info.status_input1 == 0 and self.hcu_info.status_input2 == 0:           # Conveyor is available
													self.troubleshoot_mess("info", "AutoMode -> AGV cần trả tray 11 && 12", 0)
													self.data_hcuIorequest.output1 = 1
													self.data_hcuIorequest.output2 = 1
													self.data_mcuIorequest.output1 = 1
													self.case_step = 4
												else:
													# Because of tray on the storage, AGV cannot transfer and announce warning
													# if remove the tray, After 5s, AGV will continue transfering
													self.flagWarning_RBtransmit11_full = 1
													self.flagWarning_RBtransmit12_full = 1
													self.ct_refresh_transmitFloor1_full = rospy.get_time()
													self.case_substep = 2
											
											elif self.case_substep == 2:
												if self.hcu_info.status_input1 == 1 or self.hcu_info.status_input2 == 1:           # Conveyor is available
													self.ct_refresh_transmitFloor1_full = rospy.get_time()
												else:
													if rospy.get_time() - self.ct_refresh_transmitFloor1_full >= self.TIME_REFRESH_ERR:
														self.troubleshoot_mess("info", "AutoMode -> Lỗi trả thùng 11 && 12 đã tự động được xóa", 0)
														self.flagWarning_RBtransmit11_full = 0
														self.flagWarning_RBtransmit12_full = 1
														self.case_substep = 1
														self.case_step = 4									
									
									else:
										self.flagWarning_NRgeneral = 1
										self.data_hcuIorequest = HCU_io_request()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)
								
								elif self.case_step == 4:                   ## thực hiện trả hàng
									if self.mcu_info.status_input2 == 0:
										if self.mission_val == 0b0001:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE: 
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.case_step = 5

										elif self.mission_val == 0b0010:
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor12.status == self.CYTASKTRSM_DONE: 
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 5

										elif self.mission_val == 0b0011:
											self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT							
											self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
											if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE: 
												self.control_conveyors.No1_mission = self.CYTASK_STOP
												self.control_conveyors.No2_mission = self.CYTASK_STOP
												self.case_step = 5
									else:
										self.flagWarning_NRgeneral = 1
										self.control_conveyors = Control_conveyors()
										self.troubleshoot_mess("warn", "AutoMode -> Cụm máy băng tải bị lỗi", 0)

								elif self.case_step == 5:             # Finish and reset
									self.troubleshoot_mess("info", "AutoMode -> AGV đã nhận xong khay trống", 0)
									# self.check_NRdone()
									self.completed_after_mission = 1

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
				self.data_hcuIorequest = HCU_io_request()
				self.data_mcuIorequest = MCU_io_request()
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
			# self.Traffic_infoRespond.task_num = self.case_step
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
			if (d > float(1/self.FREQ_PUBBOARD)): # < 20hz 
				self.pre_timeBoard = time_curr

				# -------- Request Board OC - Conveyors -------- #
				self.pub_controlConveyors.publish(self.control_conveyors)

				# -------- Request Board Main -------- #
				if self.flag_error == 0:
					# tat Loa khi sac thanh cong!
					if self.completed_after_mission == 1 and self.mode_operate == self.MODE_AUTO and self.mission_after == 10 and self.mission_before == 66:
						# print ("NOW HERE")
						if self.charger_write == self.CHARGER_ON:
							if self.psu_info.charge_current >= self.charger_valueOrigin :
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
				# self.pub_Main(self.charger_write, self.EMC_write, self.EMC_reset)  # MISSION

				# -- Battery info
				self.pin_info.pinState = self.battery_info.pinState
				self.pin_info.pinVolt = self.battery_info.pinVolt
				self.pin_info.pinCurr = self.battery_info.pinCurr
				self.pin_info.pinPercent = self.battery_info.pinPercent
				self.pin_info.timeCharge = self.convert_intTotime(self.battery_info.timeCharge)
				self.pin_info.timeChargePropose = self.convert_intTotime(self.battery_info.timeChargePropose)
				self.pub_pin.publish(self.pin_info)

				# -------- Request Board HC -------- #
				self.data_hcuLedsound.RGB1 = self.led_effect 
				self.data_hcuLedsound.RGB2 = self.led_effect 
				self.data_hcuLedsound.sound_type = self.speaker_effect 
				self.pub_hcuLedsound.publish(self.data_hcuLedsound)
				self.pub_hcuIorequest.publish(self.data_hcuIorequest)

				# -------- Request PSU and MCU board -------- #
				self.data_mcuSTRrequest.STR_reset = self.EMC_reset
				self.data_mcuSTRrequest.STR_write = self.EMC_write
				self.pub_mcuSTRrequest.publish(self.data_mcuSTRrequest)
				self.pub_mcuIorequest.publish(self.data_mcuIorequest)

				self.data_PSUrequest.charge_write = self.charger_write
				self.pub_PSUrequest.publish(self.data_PSUrequest)

				# -------- Request CPD Board -------- #
				# self.pub_controlCPD.publish(self.control_CPD)

				# -------- Request Task Driver Motor -------- #
				self.pub_taskDriver.publish(self.task_driver)

			# -------------------------------------------------------------- #
			time_curr = rospy.get_time()
			d = (time_curr - self.pre_timePub)
			if (d > float(1/self.FREQ_PUB)): # < 20hz 
				self.pre_timePub = time_curr
				# ---------------- Cancel Mission ---------------- #
				if self.cancelMission_control.data == 1:
					self.flag_cancelMission = 1
					# self.flagWarning_NRpos1 = 0
					# self.flagWarning_NRpos2 = 0
					# self.flagWarning_NRdone = 0

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
			# self.pub_AGVToyoSignal.publish(self.AGVToyo_signal)

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
