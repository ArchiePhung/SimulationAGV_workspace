#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 28/06/2023
"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal

import os
import re  
import subprocess
import argparse
from datetime import datetime

import sqlite3

# -- 
from app_interface import *

class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		# --
		self.name_card = rospy.get_param("name_card", "wlp3s0b1")
		self.name_card = "wlp3s0b1"

		self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		#self.address_traffic = "192.168.1.92"
		self.pre_timePing = time.time()
		# --
		rospy.init_node('app_ros', anonymous=False)
		self.rate = rospy.Rate(20)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(540)
		self.widget.setFixedWidth(1024)
		# --
		self.valueLable = valueLable()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# -----------------------------------------------------------
		# -- Break
		rospy.Subscriber("/enable_brake", Bool, self.callback_brakeControl) 
		self.status_brake = Bool()

		# -- Driver1
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1) 
		self.driver1_respond = Driver_respond()

		# -- Driver2
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2) 
		self.driver2_respond = Driver_respond()

		# -- HC
		rospy.Subscriber("/HC_info", HC_info, self.callback_HC) 
		self.HC_info = HC_info()

		# -- Main
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.main_info = POWER_info()

		# -- CPD Boad
		rospy.Subscriber("/lift_status", Lift_status, self.callback_OC_board) # lay thong tin trang thai mach dieu khien ban nang.
		self.OC_status = Lift_status()

		# -- Status Port
		rospy.Subscriber("/status_port", Status_port, self.callback_statusPort) 
		self.status_port = Status_port()

		# ------------------------------
		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.callback_nav350) 
		self.nav350_data = Nav350_data()

		# -- data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.callback_safetyNAV) 
		self.safety_NAV = Int8()

		# -- Pose robot
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPose) 
		self.robotPose_nav = PoseStamped()

		# -- Traffic cmd
		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()

		# -- Traffic cmd
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdRequest_callback) 
		self.NN_cmdRequest = NN_cmdRequest()

		# -- Pose robot
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_NN_infoRequest) 
		self.NN_infoRequest = NN_infoRequest()

		# -- info AGV
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()

		# -- info move
		rospy.Subscriber("/status_goal_control", Status_goal_control, self.goalControl_callback)
		self.status_goalControl = Status_goal_control() # sub from move_base

		# -- Launch
		rospy.Subscriber("/status_launch", Status_launch, self.callback_statusLaunch)
		self.status_launch = Status_launch()

		# -- Info Move
		# rospy.Subscriber("/navigation_respond", Navigation_respond, self.callback_navigationRespond)
		# self.navigation_respond = Navigation_respond()

		# -----------------------------------------------------------
		rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
		self.cancelMission_status = Int16()
		# --
		self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
		self.cancelMission_control = Int16()
		# --
		self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)
		self.app_button = App_button()
		self.pre_app_setColor = App_color()

		# --
		rospy.Subscriber("/nav350_reflectors", Reflector_array, self.callback_nav350Reflectors) 
		self.nav350_reflectors = Reflector_array()
		self.arrReflector = []
		# --
		self.name_agv = ""
		self.ip_agv = ""
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRun_byhand_tryTarget = 3
		self.modeRun_cancelMission = 5

		# self.app_button.bt_speaker = 1

	def anlis_ref2(self):
		self.arrReflector = []

		max_x = -1000
		max_y = -1000

		length = self.nav350_reflectors.num_reflector
		for i in range(length):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)
			ang = self.limitAngle(ang0)

			p_x, p_y = self.convert_position(dis, ang)

		if max_x < abs(p_x):
			max_x = abs(p_x)

		if max_y < abs(p_y):
			max_y = abs(p_y)

		rate_show = 0.0
		rate_xy = float(max_x/max_y)
		if rate_xy > 0.5:
			rate_show = 1.0 # (max_x*1000)/431.
		else:
			rate_show = 1.0 # (max_y*1000)/811.
		# ----
		
		rate_show = 0.07
		# print ("----------")
		for i in range(length):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000. + self.welcomeScreen.valueLable.angleCompare)
			ang = self.limitAngle(ang0)

			p_x, p_y = self.convert_position(dis, ang)
			rp_x = int(p_x/rate_show)
			rp_y = int(p_y/rate_show)
			sh_x = rp_y + 400
			sh_y = rp_x + 200
			
			# print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )
			# --
			reflector = Reflector()
			reflector.x = sh_x
			reflector.y = sh_y
			reflector.localID  = str(self.nav350_reflectors.reflectors[i].LocalID)
			reflector.globalID = str(self.nav350_reflectors.reflectors[i].GlobalID)

			self.arrReflector.append(reflector)

	def callback_nav350Reflectors(self, data):
		self.nav350_reflectors = data
		# print ("NAV NAV")
		# self.anlis_ref1()
		self.anlis_ref2()
		# --

			
	def callback_brakeControl(self, data):
		self.status_brake = data
		
	def callback_driver1(self, data):
		self.driver1_respond = data

	def callback_driver2(self, data):
		self.driver2_respond = data

	def callback_HC(self, data):
		self.HC_info = data

	def callback_Main(self, data):
		self.main_info = data
		
	def callback_OC_board(self, data):
		self.OC_status = data

	def callback_statusPort(self, data):
		self.status_port = data

	def goalControl_callback(self, data):
		self.status_goalControl = data
		
	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callback_nav350(self, data):
		self.nav350_data = data

	def callback_safetyNAV(self, data):
		self.safety_NAV = data

	def callback_robotPose(self, data):
		self.robotPose_nav = data

	def callback_server_cmdRequest(self, data):
		self.server_cmdRequest = data

	def NN_cmdRequest_callback(self, data):
		self.NN_cmdRequest = data	

	def callback_NN_infoRequest(self, data):
		self.NN_infoRequest = data

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data	

	def callback_statusLaunch(self, data):
		self.status_launch = data

	def callback_navigationRespond(self, data):
		self.navigation_respond = data

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

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

	def convert_position(self, distance, angle):
		x = 0
		y = 0
		x = distance*cos(angle)
		y = distance*sin(angle)
		# y = distance*cos(angle)
		# x = distance*sin(angle)
		return x, y

	def ping_traffic(self, address):
		try:
			ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
			# print(ping)
			vitri = str(ping).find("time")
			time_ping = str(ping)[(vitri+5):(vitri+9)]
			# print (time_ping)
			return str(float(time_ping))
		except Exception:
			return '-1'

	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			print ("address: ", address)
			return address
		except Exception:
			return "-1"

	def run_screen(self):
		self.widget.show()
		try:
			# print ("run 1")
			sys.exit(self.app.exec_())
			# print ("run 2")
		except:
			pass
			# print("Exiting 1")
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	# --
	def get_MAC(self, name_card): # name_card : str()
		try:
			MAC = ''
			output = os.popen("ip addr show {}".format(name_card) ).read()
			pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
			pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

			if (pos1 >= 0 and pos2 > 0):
				MAC = str(output)[pos1+11:pos2]

			print ("MAC: ", MAC)
			return MAC
		except Exception:
			return "-1"
	# --
	def get_qualityWifi(self, name_card): # int
		try:
			quality_data = '0'
			output = os.popen("iwconfig {}".format(name_card)).read()
			pos_quality = str(output).find('Link Quality=')
			# -
			if pos_quality >= 0:
				quality_data = str(output)[pos_quality+13:pos_quality+15]
			# print ("quality_data: ", int(quality_data) )
			# -
			return int(quality_data)
		except Exception:
			return 0
	# ---
	def get_hostname(self):
		try:
			output = os.popen("hostname").read()
			# print ("output: ", output)
			leng = len(output)
			hostname = str(output)[0:leng-1]
			print ("hostname: ", hostname)
			return hostname
		except Exception:
			return "-1"

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def convert_errorAll(self, val):
		switcher={
			0:'AGV Hoạt Động Bình Thường',
			311:'Mất kết nối với Mạch STI-RTC',			
			361:'Mất Kết Nối Với Mạch STI-CPD', # 

			351:'Mất Kết Nối Với Mạch STI-HC', # 
			352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 

			341:'Mất Kết Nối Với Mạch STI-OC', #
			342:'Mất Cổng USB của USB của Mạch STI-OC', # 
			343:'Không Giao Tiếp CAN Với Mạch STI-OC', # 
			344:'Không Giao Tiếp Với Mạch STI-OC1', # 
			345:'Không Giao Tiếp Với Mạch STI-OC2', # 
			346:'Không Giao Tiếp Với Mạch STI-OC3', # 

			323:'Mạng CAN Không Gửi Được', # 
			321:'Mất Kết Nối Với Mạch STI-Main', # 
			322:'Mất Cổng USB của USB của Mạch STI-Main', # 
			251:'Mất Kết Nối Với Driver1', # 
			252:'Lỗi Động Cơ Số 1', # 
			261:'Mất Kết Nối Với Driver2', # 
			262:'Lỗi Động Cơ Số 2', # 
			231:'Mất Kết Nối Với Cảm Biến Góc', # 
			232:'Mất Cổng USB của Cảm Biến IMU', # 
			221:'Mất Kết Nối Với Cảm Biến NAV350', # 
			181:'LoadCell-Ket Noi', # 
			182:'LoadCell-Dau noi', # 
			183:'LoadCell-USB', # 
			184:'Quá Tải 700kg', # 
			222:'Mất Tọa Độ NAV350', # 
			141:'Lỗi Không Chạm Được Cảm Biến Bàn Nâng', # 
			121:'Trạng Thái Dừng Khẩn - EMG', # 
			122:'AGV Bị Chạm Blsock', #
			272:'Không Phát Hiện Được Đủ Gương', #
			281:'Mất TF Parking', #
			282:'Mất Gói GoalControl', #
			441:'AGV Đã Di Chuyển Hết Điểm', #
			442:'AGV Đang Dừng Để Nhường Đường Cho AGV Khác', # 
			477:'Không Có Kệ Tại Vị Trí', # 
			411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
			412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Kệ', # 
			431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
			451:'Điện Áp Của AGV Đang Rất Thấp', # 
			452:'AGV Không Sạc Được Pin', #
			453:'Không Phát Hiện Được Đủ Gương', #

		}
		return switcher.get(val, 'UNK')

	def show_job(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			1:'Kiểm Tra Lại Nhiệm Vụ', # 
			2:'Thực Hiện Nhiệm Vụ Trước', # 
			3:'Kiểm Tra Trạng Thái Kệ',
			4:'Di Chuyển Ra Khởi Vị Trí', # 
			5:'Di Chuyển Giữa Các Điểm', #
			6:'Di Chuyển Vào Vị Trí Thao Tác', # 
			7:'Thực Hiện Nhiệm Vụ Sau', # 
			8:'Đợi Lệnh Mới', # 
			9:'Đợi Hoàn Thành Lệnh Cũ', # 
			20:'Chế Độ Bằng Tay', # 
			30:'Chế Độ Tự Động', # 
			50:'Kiểm Tra Vị Trí Trả Hàng', # 
		}
		return switcher.get(val, job_now)

	def show_misson(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			65:'Nâng Kệ', # 
			1:'Nâng Kệ', # 
			66:'Hạ Kệ', #
			2:'Hạ Kệ', #
			6:'Sạc Pin', # 
			10:'Hạ Kệ\nSạc Pin' # 
		}
		return switcher.get(val, job_now)

	def controlColor(self):
		self.statusColor.lbc_safety_up = self.safety_NAV.data
		# -- HC_info
		self.statusColor.lbc_safety_ahead = self.HC_info.zone_sick_ahead
		self.statusColor.lbc_safety_behind = self.HC_info.zone_sick_behind
		# --
		self.statusColor.lbc_button_clearError = self.main_info.stsButton_reset
		self.statusColor.lbc_button_power = self.main_info.stsButton_power
		self.statusColor.lbc_emg = self.main_info.EMC_status
		self.statusColor.lbc_blsock = self.HC_info.vacham

		# -- Port
		self.statusColor.lbc_port_rtc    = self.status_port.rtc
		self.statusColor.lbc_port_rs485  = self.status_port.driverall
		self.statusColor.lbc_port_nav350 = self.status_port.nav350

		# --
		self.statusColor.lbc_limit_up = self.OC_status.sensorUp.data
		self.statusColor.lbc_limit_down = self.OC_status.sensorDown.data
		self.statusColor.lbc_detect_lifter = self.OC_status.sensorLift.data

	def controlAll(self):
		# -- Mode show
		if (self.NN_infoRespond.mode == 0):   # - launch
			self.valueLable.modeRuning = self.modeRun_launch

		elif (self.NN_infoRespond.mode == 1): # -- md_by_hand
			self.valueLable.modeRuning = self.modeRun_byhand

		elif (self.NN_infoRespond.mode == 2): # -- md_auto
			self.valueLable.modeRuning = self.modeRun_auto

		# -- Battery
		if (self.main_info.charge_current > 0.1):
			self.statusColor.lbc_battery = 4
		else:
			if (self.main_info.voltages < 23.5):
				self.statusColor.lbc_battery = 3
			elif (self.main_info.voltages >= 23.5 and self.main_info.voltages < 24.5):
				self.statusColor.lbc_battery = 2
			else:
				self.statusColor.lbc_battery = 1

		bat = round(self.main_info.voltages, 1)
		if bat > 25.5:
			bat = 25.5
		self.valueLable.lbv_battery = "  " + str(bat)

		# -- status AGV
		self.statusColor.cb_status = self.NN_infoRespond.status
		lg_err = len(self.NN_infoRespond.listError)
		self.valueLable.listError = []
		if (lg_err == 0):
			self.valueLable.listError.append( self.convert_errorAll(0) )
		else:
			length = len(self.valueLable.list_logError)
			if length > 15:
				self.valueLable.list_logError = []
			# -
			for i in range(lg_err):
				self.valueLable.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
				# -
				if self.NN_infoRespond.listError[i] < 400:
					self.valueLable.list_logError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
		# -
		self.valueLable.lbv_name_agv = self.NN_infoRequest.name_agv
		self.valueLable.lbv_numbeReflector = str(self.nav350_data.number_reflectors)
		self.valueLable.lbv_reflectorDetect = str(self.nav350_reflectors.num_reflector)

		# -- Ping
		# deltaTime_ping = (time.time() - self.pre_timePing)%60
		# if (deltaTime_ping > 2.0):
		# 	self.pre_timePing = time.time()
		# 	self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
		# 	# -
		# 	self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)

		# -- 
		self.valueLable.lbv_coordinates_x = str(round(self.robotPose_nav.pose.position.x, 3))
		self.valueLable.lbv_coordinates_y = str(round(self.robotPose_nav.pose.position.y, 3))
		angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
		
		if angle < 0:
			angle_robot = 2*pi + angle
		else:
			angle_robot = angle
		self.valueLable.lbv_coordinates_r = str( round( degrees(angle_robot), 3) )
		# --
		self.valueLable.lbv_route_target = str(self.NN_cmdRequest.target_id) + "\n" + str(self.NN_cmdRequest.target_x) + "\n" + str(self.NN_cmdRequest.target_y) + "\n" + str(round(degrees(self.NN_cmdRequest.target_z), 2)) + "\n" + str(self.NN_cmdRequest.offset)
		# # -
		if len(self.NN_cmdRequest.list_id) >= 5:
			self.valueLable.lbv_route_point0 = str(self.NN_cmdRequest.list_id[0]) + "\n" + str(self.NN_cmdRequest.list_x[0]) + "\n" + str(self.NN_cmdRequest.list_y[0]) + "\n" + str(self.NN_cmdRequest.list_speed[0])
			self.valueLable.lbv_route_point1 = str(self.NN_cmdRequest.list_id[1]) + "\n" + str(self.NN_cmdRequest.list_x[1]) + "\n" + str(self.NN_cmdRequest.list_y[1]) + "\n" + str(self.NN_cmdRequest.list_speed[1])
			self.valueLable.lbv_route_point2 = str(self.NN_cmdRequest.list_id[2]) + "\n" + str(self.NN_cmdRequest.list_x[2]) + "\n" + str(self.NN_cmdRequest.list_y[2]) + "\n" + str(self.NN_cmdRequest.list_speed[2])
			self.valueLable.lbv_route_point3 = str(self.NN_cmdRequest.list_id[3]) + "\n" + str(self.NN_cmdRequest.list_x[3]) + "\n" + str(self.NN_cmdRequest.list_y[3]) + "\n" + str(self.NN_cmdRequest.list_speed[3])
			self.valueLable.lbv_route_point4 = str(self.NN_cmdRequest.list_id[4]) + "\n" + str(self.NN_cmdRequest.list_x[4]) + "\n" + str(self.NN_cmdRequest.list_y[4]) + "\n" + str(self.NN_cmdRequest.list_speed[4])
		
		self.valueLable.lbv_route_job1 = str(self.NN_cmdRequest.before_mission)
		self.valueLable.lbv_route_job2 = str(self.NN_cmdRequest.after_mission)

		self.valueLable.lbv_route_job1_mean = self.show_misson(self.NN_cmdRequest.before_mission)
		self.valueLable.lbv_route_job2_mean = self.show_misson(self.NN_cmdRequest.after_mission)

		self.valueLable.lbv_route_message = self.NN_cmdRequest.command
		self.valueLable.lbv_jobRuning = self.show_job(self.NN_infoRespond.process)
		# -- 
		self.valueLable.lbv_goalFollow_id = str(self.status_goalControl.ID_follow)

		# -- Launch
		self.valueLable.percentLaunch = self.status_launch.persent
		self.valueLable.lbv_launhing = self.status_launch.notification
		self.valueLable.lbv_numberLaunch = self.status_launch.position

		self.valueLable.lbv_notification_driver1 = self.driver1_respond.message_error
		self.valueLable.lbv_notification_driver2 = self.driver2_respond.message_error


	def readButton(self):
		# -- 
		self.app_button.bt_cancelMission = self.welcomeScreen.statusButton.bt_cancelMission
		self.app_button.bt_passAuto 	 = self.welcomeScreen.statusButton.bt_passAuto
		self.app_button.bt_passHand 	 = self.welcomeScreen.statusButton.bt_passHand
		# self.app_button.bt_setting 		 = self.welcomeScreen.statusButton.bt_setting
		self.app_button.bt_clearError 	 = self.welcomeScreen.statusButton.bt_clearError
		# --
		self.app_button.bt_forwards 	  = self.welcomeScreen.statusButton.bt_forwards
		self.app_button.bt_backwards	  = self.welcomeScreen.statusButton.bt_backwards
		self.app_button.bt_rotation_left  = self.welcomeScreen.statusButton.bt_rotation_left
		self.app_button.bt_rotation_right = self.welcomeScreen.statusButton.bt_rotation_right
		self.app_button.bt_stop 		  = self.welcomeScreen.statusButton.bt_stop
		# --
		self.app_button.bt_chg_on	= self.welcomeScreen.statusButton.bt_chg_on
		self.app_button.bt_chg_off	= self.welcomeScreen.statusButton.bt_chg_off

		self.app_button.bt_spk_on  = self.welcomeScreen.statusButton.bt_spk_on
		self.app_button.bt_spk_off  = self.welcomeScreen.statusButton.bt_spk_off

		self.app_button.bt_disableBrake	= self.welcomeScreen.statusButton.bt_disableBrake

		# -- 
		self.app_button.bt_lift_up	 = self.welcomeScreen.statusButton.bt_lift_up
		self.app_button.bt_lift_down = self.welcomeScreen.statusButton.bt_lift_down
		# -
		self.app_button.vs_speed = self.welcomeScreen.statusButton.vs_speed
		self.app_button.bt_resetFrameWork = self.welcomeScreen.statusButton.bt_resetFrameWork

		# -
		self.app_button.bt_tryTarget_start = self.welcomeScreen.statusButton.bt_tryTarget_start
		self.app_button.bt_tryTarget_stop = self.welcomeScreen.statusButton.bt_tryTarget_stop
		self.app_button.bt_tryTarget_reset = self.welcomeScreen.statusButton.bt_tryTarget_reset
		self.app_button.ck_tryTarget_safety = self.welcomeScreen.statusButton.ck_tryTarget_safety
		# -
		self.app_button.tryTarget_x = self.welcomeScreen.valueLable.lbv_tryTarget_x
		self.app_button.tryTarget_y = self.welcomeScreen.valueLable.lbv_tryTarget_y
		self.app_button.tryTarget_r = self.welcomeScreen.valueLable.lbv_tryTarget_r
		self.app_button.tryTarget_d = self.welcomeScreen.valueLable.lbv_tryTarget_d
		

	def run(self):
		# -- 
		self.valueLable.lbv_ip = self.get_ipAuto(self.name_card)
		self.valueLable.lbv_mac = self.get_MAC(self.name_card)
		self.valueLable.lbv_namePc = self.get_hostname()

		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.controlAll()
			self.controlColor()
			# --
			self.readButton()
			self.pub_button.publish(self.app_button)

			if self.cancelMission_status.data == 1:
				self.welcomeScreen.statusButton.bt_cancelMission = 0
				self.cancelMission_control.data = 0

			if self.welcomeScreen.statusButton.bt_cancelMission == 1:
				self.cancelMission_control.data = 1

			self.pub_cancelMission.publish(self.cancelMission_control)

			# ----------------------
			self.welcomeScreen.valueLable = self.valueLable
			self.welcomeScreen.statusColor = self.statusColor
			# -- 
			self.welcomeScreen.robotPoseNow = self.robotPose_nav.pose
			# -
			self.welcomeScreen.valueLable.arrReflector = self.arrReflector
			self.rate.sleep()

		self.is_exist = 0
		self.kill_app()

		print('Thread #%s stopped' % self.threadID)

	# def check_wifi(self):
	# 	while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
	# 		# -- Ping
	# 		self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
	# 		# -
	# 		self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)
	# 		self.time.sleep(1.5)

class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass
 
def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit

def main():
	# Register the signal handlers
	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)

	print('Starting main program')

	# Start the job threads
	try:
		thread1 = Program(1)
		thread1.start()

		# Keep the main thread running, otherwise signals are ignored.
		thread1.run_screen()
		thread1.is_exist = 0

	except ServiceExit:
		thread1.shutdown_flag.set()
		thread1.join()
		print('Exiting main program')
 
if __name__ == '__main__':
	main()
