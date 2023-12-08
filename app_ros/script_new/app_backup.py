#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Hoang van Quang
Company: STI Viet Nam
date: 09/03/2023
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

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * # QDialog, QApplication, QWidget
from PyQt5.QtGui import * # QPixmap
from PyQt5.QtCore import * # QTimer, QDateTime, Qt

import sqlite3

sys.path.append('/home/stivietnam/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

# print('\n'.join(sys.path))

import roslib
import rospy

from ros_canBus.msg import *
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class statusButton:
	def __init__(self):
		self.bt_passAuto = 0
		self.bt_passHand = 0
		self.bt_cancelMission = 0
		self.bt_setting = 0
		self.bt_clearError = 0

		self.bt_forwards = 0
		self.bt_backwards = 0
		self.bt_rotation_left = 0
		self.bt_rotation_right = 0
		self.bt_stop = 0

		self.bt_coorAverage = 0
		self.bt_disPointA = 0
		self.bt_disPointB = 0

		self.bt_charger = 0
		self.bt_speaker = 0
		self.bt_brake = 0

		self.bt_lift_up = 0
		self.bt_lift_down = 0
		self.bt_lift_reset = 0

		self.bt_hideSetting = 0
		# -
		self.bt_resetFrameWork = 0
		self.vs_speed = 50

class statusColor:
	def __init__(self):
	# --
		self.lbc_safety_up = 0
		self.lbc_safety_ahead = 0
		self.lbc_safety_behind = 0
		# --
		self.cb_status = 0
		self.lbc_battery = 0

		# --
		self.lbc_button_clearError = 0
		self.lbc_button_power = 0
		self.lbc_blsock = 0
		self.lbc_emg = 0

		self.lbc_limit_up = 0
		self.lbc_limit_down = 0
		self.lbc_detect_lifter = 0

		self.lbc_port_rtc = 0
		self.lbc_port_rs485 = 0
		self.lbc_port_nav350 = 0
		# self. = 0
		# self. = 0
		# self. = 0
		# self. = 0
		# self. = 0

class valueLable:
	def __init__(self):
		self.modeRuning = 0

		self.lbv_name_agv = ''
		self.lbv_ip = ''
		self.lbv_battery = ''
		self.lbv_date = ''

		self.lbv_coordinates_x = ''
		self.lbv_coordinates_y = ''
		self.lbv_coordinates_r = ''

		self.lbv_numbeReflector = ''
		self.lbv_pingServer = ''
		self.lbv_jobRuning = ''
		self.lbv_goalFollow_id = ''
		
		self.lbv_route_target = ''
		self.lbv_route_point0 = ''
		self.lbv_route_point1 = ''
		self.lbv_route_point2 = ''
		self.lbv_route_point3 = ''
		self.lbv_route_point4 = ''
		self.lbv_route_job1 = ''
		self.lbv_route_job2 = ''
		self.lbv_route_message = ''

		self.lbv_conveyorA = ''
		self.lbv_conveyorB = ''

		self.lbv_coorAverage_x = ''
		self.lbv_coorAverage_y = ''
		self.lbv_coorAverage_r = ''
		self.lbv_coorAverage_times = ''
		self.lbv_deltaDistance = ''

		self.lbv_velLeft = ''
		self.lbv_velRight = ''

		self.lbv_mac = ''
		self.lbv_namePc = ''

		self.lbv_launhing = ''
		self.lbv_numberLaunch = ''
		self.percentLaunch = 0
		self.listError = ['A', 'B', 'C']
		self.listError_pre = []

		self.lbv_notification_driver1 = ''
		self.lbv_notification_driver2 = ''

		self.lbv_qualityWifi = 0
		# -
		self.list_logError = []
		self.list_logError_pre = []
		# --
		self.lbv_navi_job = ''
		self.lbv_navi_type = ''
		self.lbv_debug1 = ''
		self.lbv_debug2 = ''
		# -
		self.lbv_reflectorDetect = ''
		# -
		self.arrReflector = []
		self.angleCompare = 0
		# self. = ''

class Reflector:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.localID = 0
		self.globalID = 0

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/stivietnam/catkin_ws/src/app_ros/interface/app.ui", self)
		# --
		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.valueLable   = valueLable()
		# --
		# self.statusButton.bt_speaker = 1
		# self.setWindowTitle("my name")
		# --
		# self.fr_run.show()
		# self.fr_agv.show()

		# self.fr_password.hide()
		# self.fr_setting.hide()
		# self.fr_controlHand.hide()
		# -
		self.bt_controlConveyor_show.released.connect(self.released_controlConveyor_show)
		self.bt_controlConveyor_hide.released.connect(self.released_controlConveyor_hide)
		# -- -- -- 
		self.bt_exit.pressed.connect(self.out)
		# -
		self.bt_cancelMission.pressed.connect(self.pressed_cancelMission)
		self.bt_cancelMission.released.connect(self.released_cancelMission)
		# -
		self.bt_passHand.pressed.connect(self.pressed_passHand)
		self.bt_passHand.released.connect(self.released_passHand)
		# -
		self.bt_passAuto.pressed.connect(self.pressed_passAuto)
		self.bt_passAuto.released.connect(self.released_passAuto)
		# -
		self.bt_clearError.pressed.connect(self.pressed_clearError)
		self.bt_clearError.released.connect(self.released_clearError)
		# -- --
		self.bt_speaker_on.clicked.connect(self.clicked_speaker_on)
		self.bt_speaker_off.clicked.connect(self.clicked_speaker_off)
		# -
		self.bt_charger_on.clicked.connect(self.clicked_charger_on)
		self.bt_charger_off.clicked.connect(self.clicked_charger_off)
		# -- 
		self.bt_disableBrake_on.clicked.connect(self.clicked_brakeOn)
		self.bt_disableBrake_off.clicked.connect(self.clicked_brakeOff)

		# --
		self.bt_forwards.clicked.connect(self.clicked_forwards)
		self.bt_backwards.clicked.connect(self.clicked_backwards)
		# -
		self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
		self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
		# -
		self.bt_stop.clicked.connect(self.clicked_stop)
		# -- Setting devices
		self.bt_setting.pressed.connect(self.pressed_setting)
		self.bt_setting.released.connect(self.released_setting)
		self.setting_status = 0
		self.timeSave_setting = rospy.Time.now()
		# -
		self.bt_hideSetting.clicked.connect(self.clicked_hideSetting)
		# -
		self.bt_pw_cancel.clicked.connect(self.clicked_password_cancel)
		self.bt_pw_agree.clicked.connect(self.clicked_password_agree)
		self.password_data = ""
		self.password_right = "0111"
		self.bt_pw_0.clicked.connect(self.clicked_password_n0)
		self.bt_pw_1.clicked.connect(self.clicked_password_n1)
		self.bt_pw_2.clicked.connect(self.clicked_password_n2)
		self.bt_pw_3.clicked.connect(self.clicked_password_n3)
		self.bt_pw_4.clicked.connect(self.clicked_password_n4)
		self.bt_pw_5.clicked.connect(self.clicked_password_n5)
		self.bt_pw_6.clicked.connect(self.clicked_password_n6)
		self.bt_pw_7.clicked.connect(self.clicked_password_n7)
		self.bt_pw_8.clicked.connect(self.clicked_password_n8)
		self.bt_pw_9.clicked.connect(self.clicked_password_n9)
		self.bt_pw_clear.clicked.connect(self.clicked_password_clear)
		self.bt_pw_delete.clicked.connect(self.clicked_password_delete)

		# --
		self.bt_lift_up.pressed.connect(self.pressed_liftUp)
		self.bt_lift_down.pressed.connect(self.pressed_liftDown)
		self.bt_lift_reset.pressed.connect(self.pressed_liftReset)

		# --
		self.bt_coorAverage.pressed.connect(self.pressed_bt_coorAverage)
		self.bt_coorAverage.released.connect(self.released_bt_coorAverage)

		self.bt_disPointA.clicked.connect(self.clicked_pointA)
		self.bt_disPointB.clicked.connect(self.clicked_pointB)
		# -- 

		# -- Set speed manual
		self.bt_upSpeed.pressed.connect(self.pressed_upSpeed)
		self.bt_reduceSpeed.pressed.connect(self.pressed_reduceSpeed)

		# -- Reset FrameWork
		self.bt_resetFrameWork.pressed.connect(self.pressed_resetFrameWork)
		self.bt_resetFrameWork.released.connect(self.released_resetFrameWork)
		# --
		self.bt_reflectorCheck.pressed.connect(self.pressed_reflectorCheck)
		# -
		self.bt_hideNav350.pressed.connect(self.pressed_hideNav350)
		self.bt_refresh_showRelector.pressed.connect(self.pressed_refresh_showRelector)

		# -- -- -- Timer updata data
		# -- Fast
		timer_fast = QTimer(self)
		timer_fast.timeout.connect(self.process_fast)
		timer_fast.start(50)
		# -- normal
		timer_normal = QTimer(self)
		timer_normal.timeout.connect(self.process_normal)
		timer_normal.start(996)
		# -- Slow
		timer_slow = QTimer(self)
		timer_slow.timeout.connect(self.process_slow)
		timer_slow.start(3000)
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRuning = self.modeRun_launch
		# --

		self.password_data = ""
		# --
		self.timeSave_cancelMisson = rospy.Time.now()
		self.cancelMission_status = 0
		# -- 
		self.isShow_setting = 0
		self.isShow_reflectorCheck = 1
		# -
		self.robotPoseNow = Pose()
		self.pointA = Point()
		self.pointB = Point()
		# -
		self.bt_coorAverage_status = 0
		# -
		self.countTime_coorAverage = 0
		self.total_x = 0.0
		self.total_y = 0.0
		self.total_angle = 0.0
		# - 
		self.enable_showToyoWrite = 0
		self.timeSave_showToyoWrite = rospy.Time.now()
		# -
		self.isShow_moveHand = 1
		# - 
		self.flag_updateShowReflector = 0
		# self.show_reflector()

	def process_fast(self):
		self.pb_qualityWifi.setValue(self.valueLable.lbv_qualityWifi)
		self.pb_speed.setValue(self.statusButton.vs_speed)

		# - combo box
		if (self.valueLable.listError != self.valueLable.listError_pre):
			self.valueLable.listError_pre = self.valueLable.listError
			self.cb_status.clear()
			lg = len(self.valueLable.listError) 
			for i in range(lg):
				self.cb_status.addItem(self.valueLable.listError[i])
		# -
		if (self.valueLable.list_logError != self.valueLable.list_logError_pre):
			self.valueLable.list_logError_pre = self.valueLable.list_logError
			self.cb_logError.clear()
			lg = len(self.valueLable.list_logError) 
			for i in range(lg):
				self.cb_logError.addItem(self.valueLable.list_logError[i])
		# - 
		self.statusButton.bt_setting = self.isShow_setting
		# -- 
		self.coorAverage_run()
		# # --
		self.set_labelValue()
		# # --
		self.set_labelColor()
		# # --
		self.controlShow_followMode()
		# # --
		# self.update_setting()

		# -- show check devices
		if (self.setting_status == 1):
			delta_t = rospy.Time.now() - self.timeSave_setting
			# -- Chi kich hoat khi dang o che do Bang Tay.
			if (delta_t.to_sec() > 1.5) and self.valueLable.modeRuning == 1:
				self.isShow_setting = 1
				self.isShow_reflectorCheck = 0
				self.password_data = ""
		else:
			self.timeSave_setting = rospy.Time.now()

		# -- add 21/01/2022 - show cancelMission
		if (self.cancelMission_status == 1):
			delta_c = rospy.Time.now() - self.timeSave_cancelMisson
			if (delta_c.to_sec() > 0.5):
				self.fr_agv.hide()
				self.fr_password.show()
				self.clicked_stop()
		else:
			self.timeSave_cancelMisson = rospy.Time.now()

		# --
		self.show_password()
		# --
		# --
		# if (self.statusButton.bt_stop == 1):
		# 	if (self.data_color.lbc_status_break == 1):
		# 		self.bt_stop.setStyleSheet("background-color: orange;")
		# 	elif (self.data_color.lbc_status_break == 2):
		# 		self.bt_stop.setStyleSheet("background-color: red;")
		# 	else:
		# 		self.bt_stop.setStyleSheet("background-color: blue;")
		# else:
		# 	self.bt_stop.setStyleSheet("background-color: white;")

		if self.flag_updateShowReflector == 1:
			self.flag_updateShowReflector = 0
			length = len(self.valueLable.arrReflector)
			print ("length: ", length)

			# for i in range(length):
			# 	# pass
			# 	label1 = QLabel(self)
			# 	# label1.setText(str(self.valueLable.arrReflector[i].localID))
			# 	label1.setParent(self.fr_showReflector)
			# 	# label1.move(self.valueLable.arrReflector[i].x, self.valueLable.arrReflector[i].y)
			# 	label1.resize(25, 25)
			# 	label1.setStyleSheet("border: 1px solid red; border-radius: 10px")
			# 	label1.setAlignment(Qt.AlignCenter)
			# 	label1.setFont(QFont('Arial', 8))
			# print ("abc")

			self.show_reflector()
	# -
	def show_reflector(self):
		# --
		# self.lb_nav350.setText("NAV")
		# self.lb_nav350.move(400, 250)
		# self.lb_nav350.setStyleSheet("border: 1px solid blue; ")
		# self.lb_nav350.setAlignment(Qt.AlignCenter)
		# self.lb_nav350.setFont(QFont('Arial', 8))
		# self.lb_nav350.setParent(self.fr_showReflector)

		self.valueLable.angleCompare = self.dial_angleCompare.value()
		# --
		self.lb_rf_0.move(0, 0)
		self.lb_rf_1.move(0, 20)
		self.lb_rf_2.move(0, 40)
		self.lb_rf_3.move(0, 60)
		self.lb_rf_4.move(0, 80)
		self.lb_rf_5.move(0, 100)
		self.lb_rf_6.move(0, 120)
		self.lb_rf_7.move(0, 140)
		self.lb_rf_8.move(0, 160)
		self.lb_rf_9.move(0, 180)
		self.lb_rf_10.move(0, 200)
		self.lb_rf_11.move(0, 220)
		# --

		length = len(self.valueLable.arrReflector)
		if length > 0:
			self.lb_rf_0.setText(str(self.valueLable.arrReflector[0].globalID)) # localID
			self.lb_rf_0.move(self.valueLable.arrReflector[0].x, self.valueLable.arrReflector[0].y)
			if self.valueLable.arrReflector[0].globalID == '-1':
				self.lb_rf_0.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_0.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 1:
			self.lb_rf_1.setText(str(self.valueLable.arrReflector[1].globalID))
			self.lb_rf_1.move(self.valueLable.arrReflector[1].x, self.valueLable.arrReflector[1].y)
			if self.valueLable.arrReflector[1].globalID == '-1':
				self.lb_rf_1.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_1.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 2:
			self.lb_rf_2.setText(str(self.valueLable.arrReflector[2].globalID))
			self.lb_rf_2.move(self.valueLable.arrReflector[2].x, self.valueLable.arrReflector[2].y)
			if self.valueLable.arrReflector[2].globalID == '-1':
				self.lb_rf_2.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_2.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 3:
			self.lb_rf_3.setText(str(self.valueLable.arrReflector[3].globalID))
			self.lb_rf_3.move(self.valueLable.arrReflector[3].x, self.valueLable.arrReflector[3].y)
			if self.valueLable.arrReflector[3].globalID == '-1':
				self.lb_rf_3.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_3.setStyleSheet("border: 1px solid green; border-radius: 10px")


		if length > 4:
			self.lb_rf_4.setText(str(self.valueLable.arrReflector[4].globalID))
			self.lb_rf_4.move(self.valueLable.arrReflector[4].x, self.valueLable.arrReflector[4].y)
			if self.valueLable.arrReflector[4].globalID == '-1':
				self.lb_rf_4.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_4.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 5:
			self.lb_rf_5.setText(str(self.valueLable.arrReflector[5].globalID))
			self.lb_rf_5.move(self.valueLable.arrReflector[5].x, self.valueLable.arrReflector[5].y)
			if self.valueLable.arrReflector[5].globalID == '-1':
				self.lb_rf_5.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_5.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 6:
			self.lb_rf_6.setText(str(self.valueLable.arrReflector[6].globalID))
			self.lb_rf_6.move(self.valueLable.arrReflector[6].x, self.valueLable.arrReflector[6].y)
			if self.valueLable.arrReflector[6].globalID == '-1':
				self.lb_rf_6.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_6.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 7:
			self.lb_rf_7.setText(str(self.valueLable.arrReflector[7].globalID))
			self.lb_rf_7.move(self.valueLable.arrReflector[7].x, self.valueLable.arrReflector[7].y)
			if self.valueLable.arrReflector[7].globalID == '-1':
				self.lb_rf_7.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_7.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 8:
			self.lb_rf_8.setText(str(self.valueLable.arrReflector[8].globalID))
			self.lb_rf_8.move(self.valueLable.arrReflector[8].x, self.valueLable.arrReflector[8].y)
			if self.valueLable.arrReflector[8].globalID == '-1':
				self.lb_rf_8.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_8.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 9:
			self.lb_rf_9.setText(str(self.valueLable.arrReflector[9].globalID))
			self.lb_rf_9.move(self.valueLable.arrReflector[9].x, self.valueLable.arrReflector[9].y)
			if self.valueLable.arrReflector[9].globalID == '-1':
				self.lb_rf_9.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_9.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 10:
			self.lb_rf_10.setText(str(self.valueLable.arrReflector[10].globalID))
			self.lb_rf_10.move(self.valueLable.arrReflector[10].x, self.valueLable.arrReflector[10].y)
			if self.valueLable.arrReflector[10].globalID == '-1':
				self.lb_rf_10.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_10.setStyleSheet("border: 1px solid green; border-radius: 10px")

		if length > 11:
			self.lb_rf_11.setText(str(self.valueLable.arrReflector[11].globalID))
			self.lb_rf_11.move(self.valueLable.arrReflector[11].x, self.valueLable.arrReflector[11].y)
			if self.valueLable.arrReflector[11].globalID == '-1':
				self.lb_rf_11.setStyleSheet("border: 1px solid red; border-radius: 10px")
			else:
				self.lb_rf_11.setStyleSheet("border: 1px solid green; border-radius: 10px")

		# for i in range(4):
		# 	label1 = QLabel(self)
		# 	label1.setText(str(i + 8))
		# 	label1.move(100 + 50*i, 150 + 50*i)
		# 	label1.resize(22, 22)
		# 	label1.setStyleSheet("border: 1px solid red; border-radius: 10px")
		# 	label1.setAlignment(Qt.AlignCenter)
		# 	label1.setFont(QFont('Arial', 8))
		# 	lable_NAV.setParent(self.fr_showReflector)

	# -
	def released_controlConveyor_show(self):
		self.isShow_moveHand = 0
		# -
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 1
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")

	def released_controlConveyor_hide(self):
		self.isShow_moveHand = 1

	# -
	def pressed_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: blue;")
		self.cancelMission_status = 1

	def released_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: white;")
		self.cancelMission_status = 0
		self.isShow_setting = 0
	# -
	def pressed_passHand(self):
		self.statusButton.bt_passHand = 1
		self.bt_passHand.setStyleSheet("background-color: blue;")
		
	def released_passHand(self):
		self.statusButton.bt_passHand = 0
		self.bt_passHand.setStyleSheet("background-color: white;")
		self.isShow_setting = 0
	# -
	def pressed_passAuto(self):
		self.statusButton.bt_passAuto = 1
		self.clicked_stop()
		# --
		self.statusButton.bt_brake = 0
		self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_on.setStyleSheet("background-color: white;")

	def released_passAuto(self):
		self.statusButton.bt_passAuto = 0
		self.isShow_setting = 0

	# -
	def pressed_clearError(self):
		self.statusButton.bt_clearError = 1
		self.bt_clearError.setStyleSheet("background-color: blue;")
		self.clicked_stop()

	def released_clearError(self):
		self.statusButton.bt_clearError = 0
		self.bt_clearError.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# -- 
	def clicked_speaker_on(self):
		# self.statusButton.bt_speaker = 1
		# self.statusButton.bt_speaker_on = 1
		# self.statusButton.bt_speaker_off = 0
		self.bt_speaker_on.setStyleSheet("background-color: blue;")
		self.bt_speaker_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_speaker_off(self):
		# self.statusButton.bt_speaker = 0
		# self.statusButton.bt_speaker_off = 1
		# self.statusButton.bt_speaker_on = 0
		self.bt_speaker_off.setStyleSheet("background-color: blue;")
		self.bt_speaker_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# - 
	def clicked_charger_on(self):
		self.statusButton.bt_charger = 1
		self.bt_charger_on.setStyleSheet("background-color: blue;")
		self.bt_charger_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_charger_off(self):
		self.statusButton.bt_charger = 0
		self.bt_charger_off.setStyleSheet("background-color: blue;")
		self.bt_charger_on.setStyleSheet("background-color: white;")
		self.clicked_stop()

	# -- OK --
	def clicked_brakeOn(self):
		self.statusButton.bt_brake = 1
		self.bt_disableBrake_on.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_off.setStyleSheet("background-color: white;")
		self.clicked_stop()

	def clicked_brakeOff(self):
		self.statusButton.bt_brake = 0
		self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
		self.bt_disableBrake_on.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# --
	def clicked_forwards(self):
		self.statusButton.bt_forwards = 1
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_forwards.setStyleSheet("background-color: blue;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_backwards(self):
		self.statusButton.bt_backwards = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_backwards.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_left(self):
		self.statusButton.bt_rotation_left = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_stop = 0
		self.bt_rotation_left.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_right(self):
		self.statusButton.bt_rotation_right = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_backwards = 0
		self.statusButton.bt_stop = 0
		self.bt_rotation_right.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_stop(self):
		self.statusButton.bt_stop = 1
		self.statusButton.bt_forwards = 0
		self.statusButton.bt_rotation_left = 0
		self.statusButton.bt_rotation_right = 0
		self.statusButton.bt_backwards = 0
  
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: blue;")

	# --  --
	def clicked_password_agree(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.statusButton.bt_cancelMission = 1
		self.password_data = ""

	def clicked_password_cancel(self):
		self.fr_agv.show()
		self.fr_password.hide()
		self.password_data = ""

	def process_normal(self):
		self.set_dateTime()
		self.lbv_ip.setText(self.valueLable.lbv_ip)
		self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
		self.lbv_mac.setText(self.valueLable.lbv_mac)
		self.lbv_namePc.setText(self.valueLable.lbv_namePc)

	def process_slow(self):
		self.set_valueBattery(self.valueLable.lbv_battery)
		
	def clicked_hideSetting(self):
		self.isShow_setting = 0
		self.password_data = ""
		self.enable_showToyoWrite = 0

	def pressed_setting(self):
		self.bt_setting.setStyleSheet("background-color: blue;")	
		self.setting_status = 1

	def released_setting(self):
		self.bt_setting.setStyleSheet("background-color: white;")	
		self.setting_status = 0
	# - 
	def clicked_password_n0(self):
		if (len(self.password_data) < 4):
			self.password_data += "0"

	def clicked_password_n1(self):
		if (len(self.password_data) < 4):
			self.password_data += "1"

	def clicked_password_n2(self):
		if (len(self.password_data) < 4):
			self.password_data += "2"

	def clicked_password_n3(self):
		if (len(self.password_data) < 4):
			self.password_data += "3"

	def clicked_password_n4(self):
		if (len(self.password_data) < 4):
			self.password_data += "4"

	def clicked_password_n5(self):
		if (len(self.password_data) < 4):
			self.password_data += "5"

	def clicked_password_n6(self):
		if (len(self.password_data) < 4):
			self.password_data += "6"

	def clicked_password_n7(self):
		if (len(self.password_data) < 4):
			self.password_data += "7"

	def clicked_password_n8(self):
		if (len(self.password_data) < 4):
			self.password_data += "8"

	def clicked_password_n9(self):
		if (len(self.password_data) < 4):
			self.password_data += "9"

	def clicked_password_clear(self):
		self.password_data = ""

	def clicked_password_delete(self):
		lenght = len(self.password_data)
		if (lenght > 0):
			self.password_data = self.password_data[:-1]

	def pressed_bt_coorAverage(self):
		self.bt_coorAverage.setStyleSheet("background-color: blue;")
		self.bt_coorAverage_status = 1

	def released_bt_coorAverage(self):
		self.bt_coorAverage.setStyleSheet("background-color: white;")
		self.bt_coorAverage_status = 0

	def clicked_pointA(self):
		self.pointA.x = self.robotPoseNow.position.x
		self.pointA.y = self.robotPoseNow.position.y
		self.lbv_deltaDistance.setText("---")

	def clicked_pointB(self):
		self.pointB.x = self.robotPoseNow.position.x
		self.pointB.y = self.robotPoseNow.position.y
		delta_distance = self.calculate_distance(self.pointA, self.pointB)
		self.lbv_deltaDistance.setText( str(round(delta_distance, 3)) )

	def pressed_resetFrameWork(self):
		self.bt_resetFrameWork.setStyleSheet("background-color: blue;")
		self.statusButton.bt_resetFrameWork = 1

	def released_resetFrameWork(self):
		self.bt_resetFrameWork.setStyleSheet("background-color: white;")
		self.statusButton.bt_resetFrameWork = 0

	def pressed_upSpeed(self):
		self.statusButton.vs_speed += 10
		if self.statusButton.vs_speed >= 100:
			self.statusButton.vs_speed = 100

	def pressed_reduceSpeed(self):
		self.statusButton.vs_speed -= 10
		if self.statusButton.vs_speed < 10:
			self.statusButton.vs_speed = 10

	def pressed_liftUp(self):
		self.bt_lift_up.setStyleSheet("background-color: blue;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
		self.statusButton.bt_lift_up = 1
		self.statusButton.bt_lift_down = 0

	def pressed_liftDown(self):
		self.bt_lift_up.setStyleSheet("background-color: white;")
		self.bt_lift_down.setStyleSheet("background-color: blue;")
		self.statusButton.bt_lift_up = 0
		self.statusButton.bt_lift_down = 1

	def pressed_liftReset(self):
		self.bt_lift_up.setStyleSheet("background-color: white;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
		self.statusButton.bt_lift_up = 0
		self.statusButton.bt_lift_down = 0

	def pressed_reflectorCheck(self):
		self.isShow_reflectorCheck = 1
		self.isShow_setting = 0

	def pressed_hideNav350(self):
		self.isShow_reflectorCheck = 0
		self.isShow_setting = 1

	def pressed_refresh_showRelector(self):
		self.flag_updateShowReflector = 1

	def out(self):
		QApplication.quit()
		print('out')

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	# ---------------------------
	def set_dateTime(self):
		time_now = datetime.now()
		# dd/mm/YY H:M:S
		dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
		self.lbv_date.setText(dt_string)

	def set_valueBattery(self, str_value): # str_value: string()
		self.lbv_battery.setText(str_value)
	
	def set_labelColor(self):
		# ---- Safety
		if (self.statusColor.lbc_safety_up == 0):
			self.lbc_safety0.setStyleSheet("background-color: green; color: white;")
			self.lbc_safety_up.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_up == 1):
			self.lbc_safety0.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_up.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety0.setStyleSheet("background-color: yellow;")
			self.lbc_safety_up.setStyleSheet("background-color: yellow;")
		# -
		if (self.statusColor.lbc_safety_ahead == 0):
			self.lbc_safety1.setStyleSheet("background-color: green; color: white")
			self.lbc_safety_ahead.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_ahead == 1):
			self.lbc_safety1.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety1.setStyleSheet("background-color: yellow;")
			self.lbc_safety_ahead.setStyleSheet("background-color: yellow;")
		# -
		if (self.statusColor.lbc_safety_behind == 0):
			self.lbc_safety2.setStyleSheet("background-color: green; color: white")
			self.lbc_safety_behind.setStyleSheet("background-color: green; color: white")

		elif (self.statusColor.lbc_safety_behind == 1):
			self.lbc_safety2.setStyleSheet("background-color: red; color: white")
			self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
		else:
			self.lbc_safety2.setStyleSheet("background-color: yellow;")
			self.lbc_safety_behind.setStyleSheet("background-color: yellow;")

		# ---- Battery
		if (self.statusColor.lbc_battery == 0):
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 1):
			self.lbv_battery.setStyleSheet("background-color: green; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 2):
			self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (self.statusColor.lbc_battery == 3):
			self.lbv_battery.setStyleSheet("background-color: red; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (self.statusColor.lbc_battery == 4):
			self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		else: # -- charging
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")

		# ---- Thanh trang thai AGV
		if (self.statusColor.cb_status == 0):
			self.cb_status.setStyleSheet("background-color: green; color: white;")
		elif (self.statusColor.cb_status == 1):
			self.cb_status.setStyleSheet("background-color: orange; color: black;")	
		elif (self.statusColor.cb_status == 2):
			self.cb_status.setStyleSheet("background-color: red; color: white;")
		else:
			self.cb_status.setStyleSheet("background-color: white;")

		# ---- Trang thai che do dang hoat dong
		if (self.valueLable.modeRuning == self.modeRun_byhand):
			self.bt_passHand.setStyleSheet("background-color: blue;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")	

		elif (self.valueLable.modeRuning == self.modeRun_auto):
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: blue;")
		else:	
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")

		# -- Button Clear error
		if (self.statusColor.lbc_button_clearError == 1):
			self.lbc_button_clearError.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_button_clearError == 0):
			self.lbc_button_clearError.setStyleSheet("background-color: white;")

		# -- Button Power
		if (self.statusColor.lbc_button_power == 1):
			self.lbc_button_power.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_button_power == 0):
			self.lbc_button_power.setStyleSheet("background-color: white;")

		# -- Blsock
		if (self.statusColor.lbc_blsock == 1):
			self.lbc_blsock.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_blsock == 0):
			self.lbc_blsock.setStyleSheet("background-color: white;")

		# -- EMG
		if (self.statusColor.lbc_emg == 1):
			self.lbc_emg.setStyleSheet("background-color: blue;")
		elif (self.statusColor.lbc_emg == 0):
			self.lbc_emg.setStyleSheet("background-color: white;")	

		# -- Port: RTC Board
		if (self.statusColor.lbc_port_rtc == 1):
			self.lbc_port_rtc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtc == 0):
			self.lbc_port_rtc.setStyleSheet("background-color: red; color: white;")

		# -- Port: RS485
		if (self.statusColor.lbc_port_rs485 == 1):
			self.lbc_port_rs485.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rs485 == 0):
			self.lbc_port_rs485.setStyleSheet("background-color: red; color: white;")

		# -- Port: NAV350
		if (self.statusColor.lbc_port_nav350 == 1):
			self.lbc_port_nav350.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_nav350 == 0):
			self.lbc_port_nav350.setStyleSheet("background-color: red; color: white;")

		# -- Sensor Up
		if (self.statusColor.lbc_limit_up == 1):
			self.lbc_limit_up.setStyleSheet("background-color: blue; color: white;")
		elif (self.statusColor.lbc_limit_up == 0):
			self.lbc_limit_up.setStyleSheet("background-color: white; color: black;")

		# -- Sensor Down
		if (self.statusColor.lbc_limit_down == 1):
			self.lbc_limit_down.setStyleSheet("background-color: blue; color: white;")
		elif (self.statusColor.lbc_limit_down == 0):
			self.lbc_limit_down.setStyleSheet("background-color: white; color: black;")

		# -- Sensor detect lift
		if (self.statusColor.lbc_detect_lifter == 1):
			self.lbc_detect_lifter.setStyleSheet("background-color: blue; color: white;")
		elif (self.statusColor.lbc_detect_lifter == 0):
			self.lbc_detect_lifter.setStyleSheet("background-color: white; color: black;")

	def set_labelValue(self): # App_lbv()
		self.lbv_angleCompare.setText(str(self.dial_angleCompare.value()))
		# --
		self.lbv_battery.setText(self.valueLable.lbv_battery)

		self.lbv_coordinates_x.setText(self.valueLable.lbv_coordinates_x)
		self.lbv_coordinates_y.setText(self.valueLable.lbv_coordinates_y)
		self.lbv_coordinates_r.setText(self.valueLable.lbv_coordinates_r)

		self.lbv_coordinates_x1.setText(self.valueLable.lbv_coordinates_x)
		self.lbv_coordinates_y1.setText(self.valueLable.lbv_coordinates_y)
		self.lbv_coordinates_r1.setText(self.valueLable.lbv_coordinates_r)

		self.lbv_numberReflector.setText(self.valueLable.lbv_numbeReflector)

		self.lbv_pingServer.setText(self.valueLable.lbv_pingServer)

		self.lbv_route_target.setText(self.valueLable.lbv_route_target)
		self.lbv_jobRuning.setText(self.valueLable.lbv_jobRuning)
		self.lbv_goalFollow_id.setText(self.valueLable.lbv_goalFollow_id)
		self.lbv_route_point0.setText(self.valueLable.lbv_route_point0)
		self.lbv_route_point1.setText(self.valueLable.lbv_route_point1)
		self.lbv_route_point2.setText(self.valueLable.lbv_route_point2)
		self.lbv_route_point3.setText(self.valueLable.lbv_route_point3)
		self.lbv_route_point4.setText(self.valueLable.lbv_route_point4)
		self.lbv_route_job1.setText(self.valueLable.lbv_route_job1)
		self.lbv_route_job2.setText(self.valueLable.lbv_route_job2)
		
		self.lbv_route_message.setText(self.valueLable.lbv_route_message)

		# self.lbv_coorAverage_x.setText(self.valueLable.lbv_coorAverage_x)
		# self.lbv_coorAverage_y.setText(self.valueLable.lbv_coorAverage_y)
		# self.lbv_coorAverage_r.setText(self.valueLable.lbv_coorAverage_r)
		# self.lbv_coorAverage_times.setText(self.valueLable.lbv_coorAverage_times)

		self.lbv_velLeft.setText(self.valueLable.lbv_velLeft)
		self.lbv_velRight.setText(self.valueLable.lbv_velRight)

		self.lbv_notification_driver1.setText(self.valueLable.lbv_notification_driver1)
		self.lbv_notification_driver2.setText(self.valueLable.lbv_notification_driver2)

		# - 
		self.lbv_reflectorLoc.setText(self.valueLable.lbv_numbeReflector)
		self.lbv_reflectorDetect.setText(self.valueLable.lbv_reflectorDetect)
		# self..setText(self.valueLable.)
		
	def controlShow_followMode(self):
		if (self.valueLable.modeRuning == self.modeRun_launch):
			self.modeRuning = self.modeRun_launch

		elif (self.valueLable.modeRuning == self.modeRun_byhand):
			self.modeRuning = self.modeRun_byhand

		elif (self.valueLable.modeRuning == self.modeRun_auto):
			self.modeRuning = self.modeRun_auto
		else:
			self.modeRuning = self.modeRun_auto
		# --
		if (self.modeRuning == self.modeRun_launch): # -- Khoi Dong
			self.fr_launch.show()
			self.fr_run.hide()
			self.show_launch()
		else:
			self.fr_run.show()
			self.fr_launch.hide()

			if (self.modeRuning == self.modeRun_auto): # -- Tu dong
				self.fr_control.show()
				self.fr_setting.hide()
				self.fr_handMode_conveyor.hide()
				self.fr_handMode_move.hide()
				self.fr_listTask.show()
				self.isShow_moveHand = 1

			elif (self.modeRuning == self.modeRun_byhand):
				if self.isShow_setting == 1:
					self.fr_control.hide()
					self.fr_setting.show()
					self.fr_listTask.hide()
					self.fr_nav350.hide()

				elif self.isShow_reflectorCheck == 1:
					self.fr_control.hide()
					self.fr_setting.hide()
					self.fr_listTask.hide()
					self.fr_nav350.show()

				else:
					self.fr_control.show()
					self.fr_setting.hide()
					self.fr_listTask.hide()
					self.fr_nav350.hide()
					if (self.isShow_moveHand == 1):
						self.fr_handMode_conveyor.hide()
						self.fr_handMode_move.show()
					else:
						self.fr_handMode_conveyor.show()
						self.fr_handMode_move.hide()

	def coorAverage_run(self):
		if (self.bt_coorAverage_status == 1):
			self.countTime_coorAverage += 1.
			self.total_x += self.robotPoseNow.position.x
			self.total_y += self.robotPoseNow.position.y
			euler = self.quaternion_to_euler(self.robotPoseNow.orientation)
			self.total_angle += degrees(euler)

		else:
			self.timeSave_coorAverage = rospy.Time.now()
			if (self.countTime_coorAverage > 4):
				d_x = self.total_x/self.countTime_coorAverage
				d_y = self.total_y/self.countTime_coorAverage
				d_a = self.total_angle/self.countTime_coorAverage
				d_degree = 0
				if d_a < 0:
					d_degree = 360 + d_a
				else:
					d_degree = d_a

				self.lbv_coorAverage_times.setText(str(self.countTime_coorAverage))
				self.lbv_coorAverage_x.setText(str(round(d_x, 3)))
				self.lbv_coorAverage_y.setText(str(round(d_y, 3)))
				self.lbv_coorAverage_r.setText(str(round(d_degree, 2)))

			self.countTime_coorAverage = 0
			self.total_x = 0.0
			self.total_y = 0.0
			self.total_angle = 0.0

	def show_launch(self):
		self.lbv_launhing.setText(self.valueLable.lbv_launhing)
		self.lbv_numberLaunch.setText(str(self.valueLable.lbv_numberLaunch))
		# --
		value = self.valueLable.percentLaunch
		if (value < 0):
			value = 0

		if (value > 100):
			value = 100

		self.pb_launch.setValue(value)
		# --
		# -- Port: RTC Board
		if (self.statusColor.lbc_port_rtc == 1):
			self.lbc_lh_rtc.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rtc == 0):
			self.lbc_lh_rtc.setStyleSheet("background-color: red; color: white;")

		# -- Port: RS485
		if (self.statusColor.lbc_port_rs485 == 1):
			self.lbc_lh_driver.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_rs485 == 0):
			self.lbc_lh_driver.setStyleSheet("background-color: red; color: white;")

		# -- Port: NAV350
		if (self.statusColor.lbc_port_nav350 == 1):
			self.lbc_lh_lidar.setStyleSheet("background-color: blue; color: white")
		elif (self.statusColor.lbc_port_nav350 == 0):
			self.lbc_lh_lidar.setStyleSheet("background-color: red; color: white;")

	def show_password(self):
		data = ""
		lenght = len(self.password_data)

		for i in range(lenght):
			data += "*"
		
		self.lbv_pw_data.setText(data)

		if (self.password_data == self.password_right):
			self.bt_pw_agree.setEnabled(True)
		else:
			self.bt_pw_agree.setEnabled(False)

class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		# --
		self.name_card = rospy.get_param("name_card", "wlo2")
		self.name_card = "wlo2"

		self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		#self.address_traffic = "192.168.1.92"
		self.pre_timePing = time.time()
		# --
		rospy.init_node('app_ros', anonymous=False)
		self.rate = rospy.Rate(40)

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

		# -- Pose robot
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_NN_infoRequest) 
		self.NN_infoRequest = NN_infoRequest()

		# -- info AGV
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()

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
		# --
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
		print ("----------")
		for i in range(length):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang0 = radians(self.nav350_reflectors.reflectors[i].Polar_Phi/1000. + self.welcomeScreen.valueLable.angleCompare)
			ang = self.limitAngle(ang0)

			p_x, p_y = self.convert_position(dis, ang)
			rp_x = int(p_x/rate_show)
			rp_y = int(p_y/rate_show)
			sh_x = rp_y + 400
			sh_y = rp_x + 200
			
			print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )
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
		job_now = ''
		switcher={
			0:'...', # 
			1:'Kiểm Tra Trạng Thái', # kiem tra trang thai ban nang sau khi Sang che do tu dong
			2:'Di Chuyển Ra Khởi Vị Trí', # 
			3:'Di Chuyển Giữa Các Điểm', #
			4:'Di Chuyển Vào Vị Trí Thao Tác', # 
			5:'Kiểm Tra Vị Trí Trả Hàng', # 
			6:'Thực Hiện Nhiệm Vụ Sau', # 
			7:'Đợi Lệnh Mới', # 
			8:'Đợi Hoàn Thành Lệnh Cũ', # 
			20:'Chế Độ Bằng Tay' # 
		}
		return switcher.get(val, job_now)

	def show_misson(self, val):
		job_now = ''
		switcher={
			0:'...', # 
			1:'Kiem Tra Tu Dong', # kiem tra trang thai ban nang sau khi Sang che do tu dong
			2:'Nhiem Vu Truoc', # thuc hien nhiem vu truoc
			3:'Kiem Tra Ke', # kiem tra ke 
			4:'Di Ra', # 
			5:'Di Chuyen', #
			6:'Di Vao Ke', # 
			7:'Nhiem Vu Sau', # 
			10:'Sac\nPin', # 
			20:'Bang Tay' # 
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
		deltaTime_ping = (time.time() - self.pre_timePing)%60
		if (deltaTime_ping > 2.0):
			self.pre_timePing = time.time()
			self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
			# -
			self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)

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
		# self.valueLable.lbv_route_target = str(self.server_cmdRequest.target_id) + "\n" + str(self.server_cmdRequest.target_x) + "\n" + str(self.server_cmdRequest.target_y) + "\n" + str(round(degrees(self.server_cmdRequest.target_z), 2)) + "\n" + str(self.server_cmdRequest.offset)
		# # -
		# if len(self.server_cmdRequest.list_id) >= 5:
		# 	self.valueLable.lbv_route_point0 = str(self.server_cmdRequest.list_id[0]) + "\n" + str(self.server_cmdRequest.list_x[0]) + "\n" + str(self.server_cmdRequest.list_y[0]) + "\n" + str(self.server_cmdRequest.list_speed[0]) + "\n" + str(self.server_cmdRequest.list_directionTravel[0]) + "\n" + str(self.server_cmdRequest.list_angleLine[0])
		# 	self.valueLable.lbv_route_point1 = str(self.server_cmdRequest.list_id[1]) + "\n" + str(self.server_cmdRequest.list_x[1]) + "\n" + str(self.server_cmdRequest.list_y[1]) + "\n" + str(self.server_cmdRequest.list_speed[1]) + "\n" + str(self.server_cmdRequest.list_directionTravel[1]) + "\n" + str(self.server_cmdRequest.list_angleLine[1])
		# 	self.valueLable.lbv_route_point2 = str(self.server_cmdRequest.list_id[2]) + "\n" + str(self.server_cmdRequest.list_x[2]) + "\n" + str(self.server_cmdRequest.list_y[2]) + "\n" + str(self.server_cmdRequest.list_speed[2]) + "\n" + str(self.server_cmdRequest.list_directionTravel[2]) + "\n" + str(self.server_cmdRequest.list_angleLine[2])
		# 	self.valueLable.lbv_route_point3 = str(self.server_cmdRequest.list_id[3]) + "\n" + str(self.server_cmdRequest.list_x[3]) + "\n" + str(self.server_cmdRequest.list_y[3]) + "\n" + str(self.server_cmdRequest.list_speed[3]) + "\n" + str(self.server_cmdRequest.list_directionTravel[3]) + "\n" + str(self.server_cmdRequest.list_angleLine[3])
		# 	self.valueLable.lbv_route_point4 = str(self.server_cmdRequest.list_id[4]) + "\n" + str(self.server_cmdRequest.list_x[4]) + "\n" + str(self.server_cmdRequest.list_y[4]) + "\n" + str(self.server_cmdRequest.list_speed[4]) + "\n" + str(self.server_cmdRequest.list_directionTravel[4]) + "\n" + str(self.server_cmdRequest.list_angleLine[4])
		
		# self.valueLable.lbv_route_job1 = str(self.server_cmdRequest.before_mission)
		# self.valueLable.lbv_route_job2 = str(self.server_cmdRequest.after_mission)
		# self.valueLable.lbv_route_message = self.server_cmdRequest.command
		# self.valueLable.lbv_jobRuning = self.show_job(self.NN_infoRespond.process)
		# -- 
		# self.valueLable.lbv_goalFollow_id = str(self.navigation_respond.id_goalFollow)
				# lbv_coorAverage_x
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
		# self.app_button.bt_charger	= self.welcomeScreen.statusButton.bt_charger
		# self.app_button.bt_speaker  = self.welcomeScreen.statusButton.bt_speaker
		# self.app_button.bt_brake	= self.welcomeScreen.statusButton.bt_brake
		# -- 
		# self.app_button.bt_lift_up	= self.welcomeScreen.statusButton.bt_lift_up
		# self.app_button.bt_lift_down	= self.welcomeScreen.statusButton.bt_lift_down
		# -
		# self.app_button.vs_speed = self.welcomeScreen.statusButton.vs_speed
		# self.app_button.bt_resetFrameWork = self.welcomeScreen.statusButton.bt_resetFrameWork

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
			# print ("arrReflector: ", len(self.arrReflector))
			self.rate.sleep()

		self.is_exist = 0
		self.kill_app()

		print('Thread #%s stopped' % self.threadID)

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
