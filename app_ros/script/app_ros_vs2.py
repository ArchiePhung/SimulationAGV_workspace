#!/usr/bin/env python3

"""
Developer: Hoang van Quang
Company: STI Viet Nam
date: 08/12/2021
>> Edit: 19/04/2022
	Them chuc nang lay mau toa do Pose.
"""

import sys
import os
from math import sin , cos , pi , atan2
import time
import threading
import signal

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime, Qt

import sqlite3

sys.path.append('/home/stivietnam/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy
from message_pkg.msg import *
from std_msgs.msg import Int16

from datetime import datetime

# -- add 19/04/2022
from geometry_msgs.msg import Point
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/stivietnam/catkin_ws/src/app_ros/interface/app_vs2.ui", self)
		# --
		# self.setWindowTitle("my name")
		# --
		self.gb_checkDevice.hide()
		# self.gb_listTask.hide()
		self.gb_errorPose.hide()
		# -- 
		self.status_button = App_button()
		self.data_color = App_color()
		self.data_labelValue = App_lbv()
		self.data_launch = App_launch()
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
		self.bt_spk_on.clicked.connect(self.clicked_spk_on)
		self.bt_spk_off.clicked.connect(self.clicked_spk_off)
		# -
		self.bt_chg_on.clicked.connect(self.clicked_chg_on)
		self.bt_chg_off.clicked.connect(self.clicked_chg_off)
		# -
		self.bt_lift_up.clicked.connect(self.clicked_lift_up)
		self.bt_lift_down.clicked.connect(self.clicked_lift_down)
		# -- -- 
		self.bt_forwards.clicked.connect(self.clicked_forwards)
		self.bt_backwards.clicked.connect(self.clicked_backwards)
		# -
		self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
		self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
		# -
		self.bt_stop.clicked.connect(self.clicked_stop)
		# -- check devices
		self.bt_checkDevices.pressed.connect(self.pressed_checkDevices)
		self.bt_checkDevices.released.connect(self.released_checkDevices)
		self.checkDevices_status = 0
		self.timeSave_checkDevices = rospy.Time.now()
		# -
		self.bt_hideCheck.clicked.connect(self.clicked_hideCheckDevices)
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

		# -- add 19/04/2022
		self.bt_point.pressed.connect(self.pressed_bt_point)
		self.bt_point.released.connect(self.released_bt_point)
		self.bt_disPoint1.clicked.connect(self.clicked_point1)
		self.bt_disPoint2.clicked.connect(self.clicked_point2)

		self.timeSave_clickPoint = rospy.Time.now()
		self.deltaTime_clickPoint = 0

		self.bt_point_status = 0
		self.count_time = 0
		self.total_x = 0.0
		self.total_y = 0.0
		self.total_angle = 0.0
		# -
		self.point_1 = Point()
		self.point_2 = Point()

		# -- Set property
		self.lbv_device.setStyleSheet("color: red;")
		self.lbv_frameWork.setStyleSheet("color: orange;")
		# --
		# -- -- -- Timer updata data
		# -- Fast
		timer_fast = QTimer(self)
		timer_fast.timeout.connect(self.process_fast)
		timer_fast.start(200)
		# -- normal
		timer_normal = QTimer(self)
		timer_normal.timeout.connect(self.process_normal)
		timer_normal.start(996)
		# -- Slow
		timer_slow = QTimer(self)
		timer_slow.timeout.connect(self.process_slow)
		timer_slow.start(3000)
		# -- 
		self.enb_showPara = 0
		self.enb_check = 0
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRuning = self.modeRun_launch
		# --
		self.gb_agv.show()
		self.gb_password.hide()
		self.gb_checkDevice.hide()
		self.password_data = ""

		# -- add 21/01/2022
		self.timeSave_cancelMisson = rospy.Time.now()
		self.cancelMission_status = 0

	def process_fast(self):
		# -- add 19/04/2022
		self.click_point()
		# --
		self.set_labelValue(self.data_labelValue)
		# --
		self.set_labelColor(self.data_color)
		# --
		self.controlShow_followMode()
		# --
		self.update_checkDevices(self.data_color)
		# -- show check devices
		if (self.checkDevices_status == 1):
			delta_t = rospy.Time.now() - self.timeSave_checkDevices
			if (delta_t.to_sec() > 1.5):
				self.gb_agv.hide()
				self.gb_password.hide()
				self.gb_checkDevice.show()
				self.password_data = ""
		else:
			self.timeSave_checkDevices = rospy.Time.now()
		# --
		self.show_password()
		# --

		# -- add 21/01/2022 - show cancelMission
		if (self.cancelMission_status == 1):
			delta_c = rospy.Time.now() - self.timeSave_cancelMisson
			if (delta_c.to_sec() > 0.5):
				self.gb_agv.hide()
				self.gb_checkDevice.hide()
				self.gb_password.show()
				self.clicked_stop()
		else:
			self.timeSave_cancelMisson = rospy.Time.now()

	def clicked_password_agree(self):
		self.gb_agv.show()
		self.gb_password.hide()
		self.gb_checkDevice.hide()
		self.status_button.bt_cancelMission = 1
		self.password_data = ""

	def clicked_password_cancel(self):
		self.gb_agv.show()
		self.gb_password.hide()
		self.gb_checkDevice.hide()
		self.password_data = ""

	def process_normal(self):
		self.set_dateTime()
		self.set_ip(" " + self.data_labelValue.lbv_ip)
		self.set_nameAgv(" " + self.data_labelValue.lbv_name_agv)

	def process_slow(self):
		self.set_valueBattery(self.data_labelValue.lbv_battery)
		
	def clicked_hideCheckDevices(self):
		self.gb_agv.show()
		self.gb_password.hide()
		self.gb_checkDevice.hide()
		self.password_data = ""

	def pressed_checkDevices(self):
		self.checkDevices_status = 1

	def released_checkDevices(self):
		self.checkDevices_status = 0

	def pressed_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: blue;")
		self.cancelMission_status = 1
		self.clicked_stop()
		# self.gb_agv.hide()
		# self.gb_checkDevice.hide()
		# self.gb_password.show()

	def released_cancelMission(self):
		self.bt_cancelMission.setStyleSheet("background-color: white;")
		self.cancelMission_status = 0
	# -
	def pressed_passHand(self):
		self.status_button.bt_passHand = 1
		# self.bt_passHand.setStyleSheet("background-color: blue;")
		
	def released_passHand(self):
		self.status_button.bt_passHand = 0
		# self.bt_passHand.setStyleSheet("background-color: white;")
	# -
	def pressed_passAuto(self):
		self.status_button.bt_passAuto = 1
		# self.bt_passAuto.setStyleSheet("background-color: blue;")
		self.clicked_stop()
		
	def released_passAuto(self):
		self.status_button.bt_passAuto = 0
		# self.bt_passAuto.setStyleSheet("background-color: white;")
		self.clear_button_lift()
	# -
	def pressed_clearError(self):
		self.status_button.bt_clearError = 1
		self.bt_clearError.setStyleSheet("background-color: blue;")
		
	def released_clearError(self):
		self.status_button.bt_clearError = 0
		self.bt_clearError.setStyleSheet("background-color: white;")
	# -- 
	def clicked_spk_on(self):
		self.status_button.bt_spk_on = 1
		self.status_button.bt_spk_off = 0
		self.bt_spk_on.setStyleSheet("background-color: blue;")
		self.bt_spk_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_spk_off(self):
		self.status_button.bt_spk_off = 1
		self.status_button.bt_spk_on = 0
		self.bt_spk_off.setStyleSheet("background-color: blue;")
		self.bt_spk_on.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_chg_on(self):
		self.status_button.bt_chg_on = 1
		self.status_button.bt_chg_off = 0
		self.bt_chg_on.setStyleSheet("background-color: blue;")
		self.bt_chg_off.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_chg_off(self):
		self.status_button.bt_chg_off = 1
		self.status_button.bt_chg_on = 0
		self.bt_chg_off.setStyleSheet("background-color: blue;")
		self.bt_chg_on.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_lift_up(self):
		self.status_button.bt_lift_up = 1
		self.status_button.bt_lift_down = 0
		self.bt_lift_up.setStyleSheet("background-color: blue;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# - 
	def clicked_lift_down(self):
		self.status_button.bt_lift_down = 1
		self.status_button.bt_lift_up = 0
		self.bt_lift_down.setStyleSheet("background-color: blue;")
		self.bt_lift_up.setStyleSheet("background-color: white;")
		self.clicked_stop()
	# -
	def clear_button_lift(self):
		self.status_button.bt_lift_up = 0
		self.status_button.bt_lift_down = 0
		self.bt_lift_up.setStyleSheet("background-color: white;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
	# --
	def clicked_forwards(self):
		self.status_button.bt_forwards = 1
		self.status_button.bt_backwards = 0
		self.status_button.bt_rotation_left = 0
		self.status_button.bt_rotation_right = 0
		self.status_button.bt_stop = 0
		self.bt_forwards.setStyleSheet("background-color: blue;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_backwards(self):
		self.status_button.bt_backwards = 1
		self.status_button.bt_forwards = 0
		self.status_button.bt_rotation_left = 0
		self.status_button.bt_rotation_right = 0
		self.status_button.bt_stop = 0
		self.bt_backwards.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_left(self):
		self.status_button.bt_rotation_left = 1
		self.status_button.bt_forwards = 0
		self.status_button.bt_backwards = 0
		self.status_button.bt_rotation_right = 0
		self.status_button.bt_stop = 0
		self.bt_rotation_left.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_rotation_right(self):
		self.status_button.bt_rotation_right = 1
		self.status_button.bt_forwards = 0
		self.status_button.bt_rotation_left = 0
		self.status_button.bt_backwards = 0
		self.status_button.bt_stop = 0
		self.bt_rotation_right.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")
		self.bt_stop.setStyleSheet("background-color: white;")
	# - 
	def clicked_stop(self):
		self.status_button.bt_stop = 1
		self.status_button.bt_forwards = 0
		self.status_button.bt_rotation_left = 0
		self.status_button.bt_rotation_right = 0
		self.status_button.bt_backwards = 0
		self.bt_stop.setStyleSheet("background-color: blue;")
		self.bt_forwards.setStyleSheet("background-color: white;")
		self.bt_backwards.setStyleSheet("background-color: white;")
		self.bt_rotation_right.setStyleSheet("background-color: white;")
		self.bt_rotation_left.setStyleSheet("background-color: white;")

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

	# -- add 19/04/2022
	def pressed_bt_point(self):
		self.bt_point_status = 1

	def released_bt_point(self):
		self.bt_point_status = 0

	def clicked_point1(self):
		self.point_1.x = float(self.data_labelValue.lbv_x)
		self.point_1.y = float(self.data_labelValue.lbv_y)
		self.lbv_deltaDistance.setText("---")

	def clicked_point2(self):
		self.point_2.x = float(self.data_labelValue.lbv_x)
		self.point_2.y = float(self.data_labelValue.lbv_y)
		delta_distance = self.calculate_distance(self.point_1, self.point_2)
		self.lbv_deltaDistance.setText( str(round(delta_distance, 3)) )
	# -- 
	def click_point(self):
		if (self.bt_point_status == 1):
			delta_t = rospy.Time.now() - self.timeSave_clickPoint
			self.deltaTime_clickPoint = round(delta_t.to_sec(), 2)

			self.count_time += 1.
			self.total_x += float(self.data_labelValue.lbv_x)
			self.total_y += float(self.data_labelValue.lbv_y)
			self.total_angle += float(self.data_labelValue.lbv_zd)

		else:
			if (self.count_time > 4):
				d_x = self.total_x/self.count_time
				d_y = self.total_y/self.count_time
				d_a = self.total_angle/self.count_time

				show_text = str(round(d_x, 3)) + "|" + str(round(d_y, 3)) + "|" + str(round(d_a, 3))
				self.lbv_time.setText(str(self.count_time))
				self.lbv_point.setText(show_text)

			self.timeSave_clickPoint = rospy.Time.now()
			self.count_time = 0
			self.total_x = 0.0
			self.total_y = 0.0
			self.total_angle = 0.0
	# --
	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)
	# --- --- ---

	def out(self):
		QApplication.quit()
		print('out')
	# ---------------------------
	def set_dateTime(self):
		time_now = datetime.now()
		# dd/mm/YY H:M:S
		dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
		self.lbv_date.setText(dt_string)

	def set_valueBattery(self, str_value): # str_value: string()
		self.lbv_battery.setText(str_value)

	def set_nameAgv(self, str_agv): # str_value: string()
		self.lbv_name_agv.setText(str_agv)

	def set_ip(self, str_ip): # str_value: string()
		self.lbv_ip.setText(str_ip)
	
	def set_labelColor(self, data_color):
		# ---- Safety
		if (data_color.lbc_safety0 == 0):
			self.lbc_safety0.setStyleSheet("background-color: green;")
		elif (data_color.lbc_safety0 == 1):
			self.lbc_safety0.setStyleSheet("background-color: red;")
		else:
			self.lbc_safety0.setStyleSheet("background-color: yellow;")
		# -
		if (data_color.lbc_safety1 == 0):
			self.lbc_safety1.setStyleSheet("background-color: green;")
		elif (data_color.lbc_safety1 == 1):
			self.lbc_safety1.setStyleSheet("background-color: red;")
		else:
			self.lbc_safety1.setStyleSheet("background-color: yellow;")
		# -
		if (data_color.lbc_safety2 == 0):
			self.lbc_safety2.setStyleSheet("background-color: green;")
		elif (data_color.lbc_safety2 == 1):
			self.lbc_safety2.setStyleSheet("background-color: red;")
		else:
			self.lbc_safety2.setStyleSheet("background-color: yellow;")

		# ---- Battery
		if (data_color.lbv_battery == 0):
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (data_color.lbv_battery == 1):
			self.lbv_battery.setStyleSheet("background-color: green; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (data_color.lbv_battery == 2):
			self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		elif (data_color.lbv_battery == 3):
			self.lbv_battery.setStyleSheet("background-color: red; color: white;")
			self.lb_v.setStyleSheet("color: white;")
		elif (data_color.lbv_battery == 4):
			self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
			self.lb_v.setStyleSheet("color: black;")
		else: # -- charging
			self.lbv_battery.setStyleSheet("background-color: white; color: black;")
			self.lb_v.setStyleSheet("color: black;")

		# ---- Nut chuyen tu dong
		if (data_color.bt_passAuto == 0):
			self.bt_passAuto.setStyleSheet("background-color: white;")
		elif (data_color.bt_passAuto == 1):
			self.bt_passAuto.setStyleSheet("background-color: green;")
			
		# ---- Thanh trang thai AGV
		if (data_color.lbc_status == 0):
			self.lbc_status.setStyleSheet("background-color: green;")
		elif (data_color.lbc_status == 1):
			self.lbc_status.setStyleSheet("background-color: orange;")	
		elif (data_color.lbc_status == 2):
			self.lbc_status.setStyleSheet("background-color: red;")

		# ---- trang thai che do dang hoat dong
		if (self.data_labelValue.modeRuning == self.modeRun_byhand):
			self.bt_passHand.setStyleSheet("background-color: blue;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")	
			
		elif (self.data_labelValue.modeRuning == self.modeRun_auto):
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: blue;")
		else:	
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")

	def update_checkDevices(self, data_color):
		# -- Clear error
		if (data_color.lbc_clear == 1):
			self.lbc_clear.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_clear == 0):
			self.lbc_clear.setStyleSheet("background-color: white;")
		# -- Blsock
		if (data_color.lbc_blsock == 1):
			self.lbc_blsock.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_blsock == 0):
			self.lbc_blsock.setStyleSheet("background-color: white;")
		# -- EMG
		if (data_color.lbc_emg == 1):
			self.lbc_emg.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_emg == 0):
			self.lbc_emg.setStyleSheet("background-color: white;")		
		# -- lift: limit1
		if (data_color.lbc_limit1 == 1):
			self.lbc_limit1.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_limit1 == 0):
			self.lbc_limit1.setStyleSheet("background-color: white;")
		# -- lift: limit2
		if (data_color.lbc_limit2 == 1):
			self.lbc_limit2.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_limit2 == 0):
			self.lbc_limit2.setStyleSheet("background-color: white;")
		# -- lift sensor detect
		if (data_color.lbc_liftSensor == 1):
			self.lbc_liftSensor.setStyleSheet("background-color: blue;")
		elif (data_color.lbc_liftSensor == 0):
			self.lbc_liftSensor.setStyleSheet("background-color: white;")

	def set_labelValue(self, data_blv): # App_lbv()
		self.lbv_x.setText(data_blv.lbv_x)
		self.lbv_y.setText(data_blv.lbv_y)
		self.lbv_zd.setText(data_blv.lbv_zd)
		# -
		self.lbv_ex.setText(data_blv.lbv_ex)
		self.lbv_ey.setText(data_blv.lbv_ey)
		# -
		self.lbv_ping.setText(data_blv.lbv_ping)
		self.lbv_ros.setText(data_blv.lbv_ros)
		# -
		self.lbv_velLeft.setText(data_blv.lbv_velLeft)
		self.lbv_velRight.setText(data_blv.lbv_velRight)
		# -
		self.lbv_mode.setText(data_blv.lbv_mode)
		self.lbv_detect.setText(data_blv.lbv_detect)
		self.lbv_job.setText(data_blv.lbv_job)
		self.lbv_load.setText(data_blv.lbv_load)
		# - task
		self.lbv_job1.setText(data_blv.lbv_job1)
		self.lbv_point0.setText(data_blv.lbv_point0)
		self.lbv_point1.setText(data_blv.lbv_point1)
		self.lbv_point2.setText(data_blv.lbv_point2)
		self.lbv_point3.setText(data_blv.lbv_point3)
		self.lbv_point4.setText(data_blv.lbv_point4)
		self.lbv_job2.setText(data_blv.lbv_job2)
		self.lbv_pointFinal.setText(data_blv.lbv_pointFinal)
		self.lbv_plan.setText(data_blv.lbv_plan)
		# - error
		self.lbv_device.setText(data_blv.lbv_device)
		self.lbv_frameWork.setText(data_blv.lbv_frameWork)	
		self.lbv_detailError.setText(data_blv.lbv_detailError)
		# - vel driver
		self.lbv_velLeft.setText(data_blv.lbv_velLeft)
		self.lbv_velRight.setText(data_blv.lbv_velRight)
		# -- bat
		self.lbv_battery.setText(data_blv.lbv_battery)
		# -- goal
		self.lbv_idGoal.setText(data_blv.lbv_idGoal)
		# -- reflector
		self.lbv_idLocal.setText(data_blv.lbv_idLocal)
		self.lbv_idGlobal.setText(data_blv.lbv_idGlobal)
		self.lbv_distance.setText(data_blv.lbv_distance)
		self.lbv_quality.setText(data_blv.lbv_quality)

	def controlShow_followMode(self):
		if (self.data_labelValue.modeRuning == self.modeRun_launch):
			self.modeRuning = self.modeRun_launch

		elif (self.data_labelValue.modeRuning == self.modeRun_byhand):
			self.modeRuning = self.modeRun_byhand

		elif (self.data_labelValue.modeRuning == self.modeRun_auto):
			self.modeRuning = self.modeRun_auto
		else:
			self.modeRuning = self.modeRun_auto
		# -- 
		self.enb_check = self.ck_showPara.isChecked()
		# --
		if (self.modeRuning == self.modeRun_launch): # -- Khoi Dong
			self.gb_launch.show()
			self.gb_run.hide()
			self.show_launch()

		else:
			self.gb_run.show()
			self.gb_launch.hide()

			if (self.modeRuning == self.modeRun_auto): # -- Tu dong

				self.gb_controlHand.hide()
				# self.lbv_mode.setText("Tu Dong")

				if (self.enb_check == 1):
					self.gb_detailError.hide()
					self.gb_listTask.show()
				else:
					self.gb_detailError.show()
					self.gb_listTask.hide()

			elif (self.modeRuning == self.modeRun_byhand):
				self.gb_detailError.hide()
				self.gb_listTask.hide()
				self.gb_controlHand.show()

				# self.lbv_mode.setText("Bang Tay")

			# --
			# if (self.enb_check == 1):
			# 	self.gb_errorPose.show()
			# else:
			# 	self.gb_errorPose.hide()

	def show_launch(self):
		self.lbv_launhing.setText(self.data_launch.lbv_launhing)
		self.lbv_numberLaunch.setText(self.data_launch.lbv_numberLaunch)
		# --
		value = self.data_launch.pb_launch
		if (value < 0):
			value = 0

		if (value > 100):
			value = 100

		self.pb_launch.setValue(value)
		# --
		if (self.data_launch.lbc_main == 0):
			self.lbc_lh_main.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_main.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_hc == 0):
			self.lbc_lh_hc.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_hc.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_oc == 0):
			self.lbc_lh_oc.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_oc.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_imu == 0):
			self.lbc_lh_imu.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_imu.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_lidar == 0):
			self.lbc_lh_lidar.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_lidar.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_load == 0):
			self.lbc_lh_load.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_load.setStyleSheet("background-color: blue;")

		if (self.data_launch.lbc_driver == 0):
			self.lbc_lh_driver.setStyleSheet("background-color: red;")	
		else:
			self.lbc_lh_driver.setStyleSheet("background-color: blue;")

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

		rospy.init_node('app_ros', anonymous=False)
		self.rate = rospy.Rate(20)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(580)
		self.widget.setFixedWidth(1024)

		# self.widget.setWindowTitle("no title")
		# -- 
		self.is_exist = 1
		self.widget.setWindowFlag(Qt.FramelessWindowHint)
		
		# -- Pub and Sub
		rospy.Subscriber("/app_setColor", App_color, self.callBack_setColor)
		self.app_setColor = App_color()

		rospy.Subscriber("/app_setValue", App_lbv, self.callBack_setValue)
		self.app_setValue = App_lbv()
		self.pre_app_setValue = App_lbv()

		rospy.Subscriber("/app_launch", App_launch, self.callBack_launch)
		self.app_launch = App_launch()

		rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
		self.cancelMission_status = Int16()
		# --
		self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
		self.cancelMission_control = Int16()
		# --
		self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)
		self.app_button = App_button()
		self.pre_app_setColor = App_color()

		# -- -- -- time update data
		self.pre_timeSet_a = time.time()
		self.pre_timeSet_b = time.time()
		self.pre_timeSet_c = time.time()
		# -- 
		self.name_agv = ""
		self.ip_agv = ""

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callBack_setColor(self, data):
		self.app_setColor = data
		# print ("app_setColor data")

	def callBack_setValue(self, data):
		self.app_setValue = data
		# print ("setValue data")

	def callBack_launch(self, data):
		self.app_launch = data

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

	def set_init(self):
		self.welcomeScreen.set_nameAgv_ip()

	def run(self):
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):

			if (self.cancelMission_status.data == 1):
				self.welcomeScreen.status_button.bt_cancelMission = 0
				self.cancelMission_control.data = 0

			self.app_button = self.welcomeScreen.status_button

			if (self.welcomeScreen.status_button.bt_cancelMission == 1):
				self.cancelMission_control.data = 1

			self.pub_button.publish(self.app_button)
			self.pub_cancelMission.publish(self.cancelMission_control)

			# ----------------------
			self.welcomeScreen.data_labelValue = self.app_setValue
			self.welcomeScreen.data_color = self.app_setColor
			self.welcomeScreen.data_launch = self.app_launch
			
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