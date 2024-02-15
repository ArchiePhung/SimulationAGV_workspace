#!/usr/bin/env python3

"""
Developer: Hoang van Quang
Company: STI Viet Nam
date: 17/11/2021
"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime

import sqlite3

sys.path.append('/home/stivietnam/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy
from message_pkg.msg import *

from datetime import datetime



"""
	--- Button ---
	bt_cancelMission
	bt_passHand
	bt_passAuto	
	bt_clearError

	bt_hide

	bt_spk_on
	bt_spk_off

	bt_chg_on
	bt_chg_off

	bt_lift_up
	bt_lift_down

	bt_forwards
	bt_backwards
	bt_rotation_left
	bt_rotation_right
	bt_stop

	--- Prepheral ---
	Driver Left
	Driver Right
	Lidar Sensor
	Main Board
	HC Board
	OC Board
	Load Board

	--- ERROR ---
	Loi thiet bi:
		1, Driver Left
		2, Driver Right
		3, Lidar Sensor
		4, Main Board
		5, HC Board
		6, OC Board
		7, Load Board

	--- Thong so ---
	lbv_name_agv
	lbv_ip
	lbv_battery
	lbv_date

	- error -
		lbv_device
		lbv_frameWork
	- position -
		lbv_x
		lbv_y
		lbv_zd
	- position run now - 
		lbv_ex
		lbv_ey

	lbv_ping
	lbv_ros
	lbv_velLeft
	lbv_velRight

	lbv_mode
	lbv_detect
	lbv_job
	lbv_load

	lbv_job1
	lbv_point0
	lbv_point1
	lbv_point2
	lbv_point3
	lbv_point4
	lbv_job2
	lbv_pointFinal
	lbv_plan

	- color -
		lbc_safety0
		lbc_safety1
		lbc_safety2

		bt_passHand
		bt_passAuto
		lbv_battery 
		
		lbc_safety11
		lbc_safety22
		lbc_limit2
		lbc_limit1
		lbc_liftSensor
		lbc_blsock
		lbc_emg
		lbc_clear
		lbc_status
	---  ---
	---  ---
	---  ---
	---  ---
	---  ---
	---  ---
	self.gb_listTask.hide()
	self.gb_listTask.show()
"""

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/stivietnam/catkin_ws/src/app_ros/interface/app.ui", self)
		# --
		# self.setWindowTitle("my name")
		# --
		self.gb_checkDevice.hide()
		# self.gb_listTask.hide()
		# -- 
		self.status_button = App_button()
		self.data_color = App_color()
		self.data_labelValue = App_lbv()
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
		self.modeNow = 0

	def process_fast(self):
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
				self.gb_checkDevice.show()
		else:
			self.timeSave_checkDevices = rospy.Time.now()


	def process_normal(self):
		self.set_dateTime()
		self.set_ip(" " + self.data_labelValue.lbv_ip)
		self.set_nameAgv(" " + self.data_labelValue.lbv_name_agv)

	def process_slow(self):
		self.set_valueBattery(self.data_labelValue.lbv_battery)
		
	def clicked_hideCheckDevices(self):
		self.gb_agv.show()
		self.gb_checkDevice.hide()

	def pressed_checkDevices(self):
		self.checkDevices_status = 1

	def released_checkDevices(self):
		self.checkDevices_status = 0

	def pressed_cancelMission(self):
		self.status_button.bt_cancelMission = 1
		self.bt_cancelMission.setStyleSheet("background-color: blue; border-style: outset; border-width: 2px; border-radius: 15px; border-color: gray;")

	def released_cancelMission(self):
		self.status_button.bt_cancelMission = 0
		self.bt_cancelMission.setStyleSheet("background-color: white; border-style: outset; border-width: 2px; border-radius: 15px; border-color: gray;")
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
		
	def released_passAuto(self):
		self.status_button.bt_passAuto = 0
		# self.bt_passAuto.setStyleSheet("background-color: white;")
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
	# - 
	def clicked_spk_off(self):
		self.status_button.bt_spk_off = 1
		self.status_button.bt_spk_on = 0
		self.bt_spk_off.setStyleSheet("background-color: blue;")
		self.bt_spk_on.setStyleSheet("background-color: white;")
	# - 
	def clicked_chg_on(self):
		self.status_button.bt_chg_on = 1
		self.status_button.bt_chg_off = 0
		self.bt_chg_on.setStyleSheet("background-color: blue;")
		self.bt_chg_off.setStyleSheet("background-color: white;")
	# - 
	def clicked_chg_off(self):
		self.status_button.bt_chg_off = 1
		self.status_button.bt_chg_on = 0
		self.bt_chg_off.setStyleSheet("background-color: blue;")
		self.bt_chg_on.setStyleSheet("background-color: white;")
	# - 
	def clicked_lift_up(self):
		self.status_button.bt_lift_up = 1
		self.status_button.bt_lift_down = 0
		self.bt_lift_up.setStyleSheet("background-color: blue;")
		self.bt_lift_down.setStyleSheet("background-color: white;")
	# - 
	def clicked_lift_down(self):
		self.status_button.bt_lift_down = 1
		self.status_button.bt_lift_up = 0
		self.bt_lift_down.setStyleSheet("background-color: blue;")
		self.bt_lift_up.setStyleSheet("background-color: white;")
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
		if (self.data_color.bt_passHand == 0 and self.data_color.bt_passAuto == 0):
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")	

		elif (self.data_color.bt_passHand == 1 and self.data_color.bt_passAuto == 0):
			self.bt_passHand.setStyleSheet("background-color: blue;")	
			self.bt_passAuto.setStyleSheet("background-color: white;")	
			
		elif (self.data_color.bt_passHand == 0 and self.data_color.bt_passAuto == 1):
			self.bt_passHand.setStyleSheet("background-color: white;")	
			self.bt_passAuto.setStyleSheet("background-color: blue;")	

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

	def controlShow_followMode(self):
		if (self.data_color.bt_passHand == 0 and self.data_color.bt_passAuto == 0):
			self.modeNow = 0

		elif (self.data_color.bt_passHand == 1 and self.data_color.bt_passAuto == 0):
			self.modeNow = 1

		elif (self.data_color.bt_passHand == 0 and self.data_color.bt_passAuto == 1):
			self.modeNow = 2
		# -- 
		self.enb_check = self.ck_showPara.isChecked()
		if (self.modeNow == 2): # -- Tu dong
			self.gb_controlHand.hide()
			self.lbv_mode.setText("Tu Dong")
			if (self.enb_check == 1):
				self.gb_detailError.hide()
				self.gb_listTask.show()
			else:
				self.gb_detailError.show()
				self.gb_listTask.hide()
		else:
			self.gb_detailError.hide()
			self.gb_listTask.hide()
			self.gb_controlHand.show()

			if (self.modeNow == 0):
				self.lbv_mode.setText("Tu Do")
			elif (self.modeNow == 1):
				self.lbv_mode.setText("Bang Tay")
		# --
		if (self.enb_check == 1):
			self.gb_errorPose.show()
		else:
			self.gb_errorPose.hide()

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
		self.widget.setFixedHeight(540)
		self.widget.setFixedWidth(1024)
		# -- 
		self.is_exist = 1
		
		# -- Pub and Sub
		rospy.Subscriber("/app_setColor", App_color, self.callBack_setColor)
		self.app_setColor = App_color()

		rospy.Subscriber("/app_setValue", App_lbv, self.callBack_setValue)
		self.app_setValue = App_lbv()	
		self.pre_app_setValue = App_lbv()

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


	def callBack_setColor(self, data):
		self.app_setColor = data
		# print ("app_setColor data")

	def callBack_setValue(self, data):
		self.app_setValue = data
		# print ("setValue data")

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
			self.app_button = self.welcomeScreen.status_button
			self.pub_button.publish(self.app_button)

			# ----------------------
			self.welcomeScreen.data_labelValue = self.app_setValue
			self.welcomeScreen.data_color = self.app_setColor
			
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