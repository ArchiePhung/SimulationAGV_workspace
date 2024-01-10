#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phung Quy Duong(Archie)
Company: STI Viet Nam
date: 25/12/2023

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
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime, Qt

import sqlite3

sys.path.append('/home/tiger/simulation_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

# print('\n'.join(sys.path))

import roslib
import rospy

from ros_canbus.msg import *	
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class statusButton:
	def __init__(self):
		self.bt_limitAhead11 = 0
		self.bt_limitBehind11 = 0
		self.bt_ssCheckTray11 = 0
		
		self.bt_limitAhead12 = 0
		self.bt_limitBehind12 = 0
		self.bt_ssCheckTray12 = 0		

		self.bt_limitAhead21 = 0
		self.bt_limitBehind21 = 0
		self.bt_ssCheckTray21 = 0
		
		self.bt_limitAhead22 = 0
		self.bt_limitBehind22 = 0
		self.bt_ssCheckTray22 = 0	

		self.bt_cvoToyoBit1 = 0
		self.bt_cvoToyoBit2 = 0
		self.bt_cvoToyoBit3 = 0
		self.bt_cvoToyoBit4 = 0
		self.bt_cvoToyoBit5_connected = 0
		self.bt_cvoToyoBit6_done = 0
		self.bt_cvoToyoBit7_ready = 0
		self.bt_cvoToyoBit8_error = 0

class statusColor:
	def __init__(self):
	# --
		self.lbc_agvToyoBit1 = 0
		self.lbc_agvToyoBit2 = 0
		self.lbc_agvToyoBit3 = 0
		self.lbc_agvToyoBit4 = 0

		self.lbc_agvToyoBit5_connected = 0
		self.lbc_agvToyoBit6_done = 0
		self.lbc_agvToyoBit7_ready = 0
		self.lbc_agvToyoBit8_error = 0		

class statusConveyor:
	def __init__(self):
		self.cvoState11 = 0
		self.cvoState12 = 0
		self.cvoState21 = 0
		self.cvoState22 = 0

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/tiger/simulation_ws/src/app_ros600/interface/app_agv12Conveyor_test.ui", self)
		# --
		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.statusConveyor = statusConveyor()
		# --
		self.pb_limitAhead11.clicked.connect(self.clicked_limitAhead11)
		self.pb_limitBehind11.clicked.connect(self.clicked_limitBehind11)
		self.pb_ssCheckTray11.clicked.connect(self.clicked_ssCheckTray11)

		self.pb_limitAhead12.clicked.connect(self.clicked_limitAhead12)
		self.pb_limitBehind12.clicked.connect(self.clicked_limitBehind12)
		self.pb_ssCheckTray12.clicked.connect(self.clicked_ssCheckTray12)

		self.pb_limitAhead21.clicked.connect(self.clicked_limitAhead21)
		self.pb_limitBehind21.clicked.connect(self.clicked_limitBehind21)
		self.pb_ssCheckTray21.clicked.connect(self.clicked_ssCheckTray21)

		self.pb_limitAhead22.clicked.connect(self.clicked_limitAhead22)
		self.pb_limitBehind22.clicked.connect(self.clicked_limitBehind22)
		self.pb_ssCheckTray22.clicked.connect(self.clicked_ssCheckTray22)

		# -
		self.pb_cvotoyo1.clicked.connect(self.clicked_cvotoyo1)
		self.pb_cvotoyo2.clicked.connect(self.clicked_cvotoyo2)
		self.pb_cvotoyo3.clicked.connect(self.clicked_cvotoyo3)
		self.pb_cvotoyo4.clicked.connect(self.clicked_cvotoyo4)
		self.pb_cvotoyo5_connected.clicked.connect(self.clicked_cvotoyo5_connected)
		self.pb_cvotoyo6_done.clicked.connect(self.clicked_cvotoyo6_done)
		self.pb_cvotoyo7_ready.clicked.connect(self.clicked_cvotoyo7_ready)
		self.pb_cvotoyo8_error.clicked.connect(self.clicked_cvotoyo8_error)

		self.ld_status11.returnPressed.connect(lambda: self.get_statusConveyor11())
		self.ld_status12.returnPressed.connect(lambda: self.get_statusConveyor12())
		self.ld_status21.returnPressed.connect(lambda: self.get_statusConveyor21())
		self.ld_status22.returnPressed.connect(lambda: self.get_statusConveyor22())

		self.cvoState11_string = ""
		self.cvoState12_string = ""
		self.cvoState21_string = ""
		self.cvoState22_string = ""

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
		# timer_slow = QTimer(self)
		# timer_slow.timeout.connect(self.process_slow)
		# timer_slow.start(3000)

	def process_fast(self):
		# - 
		if (self.statusButton.bt_cvoToyoBit1 == 1):
			self.pb_cvotoyo1.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo1.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit2 == 1):
			self.pb_cvotoyo2.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo2.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit3 == 1):
			self.pb_cvotoyo3.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo3.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit4 == 1):
			self.pb_cvotoyo4.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo4.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit5_connected == 1):
			self.pb_cvotoyo5_connected.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo5_connected.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit6_done == 1):
			self.pb_cvotoyo6_done.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo6_done.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit7_ready == 1):
			self.pb_cvotoyo7_ready.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo7_ready.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit8_error == 1):
			self.pb_cvotoyo8_error.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo8_error.setStyleSheet("background-color: white;")

		# # --
		self.set_labelColor()
		# # --
	# -- 
	def process_normal(self):
		self.set_dateTime()

	# - conveyor 11
	def clicked_limitAhead11(self):
		self.statusButton.bt_limitAhead11 = not self.statusButton.bt_limitAhead11
		if self.statusButton.bt_limitAhead11 == 1:
			self.pb_limitAhead11.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead11.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind11(self):
		self.statusButton.bt_limitBehind11 = not self.statusButton.bt_limitBehind11
		if self.statusButton.bt_limitBehind11 == 1:
			self.pb_limitBehind11.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind11.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray11(self):
		self.statusButton.bt_ssCheckTray11 = not self.statusButton.bt_ssCheckTray11
		if self.statusButton.bt_ssCheckTray11 == 1:
			self.pb_ssCheckTray11.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray11.setStyleSheet("background-color: white;")

	# -- conveyor 12
	def clicked_limitAhead12(self):
		self.statusButton.bt_limitAhead12 = not self.statusButton.bt_limitAhead12
		if self.statusButton.bt_limitAhead12 == 1:
			self.pb_limitAhead12.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead12.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind12(self):
		self.statusButton.bt_limitBehind12 = not self.statusButton.bt_limitBehind12
		if self.statusButton.bt_limitBehind12 == 1:
			self.pb_limitBehind12.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind12.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray12(self):
		self.statusButton.bt_ssCheckTray12 = not self.statusButton.bt_ssCheckTray12
		if self.statusButton.bt_ssCheckTray12 == 1:
			self.pb_ssCheckTray12.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray12.setStyleSheet("background-color: white;")

	# -- conveyor 21
	def clicked_limitAhead21(self):
		self.statusButton.bt_limitAhead21 = not self.statusButton.bt_limitAhead21
		if self.statusButton.bt_limitAhead21 == 1:
			self.pb_limitAhead21.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead21.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind21(self):
		self.statusButton.bt_limitBehind21 = not self.statusButton.bt_limitBehind21
		if self.statusButton.bt_limitBehind21 == 1:
			self.pb_limitBehind21.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind21.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray21(self):
		self.statusButton.bt_ssCheckTray21 = not self.statusButton.bt_ssCheckTray21
		if self.statusButton.bt_ssCheckTray21 == 1:
			self.pb_ssCheckTray21.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray21.setStyleSheet("background-color: white;")

	# -- conveyor 22
	def clicked_limitAhead22(self):
		self.statusButton.bt_limitAhead22 = not self.statusButton.bt_limitAhead22
		if self.statusButton.bt_limitAhead22 == 1:
			self.pb_limitAhead22.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead22.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind22(self):
		self.statusButton.bt_limitBehind22 = not self.statusButton.bt_limitBehind22
		if self.statusButton.bt_limitBehind22 == 1:
			self.pb_limitBehind22.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind22.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray22(self):
		self.statusButton.bt_ssCheckTray22 = not self.statusButton.bt_ssCheckTray22
		if self.statusButton.bt_ssCheckTray22 == 1:
			self.pb_ssCheckTray22.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray22.setStyleSheet("background-color: white;")

	def clicked_cvotoyo1(self):
		self.statusButton.bt_cvoToyoBit1 = not self.statusButton.bt_cvoToyoBit1	

	def clicked_cvotoyo2(self):
		self.statusButton.bt_cvoToyoBit2 = not self.statusButton.bt_cvoToyoBit2

	def clicked_cvotoyo3(self):
		self.statusButton.bt_cvoToyoBit3 = not self.statusButton.bt_cvoToyoBit3

	def clicked_cvotoyo4(self):
		self.statusButton.bt_cvoToyoBit4 = not self.statusButton.bt_cvoToyoBit4	

	def clicked_cvotoyo5_connected(self):
		self.statusButton.bt_cvoToyoBit5_connected = not self.statusButton.bt_cvoToyoBit5_connected	

	def clicked_cvotoyo6_done(self):
		self.statusButton.bt_cvoToyoBit6_done = not self.statusButton.bt_cvoToyoBit6_done

	def clicked_cvotoyo7_ready(self):
		self.statusButton.bt_cvoToyoBit7_ready = not self.statusButton.bt_cvoToyoBit7_ready

	def clicked_cvotoyo8_error(self):
		self.statusButton.bt_cvoToyoBit8_error = not self.statusButton.bt_cvoToyoBit8_error	

	def get_statusConveyor11(self):
		self.cvoState11_string = self.ld_status11.text()
		self.statusConveyor.cvoState11 = int(self.cvoState11_string)

	def get_statusConveyor12(self):
		self.cvoState12_string = self.ld_status12.text()
		self.statusConveyor.cvoState12 = int(self.cvoState12_string)

	def get_statusConveyor21(self):
		self.cvoState21_string = self.ld_status21.text()
		self.statusConveyor.cvoState21 = int(self.cvoState21_string)

	def get_statusConveyor22(self):
		self.cvoState22_string = self.ld_status22.text()
		self.statusConveyor.cvoState22 = int(self.cvoState22_string)

	def out(self):
		QApplication.quit()
		print('out')

	# ---------------------------
	def set_dateTime(self):
		time_now = datetime.now()
		# dd/mm/YY H:M:S
		dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
		self.lbv_date.setText(dt_string)
	
	def set_labelColor(self):
		# -------------
		if (self.statusColor.lbc_agvToyoBit1 == 1):
			self.lb_agvtoyo1.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo1.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit2 == 1):
			self.lb_agvtoyo2.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo2.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit3 == 1):
			self.lb_agvtoyo3.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo3.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit4 == 1):
			self.lb_agvtoyo4.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo4.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit5_connected == 1):
			self.lb_agvtoyo5_connected.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo5_connected.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit6_done == 1):
			self.lb_agvtoyo6_done.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo6_done.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit7_ready == 1):
			self.lb_agvtoyo7_ready.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo7_ready.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit8_error == 1):
			self.lb_agvtoyo8_error.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo8_error.setStyleSheet("background-color: white;")
		
class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		# --
		self.name_card = rospy.get_param("name_card", "wlp3s0")
		self.name_card = "wlp3s0"

		self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		#self.address_traffic = "192.168.1.92"
		self.pre_timePing = time.time()
		# --
		rospy.init_node('appFakeSignal', anonymous=False)
		self.rate = rospy.Rate(40)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(580)
		self.widget.setFixedWidth(1024)
		# self.widget.setWindowFlag(Qt.FramelessWindowHint)
		# --
		self.statusConveyor = statusConveyor()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# -----------------------------------------------------------
		rospy.Subscriber("/AGVToyo_signal", Signal_AGVToyo, self.callBack_AGVToyo_signal)
		self.AGVToyo_signal = Signal_AGVToyo()
		# --
		self.pub_signalConveyor11 = rospy.Publisher("/signal_conveyor11", Signal_ConveyorAGV, queue_size = 4)
		self.data_signalConveyor11 = Signal_ConveyorAGV()
		# --
		self.pub_signalConveyor12 = rospy.Publisher("/signal_conveyor12", Signal_ConveyorAGV, queue_size = 4)
		self.data_signalConveyor12 = Signal_ConveyorAGV()
		# --
		self.pub_signalConveyor21 = rospy.Publisher("/signal_conveyor21", Signal_ConveyorAGV, queue_size = 4)
		self.data_signalConveyor21 = Signal_ConveyorAGV()
		# --
		self.pub_signalConveyor22 = rospy.Publisher("/signal_conveyor22", Signal_ConveyorAGV, queue_size = 4)
		self.data_signalConveyor22 = Signal_ConveyorAGV()
		# --
		self.pub_signalConveyorToyo = rospy.Publisher("/signal_conveyorToyo", Signal_ConveyorToyo, queue_size = 4)
		self.data_signalConveyorToyo = Signal_ConveyorToyo()

	def callBack_AGVToyo_signal(self, data):
		self.AGVToyo_signal = data
		
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

	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			print ("address: ", address)
			return address
		except Exception:
			return "-1"
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

	def controlColor(self):
		# -- AGV Toyo signal
		self.statusColor.lbc_agvToyoBit1 = self.AGVToyo_signal.signal_toyoBit1
		self.statusColor.lbc_agvToyoBit2 = self.AGVToyo_signal.signal_toyoBit2
		self.statusColor.lbc_agvToyoBit3 = self.AGVToyo_signal.signal_toyoBit3
		self.statusColor.lbc_agvToyoBit4 = self.AGVToyo_signal.signal_toyoBit4
		self.statusColor.lbc_agvToyoBit5_connected = self.AGVToyo_signal.signal_toyoBit5_connected
		self.statusColor.lbc_agvToyoBit6_done = self.AGVToyo_signal.signal_toyoBit6_done
		self.statusColor.lbc_agvToyoBit7_ready = self.AGVToyo_signal.signal_toyoBit7_ready
		self.statusColor.lbc_agvToyoBit8_error = self.AGVToyo_signal.signal_toyoBit8_error

		if self.AGVToyo_signal.signal_toyoBit6_done == 1:
			self.welcomeScreen.statusButton.bt_cvoToyoBit1 = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit2 = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit3 = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit4 = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit5_connected = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit6_done = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit7_ready = 0
			self.welcomeScreen.statusButton.bt_cvoToyoBit8_error = 0		

	def readButton(self):
		# -- 
		self.data_signalConveyor11.signal_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead11
		self.data_signalConveyor11.signal_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind11
		self.data_signalConveyor11.signal_ssCheckTray = self.welcomeScreen.statusButton.bt_ssCheckTray11

		# -- 
		self.data_signalConveyor12.signal_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead12
		self.data_signalConveyor12.signal_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind12
		self.data_signalConveyor12.signal_ssCheckTray = self.welcomeScreen.statusButton.bt_ssCheckTray12

		# --
		self.data_signalConveyor21.signal_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead21
		self.data_signalConveyor21.signal_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind21
		self.data_signalConveyor21.signal_ssCheckTray = self.welcomeScreen.statusButton.bt_ssCheckTray21
		
		# --
		self.data_signalConveyor22.signal_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead22
		self.data_signalConveyor22.signal_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind22
		self.data_signalConveyor22.signal_ssCheckTray = self.welcomeScreen.statusButton.bt_ssCheckTray22

		# -- Signal of Conveyor Toyo 
		self.data_signalConveyorToyo.signal_toyoBit1 = self.welcomeScreen.statusButton.bt_cvoToyoBit1
		self.data_signalConveyorToyo.signal_toyoBit2 = self.welcomeScreen.statusButton.bt_cvoToyoBit2
		self.data_signalConveyorToyo.signal_toyoBit3 = self.welcomeScreen.statusButton.bt_cvoToyoBit3
		self.data_signalConveyorToyo.signal_toyoBit4 = self.welcomeScreen.statusButton.bt_cvoToyoBit4
		self.data_signalConveyorToyo.signal_toyoBit5_connected = self.welcomeScreen.statusButton.bt_cvoToyoBit5_connected
		self.data_signalConveyorToyo.signal_toyoBit6_done = self.welcomeScreen.statusButton.bt_cvoToyoBit6_done
		self.data_signalConveyorToyo.signal_toyoBit7_ready = self.welcomeScreen.statusButton.bt_cvoToyoBit7_ready
		self.data_signalConveyorToyo.signal_toyoBit8_error = self.welcomeScreen.statusButton.bt_cvoToyoBit8_error
		
	def readLineEdit(self):
		self.data_signalConveyor11.status = self.welcomeScreen.statusConveyor.cvoState11
		self.data_signalConveyor12.status = self.welcomeScreen.statusConveyor.cvoState12
		self.data_signalConveyor21.status = self.welcomeScreen.statusConveyor.cvoState21
		self.data_signalConveyor22.status = self.welcomeScreen.statusConveyor.cvoState22

	def run(self):
		# -- 

		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.controlColor()
			# --
			self.readButton()
			# --
			self.readLineEdit()

			self.pub_signalConveyor11.publish(self.data_signalConveyor11)
			self.pub_signalConveyor12.publish(self.data_signalConveyor12)
			self.pub_signalConveyor21.publish(self.data_signalConveyor21)
			self.pub_signalConveyor22.publish(self.data_signalConveyor22)
			self.pub_signalConveyorToyo.publish(self.data_signalConveyorToyo)
			# ----------------------
			self.welcomeScreen.statusColor = self.statusColor

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
