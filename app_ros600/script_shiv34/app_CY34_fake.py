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
		# -
		self.bt_limitAhead11 = 0
		self.bt_limitBehind11 = 0
		self.bt_ssCheckTray11 = 0
		
		self.bt_limitAhead21 = 0
		self.bt_limitBehind21 = 0
		self.bt_ssCheckTray21 = 0		

		# -
		self.bt_limitAhead12 = 0
		self.bt_limitBehind12 = 0
		self.bt_ssCheckTray12 = 0
		
		self.bt_limitAhead22 = 0
		self.bt_limitBehind22 = 0
		self.bt_ssCheckTray22 = 0	

		# -
		self.bt_limitAhead13 = 0
		self.bt_limitBehind13 = 0
		self.bt_ssCheckTray13 = 0
		
		self.bt_limitAhead23 = 0
		self.bt_limitBehind23 = 0
		self.bt_ssCheckTray23 = 0

		# -
		self.bt_cvoToyoBit1 = 0
		self.bt_cvoToyoBit2 = 0
		self.bt_cvoToyoBit3 = 0
		self.bt_cvoToyoBit4 = 0
		self.bt_cvoToyoBit5 = 0
		self.bt_cvoToyoBit6 = 0
		self.bt_cvoToyoBit7 = 0
		self.bt_cvoToyoBit8 = 0
		
		# -
		self.bt_ssTray1 = 0
		self.bt_ssTray2 = 0

class statusColor:
	def __init__(self):
	# --
		self.lbc_agvToyoBit1 = 0
		self.lbc_agvToyoBit2 = 0
		self.lbc_agvToyoBit3 = 0
		self.lbc_agvToyoBit4 = 0

		self.lbc_agvToyoBit5 = 0
		self.lbc_agvToyoBit6 = 0
		self.lbc_agvToyoBit7 = 0
		self.lbc_agvToyoBit8 = 0		

class statusConveyor:
	def __init__(self):
		self.cvoState11 = 0
		self.cvoState21 = 0
		self.cvoState12 = 0
		self.cvoState22 = 0
		self.cvoState13 = 0
		self.cvoState23 = 0

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/tiger/simulation_ws/src/app_ros600/interface/app_CY34_fake.ui", self)
		# --
		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.statusConveyor = statusConveyor()
		# --
		self.pb_limitAhead11.clicked.connect(self.clicked_limitAhead11)
		self.pb_limitBehind11.clicked.connect(self.clicked_limitBehind11)
		self.pb_ssCheckTray11.clicked.connect(self.clicked_ssCheckTray11)

		self.pb_limitAhead21.clicked.connect(self.clicked_limitAhead21)
		self.pb_limitBehind21.clicked.connect(self.clicked_limitBehind21)
		self.pb_ssCheckTray21.clicked.connect(self.clicked_ssCheckTray21)
		# - 
		self.pb_limitAhead12.clicked.connect(self.clicked_limitAhead12)
		self.pb_limitBehind12.clicked.connect(self.clicked_limitBehind12)
		self.pb_ssCheckTray12.clicked.connect(self.clicked_ssCheckTray12)

		self.pb_limitAhead22.clicked.connect(self.clicked_limitAhead22)
		self.pb_limitBehind22.clicked.connect(self.clicked_limitBehind22)
		self.pb_ssCheckTray22.clicked.connect(self.clicked_ssCheckTray22)
		# -
		self.pb_limitAhead13.clicked.connect(self.clicked_limitAhead13)
		self.pb_limitBehind13.clicked.connect(self.clicked_limitBehind13)
		self.pb_ssCheckTray13.clicked.connect(self.clicked_ssCheckTray13)

		self.pb_limitAhead23.clicked.connect(self.clicked_limitAhead23)
		self.pb_limitBehind23.clicked.connect(self.clicked_limitBehind23)
		self.pb_ssCheckTray23.clicked.connect(self.clicked_ssCheckTray23)
		# -- 
		self.ld_status11.returnPressed.connect(lambda: self.get_statusConveyor11())
		self.ld_status21.returnPressed.connect(lambda: self.get_statusConveyor21())
		self.ld_status12.returnPressed.connect(lambda: self.get_statusConveyor12())
		self.ld_status22.returnPressed.connect(lambda: self.get_statusConveyor22())
		self.ld_status13.returnPressed.connect(lambda: self.get_statusConveyor13())
		self.ld_status23.returnPressed.connect(lambda: self.get_statusConveyor23())

		self.cvoState11_string = ""
		self.cvoState21_string = ""
		self.cvoState12_string = ""
		self.cvoState22_string = ""
		self.cvoState13_string = ""
		self.cvoState23_string = ""

		# -
		self.pb_cvotoyo1.clicked.connect(self.clicked_cvotoyo1)
		self.pb_cvotoyo2.clicked.connect(self.clicked_cvotoyo2)
		self.pb_cvotoyo3.clicked.connect(self.clicked_cvotoyo3)
		self.pb_cvotoyo4.clicked.connect(self.clicked_cvotoyo4)
		self.pb_cvotoyo5.clicked.connect(self.clicked_cvotoyo5)
		self.pb_cvotoyo6.clicked.connect(self.clicked_cvotoyo6)
		self.pb_cvotoyo7.clicked.connect(self.clicked_cvotoyo7)
		self.pb_cvotoyo8.clicked.connect(self.clicked_cvotoyo8)
		# -
		self.pb_ssTray1.clicked.connect(self.clicked_ssTray1)
		self.pb_ssTray2.clicked.connect(self.clicked_ssTray2)		

		# --
		self.pb_nextpage.clicked.connect(self.clicked_nextpage)
		self.pb_backpage.clicked.connect(self.clicked_backpage)

		# -- -- -- 
		self.bt_exit.pressed.connect(self.out)

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

		self.fr_CPD.hide()
		self.fr_OC.show()

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

		if (self.statusButton.bt_cvoToyoBit5 == 1):
			self.pb_cvotoyo5.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo5.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit6 == 1):
			self.pb_cvotoyo6.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo6.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit7 == 1):
			self.pb_cvotoyo7.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo7.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_cvoToyoBit8 == 1):
			self.pb_cvotoyo8.setStyleSheet("background-color: blue;")
		else:
			self.pb_cvotoyo8.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_ssTray1 == 1):
			self.pb_ssTray1.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssTray1.setStyleSheet("background-color: white;")

		if (self.statusButton.bt_ssTray2 == 1):
			self.pb_ssTray2.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssTray2.setStyleSheet("background-color: white;")

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

	# -- conveyor 13
	def clicked_limitAhead13(self):
		self.statusButton.bt_limitAhead13 = not self.statusButton.bt_limitAhead13
		if self.statusButton.bt_limitAhead13 == 1:
			self.pb_limitAhead13.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead13.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind13(self):
		self.statusButton.bt_limitBehind13 = not self.statusButton.bt_limitBehind13
		if self.statusButton.bt_limitBehind13 == 1:
			self.pb_limitBehind13.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind13.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray13(self):
		self.statusButton.bt_ssCheckTray13 = not self.statusButton.bt_ssCheckTray13
		if self.statusButton.bt_ssCheckTray13 == 1:
			self.pb_ssCheckTray13.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray13.setStyleSheet("background-color: white;")

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

	# -- conveyor 23
	def clicked_limitAhead23(self):
		self.statusButton.bt_limitAhead23 = not self.statusButton.bt_limitAhead23
		if self.statusButton.bt_limitAhead23 == 1:
			self.pb_limitAhead23.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitAhead23.setStyleSheet("background-color: white;")
	# - 
	def clicked_limitBehind23(self):
		self.statusButton.bt_limitBehind23 = not self.statusButton.bt_limitBehind23
		if self.statusButton.bt_limitBehind23 == 1:
			self.pb_limitBehind23.setStyleSheet("background-color: blue;")
		else:
			self.pb_limitBehind23.setStyleSheet("background-color: white;")
	# -
	def clicked_ssCheckTray23(self):
		self.statusButton.bt_ssCheckTray23 = not self.statusButton.bt_ssCheckTray23
		if self.statusButton.bt_ssCheckTray23 == 1:
			self.pb_ssCheckTray23.setStyleSheet("background-color: blue;")
		else:
			self.pb_ssCheckTray23.setStyleSheet("background-color: white;")

	# -- 
	def clicked_cvotoyo1(self):
		self.statusButton.bt_cvoToyoBit1 = not self.statusButton.bt_cvoToyoBit1	

	def clicked_cvotoyo2(self):
		self.statusButton.bt_cvoToyoBit2 = not self.statusButton.bt_cvoToyoBit2

	def clicked_cvotoyo3(self):
		self.statusButton.bt_cvoToyoBit3 = not self.statusButton.bt_cvoToyoBit3

	def clicked_cvotoyo4(self):
		self.statusButton.bt_cvoToyoBit4 = not self.statusButton.bt_cvoToyoBit4	

	def clicked_cvotoyo5(self):
		self.statusButton.bt_cvoToyoBit5 = not self.statusButton.bt_cvoToyoBit5	

	def clicked_cvotoyo6(self):
		self.statusButton.bt_cvoToyoBit6 = not self.statusButton.bt_cvoToyoBit6

	def clicked_cvotoyo7(self):
		self.statusButton.bt_cvoToyoBit7 = not self.statusButton.bt_cvoToyoBit7

	def clicked_cvotoyo8(self):
		self.statusButton.bt_cvoToyoBit8 = not self.statusButton.bt_cvoToyoBit8

	def clicked_ssTray1(self):
		self.statusButton.bt_ssTray1 = not self.statusButton.bt_ssTray1

	def clicked_ssTray2(self):
		self.statusButton.bt_ssTray2 = not self.statusButton.bt_ssTray2

	# --
	def get_statusConveyor11(self):
		self.cvoState11_string = self.ld_status11.text()
		self.statusConveyor.cvoState11 = int(self.cvoState11_string)

	def get_statusConveyor21(self):
		self.cvoState21_string = self.ld_status21.text()
		self.statusConveyor.cvoState21 = int(self.cvoState21_string)

	def get_statusConveyor12(self):
		self.cvoState12_string = self.ld_status12.text()
		self.statusConveyor.cvoState12 = int(self.cvoState12_string)

	def get_statusConveyor22(self):
		self.cvoState22_string = self.ld_status22.text()
		self.statusConveyor.cvoState22 = int(self.cvoState22_string)

	def get_statusConveyor13(self):
		self.cvoState13_string = self.ld_status13.text()
		self.statusConveyor.cvoState13 = int(self.cvoState13_string)

	def get_statusConveyor23(self):
		self.cvoState23_string = self.ld_status23.text()
		self.statusConveyor.cvoState23 = int(self.cvoState23_string)

	def clicked_nextpage(self):
		self.fr_CPD.show()
		self.fr_OC.hide()
	
	def clicked_backpage(self):
		self.fr_CPD.hide()
		self.fr_OC.show()

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

		if (self.statusColor.lbc_agvToyoBit5 == 1):
			self.lb_agvtoyo5.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo5.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit6 == 1):
			self.lb_agvtoyo6.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo6.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit7 == 1):
			self.lb_agvtoyo7.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo7.setStyleSheet("background-color: white;")

		if (self.statusColor.lbc_agvToyoBit8 == 1):
			self.lb_agvtoyo8.setStyleSheet("background-color: blue;")
		else:
			self.lb_agvtoyo8.setStyleSheet("background-color: white;")
		
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
		self.pub_statusConveyor11 = rospy.Publisher("/status_conveyor11", Status_conveyor, queue_size = 4)
		self.data_statusConveyor11 = Status_conveyor()
		# --
		self.pub_statusConveyor21 = rospy.Publisher("/status_conveyor21", Status_conveyor, queue_size = 4)
		self.data_statusConveyor21 = Status_conveyor()
		# --
		self.pub_statusConveyor12 = rospy.Publisher("/status_conveyor12", Status_conveyor, queue_size = 4)
		self.data_statusConveyor12 = Status_conveyor()
		# --
		self.pub_statusConveyor22 = rospy.Publisher("/status_conveyor22", Status_conveyor, queue_size = 4)
		self.data_statusConveyor22 = Status_conveyor()
		# --
		self.pub_statusConveyor13 = rospy.Publisher("/status_conveyor13", Status_conveyor, queue_size = 4)
		self.data_statusConveyor13 = Status_conveyor()
		# --
		self.pub_statusConveyor23 = rospy.Publisher("/status_conveyor23", Status_conveyor, queue_size = 4)
		self.data_statusConveyor23 = Status_conveyor()
		# --
		self.pub_CPDread = rospy.Publisher("/CPD_read", CPD_read, queue_size = 4)
		self.data_CPDread = CPD_read()
		# --
		rospy.Subscriber("/CPD_write", CPD_write, self.callBack_CPDwrite)
		self.data_CPDwrite = CPD_write()

	def callBack_CPDwrite(self, data):
		self.data_CPDwrite = data
		
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
		self.statusColor.lbc_agvToyoBit1 = self.data_CPDwrite.output1
		self.statusColor.lbc_agvToyoBit2 = self.data_CPDwrite.output2
		self.statusColor.lbc_agvToyoBit3 = self.data_CPDwrite.output3
		self.statusColor.lbc_agvToyoBit4 = self.data_CPDwrite.output4
		self.statusColor.lbc_agvToyoBit5 = self.data_CPDwrite.output5
		self.statusColor.lbc_agvToyoBit6 = self.data_CPDwrite.output6
		self.statusColor.lbc_agvToyoBit7 = self.data_CPDwrite.output7
		self.statusColor.lbc_agvToyoBit8 = self.data_CPDwrite.output8

	def readButton(self):
		# -- 
		self.data_statusConveyor11.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead11
		self.data_statusConveyor11.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind11
		self.data_statusConveyor11.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray11

		# -- 
		self.data_statusConveyor12.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead12
		self.data_statusConveyor12.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind12
		self.data_statusConveyor12.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray12

		# -- 
		self.data_statusConveyor13.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead13
		self.data_statusConveyor13.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind13
		self.data_statusConveyor13.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray13

		# --
		self.data_statusConveyor21.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead21
		self.data_statusConveyor21.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind21
		self.data_statusConveyor21.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray21
		
		# --
		self.data_statusConveyor22.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead22
		self.data_statusConveyor22.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind22
		self.data_statusConveyor22.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray22

		# --
		self.data_statusConveyor23.sensor_limitAhead = self.welcomeScreen.statusButton.bt_limitAhead23
		self.data_statusConveyor23.sensor_limitBehind = self.welcomeScreen.statusButton.bt_limitBehind23
		self.data_statusConveyor23.sensor_checkRack = self.welcomeScreen.statusButton.bt_ssCheckTray23

		# -- Signal of Conveyor Toyo 
		self.data_CPDread.input1 = self.welcomeScreen.statusButton.bt_ssTray1
		self.data_CPDread.input2 = self.welcomeScreen.statusButton.bt_ssTray2

		self.data_CPDread.input3 = self.welcomeScreen.statusButton.bt_cvoToyoBit1
		self.data_CPDread.input4 = self.welcomeScreen.statusButton.bt_cvoToyoBit2
		self.data_CPDread.input5 = self.welcomeScreen.statusButton.bt_cvoToyoBit3
		self.data_CPDread.input6 = self.welcomeScreen.statusButton.bt_cvoToyoBit4
		self.data_CPDread.input7 = self.welcomeScreen.statusButton.bt_cvoToyoBit5
		self.data_CPDread.input8 = self.welcomeScreen.statusButton.bt_cvoToyoBit6
		self.data_CPDread.input9 = self.welcomeScreen.statusButton.bt_cvoToyoBit7
		self.data_CPDread.input10 = self.welcomeScreen.statusButton.bt_cvoToyoBit8

	def readLineEdit(self):
		self.data_statusConveyor11.status = self.welcomeScreen.statusConveyor.cvoState11
		self.data_statusConveyor21.status = self.welcomeScreen.statusConveyor.cvoState21
		self.data_statusConveyor12.status = self.welcomeScreen.statusConveyor.cvoState12
		self.data_statusConveyor22.status = self.welcomeScreen.statusConveyor.cvoState22
		self.data_statusConveyor13.status = self.welcomeScreen.statusConveyor.cvoState13
		self.data_statusConveyor23.status = self.welcomeScreen.statusConveyor.cvoState23

	def run(self):
		# -- 

		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.controlColor()
			# --
			self.readButton()
			# --
			self.readLineEdit()

			self.pub_statusConveyor11.publish(self.data_statusConveyor11)
			self.pub_statusConveyor21.publish(self.data_statusConveyor21)
			self.pub_statusConveyor12.publish(self.data_statusConveyor12)
			self.pub_statusConveyor22.publish(self.data_statusConveyor22)
			self.pub_statusConveyor13.publish(self.data_statusConveyor13)
			self.pub_statusConveyor23.publish(self.data_statusConveyor23)

			self.pub_CPDread.publish(self.data_CPDread)
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
