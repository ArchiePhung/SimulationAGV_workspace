#!/usr/bin/env python3

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

class launch_variable:
	lbc_main = 0
	lbc_hc = 0
	lbc_oc = 0
	lbc_load = 0
	lbc_imu = 0
	lbc_lidar = 0
	lbc_driver = 0
	lbv_notification = ""
	pb_launch = 0
	lbv_pos = ""

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/stivietnam/catkin_ws/src/app_ros/interface/app_launch.ui", self)
		# --
		# self.setWindowTitle("my name")
		# --
		# self.gb_checkDevice.hide()
		# self.gb_listTask.hide()
		# -- 
		# self.status_button = App_button()
		# self.data_color = App_color()
		# self.data_labelValue = App_lbv()
		# -- -- -- 
		# self.bt_exit.pressed.connect(self.out)
		# -

		# -- Set property
		# self.lbv_device.setStyleSheet("color: red;")
		# self.lbv_frameWork.setStyleSheet("color: orange;")
		# --
		
		# -- -- -- Timer updata data
		# -- Fast
		timer_fast = QTimer(self)
		timer_fast.timeout.connect(self.process_fast)
		timer_fast.start(400)
		self.variable_launch = launch_variable()

	def process_fast(self):
		self.set_labelColor()
		# --``
		self.set_labelValue()

	def set_labelColor(self):
		# ---- Main board
		if (self.variable_launch.lbc_main == 1):
			self.lbc_main.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_main.setStyleSheet("background-color: red; color: white;")	
		# ---- HC board
		if (self.variable_launch.lbc_hc == 1):
			self.lbc_hc.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_hc.setStyleSheet("background-color: red; color: white;")
		# ---- OC board
		if (self.variable_launch.lbc_oc == 1):
			self.lbc_oc.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_oc.setStyleSheet("background-color: red; color: white;")
		# ---- lbc_load
		if (self.variable_launch.lbc_load == 1):
			self.lbc_load.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_load.setStyleSheet("background-color: red; color: white;")
		# ---- lbc_imu
		if (self.variable_launch.lbc_imu == 1):
			self.lbc_imu.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_imu.setStyleSheet("background-color: red; color: white;")
		# ---- lbc_lidar
		if (self.variable_launch.lbc_lidar == 1):
			self.lbc_lidar.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_lidar.setStyleSheet("background-color: red; color: white;")
		# ---- lbc_driver
		if (self.variable_launch.lbc_driver == 1):
			self.lbc_driver.setStyleSheet("background-color: green; color: white;")
		else:
			self.lbc_driver.setStyleSheet("background-color: red; color: white;")

	def set_labelValue(self): # 
		self.pb_launch.setValue(self.variable_launch.pb_launch)
		self.lbv_notification.setText(self.variable_launch.lbv_notification)
		self.lbv_pos.setText(self.variable_launch.lbv_pos)

	def out(self):
		QApplication.quit()
		print('out')
		
class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()

		rospy.init_node('app_launch', anonymous=False)
		self.rate = rospy.Rate(10)

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
		rospy.Subscriber("/status_port", Status_port, self.callBack_port)
		self.status_port = Status_port()

		rospy.Subscriber("/status_launch", Status_launch, self.callBack_launch)
		self.status_launch = Status_launch()

		# self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)


	def callBack_launch(self, data):
		self.status_launch = data

	def callBack_port(self, data):
		self.status_port = data

	def callBack_setColor(self, data):
		self.app_setColor = data

	def run_screen(self):
		self.widget.show()
		try:
			sys.exit(self.app.exec_())
		except:
			pass
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	def run(self):
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.welcomeScreen.variable_launch.lbc_main = self.status_port.main
			self.welcomeScreen.variable_launch.lbc_hc = self.status_port.hc
			self.welcomeScreen.variable_launch.lbc_oc = self.status_port.oc
			self.welcomeScreen.variable_launch.lbc_load = self.status_port.loadcell
			self.welcomeScreen.variable_launch.lbc_imu = self.status_port.imu
			self.welcomeScreen.variable_launch.lbc_lidar = self.status_port.nav350
			self.welcomeScreen.variable_launch.lbc_driver = self.status_port.driverall
			
			self.welcomeScreen.variable_launch.pb_launch = self.status_launch.persent
			self.welcomeScreen.variable_launch.lbv_pos = str(self.status_launch.position)
			self.welcomeScreen.variable_launch.lbv_notification = self.status_launch.notification
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