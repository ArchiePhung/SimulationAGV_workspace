#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phung Quy Duong(Archie)
Company: STI Viet Nam
date: 12/3/2024
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
from enum import Enum

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

		# self.numberConveyorA = 1
		# self.bt_cyA_received = 0
		# self.bt_cyA_stop = 0
		# self.bt_cyA_transmit = 0

		# self.numberConveyorB = 2
		# self.bt_cyB_received = 0
		# self.bt_cyB_stop = 0
		# self.bt_cyB_transmit = 0

		# -- Archie 
		self.numberConveyor = 1
		self.bt_signalCYrecieve = 0
		self.bt_signalCYstop = 0
		self.bt_signalCYtransmit = 0

		self.bt_signalCYTwinrecieve = 0
		self.bt_signalCYTwinstop = 0
		self.bt_signalCYTwintransmit = 0

		self.bt_charger = 0
		self.bt_speaker = 0
		self.bt_brake = 0

		self.bt_speakerSound_1 = 0
		self.bt_speakerSound_2 = 0
		self.bt_speakerSound_3 = 0

		self.bt_light_red = 0
		self.bt_light_green = 0
		self.bt_light_blue = 0
		self.bt_hideSetting = 0

		# self.bt_toyoWrite_enable = 0
		# self.bt_toyoWrite_1 = 0
		# self.bt_toyoWrite_2 = 0
		# self.bt_toyoWrite_3 = 0
		# self.bt_toyoWrite_4 = 0
		# self.bt_toyoWrite_5 = 0
		# self.bt_toyoWrite_6 = 0
		# self.bt_toyoWrite_7 = 0
		# self.bt_toyoWrite_8 = 0
		# self.ck_enable_link = 0
		# -
		self.bt_linkConveyor = 0
		self.bt_resetFrameWork = 0

		self.bt_remote = 0

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

		# self.lbc_toyo_0 = 0
		# self.lbc_toyo_1 = 0
		# self.lbc_toyo_2 = 0
		# self.lbc_toyo_3 = 0
		# self.lbc_toyo_4 = 0
		# self.lbc_toyo_5 = 0
		# self.lbc_toyo_6 = 0
		# self.lbc_toyo_7 = 0

		self.lbc_toyoReadBit1 = 0
		self.lbc_toyoReadBit2 = 0
		self.lbc_toyoReadBit3 = 0
		self.lbc_toyoReadBit4 = 0
		self.lbc_toyoReadBit5 = 0
		self.lbc_toyoReadBit6 = 0
		self.lbc_toyoReadBit7 = 0
		self.lbc_toyoReadBit8 = 0	
		# --
		self.lbc_button_clearError = 0
		self.lbc_button_power = 0
		self.lbc_blsock1 = 0
		self.lbc_blsock2 = 0
		self.lbc_emg1 = 0
		self.lbc_emg2 = 0
		self.lbc_emg3 = 0
		self.lbc_safetyRelay = 0

		# self.lbc_cyA_ssAhead = 0
		# self.lbc_cyA_ssBehind = 0
		# self.lbc_cyA_ssTray = 0

		# self.lbc_cyB_ssAhead = 0
		# self.lbc_cyB_ssBehind = 0
		# self.lbc_cyB_ssTray = 0

		self.lbc_CYssAheadSignal = 0
		self.lbc_CYssBehindSignal = 0
		self.lbc_CYssTraySignal = 0
		
		self.lbc_CYcntedSignal = 0
		self.lbc_CYdoneSignal = 0		
		self.lbc_CYreadySignal = 0
		self.lbc_CYerrorSignal = 0

		self.lbc_CYTwinssAheadSignal1 = 0
		self.lbc_CYTwinssBehindSignal1 = 0
		self.lbc_CYTwinssTraySignal1 = 0
		self.lbc_CYTwinssAheadSignal2 = 0
		self.lbc_CYTwinssBehindSignal2 = 0
		self.lbc_CYTwinssTraySignal2 = 0

		self.lbc_CYTwindoneSignal = 0		
		self.lbc_CYTwinreadySignal = 0
		self.lbc_CYTwinerrorSignal = 0

		self.lbc_port_rtc = 0
		self.lbc_port_rs485 = 0
		self.lbc_port_nav350 = 0
		# - 
		self.lbc_mission_cy11 = 0
		self.lbc_mission_cy12 = 0
		self.lbc_mission_cy21 = 0
		self.lbc_mission_cy22 = 0
		# self.lbc_mission_charge = 0

		# self.lbc_safetyConveyor1 = 0
		# self.lbc_safetyConveyor2 = 0

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

		self.lbv_CYname = ''
		self.lbv_CYTwinname = ''
		self.lbv_CYTwinssname1 = ''
		self.lbv_CYTwinssname2 = ''

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

		# -- pin details
		self.lbv_pinStatus = ''
		self.lbv_pinVolt = ''
		self.lbv_pinCurr = ''
		self.lbv_pinPercent = ''
		self.lbv_pinTimeCharge = ''
		self.lbv_pinTimeChargePropose = ''

		# -- mission
		self.lbv_mission = ''

		# -- Main board Temperature
		self.lbv_tempcpu = ''

		self.listPath = []
		self.listPath_pre = []
	
class TypeLine(Enum):
    STRAIGHTLINE = 1
    CIRCLELINE = 2
    BEZIERLINE = 3

class infoPath:
	def __init__(self):
		self.typeLine = TypeLine
		self.startPoint = Point()
		self.endPoint = Point()
		self.midPoint = Point()
		self.isTarget = False

class CanvasWidget(QWidget):
	def __init__(self):
		super().__init__()
		self.listPath = []
		self.xAGV = 0.
		self.yAGV = 0.
		self.rAGV = 0.

		self.isUpdatePath = False
		self.isUpdateAGV = False

		self.offset_width = 20 
		self.offset_height = 20
		self.dentaX = 0
		self.dentaY = 0
		self.max_X = 0.
		self.max_Y = 0.
		self.min_X = 0.
		self.min_Y = 0.
		self.min = 0.0
		self.WIDTH = 960 #m
		self.HEIGHT = 520 #m
		self.SCREEN_WIDTH = 0.153 #m
		self.SCREEN_HEIGHT = 0.09 #m
		self.METER_TO_PIXEL = 3779.5275590551
		self.SCALE_COEF = 1
		self.BORDER = 10

		# path image
		self.pathIconTarget = '/home/tiger/simulation_ws/src/app_ros600/interface/toado.png'
		self.pathIconAGV = '/home/tiger/simulation_ws/src/app_ros600/interface/iconmuiten_resize_downside.png'

	def convert_coordinates(self, x, y):
		x_new = x + int(self.BORDER/2)
		y_new = int(self.BORDER/2) - y

		return (x_new, y_new)
				    
	def makePath(self, listPath):
		self.listPath = []
		max_X = 0.
		max_Y = 0.
		min_X = 1000.
		min_Y = 1000.

		n = len(listPath)
		if n > 0:
			for p in listPath:
				if max_X < p.startPoint.x:
					max_X = p.startPoint.x

				if min_X > p.startPoint.x:
					min_X = p.startPoint.x

				if max_Y < p.startPoint.y:
					max_Y = p.startPoint.y
					
				if min_Y > p.startPoint.y:
					min_Y = p.startPoint.y

				if max_X < p.endPoint.x:
					max_X = p.endPoint.x

				if min_X > p.endPoint.x:
					min_X = p.endPoint.x

				if max_Y < p.endPoint.y:
					max_Y = p.endPoint.y
					
				if min_Y > p.endPoint.y:
					min_Y = p.endPoint.y

			self.max_X = max_X
			self.min_X = min_X
			self.max_Y = max_Y
			self.min_Y = min_Y
			self.dentaX = max_X - min_X
			print("Giá trị của max_X là: ", max_X)
			print("Giá trị của min_X là: ", min_X)
			print("Giá trị của delta_X là: ", self.dentaX)

			print('-------------------------------')

			self.dentaY = max_Y - min_Y
			print("Gía trị của max Y là: ", max_Y)
			print("Gía trị của min Y là: ", min_Y)
			print("Giá trị của delta_Y là: ", self.dentaY)

			# print("Gía trị của geometry width là: ", self.geometry().width())
			# print("Gía trị của geometry heigh là: ", self.geometry().height())
			self.min = self.dentaY
			if self.min > self.dentaX:
				self.min = self.dentaX
			count = 0

			for p in listPath:
				cvPath = infoPath()
				cvPath.typeLine = p.typeLine
				cvPath.isTarget = p.isTarget
				if self.dentaX != 0 and self.dentaY != 0:
					cvPath.startPoint.x = self.offset_width + int(((p.startPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
					cvPath.startPoint.y = self.offset_height + int(((p.startPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

					cvPath.endPoint.x = self.offset_width + int(((p.endPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
					cvPath.endPoint.y = self.offset_height + int(((p.endPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

				elif self.dentaX == 0 and self.dentaY != 0:
					cvPath.startPoint.x = int(self.geometry().width()/2)
					cvPath.startPoint.y = self.offset_height + int(((p.startPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

					cvPath.endPoint.x = int(self.geometry().width()/2)
					cvPath.endPoint.y = self.offset_height + int(((p.endPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

				elif self.dentaX != 0 and self.dentaY == 0:
					cvPath.startPoint.x = self.offset_width + int(((p.startPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
					cvPath.startPoint.y = int(self.geometry().height()/2)

					cvPath.endPoint.x = self.offset_width + int(((p.endPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
					cvPath.endPoint.y = int(self.geometry().height()/2)
				
				count += 1

				# print("start origin Point: ", p.startPoint)
				# print("end origin Point: ", p.endPoint)
				# print('---------********--------------')

				print("*************************************************")
				print("Lần đọc thứ: ", count)
				print("start Point X : ", cvPath.startPoint.x)
				print("start Point Y : ", cvPath.startPoint.y)
				print("End Point X: ", cvPath.endPoint.x)
				print("End Point Y: ", cvPath.endPoint.y)	

				self.listPath.append(cvPath)		
			print("Độ dài của list path là:", len(self.listPath))

	def makeAGV(self, x_rb, y_rb, r_rb):
		if self.dentaX != 0 and self.dentaY != 0:
			self.xAGV = self.offset_width + int(((x_rb - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
			self.yAGV = self.offset_height + int(((y_rb - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))
			self.rAGV = r_rb + 90.

		elif self.dentaX != 0 and self.dentaY == 0:
			self.xAGV = self.offset_width + int(((x_rb - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
			self.yAGV = int(self.geometry().height()/2)
			self.rAGV = r_rb + 90.			

		elif self.dentaX == 0 and self.dentaY != 0:
			self.xAGV = int(self.geometry().width()/2)
			self.yAGV = self.offset_height + int(((y_rb - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))
			self.rAGV = r_rb + 90.		

	def makePath_v2(self, listPath, r):
		self.listPath = []
		n = len(listPath)
		if n > 0:
			count = 1
			x_cv = 0
			y_cv = 0
			for p in listPath:
				print("Lần đọc thứ: ", count)
				cvPath = infoPath()
				cvPath.typeLine = p.typeLine
				cvPath.isTarget = p.isTarget
				# (x_cv, y_cv) = self.convert_coordinates(p.startPoint.x, p.startPoint.y)
				x_cv = p.startPoint.x + int(r/2)
				y_cv = int(r/2) - p.startPoint.y
				print("start origin Point : ", x_cv)
				print("start origin Point : ", y_cv)
				print("----------------------------")

				cvPath.startPoint.x = self.offset_width + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.offset_width)/r)
				cvPath.startPoint.y = self.offset_height + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.offset_height)/r)

				# x_cv = p.startPoint.x + int(r/2)
				# y_cv = int(r/2) - p.startPoint.y
				# cvPath.startPoint.x = int(x_cv*self.SCALE_COEF*self.WIDTH/r)
				# cvPath.startPoint.y = int(y_cv*self.SCALE_COEF*self.HEIGHT/r)

				print("start Point x: ", cvPath.startPoint.x)
				print("start Point y: ", cvPath.startPoint.y)
				print("*****************************")

				# (x_cv, y_cv) = self.convert_coordinates(p.endPoint.x, p.endPoint.y)
				x_cv = p.endPoint.x + int(r/2)
				y_cv = int(r/2) - p.endPoint.y
				print("End origin Point : ", x_cv)
				print("End origin Point : ", y_cv)
				print("----------------------------")
				cvPath.endPoint.x = self.offset_width + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.offset_width)/r)
				cvPath.endPoint.y = self.offset_height + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.offset_height)/r)
				# x_cv = p.endPoint.x + int(r/2)
				# y_cv = int(r/2) - p.endPoint.y
				# cvPath.endPoint.x = int(x_cv*self.SCALE_COEF*self.WIDTH/r)
				# cvPath.endPoint.y = int(y_cv*self.SCALE_COEF*self.HEIGHT/r)

				print("End Point x: ", cvPath.endPoint.x)
				print("End Point y: ", cvPath.endPoint.y)
				print("*****************************")

				count += 1

				self.listPath.append(cvPath)

		else:
			print("Danh sách list Path trống")			

	def makeAGV_v2(self, x_rb, y_rb, r_rb, r):
		# (x_cv, y_cv) = self.convert_coordinates(x_rb, y_rb)
		x_cv = x_rb + int(r/2)
		y_cv = int(r/2) - y_rb
		self.xAGV = self.offset_width + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.offset_width)/r)
		self.yAGV = self.offset_height + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.offset_height)/r)
		# x_cv = x_rb + int(r/2)
		# y_cv = int(r/2) - y_rb
		# self.xAGV = int(x_cv*self.SCALE_COEF*self.WIDTH/r)
		# self.yAGV = int(y_cv*self.SCALE_COEF*self.HEIGHT/r)		
		self.rAGV = 270 - r_rb
	
	def paintEvent(self, event):
        # Create a painting object
		ThePainter = QPainter()
		# Do the painting
		ThePainter.begin(self) # everything between here and the end() call will be drawn into the Widget
		if self.isUpdatePath:
			self.isUpdatePath = False
			ThePen=QPen(Qt.green) # set a color for a pen.  Pens draw lines and outlines
			ThePen.setWidth(10)
			ThePainter.setPen(ThePen) # set the pen into the painter
			ThePainter.drawPoint(self.listPath[0].startPoint.x, self.listPath[0].startPoint.y)

			for path in self.listPath:
				if path.isTarget:
					pixmap = QPixmap(self.pathIconTarget)
					ThePainter.drawPixmap(path.endPoint.x - 15, path.endPoint.y - 30, pixmap)
				else:
					ThePen.setWidth(10)
					ThePainter.setPen(ThePen) # set the pen into the painter
					ThePainter.drawPoint(path.endPoint.x, path.endPoint.y)

				ThePen.setWidth(2)
				ThePainter.setPen(ThePen) # set the pen into the painter
				# if path.typeLine == TypeLine.STRAIGHTLINE:
				ThePainter.drawLine(path.startPoint.x, path.startPoint.y, path.endPoint.x, path.endPoint.y)

				# elif path.typeLine == TypeLine.BEZIERLINE:
				# 	pathCubic = QPainterPath()
				# 	pathCubic.moveTo(path.startPoint.x, path.startPoint.y)
				# 	pathCubic.cubicTo(path.startPoint.x, path.startPoint.y, path.midPoint.x, path.midPoint.y, path.endPoint.x, path.endPoint.y)
				# 	ThePainter.drawPath(pathCubic)

		if self.isUpdateAGV:
			self.isUpdateAGV = False
			pixmap = QPixmap(self.pathIconAGV)
			transform = QTransform().rotate(self.rAGV)
			pixmap = pixmap.transformed(transform)
			ThePainter.drawPixmap(self.xAGV - 12, self.yAGV - 12, pixmap)

		ThePainter.end() # finishing drawing
	
class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/tiger/simulation_ws/src/app_ros600/interface/app_agv12CY_testTranjectory.ui", self)
		# --
		self.statusButton = statusButton()
		self.statusColor  = statusColor()
		self.valueLable   = valueLable()
		# --
		self.statusButton.bt_speaker = 1
		# --
        # Add a box layout into the frame
		SettingsLayout = QVBoxLayout()
		self.fr_pathPlan.setLayout(SettingsLayout)
		self.TheCanvas = CanvasWidget() # create the canvas widget that will do the painting
		SettingsLayout.addWidget(self.TheCanvas)  # add the canvas to the layout within the frame
		SettingsLayout.setContentsMargins(0, 0, 0, 0)
	
		# -- button
		self.pb_zoomIn.clicked.connect(self.clicked_zoomIn)
		self.pb_zoomOut.clicked.connect(self.clicked_zoomOut)

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
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRuning = self.modeRun_byhand    # Archie: = modeRun_launch
		# --
		self.password_data = ""
		# --
		self.timeSave_cancelMisson = rospy.Time.now()
		self.cancelMission_status = 0
		# -- 
		self.isShow_setting = 0
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
		self.isShow_testCY = 1
		# - 
		self.ratio = 10
		self.delta_ratio = 5

	def out(self):
		QApplication.quit()
		print('out')

	def process_fast(self):
		self.pb_qualityWifi.setValue(self.valueLable.lbv_qualityWifi)

	def process_normal(self):
		# self.set_dateTime()
		self.lbv_ip.setText(self.valueLable.lbv_ip)
		self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
		# self.lbv_mac.setText(self.valueLable.lbv_mac)
		# self.lbv_namePc.setText(self.valueLable.lbv_namePc)

		# --
		# if self.valueLable.listPath != self.valueLable.listPath_pre:
		self.TheCanvas.isUpdatePath = True
		# self.valueLable.listPath_pre = self.valueLable.listPath

		self.TheCanvas.makePath_v2(self.valueLable.listPath, self.ratio)
		self.TheCanvas.isUpdateAGV = True
		self.TheCanvas.makeAGV_v2(self.valueLable.lbv_coordinates_x, self.valueLable.lbv_coordinates_y, self.valueLable.lbv_coordinates_r, self.ratio)
		self.fr_pathPlan.update()

	def clicked_zoomIn(self):
		if self.ratio < 50:
			print("Nút phóng to đã được nhấn ")
			self.ratio = self.ratio + self.delta_ratio
	# - 
	def clicked_zoomOut(self):
		if self.ratio > 10:
			print("Nút thu nhỏ đã được nhấn ")
			self.ratio = self.ratio - self.delta_ratio

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
		rospy.init_node('app_node_tranjectory', anonymous=False)
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
		# self.widget.setWindowFlag(Qt.FramelessWindowHint)             # Archie need to uncomment in final
		# --
		self.valueLable = valueLable()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# -----------------------------------------------------------

		# -- Pose robot
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPose) 
		self.robotPose_nav = PoseStamped()

		# -- Traffic cmd
		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()

		# -- Navigation query 
		rospy.Subscriber("/navigation_query", Navigation_query, self.callback_Navigation_query)
		self.data_navigation_query = Navigation_query()
		self.data_PathInfo = PathInfo()
		self.dataRequestMove = LineRequestMove()

		self.name_agv = ""
		self.ip_agv = ""
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		# --
		# self.app_button.bt_speaker = 1

		self.listpath = []
		self.is_firstrun = 1
		self.is_receiveQuery = 0
		self.preGoalID = 0

	def callback_robotPose(self, data):
		self.robotPose_nav = data

	def callback_server_cmdRequest(self, data):
		self.server_cmdRequest = data

	def callback_Navigation_query(self, data):
		self.data_navigation_query = data
		self.is_receiveQuery = 1
		
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
	
	def convertMsg_NavigationQuery_to_LineRequestMove(self):
		p = PathInfo()
		# n = len(self.data_navigation_query.listX)
		# print("Độ dài list nhận được là:", n)
		# for i in range(n-2):
		if self.data_navigation_query.GoalID != self.preGoalID:
			self.dataRequestMove = LineRequestMove()
			self.preGoalID = self.data_navigation_query.GoalID
			n = len(self.data_navigation_query.listX)
			# print("Độ dài list nhận được là:", n)
			for i in range(n):		# i run from 0 to 4
				if self.data_navigation_query.listID[i] != 0:
					p.typePath = 1
					p.pointOne.position.x = self.data_navigation_query.listX[i]
					p.pointOne.position.y = self.data_navigation_query.listY[i]

					p.pointSecond.position.x = self.data_navigation_query.listX[i+1]
					p.pointSecond.position.y = self.data_navigation_query.listY[i+1]

					self.dataRequestMove.pathInfo.append(p)
					p = PathInfo()
				else:
					break

			self.dataRequestMove.pathInfo.pop()
			# print("Độ dài Mảng chuyển đổi là:", len(self.dataRequestMove.pathInfo))

	def getListPath(self):
		# if self.dataRequestMove.enable:
			# numPath = len(self.dataRequestMove.pathInfo)
		if self.is_receiveQuery == 1:
			self.convertMsg_NavigationQuery_to_LineRequestMove()

			if self.listpath != self.dataRequestMove.pathInfo:
				self.listpath = self.dataRequestMove.pathInfo

				self.valueLable.listPath = []

				for p in self.listpath:
					ipath = infoPath()

					if p.pointSecond.position.x == self.data_navigation_query.GoalX and p.pointSecond.position.y == self.data_navigation_query.GoalY:
						ipath.isTarget = True
						
					if p.typePath == 1:
						ipath.typeLine = TypeLine.STRAIGHTLINE
						ipath.startPoint = Point(p.pointOne.position.x, p.pointOne.position.y, 0.)
						ipath.endPoint = Point(p.pointSecond.position.x, p.pointSecond.position.y, 0.)
						self.valueLable.listPath.append(ipath)

	def controlAll(self):
		# -- 
		self.valueLable.lbv_coordinates_x = round(self.robotPose_nav.pose.position.x, 3)
		self.valueLable.lbv_coordinates_y = round(self.robotPose_nav.pose.position.y, 3)
		angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
		
		if angle < 0:
			angle_robot = 2*pi + angle
		else:
			angle_robot = angle
		self.valueLable.lbv_coordinates_r = round(degrees(angle_robot), 3)

		self.getListPath()
	
	def run(self):
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.controlAll()
			# ----------------------
			self.welcomeScreen.valueLable = self.valueLable
			# -- 
			self.welcomeScreen.robotPoseNow = self.robotPose_nav.pose
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
