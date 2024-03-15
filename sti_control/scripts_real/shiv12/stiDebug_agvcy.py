#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date: 02/3/2023

"""
import roslib
from uptime import uptime
import sys
import time
from decimal import *
import math
import rospy
from datetime import datetime
# ip
import os
import re  
import subprocess
import argparse

from message_pkg.msg import *
from sti_msgs.msg import *
from std_msgs.msg import Float64
import psutil
#--------------------------------------------------------------------------------- ROS
class debug():
	def __init__(self):
		rospy.init_node('stiDebug', anonymous=False)
		self.rate = rospy.Rate(200)
	  	# SUB - PUB
		# Charger
		# distance
		rospy.Subscriber("/quang_duong", Float64, self.distance_callback)
		self.distanceRuned = Float64()
		# -- MAIN - POWER
		# rospy.Subscriber("/POWER_info", POWER_info, self.POWER_info_callback) 
		# self.power_info = POWER_info()
		# self.is_readMain = 0

		# -- Board PSU
		rospy.Subscriber("/PSU_info", PSU_info, self.callback_PSU) 
		self.psu_info = PSU_info()
		self.is_readPsu = 0

		# - 
		rospy.Subscriber("/PSU_request", PSU_request, self.PSU_request_callback) 
		self.psu_request = PSU_request()
		self.flag_checkCharger = 0
		# - 
		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_Server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()
		self.pre_server_cmdRequest = Server_cmdRequest()

		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdCallback)
		self.NN_cmdRequest = NN_cmdRequest()
		self.pre_NN_cmdRequest = NN_cmdRequest()		
		self.is_client = 1
		self.pre_timeCharger = 0
		self.count_requirCharger = 0
		self.count_ChagerOk = 0
		# - 
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.NN_infoCallback)
		self.NN_infoRespond = NN_infoRespond()
		self.pre_NN_infoRespond = NN_infoRespond()
		# - 
		rospy.Subscriber("/navigation_respond", Navigation_respond, self.callback_navigation_respond)
		self.navigation_respond = Navigation_respond()
		self.pre_navigation_respond = Navigation_respond()
		self.log_navigation_respond = Navigation_respond()
		# - 
		rospy.Subscriber("/navigation_query", Navigation_query, self.callback_navigation_query)
		self.navigation_query = Navigation_query()
		self.pre_navigation_query = Navigation_query()

		# - 
		rospy.Subscriber("/status_goal_control", Status_goalControl, self.goalControl_callback)
		self.status_goalControl = Status_goalControl()
		self.pre_status_goalControl = Status_goalControl()
		self.is_goalControl = 0
		# - 
		rospy.Subscriber("/parking_status", Parking_status, self.parkingStatus_callback)
		self.parking_status = Parking_status()
		self.pre_parking_status = Parking_status()
		self.is_parkingStatus = 0
		# - 
		rospy.Subscriber("/reconnect_status", Reconnect_status, self.reconnectStatus_callback)
		self.reconnect_status = Reconnect_status()
		self.pre_reconnect_status = Reconnect_status()
		self.is_reconnectStatus = 0
		# -
		rospy.Subscriber("/app_setValue", App_lbv, self.appSetValue_Callback)
		self.appSetValue = App_lbv()
		self.old_appSetValue = App_lbv()
		# - NAV350
		# rospy.Subscriber("/reconnect_status", Reconnect_status, self.reconnectStatus_callback)
		# self.reconnect_status = Reconnect_status()
		# - robot pose
		# rospy.Subscriber("/reconnect_status", Reconnect_status, self.reconnectStatus_callback)
		# self.reconnect_status = Reconnect_status()
		# - 
		self.name_card = "wlo2"
		self.address = "172.21.15.224"

		# self.address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen('ip addr show wlp3s0').read()).groups()[0]

		self.path_historyDelete = "/home/stivietnam/catkin_ws/debug/SHIV34_CY1_historyDelete.txt"
		self.path_log_NN = "/home/stivietnam/catkin_ws/debug/SHIV34_CY1_log_NN.txt"
		self.path_log_standard = "/home/stivietnam/catkin_ws/debug/SHIV34_CY1_logStandard.txt"
		self.path_log_charger = "/home/stivietnam/catkin_ws/debug/SHIV34_CY1_logCharger.txt"
		self.path_log_error = "/home/stivietnam/catkin_ws/debug/SHIV34_CY1_errorLog.txt"
		self.process = 0

		self.sevenDay = 302400 # s
		self.threeDay = 129600
		self.twoWeeks = self.sevenDay*2
		self.timeDelete = self.threeDay # 60
		self.timeCheckEnbDelete = 60
		self.pre_timeCheckDelete = time.time()

		self.check_main = 0
		self.last_time_main = 0
		self.is_print_lostConnect_main = 0
		self.is_print_Reconnect_main = 0
		
		self.is_infoNN = 1

		self.battery = 0.0
		self.pre_battery = 0.0

		self.last_time_reconnect = 0
		self.preTime_infoNN = 0

		self.pre_mess = ""
		self.preTime_uptime = 0
		self.preTime_standard = 0

		self.preTime_uptime = 0
		self.preTime_standard = 0

	def callback_Server_cmdRequest(self, dat):
		self.server_cmdRequest = dat
		self.is_client = 1

	def appSetValue_Callback(self, dat):
		self.appSetValue = dat

	def distance_callback(self, dat):
		self.distanceRuned = dat

	# def POWER_info_callback(self, dat):
	# 	self.power_info = dat
	# 	self.is_readMain = 1

	def callback_PSU(self, dat):
		self.psu_info = dat
		self.is_readPsu = 1

	def PSU_request_callback(self, dat):
		self.psu_request = dat

	def NN_cmdCallback(self, dat):
		self.NN_cmdRequest = dat
		self.is_client = 1

	def NN_infoCallback(self, dat):
		self.NN_infoRespond = dat
		self.is_infoNN = 1

	def goalControl_callback(self, dat):
		self.status_goalControl = dat
		self.is_goalControl = 1

	def parkingStatus_callback(self, dat):
		self.parking_status = dat
		self.is_parkingStatus = 1

	def reconnectStatus_callback(self, dat):
		self.reconnect_status = dat
		self.is_reconnectStatus = 1

	def callback_navigation_respond(self, data):
		self.log_navigation_respond = data

		self.navigation_respond.status = data.status
		self.navigation_respond.modeMove = data.modeMove
		self.navigation_respond.completed = data.completed
		self.navigation_respond.error = data.error
		self.navigation_respond.stop = data.stop
		self.navigation_respond.process = data.process
		self.navigation_respond.typeRun = data.typeRun
		self.navigation_respond.id_goalFollow = data.id_goalFollow
		self.navigation_respond.message = data.message

	def callback_navigation_query(self, data):
		self.navigation_query = data

	def FL_infoCallback(self, dat):
		self.FL_infoRespond = dat
		self.is_main82 = 1
		self.check_main = 1
		self.is_FL_info = 1 

	def logError(self):
		mess = ''
		if (self.old_appSetValue.lbv_device != self.appSetValue.lbv_device or self.old_appSetValue.lbv_frameWork != self.appSetValue.lbv_frameWork):
			self.old_appSetValue = self.appSetValue

			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_error, "a+")
			self.file_log.write("\n------ " + str(current_time) + " ------\n")
			self.file_log.write(str(self.appSetValue))
			self.file_log.close()

	def log_checkCharger(self): # sac dc hay ko.
		if self.psu_request.charge_write == 1:
			if self.flag_checkCharger == 1:
				t = (time.time() - self.pre_timeCharger)%60
				if (t > 10):
					print ("Log Charger: ", t)
					self.pre_timeCharger = time.time()
					self.flag_checkCharger = 0
					self.count_requirCharger += 1
					cur = self.psu_info.charge_current
					now = datetime.now()
					current_time = now.strftime("%B/%d|%H:%M:%S")
					if cur > 0.4:
						self.count_ChagerOk += 1

					self.file_log = open(self.path_log_charger, "a+")
					self.file_log.write("\n" + str(self.count_requirCharger) + ", "+ "OK: " + str(self.count_ChagerOk) + " I: " + str(cur) + "| T: " + str(current_time))
					self.file_log.close()
		else:
			self.pre_timeCharger = time.time()
			self.flag_checkCharger = 1

	def log_standard(self):
		t = (time.time() - self.preTime_standard)%60
		if (t > 10): # 10 s	
			self.preTime_standard = time.time()
			now = datetime.now()
				
			self.file_log = open(self.path_log_standard, "a+")
			# - date, time
			current_time = now.strftime("%B/%d|%H:%M:%S")	
			self.file_log.write("\n" + str(current_time) )
			# - battery
			bat = round(self.psu_info.voltages, 2)
			self.file_log.write(" " + str(bat) )
			# - uptime
			times = uptime()
			hours = int(times/3600)
			minutes = int((times - hours*3600)/60)
			seconds = int(times - hours*3600 - minutes*60)
			self.file_log.write(" " + str(hours) + ":" + str(minutes) + ":" + str(seconds))
			# - distance runed
			self.file_log.write(" " + str(round(self.distanceRuned.data, 2)))
			# - db wifi
			try:
				db = subprocess.check_output("iwconfig {}".format(self.name_card), shell=True)
				vitri = str(db).find("Signal level")
				# print("vitri: ", vitri)
				db_ = str(db)[(vitri+13):(vitri+16)]
				# print("db= {}".format(float(db_)))
				self.file_log.write(" " + str(float(db_)) )
				# print("db= {}".format(float(db_)))

			except Exception:
				self.file_log.write(" " + "-1")

			# - ping server
			try:
				ping = subprocess.check_output("ping -{} 1 {}".format('c',self.address), shell=True)
				# print(ping)
				vitri = str(ping).find("time")
				time_ping = str(ping)[(vitri+5):(vitri+9)]
				self.file_log.write(" " + str(float(time_ping)) )
				# print("time= {}".format(float(time_ping)))

			except Exception:
				# print("no ping")			
				self.file_log.write(" " + "-1")

			# - temperature
			try:
				sensors = subprocess.check_output("sensors", shell=True)
				vitri = str(sensors).find("Package id")
				temperature = str(sensors)[(vitri+16):(vitri+20)]
				# print("temperature= {}".format(float(temperature)))
				self.file_log.write(" " + str(float(temperature)))

			except Exception:
				# print("no temperature")
				self.file_log.write(" " + "-1")
			# - cpu
			try:
				cpu = psutil.cpu_percent() # cpu 
				# print("cpu= {}".format(float(cpu)))
				self.file_log.write(" " + str(float(cpu)))
			except Exception:
				# print("no cpu")
				self.file_log.write(" " + "-1")

			# - RAM
			try:
				ram = psutil.virtual_memory().available * 100 / psutil.virtual_memory().total # ram
				# print("ram= {}".format(float(ram)))
				self.file_log.write(" " + str(float(ram)))
			except Exception:
				# print("no ram")
				self.file_log.write(" " + "-1")

			# - Request Charger
			try:
				req_charger = self.psu_request.charge_write
				self.file_log.write(" " + str(float(req_charger)))
			except Exception:
				# print("no ram")
				self.file_log.write(" " + "-1")

			# - Charger CURRENT
			try:
				charge_current = self.psu_info.charge_current
				self.file_log.write(" " + str(float(charge_current)))
			except Exception:
				# print("no ram")
				self.file_log.write(" " + "-1")

			self.file_log.close()

	def log_reconnect_v0(self): # ok
		t = (time.time() - self.last_time_reconnect)%60
		if (t > 1): # 1 s
			if self.is_reconnectStatus == 1:
				self.last_time_reconnect = time.time()
				self.is_reconnectStatus = 0

				now = datetime.now()
				current_time = now.strftime("%B/%d|%H:%M:%S")

				self.file_log = open(self.path_log_NN, "a+")
				self.file_log.write("\rReconnect_status| " + str(current_time))
				self.file_log.write(str(self.reconnect_status))
				self.file_log.write("\n----------------------------------------")
				self.file_log.close()

	def log_NN_info(self): # ok
		t = (time.time() - self.preTime_infoNN)%60
		if (t > 2): # 1 s
			self.preTime_infoNN = time.time()

			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\rNN info| " + str(current_time) + "\n")
			self.file_log.write(str(self.NN_infoRespond))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()

	def log_AGV_all(self):
		if (self.pre_NN_infoRespond.task_status != self.NN_infoRespond.task_status) or (self.pre_NN_infoRespond.mode != self.NN_infoRespond.mode) or (self.pre_NN_infoRespond.status != self.NN_infoRespond.status) or (self.pre_NN_infoRespond.listError != self.NN_infoRespond.listError): # ok
			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\nNN_infoRespond| ")
			self.file_log.write(str(current_time) + "\n")
			self.file_log.write(str(self.NN_infoRespond))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()
			self.pre_NN_infoRespond = self.NN_infoRespond

		if (self.pre_server_cmdRequest != self.server_cmdRequest): # ok
			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\nNN cmd request| ")
			self.file_log.write(str(current_time) + "\n")
			self.file_log.write(str(self.server_cmdRequest))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()
			self.pre_server_cmdRequest = self.server_cmdRequest

		if (self.pre_navigation_query != self.navigation_query): # ok
			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\nNavigation_query| " + str(current_time) + "\n")
			self.file_log.write(str(self.navigation_query))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()
			self.pre_navigation_query = self.navigation_query				

		if (self.pre_navigation_respond != self.navigation_respond): # ok
			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_NN, "a+")
			self.file_log.write("\nNavigation_respond| " + str(current_time) + "\n")
			self.file_log.write(str(self.log_navigation_respond))
			self.file_log.write("\n----------------------------------------")
			self.file_log.close()
			
			# self.pre_navigation_respond = self.navigation_respond
			self.pre_navigation_respond.status = self.navigation_respond.status
			self.pre_navigation_respond.modeMove = self.navigation_respond.modeMove
			self.pre_navigation_respond.completed = self.navigation_respond.completed
			self.pre_navigation_respond.error = self.navigation_respond.error
			self.pre_navigation_respond.stop = self.navigation_respond.stop
			self.pre_navigation_respond.process = self.navigation_respond.process
			self.pre_navigation_respond.typeRun = self.navigation_respond.typeRun
			self.pre_navigation_respond.id_goalFollow = self.navigation_respond.id_goalFollow
			self.pre_navigation_respond.message = self.navigation_respond.message

	def log_nav350(self):
		# -- Number Reflector - Status Nav350 - Robot Pose
		
		pass

	def log_mess(self, typ, mess, val):
		if self.pre_mess != mess:
			if typ == "info":
				rospy.loginfo (mess + ": %s", val)
			elif typ == "warn":
				rospy.logwarn (mess + ": %s", val)
			else:
				rospy.logerr (mess + ": %s", val)	
		self.pre_mess = mess

	def enb_checkDelete(self):
		t_now = time.time()
		t = (t_now - self.pre_timeCheckDelete)%60
		if (t > self.timeCheckEnbDelete): # s
			self.pre_timeCheckDelete = t_now
			# print ("check enb delete")
			return 1
		else:
			return 0

	def enb_delete(self):
		is_ok = 0
		timePre = 0
		rowNow = 0
	  # 1, Lay dong cuoi cung.
		# file_log = open(self.path_historyDelete, "a+")
		with open(self.path_historyDelete) as file_log:
			rowNow = sum(1 for _ in file_log) - 1
			# print ("rowNow11:", rowNow)
		file_log.close()
  	  # 2, Lay thoi gian cuoi cung.
		# file_log = open(self.path_historyDelete, "a+")
		with open(self.path_historyDelete) as file_log:
			raw_text = file_log.readlines()
		
		try:
			textAll = raw_text[rowNow]
			# print ("textAll:", textAll)
			
			textFinal = textAll.split('|')
			timePre = int(textFinal[0])
			# print ("TIME PRE:", timePre)
			is_ok = 1
		except:
			is_ok = -1
			print ("ERROR READ DATA")
		file_log.close()

	  # 3, So sanh thoi gian truoc va sau -> Ghi log + xoa.
		if is_ok == 1:
			if time.time() - timePre > self.timeDelete: #sevenDay
				return 1
			else:
				return 0
		else:
			return -1

	def deleteFIleLog(self):
		# -- Log history
		now = datetime.now()
		current_time = now.strftime("%B/%d|%H:%M:%S")

		file_log = open(self.path_historyDelete, "a+")
		file_log.write(str(int(time.time())) + "|" + str(current_time) +'\n')
		file_log.close()
		
		# -- delete file
		try:
			os.remove(self.path_log_error)
			print ("DELETE FILE COMPLETED LOG ERROR")
		except:
			print ("ERROR DELETE FILE LOG ERROR")

		# -- delete file 1
		try:
			os.remove(self.path_log_NN)
			print ("DELETE FILE COMPLETED LOG NN")
		except:
			print ("ERROR DELETE FILE LOG NN")

		try:
			os.remove(self.path_log_standard)
			os.system("yes | rosclean purge -y") # xoa log ROS
			print ("DELETE FILE COMPLETED LOG STANDARD")
		except:
			print ("ERROR DELETE FILE LOG STANDARD")
			
		# -- delete file 2
		try:
			os.remove(self.path_log_NN)
			print ("DELETE FILE COMPLETED LOG NN")
		except:
			print ("ERROR DELETE FILE LOG NN")

		try:
			os.remove(self.path_log_standard)
			os.system("yes | rosclean purge -y") # xoa log ROS
			print ("DELETE FILE COMPLETED LOG STANDARD")
		except:
			print ("ERROR DELETE FILE LOG STANDARD")

	def createFIleHhistoryDelete(self):
		# -- Log history
		now = datetime.now()
		current_time = now.strftime("%B/%d|%H:%M:%S")

		file_log = open(self.path_historyDelete, "a+")
		file_log.write(str(int(time.time())) + "|" + str(current_time) +'\n')
		file_log.close()
		print ("CREATE FILE HISTORY NEW")
			
	def run(self):
		while not rospy.is_shutdown():
			if self.process == 0:	# chờ cac node khoi dong xong.
				ct = 0
				if self.is_infoNN == 1:
					ct = 7
				else:
					self.log_mess("info", "Wait sti_control info NN", 0)

				if (ct == 7):
					self.log_mess("info", "All Right", 0)
					# self.file_log = open(self.path_log_charger, "a+")
					# self.file_log.write("\n-------------------------------------")
					# self.file_log.close()
					self.process = 2

			elif self.process == 2:
				if self.enb_checkDelete() == 1:
					dlt = self.enb_delete()
					if dlt == 0:
						self.process = 3
					elif dlt == 1:
						self.deleteFIleLog()
					elif dlt == -1:
						self.createFIleHhistoryDelete()
				else:
					self.process = 3

			elif self.process == 3:
				self.log_standard()
				self.log_AGV_all()
				# self.log_NN_info()
				# self.log_checkCharger()

				self.process = 2

			self.rate.sleep()

def main():
	class_1 = debug()		
	class_1.run()

if __name__ == '__main__':
	main()

