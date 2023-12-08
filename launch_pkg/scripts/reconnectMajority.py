#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Authors : BEE
# DATE: 01/07/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import roslaunch
import os
import subprocess

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan , Image, Imu

from sti_msgs.msg import *
from message_pkg.msg import *
"""
yêu cầu thông tin để kết nối lại 1 node:
1, Topic để kiểm tra node đó có đang hoạt động không.
2, Tên node để shutdown node đó.
3, File launch khởi tạo node.
----------------
Nếu cổng vật lý vẫn còn.
"""

class Reconnect:
	def __init__(self, nameTopic_sub, time_checkLost, time_waitLaunch, file_launch):
		# -- parameter
		self.time_checkLost = time_checkLost
		self.time_waitLaunch = time_waitLaunch # wait after launch
		self.fileLaunch = file_launch
		self.nameTopic_sub = nameTopic_sub
		self.nameNode = ''
		self.is_nameReaded = 0
        # -- launch
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		# -- variable
		self.lastTime_waitConnect = time.time()
		self.time_readed = time.time()
		self.enable_check = 0
		self.process = 0
		self.numberReconnect = 0
		# -- 
		self.lastTime_waitShutdown = time.time()
		
	def read_nameNode(self, topic):
		try:
			output = subprocess.check_output("rostopic info {}".format(topic), shell= True)
			# print("out: ", output)

			pos1 = str(output).find('Publishers:') # tuyet doi ko sua linh tinh.
			pos2 = str(output).find(' (http://')   # tuyet doi ko sua linh tinh.
			# print("pos2: ", pos2)
			if (pos1 >= 0):
				name = str(output)[pos1 + 16 :pos2]
				p1 = name.find('Subscribers:')
				p2 = name.find('*')
				if (p1 == -1 and p2 == -1):
					return name
				else:
					print ("p1: ", p1)
					print ("p2: ", p2)
					print ("Read name error: " + name + "|" + self.nameTopic_sub)
					return ''
			else:
				# print ("Not!")
				return ''

		except Exception as e:
			# print ("Error!")
			return ''

	def run_reconnect_vs2(self, enb_check, time_readed): # have kill node via name_topic pub.
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = self.read_nameNode(self.nameTopic_sub)
			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (self.is_nameReaded):
				# self.nameNode = "/lineMagnetic"
				os.system("rosnode kill " + self.nameNode)
				print ("rosnode kill " + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect_vs3(self, enb_check, time_readed, off_kill): # have kill node via name_topic pub. + Add funtion turn of rosnode kill
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = self.read_nameNode(self.nameTopic_sub)
			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (off_kill == 0):
				if (self.is_nameReaded):
					# self.nameNode = "/lineMagnetic"
					os.system("rosnode kill " + self.nameNode)
					print ("rosnode kill " + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect(self, enb_check, time_readed): 
		self.enable_check = enb_check
		self.time_readed = time_readed

		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))
			self.lastTime_waitShutdown = time.time()

			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				
		return self.process, self.numberReconnect

class Reconnect_node():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('reconnect_try', anonymous=False)
		self.rate = rospy.Rate(100)
		# -- Port
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.port_status = Status_port()
		# -- PUB
		self.pub_statusReconnect = rospy.Publisher('/status_reconnect', Status_reconnect, queue_size= 10)
		self.statusReconnect = Status_reconnect()
		self.pre_timePub = time.time()
		self.cycle_timePub = 0.1 # s
		# ---------------------
		# -- -- Main 
		self.path_main = rospy.get_param("path_main", '')
		self.timeLost_main = rospy.get_param("timeLost_main", 3)
		self.timeWait_main = rospy.get_param("timeWait_main", 5)
		self.topicSub_main = "/POWER_info"
		self.isRuned_main = 0
		self.timeReaded_main = time.time()
		rospy.Subscriber(self.topicSub_main, POWER_info, self.callback_main)
		# -- .
		self.reconnect_main = Reconnect(self.topicSub_main, self.timeLost_main, self.timeWait_main, self.path_main)
		# ---------------------
		# -- -- OC 
		self.path_oc = rospy.get_param("path_oc", '')
		self.timeLost_oc = rospy.get_param("timeLost_oc", 3)
		self.timeWait_oc = rospy.get_param("timeWait_oc", 5)
		self.topicSub_oc = "/lift_status"		
		self.isRuned_oc = 0
		self.timeReaded_oc = time.time()
		rospy.Subscriber(self.topicSub_oc, Lift_status, self.callback_oc)
		# -- .
		self.reconnect_oc = Reconnect(self.topicSub_oc, self.timeLost_oc, self.timeWait_oc, self.path_oc)
		# ---------------------
		# -- -- HC 
		self.path_hc = rospy.get_param("path_hc", '')
		self.timeLost_hc = rospy.get_param("timeLost_hc", 3)
		self.timeWait_hc = rospy.get_param("timeWait_hc", 5)
		self.topicSub_hc = "/HC_info"		
		self.isRuned_hc = 0
		self.timeReaded_hc = time.time()
		rospy.Subscriber(self.topicSub_hc, HC_info, self.callback_hc)
		# -- .
		self.reconnect_hc = Reconnect(self.topicSub_hc, self.timeLost_hc, self.timeWait_hc, self.path_hc)
		# ---------------------
		# -- -- IMU
		self.path_imu = rospy.get_param("path_imu", '')
		self.timeLost_imu = rospy.get_param("timeLost_imu", 3)
		self.timeWait_imu = rospy.get_param("timeWait_imu", 5)
		self.topicSub_imu = "/imu/data"		
		self.isRuned_imu = 0
		self.timeReaded_imu = time.time()
		rospy.Subscriber(self.topicSub_imu, Imu, self.callback_imu)
		# -- .
		self.reconnect_imu = Reconnect(self.topicSub_imu, self.timeLost_imu, self.timeWait_imu, self.path_imu)
		# ---------------------
		# -- -- NAV350 
		self.path_nav350 = rospy.get_param("path_nav350", '')
		self.timeLost_nav350 = rospy.get_param("timeLost_nav350", 3)
		self.timeWait_nav350 = rospy.get_param("timeWait_nav350", 5)
		self.topicSub_nav350 = "/nav350_data"		
		self.isRuned_nav350 = 0
		self.timeReaded_nav350 = time.time()
		rospy.Subscriber(self.topicSub_nav350, Nav350_data, self.callback_nav350)
		# -- .
		self.reconnect_nav350 = Reconnect(self.topicSub_nav350, self.timeLost_nav350, self.timeWait_nav350, self.path_nav350)
		# ---------------------
		# -- -- Driverall ------------------------------------------------
		self.path_driverAll = rospy.get_param("path_driverAll", '')
		# -- -- -- Driver1
		self.timeLost_driver1 = rospy.get_param("timeLost_driver", 3)
		self.timeWait_driver1 = rospy.get_param("timeWait_driver", 5)
		self.topicSub_driver1 = "/driver1_respond"
		self.isRuned_driver1 = 0
		self.timeReaded_driver1 = time.time()
		rospy.Subscriber(self.topicSub_driver1, Driver_respond, self.callback_driver1)
		# -- .
		self.reconnect_driver1 = Reconnect(self.topicSub_driver1, self.timeLost_driver1, self.timeWait_driver1, self.path_driverAll)
		# -- -- Driver2 
		# self.timeLost_driver2 = rospy.get_param("timeLost_driver2", 3)
		# self.timeWait_driver2 = rospy.get_param("timeWait_driver2", 5)
		# self.topicSub_driver2 = "/driver2_respond"
		# self.isRuned_driver2 = 0
		# self.timeReaded_driver2 = time.time()
		# self.nameNode_driver2 = ''
		# rospy.Subscriber(self.topicSub_driver2, Driver_respond, self.callback_driver2)
		# -- .
		# self.reconnect_driver2 = Reconnect(self.topicSub_driver2, self.timeLost_driver2, self.timeWait_driver2, self.path_driverAll)
		# -- -- LoadCell
		# self.timeLost_loadcell = rospy.get_param("timeLost_loadcell", 3)
		# self.timeWait_loadcell = rospy.get_param("timeWait_loadcell", 5)
		# self.topicSub_loadcell = "/driver2_respond"
		# self.isRuned_loadcell = 0
		# self.timeReaded_loadcell = time.time()
		# self.nameNode_loadcell = ''
		# ---------------------------------------------- Close driver ALL ----------------------------
		# ---------------------
		# -- -- Parking
		self.path_parking = rospy.get_param("path_parkingControl", '')
		self.timeLost_parking = rospy.get_param("timeLost_parking", 3)
		self.timeWait_parking = rospy.get_param("timeWait_parking", 5)
		self.topicSub_parking = "/parking_respond"		
		self.isRuned_parking = 0
		self.timeReaded_parking = time.time()
		rospy.Subscriber(self.topicSub_parking, Parking_respond, self.callback_parking)
		# -- 
		self.reconnect_parking = Reconnect(self.topicSub_parking, self.timeLost_parking, self.timeWait_parking, self.path_parking)

	def callback_port(self, data):
		self.port_status = data

	def callback_main(self, data):
		self.isRuned_main = 1
		self.timeReaded_main = time.time()

	def callback_oc(self, data):
		self.isRuned_oc = 1
		self.timeReaded_oc = time.time()

	def callback_hc(self, data):
		self.isRuned_hc = 1
		self.timeReaded_hc = time.time()

	# def callback_loadcell(self, data):
	# 	self.isRuned_loadcell = 1
	# 	self.timeReaded_loadcell = time.time()

	def callback_imu(self, data):
		self.isRuned_imu = 1
		self.timeReaded_imu = time.time()

	def callback_nav350(self, data):
		self.isRuned_nav350 = 1
		self.timeReaded_nav350 = time.time()

	def callback_driver1(self, data):
		self.isRuned_driver1 = 1
		self.timeReaded_driver1 = time.time()

	# def callback_driver2(self, data):
	# 	self.isRuned_driver2 = 1
	# 	self.timeReaded_driver2 = time.time()
		
	def callback_parking(self, data):
		self.isRuned_parking = 1
		self.timeReaded_parking = time.time()

	def reconnect_node(self):
		# -- main
		process, num = self.reconnect_main.run_reconnect_vs2(self.isRuned_main, self.timeReaded_main)
		self.statusReconnect.main.sts = process
		self.statusReconnect.main.times = num
		# -- oc
		process, num = self.reconnect_oc.run_reconnect_vs2(self.isRuned_oc, self.timeReaded_oc)
		self.statusReconnect.oc.sts = process
		self.statusReconnect.oc.times = num
		# -- hc
		process, num = self.reconnect_hc.run_reconnect_vs2(self.isRuned_hc, self.timeReaded_hc)
		self.statusReconnect.hc.sts = process
		self.statusReconnect.hc.times = num
		# -- imu
		process, num = self.reconnect_imu.run_reconnect_vs2(self.isRuned_imu, self.timeReaded_imu)
		self.statusReconnect.imu.sts = process
		self.statusReconnect.imu.times = num
		# -- nav350
		process, num = self.reconnect_nav350.run_reconnect_vs2(self.isRuned_nav350, self.timeReaded_nav350)
		self.statusReconnect.lidar.sts = process
		self.statusReconnect.lidar.times = num
		# -- driverAll
		process, num = self.reconnect_driver1.run_reconnect_vs2(self.isRuned_driver1, self.timeReaded_driver1)
		self.statusReconnect.driverAll.sts = process
		self.statusReconnect.driverAll.times = num
		# -- driver2
		# process, num = self.reconnect_driver2.run_reconnect_vs2(self.isRuned_driver2, self.timeReaded_driver2)
		# self.statusReconnect.driver2.sts = process
		# self.statusReconnect.driver2.times = num
		# -- parking
		process, num = self.reconnect_parking.run_reconnect(self.isRuned_parking, self.timeReaded_parking)
		self.statusReconnect.parking.sts = process
		self.statusReconnect.parking.times = num

		# -- -- -- 
		tim = (time.time() - self.pre_timePub)%60
		if (tim > self.cycle_timePub):
			self.pre_timePub = time.time()
			self.pub_statusReconnect.publish(self.statusReconnect)

	def run(self):
		while not rospy.is_shutdown():
			self.reconnect_node()

			self.rate.sleep()

		print('Programer stopped')

def main():
	print('Starting main program')

	program = Reconnect_node()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
