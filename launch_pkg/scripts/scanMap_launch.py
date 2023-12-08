#!/usr/bin/env python
# Author: HOANG VAN QUANG - BEE
# DATE: 22/06/2021

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

from std_msgs.msg import Int16
from sti_msgs.msg import *
from message_pkg.msg import *

import roslaunch
import rospy
import string
import time
import os

class Launch:
	def __init__(self, file_launch):
		# -- parameter
		self.fileLaunch = file_launch
		# -- launch
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		# -- variable
		self.process = 0
        
	def start(self):
		if (self.process == 0): # - Launch
			# print ("Launch node!")
			launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
			launch.start()
			self.process = 1  

class scanMap():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('scanMap_launch', anonymous=False)
		self.rate = rospy.Rate(10)

		self.count_node = 0
		self.notification = ''
		self.step = 0
		self.timeWait = 0.4 # s

		self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
		self.stausLaunch = Status_launch()

		# -- module - firstWork.
		self.path_firstWork = rospy.get_param('path_firstWork', '')
		self.launch_firstWork = Launch(self.path_firstWork)
		rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
		self.is_firstWork = 0
		self.count_node += 1

		# -- module - checkPort.
		self.path_checkPort = rospy.get_param('path_checkPort', '')
		self.launch_checkPort = Launch(self.path_checkPort)
		rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
		self.is_checkPort = 0
		self.count_node += 1

		# -- module - hmi.
		# self.path_hmi = rospy.get_param('path_hmi', '')
		# self.launch_hmi = Launch(self.path_hmi)
		# rospy.Subscriber('/HMI_allButton', HMI_allButton, self.callBack_hmi)
		# self.is_hmi = 0
		# self.count_node += 1

		# -- module - main.
		self.path_main = rospy.get_param('path_main', '')
		self.launch_main = Launch(self.path_main)
		rospy.Subscriber('/POWER_info', POWER_info, self.callBack_main)
		self.is_main = 0
		self.count_node += 1

		# -- module - sc.
		self.path_sc = rospy.get_param('path_sc', '')
		self.launch_sc = Launch(self.path_sc)
		rospy.Subscriber('/imu_version1', Imu_version1 , self.callBack_sc)
		self.is_sc = 0
		self.count_node += 1

		# -- module - driverLeft.
		self.path_driverLeft = rospy.get_param('path_driverLeft', '')
		self.launch_driverLeft = Launch(self.path_driverLeft)
		rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
		self.is_driverLeft = 0
		self.count_node += 1

		# -- module - driverRight.
		self.path_driverRight = rospy.get_param('path_driverRight', '')
		self.launch_driverRight = Launch(self.path_driverRight)
		rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		self.is_driverRight = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/driver1_query', Driver_query, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- module - tf scan.
		self.path_tfScan = rospy.get_param('path_tfScan', '')
		self.launch_tfScan = Launch(self.path_tfScan)
		# rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		self.is_tfScan = 1
		self.count_node += 1

		# -- module - reconnectAll.
		# self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		# self.launch_reconnectAll = Launch(self.path_reconnectAll)
		# rospy.Subscriber('/reconnect_status', , self.callBack_reconnectAll)
		# self.is_reconnectAll = 0
		# self.count_node += 1

		# -- module - lidar.
		self.path_lidarFull = rospy.get_param('path_lidarFull', '')
		self.launch_lidarFull = Launch(self.path_lidarFull)
		rospy.Subscriber('/scan', LaserScan, self.callBack_lidarFull)
		self.is_lidarFull = 0
		self.count_node += 1

		# -- module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber('/safety_zone', Zone_lidar_2head, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - naviManual.
		self.path_naviManual = rospy.get_param('path_naviManual', '')
		self.launch_naviManual = Launch(self.path_naviManual)
		rospy.Subscriber('/cmd_vel', Twist, self.callBack_naviManual)
		self.is_naviManual = 0
		self.count_node += 1

		# -- module - cartographer.
		self.path_cartographer = rospy.get_param('path_cartographer', '')
		self.launch_cartographer = Launch(self.path_cartographer)
		# rospy.Subscriber('/', Int16, self.callBack_cartographer)
		self.is_cartographer = 1
		self.count_node += 1

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	# def callBack_hmi(self, data):
	# 	self.is_hmi = 1

	def callBack_main(self, data):
		self.is_main = 1

	def callBack_sc(self, data):
		self.is_sc = 1

	def callBack_driverLeft(self, data):
		self.is_driverLeft = 1

	def callBack_driverRight(self, data):
		self.is_driverRight = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1

	def callBack_kinematic(self, data):
		self.is_kinematic = 1

	# def callBack_odomEncoder(self, data):
	# 	self.is_odomEncoder = 1

	def callBack_lidarFull(self, data):
		self.is_lidarFull = 1

	# def callBack_odomLidar(self, data):
	# 	self.is_odomLidar = 1

	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1

	def callBack_naviManual(self, data):
		self.is_naviManual = 1

	def callBack_cartographer(self, data):
		self.is_cartographer = 1

	def run(self):
		while not rospy.is_shutdown():
			print "runing"
			# -- firstWork
			if (self.step == 0):
			    self.notification = 'launch_firstWork'
			    self.launch_firstWork.start()
			    if (self.is_firstWork == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- checkPort
			elif (self.step == 1):
				self.notification = 'launch_checkPort'
				self.launch_checkPort.start()
				if (self.is_checkPort == 1):
					self.step += 1
					time.sleep(self.timeWait)
	            		
            # -- hmi
            # elif (self.step == 2):
            #     self.notification = 'launch_hmi'
            #     self.launch_hmi.start()
            #     if (self.is_hmi == 1):
            #         self.step += 1
            # 		time.sleep(self.timeWait)
            		
			# -- main
			elif (self.step == 2):
				self.notification = 'launch_main'
				self.launch_main.start()
				if (self.is_main == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
			# -- sc
			elif (self.step == 3):
				self.notification = 'launch_sc'
				self.launch_sc.start()
				if (self.is_sc == 1):
					self.step += 1
					time.sleep(self.timeWait)
					
			# -- driverLeft
			elif (self.step == 4):
				self.notification = 'launch_driverLeft'
				self.launch_driverLeft.start()
				if (self.is_driverLeft == 1):
					self.step += 1
					time.sleep(self.timeWait)
					
            # -- driverRight
			elif (self.step == 5):
				self.notification = 'launch_driverRight'
				self.launch_driverRight.start()
				if (self.is_driverRight == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 6):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- tf scan
			elif (self.step == 7):
				self.notification = 'launch_tfScan'
				self.launch_tfScan.start()
				if (self.is_tfScan == 1):
					print ("77")
					time.sleep(2)
					self.step += 1
					time.sleep(self.timeWait)

            # -- lidar
			elif (self.step == 8):
				print ("88")
				self.notification = 'launch_lidarFull'
				self.launch_lidarFull.start()
				if (self.is_lidarFull == 1):
					self.step += 1
					time.sleep(self.timeWait)
			# print ("----")		
            # -- odomLidar
            # elif (self.step == 8):
            #     self.notification = 'launch_odomLidar'
            #     self.launch_odomLidar.start()
            #     if (self.is_odomLidar == 1):
            #         self.step += 1
            # 		time.sleep(self.timeWait)
            		
            # -- safetyZone
			elif (self.step == 9):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- naviManual
			elif (self.step == 10):
				self.notification = 'launch_naviManual'
				self.launch_naviManual.start()
				if (self.is_naviManual == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- cartographer
			elif (self.step == 11):
				self.notification = 'launch_cartographer'
				self.launch_cartographer.start()
				if (self.is_cartographer == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- Completed
			elif (self.step == 12):
				self.notification = 'Completed!'
            	
            # -- -- PUBLISH STATUS
			self.stausLaunch.persent = (self.step/self.count_node)*100.
			self.stausLaunch.position = self.step
			self.stausLaunch.notification = self.notification
			self.pub_stausLaunch.publish(self.stausLaunch)
			# time.sleep(0.1)
			self.rate.sleep()

def main():
	print('Program starting')
	try:
		program = scanMap()
		program.run()
	except rospy.ROSInterruptException:
		pass
	print('Programer stopped')

if __name__ == '__main__':
	main()