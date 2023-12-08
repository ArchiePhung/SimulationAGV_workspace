#!/usr/bin/env python3
# Author: HOANG VAN QUANG - BEE
# DATE  : 18/10/2021

from sensor_msgs.msg import LaserScan , Image , PointCloud2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

from sti_msgs.msg import POWER_info, HC_info # Status_port Driver_respond, Driver_respond
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
		self.time_pre = time.time()

	def start(self):
		if (self.process == 0): # - Launch
			# print ("Launch node!")
			launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
			launch.start()
			self.process = 1  

	def start_and_wait(self, timeWait): # second - Dung cho cac node ko pub.
		if (self.process == 0): # - Launch
			# print ("Launch node!")
			launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
			launch.start()
			self.process = 1
			self.time_pre = time.time()
			return 0

		elif (self.process == 1): # - Wait
			t = (time.time() - self.time_pre)%60
			if (t > timeWait):
				self.process = 2
			return 0

		elif (self.process == 2): # - Wait
			return 1

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

		# -- module - reconnectAll.
		self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		self.launch_reconnectAll = Launch(self.path_reconnectAll)
		# rospy.Subscriber('/reconnect_status', '', self.callBack_reconnectAll)
		self.is_reconnectAll = 1
		self.count_node += 1

		# -- module - Main.
		self.path_main = rospy.get_param('path_main', '')
		self.launch_main = Launch(self.path_main)
		rospy.Subscriber('/POWER_info', POWER_info, self.callBack_main)
		self.is_main = 0
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

		# -- module - nav350.
		self.path_nav350 = rospy.get_param('path_nav350', '')
		self.launch_nav350 = Launch(self.path_nav350)
		rospy.Subscriber('/scan', LaserScan, self.callBack_nav350)
		self.is_nav350 = 0
		self.count_node += 1

		# -- module - HC.
		self.path_hc = rospy.get_param('path_hc', '')
		self.launch_hc = Launch(self.path_hc)
		rospy.Subscriber('/HC_info', HC_info, self.callBack_hc)
		self.is_hc = 0
		self.count_node += 1

		# -- module - imu.
		self.path_imu = rospy.get_param('path_imu', '')
		self.launch_imu = Launch(self.path_imu)
		rospy.Subscriber('/imu/data', Imu, self.callBack_imu)
		self.is_imu = 0
		self.count_node += 1

		# -- module - imuFilter.
		self.path_imuFilter = rospy.get_param('path_imuFilter', '')
		self.launch_imuFilter = Launch(self.path_imuFilter)
		rospy.Subscriber('/imu_filter', Imu, self.callBack_imuFilter)
		self.is_imuFilter = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/driver1_query', Driver_query, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- get pose robot from nav.
		self.path_robotPoseNav = rospy.get_param('path_robotPoseNav', '')
		self.launch_robotPoseNav = Launch(self.path_robotPoseNav)
		rospy.Subscriber('/robotPose_nav', PoseStamped, self.callBack_robotPoseNav)
		self.is_robotPoseNav = 0
		self.count_node += 1

		# -- module - ekf.
		self.path_ekf = rospy.get_param('path_ekf', '')
		self.launch_ekf = Launch(self.path_ekf)
		rospy.Subscriber('/odom', Odometry, self.callBack_ekf)
		self.is_ekf = 0
		self.count_node += 1

		# -- module - posePublisher.
		self.path_posePublisher = rospy.get_param('path_posePublisher', '')
		self.launch_posePublisher = Launch(self.path_posePublisher)
		rospy.Subscriber('/robot_pose', Pose, self.callBack_posePublisher)
		self.is_posePublisher = 0
		self.count_node += 1

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1

	def callBack_main(self, data):
		self.is_main = 1

	def callBack_driverLeft(self, data):
		self.is_driverLeft = 1

	def callBack_driverRight(self, data):
		self.is_driverRight = 1

	def callBack_imu(self, data):
		self.is_imu = 1

	def callBack_imuFilter(self, data):
		self.is_imuFilter = 1

	def callBack_nav350(self, data):
		self.is_nav350 = 1

	def callBack_robotPoseNav(self, data):
		self.is_robotPoseNav = 1

	def callBack_hc(self, data):
		self.is_hc = 1

	def callBack_kinematic(self, data):
		self.is_kinematic = 1

	def callBack_rawOdom(self, data):
		self.is_rawOdom = 1

	def callBack_ekf(self, data):
		self.is_ekf = 1

	def callBack_posePublisher(self, data):
		self.is_posePublisher = 1

	def run(self):
		while not rospy.is_shutdown():
			# print "runing"
			# -- firstWork
			if (self.step == 0):
				self.notification = 'launch_firstWork'
				self.launch_firstWork.start()
				if (self.is_firstWork == 1):
					self.step += 1
					self.step = 5
					time.sleep(self.timeWait)

            # -- checkPort
			elif (self.step == 1):
				self.notification = 'launch_checkPort'
				self.launch_checkPort.start()
				if (self.is_checkPort == 1):
					self.step += 1
					# self.step = 4
					time.sleep(self.timeWait)

            # -- reconnectAll
			elif (self.step == 2):
				self.notification = 'launch_reconnectAll'
				# self.launch_reconnectAll.start()
				if (self.is_reconnectAll == 1):
					self.step += 1
					
					time.sleep(self.timeWait)

			# -- main
			elif (self.step == 3):
				self.notification = 'launch_main'
				self.launch_main.start()
				if (self.is_main == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- hc
			elif (self.step == 4):
				self.notification = 'launch_hc'
				self.launch_hc.start()
				if (self.is_hc == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- imu
			elif (self.step == 5):
				self.notification = 'launch_imu'
				self.launch_imu.start()
				if (self.is_imu == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- driverLeft
			elif (self.step == 6):
				self.notification = 'launch_driverLeft'
				self.launch_driverLeft.start()
				if (self.is_driverLeft == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- driverRight
			elif (self.step == 7):
				self.notification = 'launch_driverRight'
				self.launch_driverRight.start()
				if (self.is_driverRight == 1):
					self.step += 1
					# self.step = 8
					time.sleep(self.timeWait)

            # -- nav350
			elif (self.step == 8):
				self.notification = 'launch_nav350'
				self.launch_nav350.start()
				if (self.is_nav350 == 1):
					self.step += 1
					self.step = 10
					time.sleep(self.timeWait)

            # -- imuFilter
			elif (self.step == 9):
				self.notification = 'launch_imuFilter'
				self.launch_imuFilter.start()
				if (self.is_imuFilter == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 10):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- get pose robot from nav.
			elif (self.step == 11):
				self.notification = 'launch_robotPoseNav'
				self.launch_robotPoseNav.start()
				if (self.is_robotPoseNav == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- ekf
			elif (self.step == 12):
				self.notification = 'launch_ekf'
				self.launch_ekf.start()
				if (self.is_ekf == 1):
					self.step += 1
					self.step = 19
					time.sleep(self.timeWait)


            # # -- posePublisher
			# elif (self.step == 18):
			# 	self.notification = 'launch_posePublisher'
			# 	self.launch_posePublisher.start()
			# 	if (self.is_posePublisher == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- Completed
			elif (self.step == 19):
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