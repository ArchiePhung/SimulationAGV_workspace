#!/usr/bin/env python3
# Author: HOANG VAN QUANG - BEE
# DATE: 22/06/2021

from sensor_msgs.msg import LaserScan , Image , PointCloud2
from sensor_msgs.msg import Imu as Imu_SS
from geometry_msgs.msg import Twist
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
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
		rospy.init_node('kickoff', anonymous=False)
		self.rate = rospy.Rate(10)

		self.count_node = 0.
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
		# self.path_checkPort = rospy.get_param('path_checkPort', '')
		# self.launch_checkPort = Launch(self.path_checkPort)
		# rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
		# self.is_checkPort = 0
		# self.count_node += 1

		# -- module - reconnectAll.
		# self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		# self.launch_reconnectAll = Launch(self.path_reconnectAll)
		# rospy.Subscriber('/status_reconnect', Status_reconnect, self.callBack_reconnectAll)
		# self.is_reconnectAll = 0
		# self.count_node += 1

		# -- module - hmi.
		# self.path_hmi = rospy.get_param('path_hmi', '')
		# self.launch_hmi = Launch(self.path_hmi)
		# rospy.Subscriber('/HMI_allButton', HMI_allButton, self.callBack_hmi)
		# self.is_hmi = 0
		# self.count_node += 1

		# -- module - synthetic info.
		# self.path_syn = rospy.get_param('path_syn', '')
		# self.launch_syn = Launch(self.path_syn)
		# rospy.Subscriber('/HMI_infoGeneral', HMI_infoGeneral, self.callBack_syn)
		# self.is_syn = 0
		# self.count_node += 1

		# -- module - Main.
		# self.path_main = rospy.get_param('path_main', '')
		# self.launch_main = Launch(self.path_main)
		# rospy.Subscriber('/POWER_info', POWER_info, self.callBack_main)
		# self.is_main = 0
		# self.count_node += 1

		# -- module - MC.
		self.path_mc = rospy.get_param('path_mc', '')
		self.launch_mc = Launch(self.path_mc)
		rospy.Subscriber('/motorDrive_respond', MotorDrive_respond, self.callBack_mc)
		self.is_mc = 0
		self.count_node += 1

		# -- module - OC.
		self.path_oc = rospy.get_param('path_oc', '')
		self.launch_oc = Launch(self.path_oc)
		rospy.Subscriber('/OC_respond', OC_ForkLift_respond, self.callBack_oc)
		self.is_oc = 1
		self.count_node += 1

		# -- module - HC.
		# self.path_hc = rospy.get_param('path_hc', '')
		# self.launch_hc = Launch(self.path_hc)
		# rospy.Subscriber('/HC_info', HC_info, self.callBack_hc)
		# self.is_hc = 0
		# self.count_node += 1

		# -- module - magLine.
		# self.path_magLine = rospy.get_param('path_magLine', '')
		# self.launch_magLine = Launch(self.path_magLine)
		# rospy.Subscriber('/magneticLine1', Magnetic_line, self.callBack_magLine)
		# self.is_magLine = 0
		# self.count_node += 1

		# -- module - driverLeft.
		# self.path_driverLeft = rospy.get_param('path_driverLeft', '')
		# self.launch_driverLeft = Launch(self.path_driverLeft)
		# rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
		# self.is_driverLeft = 0
		# self.count_node += 1

		# -- module - driverRight.
		# self.path_driverRight = rospy.get_param('path_driverRight', '')
		# self.launch_driverRight = Launch(self.path_driverRight)
		# rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		# self.is_driverRight = 0
		# self.count_node += 1

		# -- module - tf.
		# self.path_tf = rospy.get_param('path_tf', '')
		# self.launch_tf = Launch(self.path_tf)
		# # rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		# self.is_tf = 0
		# self.count_node += 1

		# -- module - camera.
		# self.path_camera = rospy.get_param('path_camera', '')
		# self.launch_camera = Launch(self.path_camera)
		# rospy.Subscriber('/camera/color/image_raw', Image, self.callBack_camera)
		# self.is_camera = 0
		# self.count_node += 1

		# -- module - lidar.
		self.path_nav350 = rospy.get_param('path_nav350', '')
		self.launch_nav350 = Launch(self.path_nav350)
		rospy.Subscriber('/scan', LaserScan, self.callBack_nav350)
		self.is_nav350 = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		# rospy.Subscriber('/driver1_query', Driver_query, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- module - imuFilter.
		# self.path_imuFilter = rospy.get_param('path_imuFilter', '')
		# self.launch_imuFilter = Launch(self.path_imuFilter)
		# rospy.Subscriber('/imu_version1', Imu_version1, self.callBack_imuFilter)
		# self.is_imuFilter = 0
		# self.count_node += 1

		# -- module odomEncoder.
		# self.path_odomEncoder = rospy.get_param('path_odomEncoder', '')
		# self.launch_odomEncoder = Launch(self.path_odomEncoder)
		# rospy.Subscriber('/raw_odom', Odometry, self.callBack_odomEncoder)
		# self.is_odomEncoder = 0
		# self.count_node += 1

		# -- module - path_odomHector.
		# self.path_odomHector = rospy.get_param('path_odomHector', '')
		# self.launch_odomHector = Launch(self.path_odomHector)
		# rospy.Subscriber('/scanmatch_odom', Odometry, self.callBack_odomHector)
		# self.is_odomHector = 0
		# self.count_node += 1

		# -- module - poseLidar.
		# self.path_poseLidar = rospy.get_param('path_poseLidar', '')
		# self.launch_poseLidar = Launch(self.path_poseLidar)
		# rospy.Subscriber('/pose_lidar', PoseWithCovarianceStamped, self.callBack_poseLidar)
		# self.is_poseLidar = 0
		# self.count_node += 1

		# -- module - ekf.
		# self.path_ekf = rospy.get_param('path_ekf', '')
		# self.launch_ekf = Launch(self.path_ekf)
		# rospy.Subscriber('/odom', Odometry, self.callBack_ekf)
		# self.is_ekf = 0
		# self.count_node += 1

		# -- module - mapServer.
		# self.path_mapServer = rospy.get_param('path_mapServer', '')
		# self.launch_mapServer = Launch(self.path_mapServer)
		# rospy.Subscriber('/map', OccupancyGrid, self.callBack_mapServer)
		# self.is_mapServer = 0
		# self.count_node += 1

		# -- module - amcl.
		# self.path_amcl = rospy.get_param('path_amcl', '')
		# self.launch_amcl = Launch(self.path_amcl)
		# rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callBack_amcl)
		# self.is_amcl = 0
		# self.count_node += 1

		# -- module - posePublisher.
		self.path_robotPoseNav = rospy.get_param('path_robotPoseNav', '')
		self.launch_robotPoseNav = Launch(self.path_robotPoseNav)
		rospy.Subscriber('/robotPose_nav', PoseStamped, self.callBack_robotPoseNav)
		self.is_robotPoseNav = 0
		self.count_node += 1

		# -- module - Setpose.
		# self.path_setPose = rospy.get_param('path_setPose', '')
		# self.launch_setPose = Launch(self.path_setPose)
		# rospy.Subscriber('/setpose_status', Setpose_status, self.callBack_setPose)
		# self.is_setPose = 0
		# self.count_node += 1

		# -- module - Parking.
		# self.path_parking = rospy.get_param('path_parking', '')
		# self.launch_parking = Launch(self.path_parking)
		# rospy.Subscriber('/parking_status', Parking_status, self.callBack_parking)
		# self.is_parking = 0
		# self.count_node += 1

		# -- atuo launch file detect
		# self.path_apriltag = rospy.get_param('path_apriltag', '')
		# self.launch_apriltag = Launch(self.path_apriltag)
		# # rospy.Subscriber('/parking_status', Parking_status, self.callBack_apriltag)
		# self.is_apriltag = 0
		# self.count_node += 1

		# -- module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - navigation.
		# self.path_navigation = rospy.get_param('path_navigation', '')
		# self.launch_navigation = Launch(self.path_navigation)
		# rospy.Subscriber('/status_goal_control', Status_goal_control, self.callBack_navigation)
		# self.is_navigation = 0
		# self.count_node += 1

		# -- module - webconsole.
		# self.path_webconsole = rospy.get_param('path_webconsole', '')
		# self.launch_webconsole = Launch(self.path_webconsole)
		# self.is_webconsole = 0
		# self.count_node += 1

		# -- module - client.
		self.path_client = rospy.get_param('path_client', '')
		self.launch_client = Launch(self.path_client)
		rospy.Subscriber('/NN_infoRequest', NN_infoRequest, self.callBack_client)
		self.is_client = 0
		self.count_node += 1

		# -- module - control.
		# self.path_control = rospy.get_param('path_control', '')
		# self.launch_control = Launch(self.path_control)
		# rospy.Subscriber('/NN_infoRespond', NN_infoRespond, self.callBack_control)
		# self.is_control = 0
		# self.count_node += 1

		# -- module - debug.
		# self.path_debug = rospy.get_param('path_debug', '')
		# self.launch_debug = Launch(self.path_debug)
		# rospy.Subscriber('/', , self.callBack_debug)
		self.is_debug = 0
		self.count_node += 1
		# self.count_node += 1

		self.count_node = 8

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1
	# -- --
	def callBack_hmi(self, data):
		self.is_hmi = 1

	def callBack_syn(self, data):
		self.is_syn = 1

	def callBack_main(self, data):
		self.is_main = 1

	def callBack_mc(self, data):
		self.is_mc = 1

	def callBack_sc(self, data):
		self.is_sc = 1

	def callBack_oc(self, data):
		self.is_oc = 1

	def callBack_hc(self, data):
		self.is_hc = 1

	def callBack_magLine(self, data):
		self.is_magLine = 1

	def callBack_driverLeft(self, data):
		self.is_driverLeft = 1

	def callBack_driverRight(self, data):
		self.is_driverRight = 1

	def callBack_camera(self, data):
		self.is_camera = 1

	def callBack_nav350(self, data):
		self.is_nav350 = 1
	# -- --
	def callBack_kinematic(self, data):
		self.is_kinematic = 1

	def callBack_imuFilter(self, data):
		self.is_imuFilter = 1

	def callBack_odomEncoder(self, data):
		self.is_odomEncoder = 1

	def callBack_odomHector(self, data):
		self.is_odomHector = 1

	def callBack_poseLidar(self, data):
		self.is_poseLidar = 1

	def callBack_ekf(self, data):
		self.is_ekf = 1

	def callBack_mapServer(self, data):
		self.is_mapServer = 1

	def callBack_amcl(self, data):
		self.is_amcl = 1

	def callBack_robotPoseNav(self, data):
		self.is_robotPoseNav = 1

	def callBack_setPose(self, data):
		self.is_setPose = 1

	def callBack_parking(self, data):
		self.is_parking = 1
	# --
	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1

	def callBack_navigation(self, data):
		self.is_navigation = 1

	def callBack_client(self, data):
		self.is_client = 1

	def callBack_control(self, data):
		self.is_control = 1

	def run(self):
		while not rospy.is_shutdown():
			# print "runing"
			# -- firstWork
			if (self.step == 0):
				self.notification = 'launch_firstWork'
				self.launch_firstWork.start()
				if (self.is_firstWork == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- checkPort
			# elif (self.step == 1):
			# 	self.notification = 'launch_checkPort'
			# 	self.launch_checkPort.start()
			# 	if (self.is_checkPort == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- reconnect
			# elif (self.step == 2):
			# 	self.notification = 'launch_reconnectAll'
			# 	self.launch_reconnectAll.start()
			# 	if (self.is_reconnectAll == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)
	            		
            # -- hmi
			# elif (self.step == 3):
			# 	self.notification = 'launch_hmi'
			# 	self.launch_hmi.start()
			# 	if (self.is_hmi == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)
	            		
            # -- synthetic info
			# elif (self.step == 4):
			# 	self.notification = 'launch_syn'
			# 	self.launch_syn.start()
			# 	if (self.is_syn == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- main
			# elif (self.step == 5):
			# 	self.notification = 'launch_main'
			# 	self.launch_main.start()
			# 	if (self.is_main == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)
            		
			# -- sc
			# elif (self.step == 6):
			# 	self.notification = 'launch_sc'
			# 	self.launch_sc.start()
			# 	if (self.is_sc == 1):
			# 		self.step += 1
			# 		# self.step = 8
			# 		time.sleep(self.timeWait)
            		
			# -- mc
			elif (self.step == 1):
				self.notification = 'launch_mc'
				self.launch_mc.start()
				if (self.is_mc == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
			# -- oc
			elif (self.step == 2):
				self.notification = 'launch_oc'
				self.launch_oc.start()
				if (self.is_oc == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
			# -- hc
			# elif (self.step == 8):
			# 	self.notification = 'launch_hc'
			# 	self.launch_hc.start()
			# 	if (self.is_hc == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)
            		
			# -- magLine
			# elif (self.step == 9):
			# 	self.notification = 'launch_magLine'
			# 	self.launch_magLine.start()
			# 	if (self.is_magLine == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- driverLeft
			# elif (self.step == 10):
			# 	self.notification = 'launch_driverLeft'
			# 	self.launch_driverLeft.start()
			# 	if (self.is_driverLeft == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)
					
            # -- driverRight
			# elif (self.step == 11):
			# 	self.notification = 'launch_driverRight'
			# 	self.launch_driverRight.start()
			# 	if (self.is_driverRight == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- tf
			# elif (self.step == 12):
			# 	self.notification = 'launch_tf'
			# 	sts = self.launch_tf.start_and_wait(4.)
			# 	# if (self.is_tf == 1):
			# 	if (sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- camera
			# elif (self.step == 13):
			# 	self.notification = 'launch_camera'
			# 	self.launch_camera.start()
			# 	if (self.is_camera == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- lidar
			elif (self.step == 3):
				self.notification = 'launch_nav350'
				self.launch_nav350.start()
				if (self.is_nav350 == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 4):
				self.notification = 'launch_kinematic'
				sts = self.launch_kinematic.start_and_wait(3.)
				if (self.is_kinematic == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- imuFilter
			# elif (self.step == 16):
			# 	self.notification = 'launch_imuFilter'
			# 	self.launch_imuFilter.start()
			# 	if (self.is_imuFilter == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- odomEncoder
			# elif (self.step == 17):
			# 	self.notification = 'launch_odomEncoder'
			# 	self.launch_odomEncoder.start()
			# 	if (self.is_odomEncoder == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- odomHector
			# elif (self.step == 18):
			# 	self.notification = 'launch_odomHector'
			# 	self.launch_odomHector.start()
			# 	if (self.is_odomHector == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- poseLidar
			# elif (self.step == 19):
			# 	self.notification = 'launch_poseLidar'
			# 	self.launch_poseLidar.start()
			# 	if (self.is_poseLidar == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- ekf
			# elif (self.step == 20):
			# 	self.notification = 'launch_ekf'
			# 	self.launch_ekf.start()
			# 	if (self.is_ekf == 1):
			# 		self.step += 1
			# 		# self.step = 19
			# 		time.sleep(self.timeWait)

            # -- mapServer
			# elif (self.step == 21):
			# 	self.notification = 'launch_mapServer'
			# 	self.launch_mapServer.start()
			# 	if (self.is_mapServer == 1):
			# 		self.step += 1
			# 		# self.step = 19
			# 		time.sleep(self.timeWait)

            # -- amcl
			# elif (self.step == 22):
			# 	self.notification = 'launch_amcl'
			# 	self.launch_amcl.start()
			# 	if (self.is_amcl == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- posePublisher
			elif (self.step == 5):
				self.notification = 'launch_robotPoseNav'
				self.launch_robotPoseNav.start()
				if (self.is_robotPoseNav == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- Setpose
			# elif (self.step == 24):
			# 	self.notification = 'launch_setPose'
			# 	self.launch_setPose.start()
			# 	if (self.is_setPose == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- Parking
			# elif (self.step == 25):
			# 	self.notification = 'launch_parking'
			# 	self.launch_parking.start()
			# 	if (self.is_parking == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- Auto launch detect
			# elif (self.step == 26):
			# 	self.notification = 'launch_apriltag'
			# 	sts = self.launch_apriltag.start_and_wait(4.)
			# 	if (self.is_apriltag == 1 or sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- safetyZone
			elif (self.step == 6):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- navigation
			# elif (self.step == 28):
			# 	self.notification = 'launch_navigation'
			# 	self.launch_navigation.start()
			# 	if (self.is_navigation == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- webconsole
			# elif (self.step == 29):
			# 	self.notification = 'launch_webconsole'
			# 	sts = self.launch_webconsole.start_and_wait(3.)
			# 	if(sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- client
			elif (self.step == 7):
				self.notification = 'launch_client'
				sts = self.launch_client.start_and_wait(3.)
				if (self.is_client == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- control
			# elif (self.step == 30):
			# 	self.notification = 'launch_control'
			# 	self.launch_control.start()
			# 	if (self.is_control == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- debug
			# elif (self.step == 31):
			# 	self.notification = 'launch_debug'
			# 	sts = self.launch_debug.start_and_wait(4.)
			# 	# if (self.is_debug == 1):
			# 	if (sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- Completed
			elif (self.step == 8):
				self.notification = 'Completed!'
            	
            # -- -- PUBLISH STATUS 
			self.stausLaunch.persent = int((self.step/self.count_node)*100)
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
