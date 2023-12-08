#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8
from message_pkg.msg import *
from sensor_msgs.msg import Imu
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class test_point():
	def __init__(self):
		rospy.init_node('testPoint', anonymous=False)
		self.rate = rospy.Rate(2)

		# -- Communicate with Server
		self.pub_cmdRequest = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size= 20)
		self.NN_cmdRequest = NN_cmdRequest()
		# -- 
		# -- Mission server
		self.statusTask_liftError = 64 # trang thái nâng kệ nhueng ko có kệ.
		self.serverMission_liftUp = 1 # 65
		self.serverMission_liftDown = 2 # 66
		self.serverMission_charger = 6 # 5
		self.serverMission_unknown = 0
		self.serverMission_liftDown_charger = 5 # 6
		# --
		
	def path_1(self):
		self.NN_cmdRequest.target_id = 1	
		self.NN_cmdRequest.target_x = -20.781
		self.NN_cmdRequest.target_y = -2.317
		self.NN_cmdRequest.target_z = radians(272)
		self.NN_cmdRequest.tag = 0
		self.NN_cmdRequest.offset = 0
		self.NN_cmdRequest.list_id = [3, 2, 1, 0, 0]
		self.NN_cmdRequest.list_x =  [-13.606 ,-20.665, -20.781, 0, 0]
		self.NN_cmdRequest.list_y =  [6.213, 5.987, 2.317, 0, 0]
		self.NN_cmdRequest.list_speed = [100, 100 , 0, 0, 0]
		self.NN_cmdRequest.before_mission = 65
		self.NN_cmdRequest.after_mission = 0
		self.NN_cmdRequest.id_command = 1
		self.NN_cmdRequest.command = "Path 1"

	def path_2(self):
		self.NN_cmdRequest.target_id = 2
		self.NN_cmdRequest.target_x = -20.775
		self.NN_cmdRequest.target_y = -0.327
		self.NN_cmdRequest.target_z = radians(272) # 1.553
		self.NN_cmdRequest.tag = 0
		self.NN_cmdRequest.offset = 0
		self.NN_cmdRequest.list_id = [2, 0, 0, 0, 0]
		self.NN_cmdRequest.list_x =  [-20.775 ,0 , 0, 0, 0]
		self.NN_cmdRequest.list_y =  [-0.327, 0, 0, 0, 0]
		self.NN_cmdRequest.list_speed = [0, 0, 0, 0, 0]
		self.NN_cmdRequest.before_mission = 65
		self.NN_cmdRequest.after_mission = 0
		self.NN_cmdRequest.id_command = 1
		self.NN_cmdRequest.command = "Path 2"

	def path_3(self):
		self.NN_cmdRequest.target_id = 4
		self.NN_cmdRequest.target_x = -16.696
		self.NN_cmdRequest.target_y = -2.892
		self.NN_cmdRequest.target_z = 0 # radians(89) # 1.553
		self.NN_cmdRequest.tag = 0
		self.NN_cmdRequest.offset = 0
		self.NN_cmdRequest.list_id = [2, 4, 0, 0, 0]
		self.NN_cmdRequest.list_x =  [-20.765 , -16.696, 0, 0, 0]
		self.NN_cmdRequest.list_y =  [-2.578, -2.892, 0, 0, 0]
		self.NN_cmdRequest.list_speed = [100, 0, 0, 0, 0]
		self.NN_cmdRequest.before_mission = 65
		self.NN_cmdRequest.after_mission = 0
		self.NN_cmdRequest.id_command = 1
		self.NN_cmdRequest.command = "Path 3"

	def run(self):
		self.path_1()
		# self.path_2()
		# self.path_3()
		self.pub_cmdRequest.publish(self.NN_cmdRequest)
		self.rate.sleep()

def main():
	# Start the job threads
	class_1 = test_point()
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
	main()