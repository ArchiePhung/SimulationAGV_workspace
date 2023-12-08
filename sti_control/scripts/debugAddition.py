#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 06/12/2021

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

from message_pkg.msg import App_lbv
from sti_msgs.msg import NN_infoRespond
#--------------------------------------------------------------------------------- ROS
class debug():
	def __init__(self):
		rospy.init_node('debugAddition', anonymous=False)
		self.rate = rospy.Rate(100)
	  # SUB - PUB
		rospy.Subscriber("/app_setValue", App_lbv, self.appSetValue_Callback)
		self.appSetValue = App_lbv()
		self.old_appSetValue = App_lbv()

		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_Callback)
		self.infoAGV = NN_infoRespond()
		self.old_infoAGV = NN_infoRespond()
		
		self.present_error = 0
		self.path_log_error = "/home/stivietnam/catkin_ws/debug/SHIV34_NAV1_errorLog.txt"
		self.process = 0

	def log_status(self):
		mess = ''
		# if (self.infoAGV.listError != self.old_infoAGV.listError):
		# 	self.old_infoAGV = self.infoAGV
		if (self.old_appSetValue.lbv_device != self.appSetValue.lbv_device or self.old_appSetValue.lbv_frameWork != self.appSetValue.lbv_frameWork):
			self.old_appSetValue = self.appSetValue

			now = datetime.now()
			current_time = now.strftime("%B/%d|%H:%M:%S")

			self.file_log = open(self.path_log_error, "a+")
			self.file_log.write("\n------ " + str(current_time) + " ------\n")
			self.file_log.write(str(self.appSetValue))
			self.file_log.close()

	def appSetValue_Callback(self, dat):
		self.appSetValue = dat

	def infoAGV_Callback(self, dat):
		self.infoAGV = dat

	def run(self):
		if self.process == 0:
			print("DEBUG ADDITION ALL RIGHT!")
			self.process = 1
		else:		
			self.log_status()
			self.rate.sleep()

def main():
	# Start the job threads
	class_1 = debug()
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
	main()