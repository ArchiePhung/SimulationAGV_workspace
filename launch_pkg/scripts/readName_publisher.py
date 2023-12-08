#!/usr/bin/env python
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
from sensor_msgs.msg import LaserScan , Image

from sti_msgs.msg import *
from message_pkg.msg import *
# output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell= True)
def read_nameNode( topic):
	try:
		output = subprocess.check_output("rostopic info {}".format(topic), shell= True)
		# print("out: ", output)

		pos1 = output.find('Publishers:')
		pos2 = output.find(' (http://')
		# print("pos1: ", pos1)
		# print("pos2: ", pos2)
		if (pos1 >= 0):
			name = output[pos1 + 16 :pos2]
			# print("name: ", name)
			return name
		else:
			# print ("Not!")
			return ''

	except Exception, e:
	    # print ("Error!")
		return ''

name = read_nameNode("/HMI_allButton")
print (name)