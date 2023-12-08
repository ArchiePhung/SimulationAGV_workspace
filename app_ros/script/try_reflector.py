#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 31/10/2021

"""
import roslib
# from uptime import uptime
import sys
import time
from decimal import *
import math
import rospy
from datetime import datetime
# ip
import os
from message_pkg.msg import *
# from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees


#--------------------------------------------------------------------------------- ROS
class sti_ctr():
	def __init__(self):
		rospy.init_node('try_reflector', anonymous=False)
		self.rate = rospy.Rate(10)

		# -- Pose robot
		rospy.Subscriber("/nav350_reflectors", Reflector_array, self.callback_nav350Reflectors) 
		self.nav350_reflectors = Reflector_array()

		self.value_error = 0

	def callback_nav350Reflectors(self, data):
		self.nav350_reflectors = data
		# --
		# self.abl1(self)
		self.abl2()


	def abl2(self):
		length = self.nav350_reflectors.num_reflector
		rate_show = 0.1
		# ----
		print ("----------")
		for i in range(length):
			if self.nav350_reflectors.reflectors[i].GlobalID == 10:
				angle1 = round(self.nav350_reflectors.reflectors[i].Polar_Phi/1000., 2)
				print ("Angle1: ",  angle1)
				angle2 = self.limitAngle(radians(angle1) )
				print ("Angle2: ",  degrees(angle2))

			# dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			# ang = self.limitAngle(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)

			# p_x, p_y = self.convert_position(dis, ang)
			# print (str(round(p_x, 3)) + " | " + str(round(p_y, 3)))

			# rp_x = int((p_x)/rate_show)
			# rp_y = int((p_y)/rate_show)
			# # print (str(round(rp_x, 3)) + " | " + str(round(rp_y, 3)))

			# sh_x = rp_x + 401
			# sh_y = rp_y + 201
			
			# print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )

	def abl1(self):
		min_x = 1000
		max_x = -1000
		min_y = 1000
		max_y = -1000
		print ("----------------------------------")
		length = self.nav350_reflectors.num_reflector
		for i in range(length):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang = self.limitAngle(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)

			p_x, p_y = self.convert_position(dis, ang)
			print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(p_x, 3)) + " | " + str(round(p_y, 3)) )
			if min_x > p_x:
				min_x = p_x

			if min_y > p_y:
				min_y = p_y

			if max_x < p_x:
				max_x = p_x

			if max_y < p_y:
				max_y = p_y

		print (str(min_x) + " | " + str(max_x) + " | Y: " + str(min_y) + " | " + str(max_y))
		# s = (abs(min_x) + abs(max_x))*(abs(min_y) + abs(max_y))
		# print ("S: ", s/3.0)
		dx = abs(min_x) + abs(max_x)
		dy = abs(min_y) + abs(max_y)
		print ("dx: " + str(dx) + " |dy: " + str(dy))

		rate_show = 0.0
		rate_xy = float(dx/dy)
		if rate_xy > 0.5:
			print ("Hien thi theo X")
			rate_show = (dx*1000)/400.
			# rate_show = int(rate_show/1000.)
		else:
			print ("Hien thi theo Y")
			rate_show = (dy*1000)/820.
			# rate_show = int(rate_show/1000.)

		print ("Rate show: " + str(rate_show))
		print ("drx: " + str(int(dx*1000/rate_show)) + " |dry: " + str(int((dy*1000)/rate_show)))

		for i in range(length):
			dis = self.nav350_reflectors.reflectors[i].Polar_Dist/1000.
			ang = self.limitAngle(self.nav350_reflectors.reflectors[i].Polar_Phi/1000.)

			p_x, p_y = self.convert_position(dis, ang)
			rp_x = int(p_x*1000/rate_show)
			rp_y = int(p_y*1000/rate_show)
			sh_x = rp_y + 215
			sh_y = rp_x + 430
			print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(rp_x, 3)) + " | " + str(round(rp_y, 3)) )
			print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )
			print ("----")

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
	
	def convert_coordinate(self):
		pass

	def convert_position(self, distance, angle):
		x = 0
		y = 0
		x = distance*cos(angle)
		y = distance*sin(angle)
		return x, y
	
	def run(self):
		while not rospy.is_shutdown():
			# --
			# self.pub_infoRespond.publish(self.Server_infoRespond)
			# -- 
			self.rate.sleep()

def main():
	# Start the job threads
	program = sti_ctr()
	program.run()

if __name__ == '__main__':
	main()