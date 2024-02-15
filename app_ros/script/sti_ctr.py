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
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees


#--------------------------------------------------------------------------------- ROS
class sti_ctr():
	def __init__(self):
		rospy.init_node('sti_ctr', anonymous=False)
		self.rate = rospy.Rate(20)

		# -- Pose robot
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.robotPose_callback) 
		self.robotPose_nav = PoseStamped()

		# -- robot Pose
		self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size = 4)
		self.cmdVel = Twist()	

		# -- Communicate with Server
		rospy.Subscriber("/Server_cmdRequest", Server_cmdRequest, self.callback_cmdRequest)
		self.Server_cmdRequest = Server_cmdRequest()

		# -- -- MAIN
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_main) 
		self.main_info = POWER_info()
		self.is_readMain = 0
		self.valueVoltage = 245

		# - Measure Voltage
		self.flag_afterChager = 0 # co yeu cau sau khi sac phai doi T s moi dc do Dien ap.
		self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
		self.pre_timeVoltage = rospy.get_time()
		self.step_readVoltage = 0
		
		self.pub_requestMain = rospy.Publisher("/POWER_request", POWER_request, queue_size=100)	
		self.power_request = POWER_request()

		# --------- info
		self.pub_infoRespond = rospy.Publisher("/Server_infoRespond", Server_infoRespond, queue_size=100)
		self.Server_infoRespond = Server_infoRespond()
		# -- Move manual
		self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size = 4)
		self.cmdVel = Twist()

		# -- mode run - info
		self.modeOperate = 0 # (0: Bang tay) (1: Tu dong)
		self.robot_status = 0
		self.task_status = 0
		self.error_device = 0
		self.error_moving = 0
		self.error_perform = 0
		# -- speaker
		self.enb_runSpeaker = 0
		self.spkTyp_write = -1
		self.spkTyp_off = -1
		self.spkTyp_error = 0
		self.spkTyp_right = 1
		self.spkTyp_warn = 2
		self.spkTyp_honking = 3
		# -- changer
		self.charger_requir = 0 # lenh yeu cau tu server hoac app
		self.charger_write = 0
		self.charger_off = 0
		self.charger_on = 1
		self.charger_write = self.charger_off

		# -- Error
		self.value_error = 0

	def callback_main(self, dat):
		self.main_info = dat
		self.voltage = round(self.main_info.pin_voltage, 2)
		self.is_readMain = 1

	def robotPose_callback(self, data):
		self.robotPose_nav = data

	def appButton_callback(self, data):
		self.app_button = data
			
	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def run_maunal(self):
		cmd_vel = Twist()

		if (self.app_button.bt_forwards == True):
			cmd_vel.linear.x = 0.25
			cmd_vel.angular.z = 0.0

		if (self.app_button.bt_backwards == True):
			cmd_vel.linear.x = -0.25
			cmd_vel.angular.z = 0.0

		if (self.app_button.bt_rotation_left == True):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.3

		if (self.app_button.bt_rotation_right == True):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = -0.3

		if (self.app_button.bt_stop == True):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0

		return cmd_vel

	def Main_pub(self, charge, sound, EMC_write, EMC_reset):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		if sound == 0:
			mai.sound_on = 0	
		else:
			mai.sound_on = 1
			mai.sound_type = sound
		mai.EMC_write = EMC_write
		mai.EMC_reset = EMC_reset
		
		self.pub_requestMain.publish(mai)

	def readbatteryVoltage(self): # 
		time_curr = rospy.get_time()
		delta_time = (time_curr - self.pre_timeVoltage)
		if self.charger_requir == self.charger_on:
			self.flag_afterChager = 1
			if self.step_readVoltage == 0:  # bat sac.
				self.charger_write = self.charger_on
				if (delta_time > self.timeCheckVoltage_charger):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.charger_off
				if (delta_time > self.timeCheckVoltage_normal*3):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.main_info.pin_voltage, 1)*10
				# print "charger --"
				if  bat > 255:
					self.valueVoltage = 255
				elif bat < 0:
					self.valueVoltage = 0
				else:
					self.valueVoltage = bat

				self.pre_timeVoltage = time_curr
				self.step_readVoltage = 3

			elif self.step_readVoltage == 3: # doi.
				if (delta_time > 2):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 0

		elif self.charger_requir == self.charger_off:
			if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
				self.pre_timeVoltage = time_curr
				self.flag_afterChager = 0
				self.charger_write = self.charger_off
				self.step_readVoltage = 0
			else:
				if (delta_time > self.timeCheckVoltage_normal):
					self.pre_timeVoltage = time_curr
					bat = round(self.main_info.pin_voltage, 1)*10
					# print "normal --"
					if  bat > 255:
						self.valueVoltage = 255
					elif bat < 0:
						self.valueVoltage = 0
					else:
						self.valueVoltage = bat

	def run(self):
		while not rospy.is_shutdown():
			# --
			if (self.app_button.bt_passHand == 1):
				self.modeOperate = 0
			
			if (self.app_button.bt_passAuto == 1):
				self.modeOperate = 1

			# -- Control Mode
			if (self.modeOperate == 0): # dang o che do bang tay.
				# -- Move
				self.cmdVel = self.run_maunal()
				self.pub_cmdVel.publish(self.cmdVel)
				# -- Control Speaker
				if (self.enb_runSpeaker == 0):
					self.spkTyp_write = self.spkTyp_off
				else:
					if (self.value_error == 0):
						self.spkTyp_write = self.spkTyp_right
					else:
						self.spkTyp_write = self.spkTyp_error
			else:
				pass

			# -- -- -- Measure Voltage and Control Charger
		  	self.readbatteryVoltage()
			self.Server_infoRespond.battery = self.valueVoltage
			# -- -- -- Update info to Server
			self.Server_infoRespond.x = round(self.robotPose_nav.position.x, 3)
			self.Server_infoRespond.x = round(self.robotPose_nav.position.x, 3)
			angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)
			self.Server_infoRespond.z = round(angle, 2)

			self.Server_infoRespond.status = self.robot_status
			self.Server_infoRespond.task_status = self.task_status
			self.Server_infoRespond.error_device = self.error_device
			self.Server_infoRespond.error_moving = self.error_moving
			self.Server_infoRespond.error_perform = self.error_perform
			# --
			self.pub_infoRespond.publish(self.Server_infoRespond)
			# -- 
			self.rate.sleep()

def main():
	# Start the job threads
	program = sti_ctr()
	program.run()

if __name__ == '__main__':
	main()