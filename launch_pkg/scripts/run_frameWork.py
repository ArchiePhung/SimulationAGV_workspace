#!/usr/bin/env python
# Author: Hoang Van Quang - BEE
# Date: 11/06/2021

import roslaunch
import rospy
import time
import os
import sys
from math import *
from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16

class kickoff():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('frameWork', anonymous= False)
		self.rate = rospy.Rate(20)

		# -- 
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.callBack_infoRespond)
		self.NN_infoRespond = NN_infoRespond()
		self.is_infoRespond = 0

		self.pub_mission = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size= 10)
		self.NN_cmdRequest = NN_cmdRequest()

		self.step = 0

		self.statusTask_liftError = 64 # trang thai nang ke nhung ko co ke.
		self.serverMission_liftUp = 65
		self.serverMission_liftDown = 66
		self.serverMission_charger = 5
		self.serverMission_unknown = 0
		self.serverMission_liftDown_charger = 6
		# -- 
		self.timeSave_waitCharger = time.time()
		self.timeCheck_waitCharger = 180 # s
		# --
		self.goTo_P1_lx = [7.24, 5.72, 4.53, 3.21, 0.85, -0.23, -1.25, -2.02]
		self.goTo_P1_ly = [1.16, 0.76, 0.66, 0.44, 0.232, 0.01, 1.08, -0.18]

		self.goTo_P2_lx = [-1.25, -0.23, 0.85, 3.21, 4.53, 5.72, 7.24]
		self.goTo_P2_ly = [1.08, 0.01, 0.232, 0.44, 0.66, 0.76, 1.16]
		# -- 
		self.goTo_chr_lx = [7.24, 4.72, 2.23, 0.23, -1.25, -2.54]
		self.goTo_chr_ly = [1.16, 1.64, 1.81, 1.27, 1.08, 2.08]

		# self.goTo_chr_lx = [7.24, 5.72, 4.53, 3.21, 0.85, -0.23, -1.25, -2.54]
		# self.goTo_chr_ly = [1.16, 0.76, 0.66, 0.44, 0.232, 0.01, 1.08, 2.08]
		self.chr_numChoose = -1
		self.chr_locateNow = -1

	def callBack_infoRespond(self, data):
		self.NN_infoRespond = data
		self.is_infoRespond = 1

	def calculate_distance(self, p1_x, p1_y, p2_x, p2_y): 
		distance = sqrt(pow((p1_x - p2_x), 2) + pow((p2_y - p2_y), 2))
		return distance

	def find_nearest_point(self, point_x, point_y, pointList_x, pointList_y): # my_point | Point()
		length = len(pointList_x)
		min_distance = 1000.
		num_choose = -1
		for num in range(length):
			distance = self.calculate_distance(point_x, point_y, pointList_x[num], pointList_y[num])
			if (min_distance >= distance):
				min_distance = distance
				num_choose = num
			print ("num: ", num)
			print ("distance: ", distance)
		# return pointList_x[num_choose], pointList_y[num_choose]
		print ("num_choose: ", num_choose)
		return num_choose

	def check_arrived(self, p_x, p_y, t_x, t_y, offset):
		dis = self.calculate_distance(p_x, p_y, t_x, t_y)
		if (dis <= offset):
			return 1
		else:
			return 0

	def target_goCharger(self, pointNow_x, pointNow_y):
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = -2.54
		cmdRequest.target_y = 2.08
		cmdRequest.target_z = 1.57
		cmdRequest.tag = 41 # 41
		cmdRequest.offset = 1.6
		# cmdRequest.before_mission = 0
		# cmdRequest.after_mission = 0
		cmdRequest.before_mission = self.serverMission_liftUp
		cmdRequest.after_mission = self.serverMission_liftDown

		cmdRequest.id_command = 0
		cmdRequest.command = "Go To Charger"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_chr_lx)
		if (self.chr_locateNow == -1):
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_chr_lx, self.goTo_chr_ly)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_chr_lx[self.chr_locateNow], self.goTo_chr_ly[self.chr_locateNow], 0.2)
			if (comp == 1):
				self.chr_locateNow += 1
			
			ll_ok = length - self.chr_locateNow
			if (ll_ok > 5):
				ll_ok = 5
			if (ll_ok == 0):
				ll_ok = 1

			ll_not = 5 - ll_ok

			for i in range(ll_ok):
				list_x.append(self.goTo_chr_lx[i + self.chr_locateNow])
				list_y.append(self.goTo_chr_ly[i + self.chr_locateNow])

			for i in range(ll_not):
				list_x.append(1000.)
				list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	# def target_getItem_p1(self):
	# 	cmdRequest = NN_cmdRequest()

	# 	cmdRequest.target_x = -2.02
	# 	cmdRequest.target_y = -0.18
	# 	cmdRequest.target_z = 3.14
	# 	cmdRequest.tag = 13
	# 	cmdRequest.offset = 1.8
	# 	cmdRequest.before_mission = self.serverMission_liftDown
	# 	cmdRequest.after_mission = self.serverMission_liftUp
	# 	cmdRequest.id_command = 0
	# 	cmdRequest.command = "Get Item P1"

	# 	cmdRequest.list_x = 
	# 	cmdRequest.list_y = 
		
	# 	return cmdRequest

	# def target_returnItem_p1(self):
	# 	cmdRequest = NN_cmdRequest()

	# 	cmdRequest.target_x = -2.02
	# 	cmdRequest.target_y = -0.18
	# 	cmdRequest.target_z = 3.14
	# 	cmdRequest.tag = 13
	# 	cmdRequest.offset = 1.8
	# 	cmdRequest.before_mission = self.serverMission_liftUp
	# 	cmdRequest.after_mission = self.serverMission_liftDown
	# 	cmdRequest.id_command = 0
	# 	cmdRequest.command = "Return Item P1"

	# 	cmdRequest.list_x = 
	# 	cmdRequest.list_y = 
		
	# 	return cmdRequest

	# def target_getItem_p2(self):
	# 	cmdRequest = NN_cmdRequest()

	# 	cmdRequest.target_x = 7.24
	# 	cmdRequest.target_y = 1.16
	# 	cmdRequest.target_z = -1.57
	# 	cmdRequest.tag = 14
	# 	cmdRequest.offset = 1.8
	# 	cmdRequest.before_mission = self.serverMission_liftDown
	# 	cmdRequest.after_mission = self.serverMission_liftUp
	# 	cmdRequest.id_command = 0
	# 	cmdRequest.command = "Get Item P2"

	# 	cmdRequest.list_x = 
	# 	cmdRequest.list_y = 
		
	# 	return cmdRequest

	# def target_returnItem_p2(self):
	# 	cmdRequest = NN_cmdRequest()

	# 	cmdRequest.target_x = 7.24
	# 	cmdRequest.target_y = 1.16
	# 	cmdRequest.target_z = -1.57
	# 	cmdRequest.tag = 14
	# 	cmdRequest.offset = 1.8
	# 	cmdRequest.before_mission = self.serverMission_liftUp
	# 	cmdRequest.after_mission = self.serverMission_liftDown
	# 	cmdRequest.id_command = 0
	# 	cmdRequest.command = "Return Item P2"

	# 	cmdRequest.list_x = 
	# 	cmdRequest.list_y = 
		
	# 	return cmdRequest

	def run(self):
		while not rospy.is_shutdown():
		    # -- firstWork
			if (self.is_infoRespond):
				if (self.step == 0): # ke hang o vi tri 1 + AMR khoang khong.
					# -- Ve sac
					self.NN_cmdRequest = self.target_goCharger(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.step = 1	
						self.timeSave_waitCharger = time.time()

				elif (self.step == 1): #
					print ("step: ", self.step)
					# -- wait to charger
					delta_time = time.time() - self.timeSave_waitCharger
					if (delta_time > self.timeCheck_waitCharger):
						self.step = 2

			# elif (self.step == 2):
			# 	# -- lay hang tai vi tri 1.
			# 	self.NN_cmdRequest = self.target_getItem_p1()
			# 	if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status):
			# 		self.step = 3

			# elif (self.step == 3):
			# 	# -- tra hang tai vi tri 2.
			# 	self.NN_cmdRequest = self.target_returnItem_p2()
			# 	if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status):
			# 		self.step = 4

			# elif (self.step == 4):
			# 	# -- ve sac
			# 	self.NN_cmdRequest = self.target_getItem_p1()
			# 	if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status):
			# 		self.step = 5
			# 		self.timeSave_waitCharger = time.time()

			# elif (self.step == 5):
			# 	# -- wait to charger
			# 	delta_time = time.time() - self.timeSave_waitCharger
			# 	if (delta_time > self.timeCheck_waitCharger):
			# 		self.step = 6

			# elif (self.step == 6):
			# 	# -- Lay hang tai vi tri 2.
			# 	self.NN_cmdRequest = self.target_getItem_p2()
			# 	if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status):
			# 		self.step = 7

			# elif (self.step == 7):
			# 	# -- Tra hang tai vi tri 1.
			# 	self.NN_cmdRequest = self.target_returnItem_p1()
			# 	if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status):
			# 		self.step = 0

			self.pub_mission.publish(self.NN_cmdRequest)

			self.rate.sleep()

def main():
	print('Program starting')
	try:
	    program = kickoff()
	    program.run()
	except rospy.ROSInterruptException:
		pass
	print('Programer stopped')

if __name__ == '__main__':
	main()

