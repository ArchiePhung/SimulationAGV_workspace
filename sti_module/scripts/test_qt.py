#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

class frameWork():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('frameWork', anonymous= False)
		self.rate = rospy.Rate(10)

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
		self.timeCheck_waitCharger = 1800 # s 180
		# --
		self.goTo_P1_lx = [1.370, 0.332, 5.784, 11.538, 11.538, 11.605, 12.1, 12.05, 12.3, 12.219, 12.218]
		self.goTo_P1_ly = [0.622, 1.287, 1.486, 1.441, 1.750, 9.24, 24.057, 25.326, 30.5, 35.2, 36.955]

		self.goTo_P2_lx = [12.218, 12.219, 12.3, 12.050, 12.100, 11.605, 11.538, 5.784, 0.332, 1.370]
		self.goTo_P2_ly = [36.955, 35.200, 30.5, 25.326, 24.057, 9.240, 1.750, 1.441, 1.486, 1.287, 0.622]
		# -- 
		self.goTo_chr_lx = [7.24, 4.72, 2.23, 0.23, -1.25, -2.54]
		self.goTo_chr_ly = [1.16, 1.64, 1.81, 1.27, 1.08, 2.08]

		self.goTo_P3_lx = [0.8, -0.2]
		self.goTo_P3_ly = [1.7, 0.7]

		# self.goTo_chr_lx = [7.24, 5.72, 4.53, 3.21, 0.85, -0.23, -1.25, -2.54]
		# self.goTo_chr_ly = [1.16, 0.76, 0.66, 0.44, 0.232, 0.01, 1.08, 2.08]
		self.chr_numChoose = -1
		self.chr_locateNow = -1
		self.is_completed_charger = 0
		self.is_completed_p1 = 0
		self.is_completed_p2 = 0
		self.is_completed_p3 = 0
		# -- 
		self.value_print = 0

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
		cmdRequest.before_mission = self.serverMission_liftDown
		cmdRequest.after_mission = self.serverMission_charger

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
			if (comp == 1 and self.goTo_chr_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
			
			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_chr_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_chr_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_chr_lx[length - 1])
				list_y.append(self.goTo_chr_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def target_getP1(self, pointNow_x, pointNow_y): # -- Lấy hàng tại vị trí P1.
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = 12.218
		cmdRequest.target_y = 36.955
		cmdRequest.target_z = 3.14
		cmdRequest.tag = 23
		cmdRequest.offset = 1.7
		cmdRequest.before_mission = self.serverMission_liftDown
		cmdRequest.after_mission = self.serverMission_liftUp
		cmdRequest.id_command = 0
		cmdRequest.command = "Get Item P1"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_P1_lx)
		if (self.chr_locateNow == -1):
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_P1_lx, self.goTo_P1_ly)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_P1_lx[self.chr_locateNow], self.goTo_P1_ly[self.chr_locateNow], 0.2)
			if (comp == 1 and self.goTo_P1_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
				if (self.chr_locateNow > (length - 1)):
					self.chr_locateNow = length	- 1		
			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_P1_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_P1_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_P1_lx[length - 1])
				list_y.append(self.goTo_P1_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def target_giveP1(self, pointNow_x, pointNow_y): # -- Trả hàng tại vị trí P1.
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = 12.218
		cmdRequest.target_y = 36.955
		cmdRequest.target_z = 3.14
		cmdRequest.tag = 23
		cmdRequest.offset = 1.7
		cmdRequest.before_mission = self.serverMission_liftUp
		cmdRequest.after_mission = self.serverMission_liftDown
		cmdRequest.id_command = 0
		cmdRequest.command = "Give Item P1"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_P1_lx)
		if (self.chr_locateNow == -1):
			print ("length: ", length)
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_P1_lx, self.goTo_P1_ly)
			print ("locateNow: ", self.chr_locateNow)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_P1_lx[self.chr_locateNow], self.goTo_P1_ly[self.chr_locateNow], 0.2)
			if (comp == 1 and self.goTo_P1_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
				if (self.chr_locateNow > (length - 1)):
					self.chr_locateNow = length	- 1
			
			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_P1_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_P1_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_P1_lx[length - 1])
				list_y.append(self.goTo_P1_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def target_getP2(self, pointNow_x, pointNow_y): # -- Lấy hàng tại vị trí P2.
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = 1.370
		cmdRequest.target_y = 0.662
		cmdRequest.target_z = -1.57
		cmdRequest.tag = 3
		cmdRequest.offset = 1.7
		cmdRequest.before_mission = self.serverMission_liftDown
		cmdRequest.after_mission = self.serverMission_liftUp
		cmdRequest.id_command = 0
		cmdRequest.command = "Get Item P2"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_P2_lx)
		if (self.chr_locateNow == -1):
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_P2_lx, self.goTo_P2_ly)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_P2_lx[self.chr_locateNow], self.goTo_P2_ly[self.chr_locateNow], 0.2)
			if (comp == 1 and self.goTo_P2_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
				if (self.chr_locateNow > (length - 1)):
					self.chr_locateNow = length	- 1

			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_P2_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_P2_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_P2_lx[length - 1])
				list_y.append(self.goTo_P2_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def target_giveP2(self, pointNow_x, pointNow_y): # -- Trả hàng tại vị trí P2.
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = 1.370
		cmdRequest.target_y = 0.662
		cmdRequest.target_z = -1.57
		cmdRequest.tag = 14
		cmdRequest.offset = 1.7
		cmdRequest.before_mission = self.serverMission_liftUp
		cmdRequest.after_mission = self.serverMission_liftDown
		cmdRequest.id_command = 0
		cmdRequest.command = "Give Item P2"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_P2_lx)
		if (self.chr_locateNow == -1):
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_P2_lx, self.goTo_P2_ly)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_P2_lx[self.chr_locateNow], self.goTo_P2_ly[self.chr_locateNow], 0.2)
			if (comp == 1 and self.goTo_P2_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
				if (self.chr_locateNow > (length - 1)):
					self.chr_locateNow = length	- 1		
			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_P2_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_P2_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_P2_lx[length - 1])
				list_y.append(self.goTo_P2_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def target_giveP3(self, pointNow_x, pointNow_y): # -- Trả hàng tại vị trí P3.
		cmdRequest = NN_cmdRequest()

		cmdRequest.target_x = -0.2
		cmdRequest.target_y = 0.7
		cmdRequest.target_z = -1.7
		cmdRequest.tag = 22
		cmdRequest.offset = 1.7
		cmdRequest.before_mission = self.serverMission_liftUp
		cmdRequest.after_mission = self.serverMission_liftDown
		cmdRequest.id_command = 0
		cmdRequest.command = "Give Item P3"

		# -- list
		list_x = []
		list_y = []
		length = len(self.goTo_P3_lx)
		if (self.chr_locateNow == -1):
			self.chr_locateNow = self.find_nearest_point(pointNow_x, pointNow_y, self.goTo_P3_lx, self.goTo_P3_ly)
		else:
			comp = self.check_arrived(pointNow_x, pointNow_y, self.goTo_P3_lx[self.chr_locateNow], self.goTo_P3_ly[self.chr_locateNow], 0.2)
			if (comp == 1 and self.goTo_P3_lx[self.chr_locateNow] != cmdRequest.target_x):
				self.chr_locateNow += 1
				if (self.chr_locateNow > (length - 1)):
					self.chr_locateNow = length	- 1		
			ll_ok = length - self.chr_locateNow
			if (ll_ok != 0):
				if (ll_ok > 5):
					ll_ok = 5

				for i in range(ll_ok):
					list_x.append(self.goTo_P3_lx[i + self.chr_locateNow])
					list_y.append(self.goTo_P3_ly[i + self.chr_locateNow])

				ll_not = 5 - ll_ok
				for i in range(ll_not):
					list_x.append(1000.)
					list_y.append(1000.)
			elif (ll_ok < 1):
				list_x.append(self.goTo_P3_lx[length - 1])
				list_y.append(self.goTo_P3_ly[length - 1])

				for i in range(4):
					list_x.append(1000.)
					list_y.append(1000.)

		cmdRequest.list_x = list_x
		cmdRequest.list_y = list_y

		return cmdRequest

	def process_single(self, typ):
		if (self.is_infoRespond):
			# -- Về sạc - Khi AMR đang đứng ở 1 vị trí bất kì.
			if (typ == 0):
				if (self.is_completed_charger == 0): # 
					print ("Runing: go to charger!")
					self.NN_cmdRequest = self.target_goCharger(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_charger = 1
				else: #
					print ("Completed: go to charger!")
			# -- Lấy hàng tại P1.
			elif (typ == 1):
				if (self.is_completed_p1 == 0): # 
					print ("Runing: Get P1!")
					self.NN_cmdRequest = self.target_getP1(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_p1 = 1
				else: #
					print ("Completed: Get P1!")
			# -- Trả hàng tại P1.
			elif (typ == 2):
				if (self.is_completed_p2 == 0): #
					print ("Runing: Give P1!")
					self.NN_cmdRequest = self.target_giveP1(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_p2 = 1
				else: #
					print ("Completed Give P1!")
			# -- Lấy hàng tại P2.
			elif (typ == 3):
				if (self.is_completed_p1 == 0): # 
					print ("Runing: Get P2!")
					self.NN_cmdRequest = self.target_getP2(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_p1 = 1
				else: #
					print ("Completed Get P2!")
			# -- Trả hàng tại P2.
			elif (typ == 4):
				if (self.is_completed_p2 == 0): # 
					print ("Runing: Give P2!")
					self.NN_cmdRequest = self.target_giveP2(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_p2 = 1
				else: #
					print ("Completed Give P2!")
			# -- Trả hàng tại P3.
			elif (typ == 5):
				if (self.is_completed_p3 == 0): # 
					print ("Runing: Give P3!")
					self.NN_cmdRequest = self.target_giveP3(self.NN_infoRespond.x, self.NN_infoRespond.y)
					if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
						self.is_completed_p3 = 1
				else: #
					print ("Completed Give P3!")
		self.pub_mission.publish(self.NN_cmdRequest)

	def process_auto(self):
		if (self.is_infoRespond):
			print ("step: ", self.step)
			if (self.step == 0): # ke hang o vi tri 1 + AMR khoang khong.		
				# print ("Runing: go to charger!")
				self.NN_cmdRequest = self.target_goCharger(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step += 1
					self.chr_locateNow = -1
					self.timeSave_waitCharger = time.time()
					self.value_print = 0

			elif (self.step == 1): # -- Chờ sạc.
				delta_time = time.time() - self.timeSave_waitCharger
				if (abs(delta_time - self.value_print) > 60):
					print ("delta_time: ", delta_time)
					self.value_print = delta_time

				if (delta_time > self.timeCheck_waitCharger):
					self.step += 1
					self.chr_locateNow = -1

			elif (self.step == 2): # -- Lấy hàng tại vị trí P1.
				self.NN_cmdRequest = self.target_getP1(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step += 1
					self.chr_locateNow = -1

			elif (self.step == 3): # --  Mang hàng tại vị trí P1 trả P2.
				self.NN_cmdRequest = self.target_giveP2(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step += 1
					self.chr_locateNow = -1

			elif (self.step == 4): # -- Về sạc.		
				self.NN_cmdRequest = self.target_goCharger(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step += 1
					self.chr_locateNow = -1
					self.timeSave_waitCharger = time.time()
					self.value_print = 0

			elif (self.step == 5): # -- Chờ sạc.
				delta_time = time.time() - self.timeSave_waitCharger
				if (abs(delta_time - self.value_print) > 60):
					print ("delta_time: ", delta_time)
					self.value_print = delta_time

				if (delta_time > self.timeCheck_waitCharger):
					self.step += 1
					self.chr_locateNow = -1

			elif (self.step == 6): # -- Lấy hàng tại vị trí P2.
				self.NN_cmdRequest = self.target_getP2(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step += 1
					self.chr_locateNow = -1
			elif (self.step == 7): # --  Mang hàng tại vị trí P2 trả P1.
				self.NN_cmdRequest = self.target_giveP1(self.NN_infoRespond.x, self.NN_infoRespond.y)
				if (self.NN_cmdRequest.after_mission == self.NN_infoRespond.task_status and self.NN_cmdRequest.tag == self.NN_infoRespond.tag):
					self.step = 0
					self.chr_locateNow = -1
		self.pub_mission.publish(self.NN_cmdRequest)

	def run(self):
		while not rospy.is_shutdown():
			# -- Lấy hàng tại P1. - 1
			# -- Trả hàng tại P1. - 2
			# -- Lấy hàng tại P2. - 3
			# -- Trả hàng tại P2. - 4
			self.process_single(5)
			# self.process_auto()

			self.rate.sleep()

def main():
	print('Program starting')
	program = frameWork()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
	main()

