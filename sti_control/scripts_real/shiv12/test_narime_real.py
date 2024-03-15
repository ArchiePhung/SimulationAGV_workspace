#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
Dev: Archie Phung
Date Modify: 08/03/2024

>> Connect and test process between AGV and NARIME CONVEYOR
>> Define Toyo data                                           
	- input
		+ bit1: conveyor 1 : hcu - input 1
		+ bit2: conveyor 2 : hcu - input 2
		+ bit3: conveyor 3 : hcu - input 3
		+ bit4: conveyor 4 : hcu - input 4
		+ bit5: connected  : hcu - input 5
		+ bit6: done       : hcu - input 6
		+ bit7: ready      : mcu - input 1 
		+ bit8: error      : mcu - input 2
	- output
		+ bit1: conveyor 1 : hcu - output 1
		+ bit2: conveyor 2 : hcu - output 2
		+ bit3: conveyor 3 : hcu - output 3
		+ bit4: conveyor 4 : hcu - output 4
		+ bit5: connected  : hcu - output 5
		+ bit6: done       : hcu - output 6
		+ bit7: ready      : mcu - output 1 
		+ bit8: error      : mcu - output 2
"""

import rospy
import math 

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *
from ros_canbus.msg import *

class TestCYNarime():
	def __init__(self):
		print("ROS Initial: TestCYNarime!")
		rospy.init_node('test_cy_narime', anonymous = False, disable_signals=True) # False

		self.rate = rospy.Rate(40)

		# -- Keyboard command 
		rospy.Subscriber("/Keyboard_cmd", Keyboard_command, self.callback_Keyboard) # 
		self.keyboard_cmd = Keyboard_command()	

		# -- Board OC 12 | conveyor11
		rospy.Subscriber("/status_conveyor1", Status_conveyor, self.callback_conveyor11) # 
		self.signal_conveyor11 = Status_conveyor()
		self.timeStampe_conveyor2 = rospy.Time.now()

		# -- Board OC 12 | conveyor12
		rospy.Subscriber("/status_conveyor2", Status_conveyor, self.callback_conveyor12) # 
		self.signal_conveyor12 = Status_conveyor()

		# -- Board OC 34 | conveyor21
		rospy.Subscriber("/status_conveyor3", Status_conveyor, self.callback_conveyor21) # 
		self.signal_conveyor21 = Status_conveyor()
		self.timeStampe_conveyor1 = rospy.Time.now()

		# -- Board OC 34 | conveyor22
		rospy.Subscriber("/status_conveyor4", Status_conveyor, self.callback_conveyor22) # 
		self.signal_conveyor22 = Status_conveyor()

		# - Board OC | Control Conveyors
		self.pub_controlConveyors = rospy.Publisher("/control_conveyors", Control_conveyors, queue_size= 20)	# Dieu khien ban nang.
		self.control_conveyors = Control_conveyors()

		# -- Board HC | Toyo
		rospy.Subscriber("/HCU_info", HCU_info, self.callback_HCUinfo) # 
		self.signal_HCUinfo = HCU_info()
		# -- Board MC | Toyo
		rospy.Subscriber("/MCU_info", MCU_info, self.callback_MCUinfo) # 
		self.signal_MCUinfo = MCU_info()

		# -- Board HC pub| Toyo 
		self.pub_HCUiorequest = rospy.Publisher("/HCU_io_request", HCU_io_request, queue_size= 20)	# Dieu khien Toyo.
		self.HCUiorequest = HCU_io_request()

		self.pub_MCUiorequest = rospy.Publisher("/MCU_io_request", MCU_io_request, queue_size= 20)	# Dieu khien Toyo.
		self.MCUiorequest = MCU_io_request()

		# -- CONST --
		self.MISSION_TRSM = "2"
		self.MISSION_RECV = "1"
		self.MISSION_EMPTY = "0"

		self.CYTASK_RECEIVE = 1
		self.CYTASK_TRANSMIT = 2
		self.CYTASK_STOP = 0

		self.CYTASKRCV_DONE = 3               # 3 nhận xong, 4 trả xong                             # Archie: update scripts of OC board
		self.CYTASKTRSM_DONE = 4
		self.CYTASK_UNDONE = 0
		self.CYTASK_RUNNING = 1
		self.CYTASKRCV_TIMERR = -1
		self.CYTASKRCV_UNKERR = -3

		self.CYTASKTRSM_TIMERR = -2
		self.CYTASKTRSM_UNKERR = -4

		self.FREQUENCEPUBBOARD = 10.
		self.TIME_CHECKPOS = 10.0
		self.TIME_LOG = 1
		self.TIME_CHECKFLOOR2 = 10.0
		self.TIME_CHECKDONE = 10.0

		# -- VAR -- 
		self.mission = self.MISSION_EMPTY
		self.enb_conduct = 0
		self.mission_val = 0

		self.input = ""
		self.is_exit = 0
		# ---------- flag var --------------
		self.flag_checkpos = 0
		self.enb_log = 0

		self.flag_mission = 0
		self.flag_cy = 0
		self.flag_reset = 0
		self.flag_returncy11 = 0
		self.flag_returncy12 = 0
		self.flag_returncy21 = 0
		self.flag_returncy22 = 0
		self.flag_returnfloor1 = 0
		self.flag_returnfloor2 = 0
		self.flag_returnAuto = 0
		self.flag_restart = 0

		self.completed_reset = 0
		self.completed_get_cy = 0
		self.completed_get_mission = 0

		# --------- process var ------------
		self.case_step2 = 0
		self.case_step1 = 0
		self.step = 0	
		self.step_checkpos = 0
		self.step_log = 0
		self.step_checkfloor2 = 0
		self.step_checkdone = 0

		# ----------- time var -------------
		self.pre_timeBoard = rospy.get_time()
		self.ct_checkpos = rospy.get_time()
		self.ct_log = rospy.get_time()
		self.ct_checkfloor2 = rospy.get_time()
		self.ct_checkdone = rospy.get_time()

	def callback_Keyboard(self, data):
		self.keyboard_cmd = data
		if self.keyboard_cmd.value == "":
			self.flag_cy = 0
			self.flag_mission = 0
			self.flag_reset = 0	

		elif self.keyboard_cmd.value == "get_cy":
			self.flag_cy = 1

		elif self.keyboard_cmd.value == "get_mission":
			self.flag_mission = 1
		
		elif self.keyboard_cmd.value == "pos":
			self.flag_checkpos = 1

		elif self.keyboard_cmd.value == "cy11":
			self.flag_returncy11 = 1						

		elif self.keyboard_cmd.value == "cy12":
			self.flag_returncy12 = 1

		elif self.keyboard_cmd.value == "cy21":
			self.flag_returncy21 = 1

		elif self.keyboard_cmd.value == "cy22":
			self.flag_returncy22 = 1

		elif self.keyboard_cmd.value == "floor1":
			self.flag_returnfloor1 = 1

		elif self.keyboard_cmd.value == "floor2":
			self.flag_returnfloor2 = 1

		elif self.keyboard_cmd.value == "run":
			self.flag_returnAuto = 1

		elif self.keyboard_cmd.value == "reset":
			self.flag_reset = 1

		elif self.keyboard_cmd.value == "restart":
			self.flag_restart = 1

	def callback_conveyor12(self, data):
		self.signal_conveyor12 = data

	def callback_conveyor22(self, data):
		self.signal_conveyor22 = data

	def callback_conveyor11(self, data):
		self.signal_conveyor11 = data

	def callback_conveyor21(self, data):
		self.signal_conveyor21 = data

	def callback_HCUinfo(self, data):
		self.signal_HCUinfo = data

	def callback_MCUinfo(self, data):
		self.signal_MCUinfo = data

	def log_mess(self, mess):
		if self.enb_log == 1:
			if self.step_log == 0:
				self.ct_log = rospy.get_time()
				self.step_log = 1
				print(mess)
			else:
				if rospy.get_time() - self.ct_log >= self.TIME_LOG:
					print(mess)
					self.ct_log = rospy.get_time()
		else:
			self.step_log = 0
			
	def confirm_pos(self):                       ## Setup time check xác nhận vị trí
		if self.step_checkpos == 0:
			self.HCUiorequest.output5 = 1
			self.ct_checkpos = rospy.get_time()
			self.step_checkpos = 1
			self.enb_log = 1
		else:
			if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
				self.log_mess("Đang chờ xác nhận vị trí với NARIME_CY")
				if self.signal_HCUinfo.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
					print("AGV đã xác nhận vị trí với NARIME_CY")
					self.enb_conduct = 1
					self.flag_checkpos = 0
					self.step_checkpos = 0
					self.enb_log = 0
			
			else:
				print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
				self.enb_conduct = 0
				self.flag_checkpos = 0
				self.HCUiorequest.output5 = 0
				self.step_checkpos = 0
				self.enb_log = 0

	def get_mission(self):
		self.mission = input("Please enter mission number: ")
		if self.mission == self.MISSION_RECV:
			print("Xác nhận AGV muốn nhận hàng")
		elif self.mission == self.MISSION_TRSM:
			print("Xác nhận AGV muốn trả hàng")
		self.completed_get_mission = 1

	def get_cy_number(self):
		self.completed_get_cy = 1
		cy_val1 = 0
		cy_val2 = 0
		cy_val3 = 0
		cy_val4 = 0
		if self.mission == self.MISSION_TRSM:
			if self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0001
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0010
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0100
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1000
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0011
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0101	
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1001
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0110
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1010						
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1100
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 0:
				self.mission_val = 0b0111						
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 0 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1011
			elif self.signal_conveyor11.sensor_checkPiece == 0 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1110
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 0 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1101
			elif self.signal_conveyor11.sensor_checkPiece == 1 and self.signal_conveyor12.sensor_checkPiece == 1 and self.signal_conveyor21.sensor_checkPiece == 1 and self.signal_conveyor22.sensor_checkPiece == 1:
				self.mission_val = 0b1111
			else:
				self.mission_val = 0
				print("AGV không phát hiện thùng hàng nào để  trả!")
			
			print("Các thùng hàng cần trả là: ", bin(self.mission_val))

		elif self.mission == self.MISSION_RECV:	
			if self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 0
			elif self.signal_conveyor11.sensor_limitAhead == 1 and self.signal_conveyor11.sensor_limitBehind == 0:
				cy_val1 = 1
			elif self.signal_conveyor11.sensor_limitAhead == 0 and self.signal_conveyor11.sensor_limitBehind == 1:
				cy_val1 = 1
			else:
				cy_val1 = 1

			if self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 0
			elif self.signal_conveyor12.sensor_limitAhead == 1 and self.signal_conveyor12.sensor_limitBehind == 0:
				cy_val2 = 1
			elif self.signal_conveyor12.sensor_limitAhead == 0 and self.signal_conveyor12.sensor_limitBehind == 1:
				cy_val2 = 1
			else:
				cy_val2 = 1

			if self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 0:
				cy_val3 = 0
			elif self.signal_conveyor21.sensor_limitAhead == 1 and self.signal_conveyor21.sensor_limitBehind == 0:
				cy_val3 = 1
			elif self.signal_conveyor21.sensor_limitAhead == 0 and self.signal_conveyor21.sensor_limitBehind == 1:
				cy_val3 = 1
			else:
				cy_val3 = 1

			if self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 0:
				cy_val4 = 0
			elif self.signal_conveyor22.sensor_limitAhead == 1 and self.signal_conveyor22.sensor_limitBehind == 0:
				cy_val4 = 1
			elif self.signal_conveyor22.sensor_limitAhead == 0 and self.signal_conveyor22.sensor_limitBehind == 1:
				cy_val4 = 1
			else:
				cy_val4 = 1

			if cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 1:
				self.mission_val = 0b0001
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 1:
				self.mission_val = 0b0010
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 1:
				self.mission_val = 0b0100
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 0:
				self.mission_val = 0b1000
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 1:
				self.mission_val = 0b0011
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 1:
				self.mission_val = 0b0101
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 1 and cy_val4 == 0:
				self.mission_val = 0b1001
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 1:
				self.mission_val = 0b0110	
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 0:
				self.mission_val = 0b1010
			elif cy_val1 == 1 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 0:
				self.mission_val = 0b1100
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 1:
				self.mission_val = 0b0111
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 1 and cy_val4 == 0:
				self.mission_val = 0b1011
			elif cy_val1 == 1 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 0:
				self.mission_val = 0b1110
			elif cy_val1 == 0 and cy_val2 == 1 and cy_val3 == 0 and cy_val4 == 0:
				self.mission_val = 0b1101
			elif cy_val1 == 0 and cy_val2 == 0 and cy_val3 == 0 and cy_val4 == 0:
				self.mission_val = 0b1111
			else:
				self.mission_val = 0
				print("AGV Đã đủ hàng nên không thể nhận!")

			print("Các thùng hàng cần nhận là: ", bin(self.mission_val))
		else:
			print("Chưa xác nhận nhiệm vụ cho AGV")
			self.mission_val = 0

	def run_auto(self):
		if self.mission == self.MISSION_TRSM:
			if self.case_step2 == 0:
				self.enb_log = 1
				self.log_mess("Im here -- case_step2 == 0")
				# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
				if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
					self.HCUiorequest.output1 = 1
					self.HCUiorequest.output2 = 0

				# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
				elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
					self.HCUiorequest.output1 = 0
					self.HCUiorequest.output2 = 1

				# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
				elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
					self.HCUiorequest.output1 = 1
					self.HCUiorequest.output2 = 1

				# - 21 or 22 or (21 && 22)
				elif self.mission_val == 0b0100 or self.mission_val == 0b1000 or self.mission_val == 0b1100:
					self.case_step2 = 5

				if self.step_checkpos == 0:
					self.HCUiorequest.output5 = 1
					self.ct_checkpos = rospy.get_time()
					self.step_checkpos = 1
					self.enb_log = 1
				else:
					if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
						self.log_mess("Đang chờ xác nhận vị trí với NARIME_CY")
						if self.signal_HCUinfo.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
							print("AGV đã xác nhận vị trí với NARIME_CY")
							self.step_checkpos = 0
							self.enb_log = 0
							self.case_step2 = 1
					
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
						self.HCUiorequest.output5 = 0
						self.step_checkpos = 0
						self.enb_log = 0
						self.case_step2 = 0

			elif self.case_step2 == 1:
				self.log_mess("AGV is trasmiting tray to NARIME CY in step 1")
				if self.mission_val == 0b0001:
					self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10

				elif self.mission_val == 0b0010:
					self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10

				elif self.mission_val == 0b0011:
					self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
					self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10
					
				# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
				elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
					self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor11.status == self.CYTASKTRSM_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.case_step2 = 5
				# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
				elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
					self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step2 = 5														
				# - (11 && 12 && 21) or (11 && 12 && 22)
				elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
					self.control_conveyors.No1_mission = self.CYTASK_TRANSMIT
					self.control_conveyors.No2_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor11.status == self.CYTASKTRSM_DONE and self.signal_conveyor12.status == self.CYTASKTRSM_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step2 = 5	

			elif self.case_step2 == 5:	
				self.HCUiorequest.output1 = 0
				self.HCUiorequest.output2 = 0							
				# - Wait CYM1 confirm 
				if self.step_checkfloor2 == 0:
					self.ct_checkfloor2 = rospy.get_time()
					self.step_checkfloor2 = 1

					if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
						self.HCUiorequest.output3 = 1
					elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
						self.HCUiorequest.output4 = 1
					elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
						self.HCUiorequest.output3 = 1
						self.HCUiorequest.output4 = 1	
				else:
					if rospy.get_time() - self.ct_checkfloor2 < self.TIME_CHECKFLOOR2:
						self.log_mess("Đang chờ xác nhận vị trí với NARIME_CY")
						if self.signal_MCUinfo.status_input1 == 1:  
							print("AGV đã xác nhận vị trí với NARIME_CY")                  
							self.case_step2 = 6
							self.step_checkfloor2 = 0
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
						self.step_checkfloor2 = 0
						self.case_step2 = 0
						self.flag_returnAuto = 0
						self.enb_conduct = 0
			
			elif self.case_step2 == 6:
				self.log_mess("AGV is trasmiting tray to NARIME CY in step 2")
				if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
					self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor21.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10
				elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
					self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10																
				elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
					self.control_conveyors.No3_mission = self.CYTASK_TRANSMIT
					self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor21.status == self.CYTASKTRSM_DONE and self.signal_conveyor22.status == self.CYTASKTRSM_DONE:
						self.case_step2 = 10

			elif self.case_step2 == 10:
				# - Wait CYM1 confirm 
				if self.step_checkdone == 0:
					self.ct_checkdone = rospy.get_time()
					self.step_checkdone = 1
					self.HCUiorequest.output6 = 1
				else:
					if rospy.get_time() - self.ct_checkdone < self.TIME_CHECKDONE:
						self.log_mess("AGV transmits done! Wait confirm done signal from NARIME CY")

						if self.signal_HCUinfo.status_input6 == 1:
							print("Cho phép AGV thực hiện nhiệm vụ tiếp theo!")
							self.flag_returnAuto = 0
							self.enb_log = 0
							self.case_step2 = 0
							self.HCUiorequest = HCU_io_request()
							self.control_conveyors = Control_conveyors()		
							self.enb_conduct = 0		
							self.step_checkdone = 0
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
						self.flag_returnAuto = 0
						self.enb_log = 0
						self.case_step2 = 0
						self.HCUiorequest = HCU_io_request()
						self.control_conveyors = Control_conveyors()		
						self.enb_conduct = 0		
						self.step_checkdone = 0

		elif self.mission == self.MISSION_RECV:
			if self.case_step1 == 0:
				self.enb_log = 1
				self.log_mess("Im here -- case_step1 == 0")
				# - 11 - (11 && 21) or (11 && 22) - (11 && 21 && 22)
				if self.mission_val == 0b0001 or self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
					self.HCUiorequest.output1 = 1
					self.HCUiorequest.output2 = 0
				# - 12 - (12 && 21) or (12 && 22) - (12 && 21 && 22)
				elif self.mission_val == 0b0010 or self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
					self.HCUiorequest.output1 = 0
					self.HCUiorequest.output2 = 1
				# - 11 && 12 - (11 && 12 && 21) or (11 && 12 && 22) or (11 && 12 && 21 & 22)
				elif self.mission_val == 0b0011 or self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111:
					self.HCUiorequest.output1 = 1
					self.HCUiorequest.output2 = 1

				# - 21 or 22 or (21 && 22)
				elif self.mission_val == 0b0100 or self.mission_val == 0b1000 or self.mission_val == 0b1100:
					self.case_step1 = 5

				if self.step_checkpos == 0:
					self.HCUiorequest.output5 = 1
					self.ct_checkpos = rospy.get_time()
					self.step_checkpos = 1
					self.enb_log = 1
				else:
					if rospy.get_time() - self.ct_checkpos < self.TIME_CHECKPOS:
						self.log_mess("Đang chờ xác nhận vị trí với NARIME_CY")
						if self.signal_HCUinfo.status_input5 == 1:    # Conveyor xác nhận vị trí với AGV + phần băng tải 1 đã ở vị trí sẵn sàng
							print("AGV đã xác nhận vị trí với NARIME_CY")
							self.step_checkpos = 0
							self.enb_log = 0
							self.case_step1 = 1
					
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")   #>> Thêm cờ báo lỗi ở đây
						self.HCUiorequest.output5 = 0
						self.step_checkpos = 0
						self.enb_log = 0
						self.case_step1 = 0

			elif self.case_step1 == 1:
				self.log_mess("AGV is receiving tray to NARIME CY in step 1")
				if self.mission_val == 0b0001:
					self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.case_step1 = 10

				elif self.mission_val == 0b0010:
					self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step1 = 10	

				elif self.mission_val == 0b0011:
					self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
					self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step1 = 10
					
				# - 11 && 21 or 11 && 22 - (11 && 21 - 22)
				elif self.mission_val == 0b0101 or self.mission_val == 0b1001 or self.mission_val == 0b1101:
					self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor11.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.case_step1 = 5

				# - 12 && 21 or 12 && 22 - (12 && 21 - 22)
				elif self.mission_val == 0b0110 or self.mission_val == 0b1010 or self.mission_val == 0b1110:
					self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor12.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step1 = 5			

				# - (11 && 12 && 21) or (11 && 12 && 22)
				elif self.mission_val == 0b0111 or self.mission_val == 0b1011 or self.mission_val == 0b1111: 
					self.control_conveyors.No1_mission = self.CYTASK_RECEIVE
					self.control_conveyors.No2_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor11.status == self.CYTASKRCV_DONE and self.signal_conveyor12.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No1_mission = self.CYTASK_STOP
						self.control_conveyors.No2_mission = self.CYTASK_STOP
						self.case_step1 = 5	

			elif self.case_step1 == 5:							
				self.HCUiorequest.output1 = 0
				self.HCUiorequest.output2 = 0

				# - Wait CYM1 confirm 
				if self.step_checkfloor2 == 0:
					self.ct_checkfloor2 = rospy.get_time()
					self.step_checkfloor2 = 1

					if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111 or self.mission_val == 0b0100:
						self.HCUiorequest.output3 = 1
					elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011 or self.mission_val == 0b1000:
						self.HCUiorequest.output4 = 1
					elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111 or self.mission_val == 0b1100:
						self.HCUiorequest.output3 = 1
						self.HCUiorequest.output4 = 1			
				else:
					if rospy.get_time() - self.ct_checkfloor2 < self.TIME_CHECKFLOOR2:
						self.log_mess("Đang chờ xác nhận vị trí với NARIME_CY")
						if self.signal_MCUinfo.status_input1 == 1:  
							print("AGV đã xác nhận vị trí với NARIME_CY")                  
							self.case_step1 = 6
							self.step_checkfloor2 = 0
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
						self.step_checkfloor2 = 0
						self.case_step1 = 0
						self.flag_returnAuto = 0
						self.enb_conduct = 0
			
			elif self.case_step1 == 6:
				self.log_mess("AGV is receiving tray to NARIME CY in step 2")
				if self.mission_val == 0b0101 or self.mission_val == 0b0110 or self.mission_val == 0b0111:
					self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor21.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No3_mission = self.CYTASK_STOP
						self.enb_log = 1
						self.case_step1 = 10
				elif self.mission_val == 0b1001 or self.mission_val == 0b1010 or self.mission_val == 0b1011:
					self.control_conveyors.No4_mission = self.CYTASK_RECEIVE
					if self.signal_conveyor22.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No4_mission = self.CYTASK_STOP
						self.enb_log = 1
						self.case_step1 = 10																
				elif self.mission_val == 0b1101 or self.mission_val == 0b1110 or self.mission_val == 0b1111:
					self.control_conveyors.No3_mission = self.CYTASK_RECEIVE
					self.control_conveyors.No4_mission = self.CYTASK_TRANSMIT
					if self.signal_conveyor21.status == self.CYTASKRCV_DONE and self.signal_conveyor22.status == self.CYTASKRCV_DONE:
						self.control_conveyors.No3_mission = self.CYTASK_STOP
						self.control_conveyors.No4_mission = self.CYTASK_STOP
						self.enb_log = 1
						self.case_step1 = 10

			elif self.case_step1 == 10:
				# - Wait CYM1 confirm 
				if self.step_checkdone == 0:
					self.ct_checkdone = rospy.get_time()
					self.step_checkdone = 1
					self.HCUiorequest.output6 = 1
				else:
					if rospy.get_time() - self.ct_checkdone < self.TIME_CHECKDONE:
						self.log_mess("AGV transmits done! Wait confirm done signal from NARIME CY")

						if self.signal_HCUinfo.status_input6 == 1:
							print("Cho phép AGV thực hiện nhiệm vụ tiếp theo!")
							self.flag_returnAuto = 0
							self.enb_log = 0
							self.case_step1 = 0
							self.HCUiorequest = HCU_io_request()
							self.control_conveyors = Control_conveyors()		
							self.enb_conduct = 0		
							self.step_checkdone = 0
					else:
						print("Connection Timeout! AGV không xác nhận vị trí với NARIME_CY")     # >>>> Thêm cờ báo lỗi ở đây
						self.flag_returnAuto = 0
						self.enb_log = 0
						self.case_step1 = 0
						self.HCUiorequest = HCU_io_request()
						self.control_conveyors = Control_conveyors()		
						self.enb_conduct = 0		
						self.step_checkdone = 0

		elif self.enb_conduct == 0:
			print("AGV chưa xác nhận vị trí")
			self.flag_returnAuto = 0

	def test_CYseparate(self, b1, b2, b3, b4, mss1, mss2, mss3, mss4, mssdone1, mssdone2, mssdone3, mssdone4):
		if self.enb_conduct == 1:
			if self.step == 0:
				if b3 == 0 and b4 == 0:
					self.HCUiorequest.output1 = b1
					self.HCUiorequest.output2 = b2
					self.step = 1
				else:
					self.HCUiorequest.output3 = b3
					self.HCUiorequest.output4 = b4
					self.HCUiorequest.output5 = 1
					if self.signal_MCUinfo.status_input1 == 1:
						self.step = 1
						self.HCUiorequest.output5 = 0
						print("AGV đã sẵn sàng trả/nhận tại tầng 2")

			elif self.step == 1:
				self.control_conveyors.No1_mission = mss1
				self.control_conveyors.No2_mission = mss2
				self.control_conveyors.No3_mission = mss3
				self.control_conveyors.No4_mission = mss4

				if self.signal_conveyor11.status == mssdone1 and self.signal_conveyor12.status == mssdone2 and self.signal_conveyor21.status == mssdone3 and self.signal_conveyor22.status == mssdone4:
					self.step = 2
					self.control_conveyors = Control_conveyors()
					self.enb_log = 1

			elif self.step == 2:
				# print("AGV đã trả/nhận xong hàng ")
				self.log_mess("Wait for confirm done signal from NARIME Conveyor")
				self.HCUiorequest.output6 = 1

				if self.signal_HCUinfo.status_input6 == 1:
					print("Cho phép AGV thực hiện nhiệm vụ tiếp theo!")
					self.step = 3
					self.mission = "0"
					self.HCUiorequest = HCU_io_request()
					self.control_conveyors = Control_conveyors()
					self.enb_log = 0
		else:
			print("Please confirm pos with NARIME CY")
			self.step = 3

	def reset(self):
		self.completed_reset = 1

		self.flag_checkpos = 0
		self.flag_mission = 0
		self.flag_cy = 0
		self.flag_reset = 0
		self.flag_returncy11 = 0
		self.flag_returncy12 = 0
		self.flag_returncy21 = 0
		self.flag_returncy22 = 0
		self.flag_returnfloor1 = 0
		self.flag_returnfloor2 = 0
		self.flag_returnAuto = 0

		self.enb_conduct = 0
		self.step = 0
		self.step_log = 0
		self.case_step1 = 0
		self.case_step2 = 0

		self.mission = "0"
		self.HCUiorequest = HCU_io_request()
		self.control_conveyors = Control_conveyors()		
		print("Reset hệ thống!")

	def restart(self):
		self.completed_get_cy = 0
		self.completed_get_mission = 0
		self.completed_reset = 0
		self.flag_restart = 0
		print("Restart hệ thống!")		

	def shutdown(self):
		self.is_exit = 1

	def run(self):
		try:
			if self.is_exit == 0:
				while not rospy.is_shutdown():
					if self.flag_restart == 1:
						self.restart()

					if self.flag_cy == 1 and self.completed_get_cy == 0:
						self.get_cy_number()
					
					if self.flag_mission == 1 and self.completed_get_mission == 0:
						self.get_mission()

					if self.flag_reset == 1 and self.completed_reset == 0:
						self.reset()

					if self.flag_checkpos == 1:
						self.confirm_pos()

					if self.flag_returnAuto == 1:
						self.run_auto()

					if self.flag_returncy11 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(1,0,0,0,2,0,0,0,4,0,0,0)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(1,0,0,0,1,0,0,0,3,0,0,0)
						elif self.mission == "0":
							self.flag_returncy11 = 0
							print("Please enter mission first!")

						if self.step == 3:
							self.flag_returncy11 = 0
							self.step = 0

					elif self.flag_returncy12 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(0,1,0,0,0,2,0,0,0,4,0,0)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(0,1,0,0,0,1,0,0,0,3,0,0)
						elif self.mission == "0":
							self.flag_returncy12 = 0
							print("Please enter mission first!")
						if self.step == 3:
							self.flag_returncy12 = 0
							self.step = 0

					elif self.flag_returncy21 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(0,0,1,0,0,0,2,0,0,0,4,0)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(0,0,1,0,0,0,1,0,0,0,3,0)
						elif self.mission == "0":
							self.flag_returncy21 = 0
							print("Please enter mission first!")

						if self.step == 3:
							self.flag_returncy21 = 0
							self.step = 0

					elif self.flag_returncy22 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(0,0,0,1,0,0,0,2,0,0,0,4)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(0,0,0,1,0,0,0,1,0,0,0,3)
						elif self.mission == "0":
							self.flag_returncy22 = 0
							print("Please enter mission first!")

						if self.step == 3:
							self.flag_returncy22 = 0
							self.step = 0

					elif self.flag_returnfloor1 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(1,1,0,0,2,2,0,0,4,4,0,0)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(1,1,0,0,1,1,0,0,3,3,0,0)
						elif self.mission == "0":
							self.flag_returnfloor1 = 0
							print("Please enter mission first!")

						if self.step == 3:
							self.flag_returnfloor1 = 0
							self.step = 0

					elif self.flag_returnfloor2 == 1:
						if self.mission == self.MISSION_TRSM:
							self.test_CYseparate(0,0,1,1,0,0,2,2,0,0,4,4)
						elif self.mission == self.MISSION_RECV:
							self.test_CYseparate(0,0,1,1,0,0,1,1,0,0,3,3)
						elif self.mission == "0":
							self.flag_returnfloor2 = 0
							print("Please enter mission first!")							

						if self.step == 3:
							self.flag_returnfloor2 = 0
							self.step = 0

					# -- BOARD -- 
					time_curr = rospy.get_time()
					d = (time_curr - self.pre_timeBoard)
					if (d > float(1/self.FREQUENCEPUBBOARD)): # < 20hz 
						self.pre_timeBoard = time_curr

						# -------- Request Board OC - Conveyors -------- #
						self.pub_controlConveyors.publish(self.control_conveyors)

					# -- GUI TEST -- 
					self.pub_HCUiorequest.publish(self.HCUiorequest)

					self.rate.sleep()
		except KeyboardInterrupt:
			rospy.on_shutdown(self.shutdown)
			self.is_exit = 1
			print('!!FINISH!!')

def main():
	print('Starting main program')
	program = TestCYNarime()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    