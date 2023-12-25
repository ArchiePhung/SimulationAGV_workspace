#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 03/08/2022
# UPATE: 17/03/2023

# from message_pkg.msg import *
from message_pkg.msg import CAN_send, CAN_status, CAN_received, CPD_read, CPD_write
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canBus.msg import *

class CAN_ROS():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('CAN_ROS', anonymous=False)
		self.rate = rospy.Rate(50)

		# ------------- PARAMETER ------------- #
		self.ID_RTC  = rospy.get_param('ID_RTC', 2)
		self.ID_RTC  = 1
		self.ID_HC   = rospy.get_param('ID_HC', 1)
		self.ID_HC   = 2
		self.ID_MAIN = rospy.get_param('ID_MAIN', 3)
		self.ID_MAIN = 3

		self.ID_CONVEYOR_No12 = rospy.get_param('ID_CONVEYOR_No12', 4)
		self.ID_CONVEYOR_No12 = 4
		self.ID_CONVEYOR_No34 = rospy.get_param('ID_CONVEYOR_No34', 5)
		self.ID_CONVEYOR_No34 = 5
		self.ID_CONVEYOR_No56 = rospy.get_param('ID_CONVEYOR_No56', 6)
		self.ID_CONVEYOR_No56 = 6
		self.ID_CPD = rospy.get_param('ID_CPD', 7)
		self.ID_CPD = 7
		# ------------- ROS ------------- #
		# ----- CONVEYER ----- #
		# - PUBLISH
		self.pub_statusConveyor1 = rospy.Publisher("/status_conveyor13", Status_conveyor, queue_size= 40)
		self.status_conveyor13 = Status_conveyor()

		self.pub_statusConveyor2 = rospy.Publisher("/status_conveyor23", Status_conveyor, queue_size= 40)
		self.status_conveyor23 = Status_conveyor()

		self.pub_statusConveyor3 = rospy.Publisher("/status_conveyor12", Status_conveyor, queue_size= 40)
		self.status_conveyor12 = Status_conveyor()

		self.pub_statusConveyor4 = rospy.Publisher("/status_conveyor22", Status_conveyor, queue_size= 40)
		self.status_conveyor22 = Status_conveyor()

		self.pub_statusConveyor5 = rospy.Publisher("/status_conveyor11", Status_conveyor, queue_size= 40)
		self.status_conveyor11 = Status_conveyor()

		self.pub_statusConveyor6 = rospy.Publisher("/status_conveyor21", Status_conveyor, queue_size= 40)
		self.status_conveyor21 = Status_conveyor()
		# --
		self.pub_statusMain = rospy.Publisher("/POWER_info", POWER_info, queue_size = 20)
		self.status_Main = POWER_info()
		# -- 
		self.pub_statusHC = rospy.Publisher("/HC_infoRaw", HC_info, queue_size = 200)
		self.status_HC = HC_info()
		# -- 
		self.pub_readCPD = rospy.Publisher("/CPD_read", CPD_read, queue_size = 20)
		self.CPD_dataRead = CPD_read()

		# - SUBCRIBER
		# ------------- CAN ------------- #
		# rospy.Subscriber("/CAN_status", CAN_status, self.statusCAN_callback)
		# self.CAN_status = CAN_status()

		rospy.Subscriber("/CAN_received", CAN_received, self.CAN_callback)
		self.data_receivedCAN = CAN_received()

		self.pub_sendCAN = rospy.Publisher("/CAN_send", CAN_send, queue_size= 40)
		self.data_sendCan = CAN_send()
		self.frequence_sendCAN = 14. # - Hz
		self.saveTime_sendCAN = time.time()
		self.sort_send = 0

		# ------------- Control ------------- #
		rospy.Subscriber("/control_conveyors", Control_conveyors, self.controlOC_callback)
		self.data_controlConveyors = Control_conveyors()

		rospy.Subscriber("/POWER_request", POWER_request, self.controlMain_callback)
		self.data_controlMain = POWER_request()

		rospy.Subscriber("/HC_request", HC_request, self.controlHC_callback)
		self.data_controlHC = HC_request()

		rospy.Subscriber("/CPD_write", CPD_write, self.controlCPD_callback)
		self.data_controlCPD = CPD_write()

		rospy.Subscriber("/HC_fieldRequest1", Int8, self.controlFieldSick1_callback)
		self.data_controlFieldSick1 = Int8()

		rospy.Subscriber("/HC_fieldRequest2", Int8, self.controlFieldSick2_callback)
		self.data_controlFieldSick2 = Int8()
		# ------------- VAR ------------- #

	# -------------------
	def controlFieldSick1_callback(self, data):
		self.data_controlFieldSick1 = data

	def controlFieldSick2_callback(self, data):
		self.data_controlFieldSick2 = data

	def controlCPD_callback(self, data):
		self.data_controlCPD = data

	def controlHC_callback(self, data):
		self.data_controlHC = data

	def controlMain_callback(self, data):
		self.data_controlMain = data

	def controlOC_callback(self, data):
		self.data_controlConveyors = data
	# ------------------- 
	def CAN_callback(self, data):
		self.data_receivedCAN = data
		# -- RECEIVED CAN
		self.analysisFrame_receivedCAN()

	def statusCAN_callback(self, data):
		self.CAN_status = data

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	def convert_16bit_int(self, bitArr):
		int_out = 0
		for i in range(16):
			int_out += bitArr[i]*pow(2, i)
		return int_out

	def getByte_fromInt16(self, valueIn, pos):
		byte1 = int(valueIn/256)
		byte0 =  valueIn - byte1*256
		if (pos == 0):
			return byte0
		else:
			return byte1

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def getBit_fromInt16(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(16):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return int(bit_out)

			if (value_now < 1):
				return 0		
		return 0

	def syntheticFrame_sendCAN(self):
		# -- HC
		if (self.sort_send == 0):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_HC
			self.data_sendCan.byte1 = self.data_controlHC.RBG1
			self.data_sendCan.byte2 = self.data_controlHC.RBG2
			self.data_sendCan.byte3 = self.data_controlFieldSick2.data
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 1

		# -- Main
		elif (self.sort_send == 1):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_MAIN
			self.data_sendCan.byte1 = self.data_controlMain.sound_on
			self.data_sendCan.byte2 = self.data_controlMain.sound_type
			self.data_sendCan.byte3 = self.data_controlMain.charge
			self.data_sendCan.byte4 = self.data_controlMain.EMC_reset
			self.data_sendCan.byte5 = self.data_controlMain.EMC_write
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 2

		# -- CPD
		elif (self.sort_send == 2):
			bitArr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_CPD

			bitArr[0] = self.data_controlCPD.output1
			bitArr[1] = self.data_controlCPD.output2
			bitArr[2] = self.data_controlCPD.output3
			bitArr[3] = self.data_controlCPD.output4
			bitArr[4] = self.data_controlCPD.output5
			bitArr[5] = self.data_controlCPD.output6
			bitArr[6] = self.data_controlCPD.output7
			bitArr[7] = self.data_controlCPD.output8
			bitArr[8] = self.data_controlCPD.output9
			bitArr[9] = self.data_controlCPD.output10
			bitArr[10] = self.data_controlCPD.output11
			bitArr[11] = self.data_controlCPD.output12

			int16bit = self.convert_16bit_int(bitArr)
			byte0 = self.getByte_fromInt16(int16bit, 0)
			byte1 = self.getByte_fromInt16(int16bit, 1)

			# print ("int: " + str(int16bit) + " Byte: " + str(byte0) + " | " + str(byte1) )
			self.data_sendCan.byte1 = byte0
			self.data_sendCan.byte2 = byte1
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 3

		# -- CONVEYOR_No12
		elif (self.sort_send == 3):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_CONVEYOR_No12
			self.data_sendCan.byte1 = self.data_controlConveyors.No1_mission
			self.data_sendCan.byte2 = self.data_controlConveyors.No1_speed

			self.data_sendCan.byte3 = self.data_controlConveyors.No2_mission
			self.data_sendCan.byte4 = self.data_controlConveyors.No2_speed

			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 4

		# -- CONVEYOR_No34
		elif (self.sort_send == 4):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_CONVEYOR_No34
			self.data_sendCan.byte1 = self.data_controlConveyors.No3_mission
			self.data_sendCan.byte2 = self.data_controlConveyors.No3_speed

			self.data_sendCan.byte3 = self.data_controlConveyors.No4_mission
			self.data_sendCan.byte4 = self.data_controlConveyors.No4_speed

			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 5

		# -- CONVEYOR_No56
		elif (self.sort_send == 5):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_CONVEYOR_No56
			self.data_sendCan.byte1 = self.data_controlConveyors.No5_mission
			self.data_sendCan.byte2 = self.data_controlConveyors.No5_speed

			self.data_sendCan.byte3 = self.data_controlConveyors.No6_mission
			self.data_sendCan.byte4 = self.data_controlConveyors.No6_speed

			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 0

		
	def analysisFrame_receivedCAN(self):
		# -- HC
		if self.data_receivedCAN.idSend == self.ID_HC:
			self.status_HC.status 			= 0
			self.status_HC.zone_sick_ahead  = int(self.data_receivedCAN.byte4)
			self.status_HC.zone_sick_behind = int(self.data_receivedCAN.byte5)
			self.status_HC.vacham 			= int(self.data_receivedCAN.byte6)

			# print ("---------")
			# print ("HC - byte4: ", self.data_receivedCAN.byte4)
			# print ("HC - byte5: ", self.data_receivedCAN.byte5)
			# print ("HC - byte6: ", self.data_receivedCAN.byte6)
			self.pub_statusHC.publish(self.status_HC)
			# print ("--- HC")

		# -- Main
		elif self.data_receivedCAN.idSend == self.ID_MAIN:
			self.status_Main.voltages 	 = self.convert_4byte_int(self.data_receivedCAN.byte0, self.data_receivedCAN.byte1, 0, 0)/10.
			self.status_Main.voltages_analog = 0

			self.status_Main.charge_current  = self.convert_4byte_int(self.data_receivedCAN.byte2, self.data_receivedCAN.byte3, 0, 0)
			self.status_Main.charge_analog   = 0

			# print (self.status_Main.stsButton_reset)
			self.status_Main.stsButton_reset = self.data_receivedCAN.byte4

			self.status_Main.stsButton_power = self.data_receivedCAN.byte5
			self.status_Main.EMC_status 	 = self.data_receivedCAN.byte6
			self.status_Main.CAN_status   	 = self.data_receivedCAN.byte7
			self.pub_statusMain.publish(self.status_Main)
			# print ("--- --- Main")

		# -- CPD
		elif (self.data_receivedCAN.idSend == self.ID_CPD):
			self.CPD_dataRead.status = self.data_receivedCAN.byte0
			int16Bit = self.convert_4byte_int(self.data_receivedCAN.byte1, self.data_receivedCAN.byte2, 0, 0)
			self.CPD_dataRead.input1  = self.getBit_fromInt16(int16Bit, 0)
			self.CPD_dataRead.input2  = self.getBit_fromInt16(int16Bit, 1)
			self.CPD_dataRead.input3  = self.getBit_fromInt16(int16Bit, 2)
			self.CPD_dataRead.input4  = self.getBit_fromInt16(int16Bit, 3)
			self.CPD_dataRead.input5  = self.getBit_fromInt16(int16Bit, 4)
			self.CPD_dataRead.input6  = self.getBit_fromInt16(int16Bit, 5)
			self.CPD_dataRead.input7  = self.getBit_fromInt16(int16Bit, 6)
			self.CPD_dataRead.input8  = self.getBit_fromInt16(int16Bit, 7)
			self.CPD_dataRead.input9  = self.getBit_fromInt16(int16Bit, 8)
			self.CPD_dataRead.input10 = self.getBit_fromInt16(int16Bit, 9)
			self.CPD_dataRead.input11 = self.getBit_fromInt16(int16Bit, 10)
			self.CPD_dataRead.input12 = self.getBit_fromInt16(int16Bit, 11)

			self.pub_readCPD.publish(self.CPD_dataRead)
			# print ("--- --- --- CPD")

		# -- OC No.1 - CONVEYOR_No12
		elif (self.data_receivedCAN.idSend == self.ID_CONVEYOR_No12):
			self.status_conveyor13.status 			  = int(self.data_receivedCAN.byte0)
			self.status_conveyor13.sensor_limitAhead  = self.data_receivedCAN.byte1
			self.status_conveyor13.sensor_limitBehind = self.data_receivedCAN.byte2
			self.status_conveyor13.sensor_checkRack   = self.data_receivedCAN.byte3

			self.status_conveyor21.status 			  = int(self.data_receivedCAN.byte4)
			self.status_conveyor21.sensor_limitAhead  = self.data_receivedCAN.byte5
			self.status_conveyor21.sensor_limitBehind = self.data_receivedCAN.byte6
			self.status_conveyor21.sensor_checkRack   = self.data_receivedCAN.byte7

			self.pub_statusConveyor1.publish(self.status_conveyor13)
			self.pub_statusConveyor2.publish(self.status_conveyor21)
			# print ("--- --- --- --- OC12")

		# -- OC No.2 - CONVEYOR_No34
		elif (self.data_receivedCAN.idSend == self.ID_CONVEYOR_No34):
			self.status_conveyor12.status 			  = int(self.data_receivedCAN.byte0)
			self.status_conveyor12.sensor_limitAhead  = self.data_receivedCAN.byte1
			self.status_conveyor12.sensor_limitBehind = self.data_receivedCAN.byte2
			self.status_conveyor12.sensor_checkRack   = self.data_receivedCAN.byte3

			self.status_conveyor22.status 			  = int(self.data_receivedCAN.byte4)
			self.status_conveyor22.sensor_limitAhead  = self.data_receivedCAN.byte5
			self.status_conveyor22.sensor_limitBehind = self.data_receivedCAN.byte6
			self.status_conveyor22.sensor_checkRack   = self.data_receivedCAN.byte7

			self.pub_statusConveyor3.publish(self.status_conveyor12)
			self.pub_statusConveyor4.publish(self.status_conveyor22)
			# print ("--- --- --- --- --- OC34")

		# -- OC No.3 - CONVEYOR_No56
		elif (self.data_receivedCAN.idSend == self.ID_CONVEYOR_No56):
			self.status_conveyor11.status 			  = int(self.data_receivedCAN.byte0)
			self.status_conveyor11.sensor_limitAhead  = self.data_receivedCAN.byte1
			self.status_conveyor11.sensor_limitBehind = self.data_receivedCAN.byte2
			self.status_conveyor11.sensor_checkRack   = self.data_receivedCAN.byte3

			self.status_conveyor21.status 			  = int(self.data_receivedCAN.byte4)
			self.status_conveyor21.sensor_limitAhead  = self.data_receivedCAN.byte5
			self.status_conveyor21.sensor_limitBehind = self.data_receivedCAN.byte6
			self.status_conveyor21.sensor_checkRack   = self.data_receivedCAN.byte7

			self.pub_statusConveyor5.publish(self.status_conveyor11)
			self.pub_statusConveyor6.publish(self.status_conveyor21)
			# print ("--- --- --- --- --- --- OC56")

	def try_run(self):
		val = 100
		print ("Bit0", self.getBit_fromInt8(val, 0))
		print ("Bit1", self.getBit_fromInt8(val, 1))
		print ("Bit2", self.getBit_fromInt8(val, 2))
		print ("Bit3", self.getBit_fromInt8(val, 3))
		print ("Bit4", self.getBit_fromInt8(val, 4))
		print ("Bit5", self.getBit_fromInt8(val, 5))
		print ("Bit6", self.getBit_fromInt8(val, 6))
		print ("Bit7", self.getBit_fromInt8(val, 7))

	def string_arry(self):
		arr_str = "0123456"
		print ("OUT0: ", arr_str[2:4])
		print ("OUT1: ", arr_str[0:2])
		print ("OUT2: ", arr_str[:2])
		print ("OUT3: ", arr_str[:-3])
		print ("OUT4: ", arr_str[-2:0])
		print ("OUT5: ", arr_str[2:0])

	def run(self):
		while not rospy.is_shutdown():
			# -- SEND CAN
			delta_time = (time.time() - self.saveTime_sendCAN)%60 
			if (delta_time > 1/self.frequence_sendCAN):
				self.saveTime_sendCAN = time.time()
				self.syntheticFrame_sendCAN()
				self.pub_sendCAN.publish(self.data_sendCan)

			self.rate.sleep()

def main():
	print('Program starting')
	program = CAN_ROS()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()



