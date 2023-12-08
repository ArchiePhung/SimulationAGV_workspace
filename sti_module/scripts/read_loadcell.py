#!/usr/bin/env python3

import rospy
import sys
import time
import threading
import signal
import struct

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Bool

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from decimal import *
import math

class ReadLoadCell():
	def __init__(self):
        # status = 1 (all right) | != 1 error.
		self.PORT =  '/dev/stibase_motor'
		self.BAUDRATE = 57600 # 57600 115200

        # -------------------
		self.ID_Slave = 3

        # -- Registers
		self.reg_1 = 0
		self.reg_2 = 1
		self.reg_3 = 2

        # ---------------------------------------- Modbus
		print("Modbus run!")
		for i in range(3):
			try:
				self.MODBUS = modbus_rtu.RtuMaster(
					serial.Serial(port= self.PORT, baudrate= self.BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
				)

				self.MODBUS.open 
				self.MODBUS.set_timeout(1.0)
				self.MODBUS.set_verbose(True)
				print("Modbus connected !")
				break

			except modbus_tk.modbus.ModbusError as exc:
				print("Modbus false!") 
				sys.exit()
        # ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('read_LoadCell', anonymous=False)
		self.rate = rospy.Rate(20)

	def int_to_byte(self, val): # int to a bytes
		if val > 255:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 255
		elif val < 0:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 0
		return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

	def int_to_bytes(self, val, n):
		ss = b''
		x = 0
		t = 0
		for i in range(0, n):
			t += pow(256, n - i)*x
			x = int((val - t)/pow(256, n - i - 1) )
			ss += self.int_to_byte(x)
		return ss

	def byte_to_int(self, byt): # byte to int (standard)
		x = 0
		x = int(byt)
		return x

	def bytes_list_to_coor_vs1(self, list_byte, pos_byte): # 4 byte
		n = 0
		x = 0
		v = pos_byte + 1

		for t in range(4):
			x = int(list_byte[pos_byte + t])
			n += x*pow(256, 3 - t)

		val = 0
		if (n > pow(256, 3)):
			val = (n - pow(256, 4))
		else:
			val  = n
		return val

	def float_to_bytes(self, value_in):
		out = b''
		out = bytearray(struct.pack("f", value_in))
		return out

	def convert_(self):
		byte_ = b'\xE7\x00\x05\x01' # 93001D01
		# bytes_as_bits = ''.join(format(byte, '08b')[::-1] for byte in byte_) # dao bit 0 <-> 1
		bytes_as_bits = ''.join(format(byte, '08b') for byte in byte_) # 
		print ("bit: ", bytes_as_bits)

	def check_bit(self):
		# byte_ = b''
		# hex_string = "E7000501"
		# byte_ = bytes.fromhex(hex_string)
		# # bytes_as_bits = ''.join(format(byte, '08b')[::-1] for byte in byte_) # dao bit 0 <-> 1
		# bytes_as_bits = ''.join(format(byte, '08b') for byte in byte_) # 
		# print ("bit: ", bytes_as_bits)
		val = self.arrBin_to_dec("1110", 0, 4)
		print("val: ", val)

	def arrBit_to_MaxtrixHex(self, arrBit):
		matrix_Hex = []
		arrHexNow = []
		# ---- byte0
		if (arrBit[0] == '1'):
			arrHexNow.append(8)

		if (arrBit[1] == '1'):
			arrHexNow.append(4)

		if (arrBit[2] == '1'):
			arrHexNow.append(2)

		if (arrBit[3] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte1
		if (arrBit[4] == '1'):
			arrHexNow.append(8)

		if (arrBit[5] == '1'):
			arrHexNow.append(4)

		if (arrBit[6] == '1'):
			arrHexNow.append(2)

		if (arrBit[7] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte2
		if (arrBit[8] == '1'):
			arrHexNow.append(8)

		if (arrBit[9] == '1'):
			arrHexNow.append(4)

		if (arrBit[10] == '1'):
			arrHexNow.append(2)

		if (arrBit[11] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte3
		if (arrBit[12] == '1'):
			arrHexNow.append(8)

		if (arrBit[13] == '1'):
			arrHexNow.append(4)

		if (arrBit[14] == '1'):
			arrHexNow.append(2)

		if (arrBit[15] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte4
		if (arrBit[16] == '1'):
			arrHexNow.append(8)

		if (arrBit[17] == '1'):
			arrHexNow.append(4)

		if (arrBit[18] == '1'):
			arrHexNow.append(2)

		if (arrBit[19] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte5
		if (arrBit[20] == '1'):
			arrHexNow.append(8)

		if (arrBit[21] == '1'):
			arrHexNow.append(4)

		if (arrBit[22] == '1'):
			arrHexNow.append(2)

		if (arrBit[23] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte6
		if (arrBit[24] == '1'):
			arrHexNow.append(8)

		if (arrBit[25] == '1'):
			arrHexNow.append(4)

		if (arrBit[26] == '1'):
			arrHexNow.append(2)

		if (arrBit[27] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ---- byte7
		if (arrBit[28] == '1'):
			arrHexNow.append(8)

		if (arrBit[29] == '1'):
			arrHexNow.append(4)

		if (arrBit[30] == '1'):
			arrHexNow.append(2)

		if (arrBit[31] == '1'):
			arrHexNow.append(1)
		matrix_Hex.append(arrHexNow)
		arrHexNow = []
		# ----
		for a in range(8):
			length = len(matrix_Hex[a])
			print ("length: ", length)
			strs = ""
			for b in range(length):
				strs += str(matrix_Hex[a][b]) + "|"
			print ("B" + str(a) + ": " + strs)
		return matrix_Hex


	def arrBin_to_dec(self, arr, pos, num):
		val_out = 0
		end = pos + num 
		for vt in range(pos, end):
			print ("vt: ", vt)
			val_out += int(arr[end - vt - 1])*pow(2, vt)
		return val_out

	def getData(self):
		while not rospy.is_shutdown():
			try:
				time.sleep(0.4)
				rawData1 = self.MODBUS.execute(self.ID_Slave, cst.READ_HOLDING_REGISTERS, self.reg_1, 1)
				time.sleep(0.4)
				rawData2 = self.MODBUS.execute(self.ID_Slave, cst.READ_HOLDING_REGISTERS, self.reg_2, 1)
				time.sleep(0.4)
				rawData3 = self.MODBUS.execute(self.ID_Slave, cst.READ_HOLDING_REGISTERS, self.reg_3, 1)
				print("data1: " + str(rawData1[0]) + " | data2: " + str(rawData2[0]) )
				print("data1: " + str(self.int_to_bytes(rawData1[0], 2) ) + " | data2: " + str(self.int_to_bytes(rawData2[0], 2) ) + " | data3: " + str(rawData3[0]) )

				st = b''
				a1 = b''
				a2 = b''
				a1 = self.int_to_bytes(rawData1[0], 2)

				a10 = self.int_to_byte(a1[0])
				a11 = self.int_to_byte(a1[1])

				a2 = self.int_to_bytes(rawData2[0], 2)
				a20 = self.int_to_byte(a2[0])
				a21 = self.int_to_byte(a2[1])

				st += a11
				st += a10
				st += a21
				st += a20

				print ("St_val: ", struct.unpack('f', st)[0])

			except:
			# except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_feedback error")
				# sys.exit()
			self.rate.sleep()

	def run(self):
		self.show_infoState('11100111000000000000010100000001')

def main():
	print('Starting main program')
	program = ReadLoadCell()
	# program.convert_()
	program.run()
	# program.check_bit()
	# program.getData()
		# time.sleep(0.1)

if __name__ == '__main__':
	main()
