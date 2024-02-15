#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 17/11/2021

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
import re  
import subprocess
import argparse
from message_pkg.msg import *
from sti_msgs.msg import *
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
#--------------------------------------------------------------------------------- ROS
class data_synthesis():
	def __init__(self):
		rospy.init_node('data_synthesis', anonymous=False)
		self.rate = rospy.Rate(20)

		self.name_card = rospy.get_param("name_card", "wlp0s20f3")
		self.name_card = "wlp0s20f3"
		self.transmission_ratio = rospy.get_param("transmission_ratio", 30.0)

		# distance
		# rospy.Subscriber("/quang_duong", Float64, self.distance_callback)
		# self.distanceRuned = Float64()

		# -- Driver1
		rospy.Subscriber("/driver1_respond", Driver_respond, self.driver1_callback) 
		self.driver1_respond = Driver_respond()

		# -- Driver2
		rospy.Subscriber("/driver2_respond", Driver_respond, self.driver2_callback) 
		self.driver2_respond = Driver_respond()

		# -- NN cmd
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdRequest_callback) 
		self.NN_cmdRequest = NN_cmdRequest()

		# -- Pose robot
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.NN_infoRequest_callback) 
		self.NN_infoRequest = NN_infoRequest()

		# -- Pose robot
		rospy.Subscriber("/robotPose_nav", PoseStamped, self.robotPose_callback) 
		self.robotPose_nav = PoseStamped()

		# -- data HC
		rospy.Subscriber("/HC_info", HC_info, self.HC_info_callback) 
		self.HC_info = HC_info()

		# -- data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.safety_NAV_callback) 
		self.safety_NAV = Int8()

		# -- data nav
		rospy.Subscriber("/nav350_data", Nav350_data, self.nav350_callback) 
		self.nav350_data = Nav350_data()

		# -- lift
		rospy.Subscriber("/lift_status", Lift_status, self.callback_oc) # lay thong tin trang thai mach dieu khien ban nang.
		self.lift_status = Lift_status()

		# -- Color
		self.pub_dataColor = rospy.Publisher("/app_setColor", App_color, queue_size = 4)
		self.app_setColor = App_color()
		# -- Label value
		self.pub_setValue = rospy.Publisher("/app_setValue", App_lbv, queue_size = 4)
		self.app_setValue = App_lbv()
		# -- info AGV
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.infoAGV = NN_infoRespond()
		# -- info move
		rospy.Subscriber("/status_goal_control", Status_goal_control, self.goalControl_callback)
		self.status_goalControl = Status_goal_control() # sub from move_base
		# -- parking
		rospy.Subscriber("/parking_respond", Parking_respond, self.parking_callback)
		self.parking_status = Parking_respond()
		# --
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_main) 
		self.main_info = POWER_info()
		# --
		# -- loadCell
		rospy.Subscriber("/loadcell_respond", Loadcell_respond, self.loadcell_callback) 
		self.loadCell_respond = Loadcell_respond()
		# --
		self.address = "192.168.1.69"
		# -- 
		self.pre_timePing = time.time()
		self.weight_origin = 78

	def loadcell_callback(self, dat):
		self.loadCell_respond = dat

	def callback_oc(self, dat):
		self.lift_status = dat

	def callback_main(self, dat):
		self.main_info = dat

	def parking_callback(self, data):
		self.parking_status = data

	def goalControl_callback(self, data):
		self.status_goalControl = data

	def driver1_callback(self, data):
		self.driver1_respond = data

	def driver2_callback(self, data):
		self.driver2_respond = data

	def NN_cmdRequest_callback(self, data):
		self.NN_cmdRequest = data	

	def NN_infoRequest_callback(self, data):
		self.NN_infoRequest = data	

	def safety_NAV_callback(self, data):
		self.safety_NAV = data	

	def HC_info_callback(self, data):
		self.HC_info = data	

	def infoAGV_callback(self, data):
		self.infoAGV = data	

	def robotPose_callback(self, data):
		self.robotPose_nav = data

	def nav350_callback(self, data):
		self.nav350_data = data

	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			print ("address: ", address)
			return address
		except Exception:
			return "-1"

	def getPing_rosWifi(self, address):
		try:
			# -- wifi
			# ping_wifi = str(subprocess.check_output("ping -{} 1 {}".format('c', address), shell=True) )
			# vitri_wifi = ping_wifi.find("time=")
			# time_pingWifi = ping_wifi[(vitri_wifi + 5):(vitri_wifi + 9)]
			time_pingWifi = 0.0
			# -- ros
			ping_ros = str(subprocess.check_output("rosnode ping -c 1 /rosout", shell=True) )
			vitri_ros = ping_ros.find("time=")
			time_pingRos = ping_ros[(vitri_ros + 5):(vitri_ros + 9)]

			# print ("time_pingWifi: ", time_pingWifi)
			# print ("time_pingRos: ", time_pingRos)
			
			return time_pingWifi, time_pingRos

		except Exception:
		# 	print ("time_pingWifi: ", -1)
		# 	print ("time_pingRos: ", -1)
			return -1, -1
			
	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def convert_misson(self, val):
		switcher={
			0:'Khong\nXac Dinh', # 
			65:'Nang Ke', # 
			66:'Ha Ke', # 
			6:'Ha Ke\nSac Pin', # 
			5:'Sac Pin' # 
		}
		return switcher.get(val, 'Khong\nXac Dinh')

	def convert_errorDevice(self, val):
		switcher={
			351:'Mach HC', # 
			341:'Mach OC', # 
			321:'Mach Main', # 
			251:'Driver Motor1', # 
			261:'Driver Motor2', # 
			231:'CB IMU', #
			221:'CB NAV', #
			141:'CB Ban Nang' # 
		}
		return switcher.get(val, '')

	def convert_errorPerform(self, val):
		switcher={
			# :'Qua tai trong', # 
			471:'Khong co ke', # 
			# :'Co vat can', # 
			121:'Bi Nhan EMG', # 
			122:'Bi Cham Blsock', #
			272:'Khong phat hien Guong', #
			431:'Mat Ket Noi Server', #
			451:'Dien AP Thap' # 
		}
		return switcher.get(val, '')

	def convert_errorAll(self, val):
		switcher={
			351:'Mach HC', # 
			341:'OC-Ket noi', #
			342:'OC-USB', # 
			321:'Mach Main', # 
			251:'Mat Motor1', # 
			252:'Loi Motor1', # 
			261:'Mat Motor2', # 
			262:'Loi Motor2', # 
			231:'Cam Bien Goc', #
			232:'IMU-USB', #
			221:'Cam Bien NAV', #
			181:'LoadCell-Ket Noi', #
			182:'LoadCell-Dau noi', #
			183:'LoadCell-USB', #
			184:'Qua Tai 700kg', #
			222:'Mat Toa Do', #
			141:'Cam Bien Ban Nang', # 
			121:'Bi Nhan EMG', # 
			122:'Bi Cham Blsock', #
			272:'Khong Phat Hien Guong', #
			441:'AGV di chuyen het diem', # 
			471:'Khong co ke', # 
			411:'Co vat can', # 
			431:'Mat Ket Noi Server', #
			451:'Dien AP Thap' # 
		}
		return switcher.get(val, '')

	def driver_errorType_Alarm(self, err):
		# Reset using the ALARMRESET input: <
		mess = ' '
		switcher={
			0:   'All right!',
			48:  'Overload',
			40:  'Sensor error',
			66:  'Initial sensor error',
			34:  'Overvoltage',
			37:  'Undervoltage',
			49:  'Overspeed',
			32:  'Overcurrent',
			65:  'EEPROM error',
			33:  'Main circuit overheat',
			110: 'External stop∗1',
			70:  'Initial operation error∗2',
			129: 'Network bus error',
			131: 'Communication switch setting error',
			132: 'RS-485 communication error',
			133: 'RS-485 communication timeout',
			142: 'Network converter error ',
			45:  'Main circuit output error∗3'
		}
		return switcher.get(err, 'Not found!')

	def driver_errorType_All(self, err):
		mess = ' '
		switcher={
			0:   'All right!',
			132: 'RS-485 communication error',
			136: 'Command not yet defined',
			137: 'User I/F communication in progress',
			138: 'NV memory processing in progress',
			140: 'Outside setting range',
			141: 'Command execute disable',
			33:  'Main circuit overheat',
			37:  'Undervoltage∗',
			48:  'Overload∗',
			108: 'Operation error '
		}
		return switcher.get(err, 'Not found!')

	# def convert_errorDetail(self, val):
	# 	switcher={
	# 		:'Mat Port IMU', # 
	# 		:'Mat Port Main', # 
	# 		:'Mat Port OC', # 
	# 		:'Mat Port NAV', # 
	# 		:'Mat Port Driver', # 
	# 		:'Motor1: Dien Ap Thap', # 
	# 		:'Motor1: Qua Tai', #
	# 		:'Motor1: Loi Dieu khien', #
	# 		:'Motor1: Dien Ap Thap', # 
	# 		:'Motor1: Qua Tai', # 
	# 		231:'Cam Bien Goc', #
	# 		221:'Cam Bien NAV', #
	# 		222:'Mat Toa Do', #
	# 		141:'Cam Bien Ban Nang', # 
			
	# 		# :'Qua tai trong', # 
	# 		121:'Bi Nhan EMG', # 
	# 		122:'Bi Cham Blsock', #
	# 		272:'Khong phat hien Guong', #
	# 		441:'AGV di chuyen het diem', # 
	# 		471:'Khong co ke', # 
	# 		411:'Co vat can', # 
	# 		431:'Mat Ket Noi Server', #
	# 		451:'Dien AP Thap' # 
	# 	}
	# 	return switcher.get(val, '')


	def show_job(self, val):
		job_now = ''
		switcher={
			0:'Khong Xac Dinh', # 
			1:'Kiem Tra Tu Dong', # kiem tra trang thai ban nang sau khi Sang che do tu dong
			2:'Nhiem Vu Truoc', # thuc hien nhiem vu truoc
			3:'Kiem Tra Ke', # kiem tra ke 
			4:'Di Ra', # 
			5:'Di Chuyen', #
			6:'Di Vao Ke', # 
			7:'Nhiem Vu Sau', # 
			8:'Doi Lenh Moi', # 
			20:'Bang Tay' # 
		}
		return switcher.get(val, job_now)

	def run(self):
		self.address = self.get_ipAuto(self.name_card)
		self.app_setValue.lbv_ip = self.address
		print ("ok --")
		while not rospy.is_shutdown():
			# --
			deltaTime_ping = (time.time() - self.pre_timePing)%60
			if (deltaTime_ping > 1.0):
				self.pre_timePing = time.time()
				wifi = ''
				ros = ''
				# wifi, ros = self.getPing_rosWifi(self.address)
				self.app_setValue.lbv_ping = wifi
				self.app_setValue.lbv_ros = ros

			# -- Mode show
			# self.infoAGV.mode = 2
			if (self.infoAGV.mode == 0): # - not
				self.app_setColor.bt_passHand = 0
				self.app_setColor.bt_passAuto = 0

			elif (self.infoAGV.mode == 1): # -- md_by_hand
				self.app_setColor.bt_passHand = 1
				self.app_setColor.bt_passAuto = 0

			elif (self.infoAGV.mode == 2): # -- md_auto
				self.app_setColor.bt_passHand = 0
				self.app_setColor.bt_passAuto = 1

			# -- Mau sac khi sac pin
			if (self.main_info.charge_current > 2.5):
				self.app_setColor.lbv_battery = 4
			else:
				if (self.main_info.pin_voltage < 23.5):
					self.app_setColor.lbv_battery = 3
				elif (self.main_info.pin_voltage >= 23.5 and self.main_info.pin_voltage < 24.5):
					self.app_setColor.lbv_battery = 2
				else:
					self.app_setColor.lbv_battery = 1

			# -- status AGV
			self.app_setColor.lbc_status = self.infoAGV.status
			self.app_setValue.lbv_name_agv = self.NN_infoRequest.name_agv
			self.app_setValue.lbv_battery = "  " + str(round(self.infoAGV.battery*0.1, 1) )
			# -- -- -- -- --
			lenght_error = len(self.infoAGV.listError)
			str_errorDevice = ''
			str_errorPerform = ''
			if (lenght_error == 0):
				str_errorDevice = 'AGV Van Hanh Binh Thuong'
			else:
				for i in range(lenght_error):
					if (self.infoAGV.listError[i] < 400):
						str_errorDevice += self.convert_errorAll(self.infoAGV.listError[i])
						str_errorDevice += ' | '
					else:
						str_errorPerform = self.convert_errorAll(self.infoAGV.listError[i])
						str_errorPerform += ' | '

			self.app_setValue.lbv_device = str_errorDevice
			self.app_setValue.lbv_frameWork = str_errorPerform
			# self.app_setValue.lbv_frameWork = str(self.infoAGV.error_perform)

			# -- Pose robot
			self.app_setValue.lbv_x = str(round(self.robotPose_nav.pose.position.x, 3))
			self.app_setValue.lbv_y = str(round(self.robotPose_nav.pose.position.y, 3))
			angle = self.quaternion_to_euler(self.robotPose_nav.pose.orientation)

			angle_180 = degrees(angle)
			angle_360 = 0.0
			if (angle_180 < 0):
				angle_360 = round(360 + angle_180, 2)
			else:
				angle_360 = round(angle_180, 2)

			self.app_setValue.lbv_zd = str(angle_360)
			self.app_setValue.lbv_detect = str(self.nav350_data.number_reflectors)
			# --
			if (self.safety_NAV.data == 0):
				self.app_setColor.lbc_safety0 = 0
			else:
				self.app_setColor.lbc_safety0 = 1
			# --
			if (self.HC_info.zone_sick_ahead == 0):
				self.app_setColor.lbc_safety1 = 0
			elif (self.HC_info.zone_sick_ahead == 1):
				self.app_setColor.lbc_safety1 = 1
			else:
				self.app_setColor.lbc_safety1 = 2
			# --
			if (self.HC_info.zone_sick_behind == 0):
				self.app_setColor.lbc_safety2 = 0
			elif (self.HC_info.zone_sick_behind == 1):
				self.app_setColor.lbc_safety2 = 1
			else:
				self.app_setColor.lbc_safety2 = 2
			# -- -- -- -- -- cmd
			self.app_setValue.lbv_job = self.show_job(self.infoAGV.process)
			# --
			self.app_setValue.lbv_job1 = self.convert_misson(self.NN_cmdRequest.before_mission)
			self.app_setValue.lbv_job2 = self.convert_misson(self.NN_cmdRequest.after_mission)

			angle_dec = 0
			if (self.NN_cmdRequest.target_z < 0):
				angle_dec = 360 + degrees(self.NN_cmdRequest.target_z)
			else:
				angle_dec = degrees(self.NN_cmdRequest.target_z)

			self.app_setValue.lbv_pointFinal = str(self.NN_cmdRequest.target_x) + "\n" + str(self.NN_cmdRequest.target_y) + "\n" + str(round(angle_dec, 2)) + "\n" + str(self.NN_cmdRequest.offset)
			# -- 
			if (len(self.NN_cmdRequest.list_x) >= 5):
				self.app_setValue.lbv_point0 = str(self.NN_cmdRequest.list_id[0]) + "\n" + str(self.NN_cmdRequest.list_x[0]) + "\n" + str(self.NN_cmdRequest.list_y[0]) + "\n" + str(self.NN_cmdRequest.list_speed[0])
				self.app_setValue.lbv_point1 = str(self.NN_cmdRequest.list_id[1]) + "\n" + str(self.NN_cmdRequest.list_x[1]) + "\n" + str(self.NN_cmdRequest.list_y[1]) + "\n" + str(self.NN_cmdRequest.list_speed[1])
				self.app_setValue.lbv_point2 = str(self.NN_cmdRequest.list_id[2]) + "\n" + str(self.NN_cmdRequest.list_x[2]) + "\n" + str(self.NN_cmdRequest.list_y[2]) + "\n" + str(self.NN_cmdRequest.list_speed[2])
				self.app_setValue.lbv_point3 = str(self.NN_cmdRequest.list_id[3]) + "\n" + str(self.NN_cmdRequest.list_x[3]) + "\n" + str(self.NN_cmdRequest.list_y[3]) + "\n" + str(self.NN_cmdRequest.list_speed[3])
				self.app_setValue.lbv_point4 = str(self.NN_cmdRequest.list_id[4]) + "\n" + str(self.NN_cmdRequest.list_x[4]) + "\n" + str(self.NN_cmdRequest.list_y[4]) + "\n" + str(self.NN_cmdRequest.list_speed[4])
			# --
			self.app_setValue.lbv_plan = self.NN_cmdRequest.command
			# --
			# --
			self.app_setValue.lbv_velLeft = str(self.driver1_respond.speed)
			self.app_setValue.lbv_velRight = str(self.driver2_respond.speed)
			# --
			self.app_setValue.lbv_idGoal = str(self.status_goalControl.ID_follow)
			# --
			self.app_setValue.lbv_ex = str(round(self.parking_status.ss_x, 3))
			self.app_setValue.lbv_ey = str(round(self.parking_status.ss_y, 3))
			self.app_setValue.lbv_eg = str(round(self.parking_status.ss_a, 3))

			# -- loadcell
			weight = 0
			weight =  int(self.loadCell_respond.weight) - self.weight_origin
			if (weight < 0):
				weight = 0

			self.app_setValue.lbv_load = str(weight)
			# -- sensorUp
			if (self.lift_status.sensorUp.data == 0):
				self.app_setColor.lbc_limit2 = 0
			else:
				self.app_setColor.lbc_limit2 = 1
			# -- sensorDown
			if (self.lift_status.sensorDown.data == 0):
				self.app_setColor.lbc_limit1 = 0
			else:
				self.app_setColor.lbc_limit1 = 1
			# -- sensorLift
			if (self.lift_status.sensorLift.data == 0):
				self.app_setColor.lbc_liftSensor = 0
			else:
				self.app_setColor.lbc_liftSensor = 1

			# -- Nut xoa loi
			if (self.main_info.button2 == True):
				self.app_setColor.lbc_clear = 1
			else:
				self.app_setColor.lbc_clear = 0

			if (self.main_info.EMC_status == False):
				self.app_setColor.lbc_emg = 1
			else:
				self.app_setColor.lbc_emg = 0

			if (self.HC_info.vacham == 1):
				self.app_setColor.lbc_blsock = 0
			else:
				self.app_setColor.lbc_blsock = 1

			self.pub_setValue.publish(self.app_setValue)
			self.pub_dataColor.publish(self.app_setColor)

			self.rate.sleep()

def main():
	# Start the job threads
	program = data_synthesis()
	# ip = program.get_ipAuto("wlo2")
	program.run()

if __name__ == '__main__':
	main()