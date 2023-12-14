#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# DATE: 22/10/2022
# UPDATE: 22/02/2023
# AUTHOR: HOANG VAN QUANG - BEE
# Update: 18/05/2023
# Update: 30/05/2023: Lỗi đi chéo tại vị trí nhận/trả hàng.

import rospy
import sys
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import *

from geometry_msgs.msg import Twist, Pose, Point, Quaternion

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees, acos, fabs
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int8
from message_pkg.msg import *

class Velocities:
	def __init__(self, vel_x = 0.0, vel_y = 0.0, vel_r = 0.0):
		self.x = vel_x # - m/s
		self.y = vel_y # - m/s
		self.r = vel_r # - rad/s

class PoseSimple:
	def __init__(self, x = 0.0, y = 0.0, r = 0.0):
		self.x = x 		# - mm
		self.y = y 		# - mm
		self.angle = r 	# - rad

class GoalFollow:
	def __init__(self):
		self.id = 0
		self.pose = Pose()			# - Pose()
		self.point = Point()			# - Posi ()
		
		self.angleLine = 0 			# - rad (5 | not care)
		self.angleFinal = 0 		# - rad (5 | not care)

		self.directionTravel = 0 	# 

		self.roadWidth = 0 			# - m  (-1 | not care)
		self.deltaAngle = 0 		# - rad.
		self.speed = 0     			# - (0.2 - 0.8 m/s) (not care - 0).

class rickshaw_navigation():
	def __init__(self):
		print("ROS Initial: navigation!")
		rospy.init_node('rickshaw_navigation', anonymous = False) # False

		self.control_frequence = rospy.get_param("control_frequence", 10)
		self.control_frequence = 100
		self.debug = rospy.get_param("debug", 1)
		# --
		self.linear_max = rospy.get_param("linear_max", 0.24) # m/s
		self.linear_max = 0.68         #0.68
		self.linear_min = rospy.get_param("linear_min", 0.006)
		self.linear_min = 0.006
		# --
		self.rotation_max = rospy.get_param("rotation_max", 0.4) # rad/s
		self.rotation_max = 0.4
		self.rotation_min = rospy.get_param("rotation_min", 0.01)
		self.rotation_min = 0.006
		# --
		self.accuracy_position = rospy.get_param("accuracy_position", 0.02) # m
		self.accuracy_position = 0.01
		self.accuracy_angle = rospy.get_param("accuracy_angle", radians(0.5)) # rad
		self.accuracy_angle = radians(1)
		# --
		self.deceleration_distance = rospy.get_param("deceleration_distance_", 0.4) # m
		self.deceleration_distance = 0.6

		self.deceleration_angle = rospy.get_param("deceleration_angle", radians(15)) # rad
		self.deceleration_angle = radians(30)
		# --
		self.acceleration_distance_ = rospy.get_param("acceleration_distance_", 0.3) # m
		self.acceleration_angle = rospy.get_param("acceleration_angle", radians(20)) # rad
		# -- 
		self.rate = rospy.Rate(self.control_frequence)
		self.angleThreshold_mustRotation = radians(40) # rad
		# -- thông số khi đi thẳng.
		self.angleThreshold_adjustment = radians(0.4) # rad - ngưỡng điều chỉnh khi đi thẳng.

		# -------------------------------- PUBLISH AND SUBCRIBER ---------------------------------

		self.pub_requestFields = rospy.Publisher("/HC_fieldRequest", Int8, queue_size = 60)
		# -- 
		self.pub_navigationRespond = rospy.Publisher("/navigation_respond", Navigation_respond, queue_size = 60)
		self.navigationRespond = Navigation_respond()
		# -- 
		self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size = 60)
		self.cmd_vel = Twist()
		# -- -- -- -- 
		rospy.Subscriber('/safety_NAV', Int8, self.callback_safetyNAV)
		self.safetyNAV = Int8()
		self.is_safetyNAV = 0
		self.timeStampe_safetyNAV = rospy.Time.now()
		# --
		rospy.Subscriber("/HC_info", HC_info, self.callback_safetyHC)
		self.safetyHC = HC_info()
		self.is_safetyHC = 0
		self.timeStampe_safetyHC = rospy.Time.now()
		# --
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.powerInfo = POWER_info()
		self.is_main = 0
		# --
		# rospy.Subscriber("/robotPose_nav", PoseStamped, self.callback_robotPoseNAV)
		# self.robotPoseNAV = PoseStamped()
		# self.is_robotPoseNAV = 0

		rospy.Subscriber('/agv_pose', Pose, self.callback_robotPoseNAV, queue_size = 20)
		self.robotPoseNAV = PoseStamped()
		self.is_robotPoseNAV = 0

		# --
		rospy.Subscriber("/robot_pose", PoseStamped, self.callback_robotPose2)
		self.robotPose_2 = PoseStamped()
		# --
		rospy.Subscriber("/navigation_query", Navigation_query, self.callback_navigationQuery)
		self.navigationQuery = Navigation_query()
		self.is_navigationQuery = 0

		self.pub_path_local = rospy.Publisher('/path_plan_local', Path, queue_size= 20)
		self.pub_path_global = rospy.Publisher('/path_global', Path, queue_size= 20)		

		# -------------------------------- VARIABLE ---------------------------------
		# -- -- -- -- Server Command
		self.SCD_GoalID = 0
		self.SCD_GoalX = 0
		self.SCD_GoalY = 0
		self.SCD_GoalAngle = 0
		self.SCD_listID = [0, 0, 0, 0, 0]
		self.SCD_listX  = [0, 0, 0, 0, 0]
		self.SCD_listY  = [0, 0, 0, 0, 0]

		self.SCD_listRoadWidth   = [0, 0, 0, 0, 0]
		self.SCD_listAngleLine   = [0, 0, 0, 0, 0]
		self.SCD_listAngleFinal  = [0, 0, 0, 0, 0]
		self.SCD_listDirectionTravel = [0, 0, 0, 0, 0]
		self.SCD_listSpeed = [0, 0, 0, 0, 0]
		# -- -- -- -- 
		self.SCD_listID_old = [0, 0, 0, 0, 0] # - Xử lý lỗi không tự đi tiếp sau khi dừng Tránh.
		# -
		self.completed_move = 0
		self.completed_angle = 0
		self.completed_all = 0
		# -- 
		self.goalFollow_now = GoalFollow()
		self.goalFollow_next = GoalFollow()
		self.idFollow_now = 0
		self.idFollow_next = 0
		self.pos_idFollow_now = -1
		self.pos_idFollow_next = -1
		self.idGoal_now = 0
		self.angleGoal_finnal = 0
		self.speed_runNow = 100 # - persent: 0 - 100%.
		# - 
		self.typeCompleted = 0
		self.typeCompleted_relatively = 0
		self.typeCompleted_absolute = 1
		# - 
		self.typeMove = 0 
		self.typeMove_simple = 0 
		self.typeMove_special = 1

		# - Di chuyển bằng đầu hay đuôi.
		self.moveBy = 0
		self.moveBy_ahead = 0
		self.moveBy_tail = 1
		# -
		self.step_moveSpecial = 0
		self.step_moveSimple = 0

		self.step_move = 0
		self.step_goPoint = 0
		# -- 
		self.flag_errorSoFar_distance = 0
		self.flag_errorSoFar_angle = 0
		# -- 
		self.distance_nearGoal = 0 # -
		# -- 
		self.message_show = ['', '', '', '', '', '', '']
		# -- 
		self.saveTime_measureHz = time.time()
		self.saveTime_pubCmdVel = time.time()
		# -- 
		self.filterCounter = 0
		self.filterPoseSimple_Total = PoseSimple()
		self.poseSimple = PoseSimple()
		# --
		self.lengthRobot = 1.8
		# -- 
		self.simple_angleRight = 0
		self.special_angleRight = 0
		self.simple_isNearGoal = 0
		self.special_isNearGoal = 0
		self.flag_removeRotationFirst = 0 # - Bỏ bước xoay ban đầu.
		self.flag_identifyInformation = 1
		# - 
		self.goalSimple = GoalFollow()
		self.poseRobot_save = Pose()
		self.distance_save = 0
		self.modeMove_now = 0
		# - 
		self.flag_resetVel = 1
		self.count_resetVel = 0
		# -- add
		self.timeWait_stoped = 0.4 # - ms
		self.saveTime_waitStop = time.time()
		self.flag_increaseVel = 1
		# -
		self.velociate_increase = 0.0
		self.saveTime_increaseVel = time.time()
		self.flag_increaseVel = 1
		# --
		self.string_debug1 = ''
		self.string_debug2 = ''

		# -- path plan
		self.path_plan = Path()
		self.path_plan.header.frame_id = 'world'
		self.path_plan.header.stamp = rospy.Time.now()

		self.allow_path_plan = True

	def point_path(self, x, y):
		point = PoseStamped()
		point.header.frame_id = 'world'        
		point.header.stamp = rospy.Time.now()
		point.pose.position.x = x
		point.pose.position.y = y
		point.pose.position.z = -1.0
		point.pose.orientation.w = 1.0
		return point

	def callback_safetyNAV(self, data):
		self.safetyNAV = data
		self.is_safetyNAV = 1
		self.timeStampe_safetyNAV = rospy.Time.now()

	def callback_safetyHC(self, data):
		self.safetyHC = data
		self.is_safetyHC = 1
		self.timeStampe_safetyHC = rospy.Time.now()

	def callback_Main(self, data):
		self.powerInfo = data
		self.is_main = 1

	def callback_navigationQuery(self, data):
		self.navigationQuery = data
		self.is_navigationQuery = 1
		
	def callback_robotPoseNAV(self, data):
		self.robotPoseNAV.pose = data
		self.is_robotPoseNAV = 1

	def callback_robotPose2(self, data):
		self.robotPose_2 = data
		self.robotPoseNAV = data
		self.is_robotPoseNAV = 1
		# print (self.robotPose_2.pose)

	# -------------- DETECT LOST ---------------------------
	def detectLost_safetyHC(self):
		delta_t = rospy.Time.now() - self.timeStampe_safetyHC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_safetyNAV(self):
		delta_t = rospy.Time.now() - self.timeStampe_safetyNAV
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_poseNAV(self):
		delta_t = rospy.Time.now() - self.robotPoseNAV.header.stamp
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0
	# -------------------------------------------------------
	def publish_cmdVel(self, vel):
		t = (time.time() - self.saveTime_pubCmdVel)%60 
		if (t >= 1/20.):
			self.pub_cmdVel.publish(vel)
			self.saveTime_pubCmdVel = time.time()

	def constrain(self, value_in, value_min, value_max):
		value_out = 0.0
		if value_in < value_min:
			value_out = value_min
		elif value_in > value_max:
			value_out = value_max
		else:
			value_out = value_in
		return value_out

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def angleLine0_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		d_x = pointB.x - pointA.x
		d_y = pointB.y - pointA.y
		ang = 0
		if d_x == 0:
			if d_y >= 0:
				ang = pi/2.
			else:
				ang = -pi/2.
		else:
			if d_y == 0:
				if d_x > 0:
					ang = 0
				else:
					ang = pi
			else:
				ang = atan2(d_y, d_x)
				if ang > pi:
					ang = ang - 2*pi
				if ang < -pi:
					ang = 2*pi + ang
		return ang

	# - New 18.05.2023
	def angleLine1_AB(P1, P2):
		delta_y = P2.y - P1.y
		delta_x = P2.x - P1.x
		return atan2(delta_y, delta_x)

	def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		a = pointA.y - pointB.y
		b = pointB.x - pointA.x
		ang = 0.0
		if b == 0:
			if a < 0:
				ang = pi/2.0
			elif a > 0:
				ang = -pi/2.0
		elif a == 0:
			if -b < 0:
				ang = 0.0
			elif -b > 0:
				ang = pi
		else:
			ang = acos(b/sqrt(b*b + a*a))
			if -a/b > 0:
				if fabs(ang) > pi/2:
					ang = -ang
				else:
					ang = ang
			else:
				if fabs(ang) > pi/2:
					ang = ang
				else:
					ang = -ang

		return ang

	def angleRobot_Goal(self, poseRobot, pointGoal): # - 
		angle_robot = self.quaternion_to_euler(poseRobot.orientation)
		angle_robotToGoal = self.angleLine_AB(poseRobot.position, pointGoal)
		delta_angle = angle_robotToGoal - angle_robot
		delta_angle = self.limitAngle(delta_angle)
		return delta_angle

	def angleThreePoint_ATB(self, pointA, pointT, pointB): # - Point()
		angle_TA = self.angleLine_AB(pointT, pointA)
		angle_TB = self.angleLine_AB(pointT, pointB)
		# -
		delta_angle = angle_TB - angle_TA
		delta_angle = abs(self.limitAngle(delta_angle))
		# print ("angle_TA: ", degrees(angle_TA) )
		# print ("angle_TB: ", degrees(angle_TB) )
		# print ("delta_angle: ", degrees(delta_angle) )
		return delta_angle

	def fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.
		distancePoint = self.calculate_distance(robotPose.position, goalPose.position)
		# --
		angle_goalToRobot = self.angleLine_AB(goalPose.position, robotPose.position)
		# angle_robotToGoal = self.angleLine_AB(robotPose.position, goalPose.position)
		# --
		angleGoal = self.quaternion_to_euler(goalPose.orientation)
		angleRobot = self.quaternion_to_euler(robotPose.orientation)
		# -- 
		# print ("angleRobot: ", degrees(angleRobot))
		# print ("angleGoal: ", degrees(angleGoal))
		# print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		# -- 
		angleNew_goalToRobot = angle_goalToRobot - angleGoal
		angleNew_goalToRobot = self.limitAngle(angleNew_goalToRobot)
		# -- 
		# print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
		# -- 
		angleRobot_new = angleRobot - angleGoal
		angleRobot_new = self.limitAngle(angleRobot_new)
		# -- 
		poseRobot_new = Pose()
		poseRobot_new.position.x = cos(angleNew_goalToRobot)*distancePoint
		poseRobot_new.position.y = sin(angleNew_goalToRobot)*distancePoint
		poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)
		# print ("poseRobot_new X: ", poseRobot_new.position.x)
		# print ("poseRobot_new Y: ", poseRobot_new.position.y)
		# print ("poseRobot_new R: ", degrees(angleRobot_new) )
		# print (str(round(poseGoal_new.position.x, 3)) + " | " + str(round(poseGoal_new.position.y, 3)))

		poseGoal_new = Pose()
		poseGoal_new.orientation = self.euler_to_quaternion(0)

		return poseRobot_new, poseGoal_new

	def find_pos(self, arr, val):
		for i in range( len(arr) ):
			if (val == arr[i] ):
				return i
		return -1

	def measure_Hz(self):
		t = (time.time() - self.saveTime_measureHz)%60 
		print ("-- Hz -- : ", 1/t)
		self.saveTime_measureHz = time.time()

	def refresh_moveSimple(self):
		self.step_moveSimple = 0
		self.show_message_smart('refresh_moveSimple', 0)

	def refresh_moveSpecial(self):
		self.step_moveSpecial = 0
		self.show_message_smart('refresh_moveSpecail', 0)

	def stop_run(self):
		self.cmd_vel = Twist()
		self.show_message_smart("STOP RUN!", 1)

	def show_message_smart(self, msg, pos_save):
		if msg != self.message_show[pos_save]:
			self.message_show[pos_save] = msg
			print (msg)

	def mediumCoordinates(self, poseSimple):
		self.filterCounter += 1
		self.filterPoseSimple_Total.x += poseSimple.x
		self.filterPoseSimple_Total.y += poseSimple.y
		self.filterPoseSimple_Total.angle += poseSimple.angle
		# -
		if self.filterCounter >= 4:
			x = round( self.filterPoseSimple_Total.x/(self.filterCounter+1) , 3)
			y = round( self.filterPoseSimple_Total.y/(self.filterCounter+1), 3)
			ang = round( self.filterPoseSimple_Total.angle/(self.filterCounter+1), 3)
			print ("filterPose: " + str(x) + " | " +  str(y) + " | " +  str(degrees(ang)) )
			self.filterCounter = 0
			self.filterPoseSimple_Total.x = poseSimple.x
			self.filterPoseSimple_Total.y = poseSimple.y
			self.filterPoseSimple_Total.angle = poseSimple.angle

	def angleFromRoadWidth(self, lengthRobot, roadWidth):
		return roadWidth/lengthRobot

	def convert_valueRange(self, value_in, range1_min, range1_max, range2_min, range2_max):
		value_out = 0
		if value_in < range1_min:
			value_out = range2_min
		elif value_in > range1_max:
			value_out = range2_max
		else:
			rate_1 = value_in/float(range1_max - range1_min)
			value_out = range2_min + rate_1*(range2_max - range2_min)
		return value_out

	def message_status(self, val_sts):
		switcher={
			0:'Stop - Completed Wait new Goal', # 
			1:'Stop - Di chuyen het diem', # 
			2:'Stop - Vuong vat can - Xoay', # 
			3:'Stop - Vuong vat can - Tinh tien', #
			4:'Stop - Vuong vat can - Tren cao', #
			5:'Runing - Pose simple - Rotaion', #
			6:'Runing - Pose special - Go strange', #
			7:'Stop - Error - Lost: ', #
			8:'Stop - Error - Wrong: ', #
			100:'' # 
		}
		return switcher.get(val_sts, '')

	# Sử dụng để loại bỏ điểm đã đi qua trong lộ trình di chuyển hiện tại.
	def is_between(self, pointRobot, goalPoint_now, goalPoint_next): # - Point()
		angle_robotNow = self.angleLine_AB(pointRobot, goalPoint_now)
		angle_nowNext = self.angleLine_AB(goalPoint_now, goalPoint_next)
			
		delta_angle = angle_nowNext - angle_robotNow
		delta_angle = self.limitAngle(delta_angle)

		# print ("angle_robotNow: ", degrees(angle_robotNow) )
		# print ("angle_nowNext: ", degrees(angle_nowNext) )
		# print ("delta_angle: ", degrees(delta_angle) )

		if abs(delta_angle) >= radians(176):
			print ("- Bet -")
			return 1
		else:
			print ("- Out -")
			return 0

	def acceleration_velociate(self, vel_start, vel_end):
		sts = 0
		if self.velociate_increase >= vel_end:
			self.velociate_increase = vel_end
			sts = 1
		else:
			delta_time = (time.time() - self.saveTime_increaseVel)%60 
			if delta_time >= 0.15: # - 150 ms tang toc 1 lan.
				self.velociate_increase += (vel_start + 0.04)
				self.saveTime_increaseVel = time.time()
			sts = 0

		return sts, self.velociate_increase

	# - add 30/05/2023
	def acceleration_velociate2(self, vel_start, vel_end):
		sts = 0
		if self.velociate_increase >= vel_end:
			self.velociate_increase = vel_end
			sts = 1
		else:
			delta_time = (time.time() - self.saveTime_increaseVel)%60 
			if delta_time >= 0.15: # - 150 ms tang toc 1 lan.
				self.velociate_increase += (vel_start + 0.032)
				self.saveTime_increaseVel = time.time()
			sts = 0

		return sts, self.velociate_increase

	def rotation_run(self, poseRobot, angle_target, accuracy_angle):
		stopRun = 0
		vel_out = Velocities()
		# -- Sai số.
		angle_robot = self.quaternion_to_euler(poseRobot.orientation)
		delta_angle =  angle_target - angle_robot
		delta_angle = self.limitAngle(delta_angle)
		# -- Điều hướng.
		if (abs(delta_angle) > accuracy_angle): # 
			coefficient_angle = delta_angle/radians(40) # self.deceleration_angle
			coefficient_angle = self.constrain(coefficient_angle, -1, 1)
			vel_out.r = coefficient_angle*0.32

			if abs(vel_out.r) < self.rotation_min and vel_out.r != 0:
				if vel_out.r > 0:
					vel_out.r = self.rotation_min
				else:
					vel_out.r = -self.rotation_min

			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_out.r = 0.0
				stopRun = 1

			return 0, vel_out, stopRun
		else:
			vel_out.r = 0.0
			return 1, vel_out, stopRun

	# -- Di chuyển lùi.
	def moveSimple_tail(self, robotPose, goalPose, accuracy_distance): # - OK
		vel_level = Velocities()
		# -- Tính toán độ lệch.
		angle_robot_tail = self.quaternion_to_euler(robotPose.orientation) + pi
		angle_robotToGoal = self.angleLine_AB(robotPose.position, goalPose.position)

		delta_distance = self.calculate_distance(robotPose.position, goalPose.position)
		delta_angle =  angle_robotToGoal - angle_robot_tail
		delta_angle = self.limitAngle(delta_angle)

		# print ("delta_angle: ", delta_angle)
		# print ("step_moveSimple: ", self.step_moveSimple)

		# -- Kiểm tra hoàn thành.
		if ( abs(delta_distance) < accuracy_distance):
			self.step_moveSimple = 10

		# -- Đáp ứng góc đầu.
		if (self.step_moveSimple == 0):
			vel_level.x = 0
			if (abs(delta_angle) > radians(1)):
				coefficient_angle = delta_angle/radians(60)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.3# 0.4		
			else:
				vel_level.r = 0.0
				self.step_moveSimple = 1

		# -- Đáp ứng khoảng cách.
		if (self.step_moveSimple == 1):
			# -- X
			coefficient_distance = abs(delta_distance)/self.deceleration_distance
			coefficient_distance = self.constrain(coefficient_distance, 0, 1)
			vel_level.x = coefficient_distance*self.linear_max*-1
			vel_level.x = self.constrain(vel_level.x, self.linear_max*-1, self.linear_min*-1)
			# -- R
			if (abs(delta_angle) > radians(0.1) ): # do lech du lon thi cho phep quay.
				coefficient_angle = delta_angle/radians(30) # self.deceleration_angle
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.4 # 0.4
			else:
				vel_level.r = 0.0

			# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
			if (abs(delta_angle) > radians(50)):
				self.step_moveSimple = 0

		# -- Hoàn thành.
		if (self.step_moveSimple == 10):
			vel_level = Velocities()

		return self.step_moveSimple, vel_level

	# -- OKK . Bắt vật cản - Bỏ điểm đầu.
	def moveSimple_near(self, robotPose, goalSimple, speedMax, accuracy_distance, distance_near, moveRotationFirst): # - OK
		vel_level = Velocities()
		angle_robot = 0
		delta_angle = 0
		angle_robotToGoal = 0
		is_nearGoal = 0
		stopRun = 0
		# -- Tính toán độ lệch khoảng cách.
		delta_distance = self.calculate_distance(robotPose.position, goalSimple.pose.position)
		# -- Tính toán độ lệch góc.
		if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
			angle_robot = self.quaternion_to_euler(robotPose.orientation)
		else:
			angle_robot = self.quaternion_to_euler(robotPose.orientation) + pi
			angle_robot = self.limitAngle(angle_robot)

		angle_robotToGoal = self.angleLine_AB(robotPose.position, goalSimple.pose.position)
		delta_angle =  angle_robotToGoal - angle_robot
		delta_angle = self.limitAngle(delta_angle)
		
		# -- Kiểm tra hướng di chuyển.
		if self.step_moveSimple == 0:
			# -- Tính toán độ lệch.
			angle_robot = self.quaternion_to_euler(robotPose.orientation)
			angle_robotToGoal = self.angleLine_AB(robotPose.position, goalSimple.pose.position)

			delta_angle = angle_robotToGoal - angle_robot
			delta_angle = self.limitAngle(delta_angle)

			# - Nếu cho phép đi 2 đầu.
			if goalSimple.directionTravel == 1:
				if abs(delta_angle) <= pi/2: # - Di chuyển xuôi.
					self.moveBy = self.moveBy_ahead
					angle_robot = self.quaternion_to_euler(robotPose.orientation)
				else: # - Di chuyển ngược.
					self.moveBy = self.moveBy_tail
					angle_robot = self.quaternion_to_euler(robotPose.orientation) + pi
					angle_robot = self.limitAngle(angle_robot)
			else:
				self.moveBy = 0

			if moveRotationFirst == 1:
				self.step_moveSimple = 2
				print ("Bo - moveRotationFirst")
			else:
				self.step_moveSimple = 1
			
		# -- Đáp ứng góc đầu.   (Xoay trước khi tiến) 
		if self.step_moveSimple == 1:
			if abs(delta_distance) < distance_near:
				is_nearGoal = 1
			else:
				is_nearGoal = 0

			# -- Tính toán độ lệch góc.
			if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
				angle_robot = self.quaternion_to_euler(robotPose.orientation)
			else:
				angle_robot = self.quaternion_to_euler(robotPose.orientation) + pi
				angle_robot = self.limitAngle(angle_robot)

			angle_robotToGoal = self.angleLine_AB(robotPose.position, goalSimple.pose.position)
			delta_angle =  angle_robotToGoal - angle_robot
			delta_angle = self.limitAngle(delta_angle)

			# -- Kiểm tra hoàn thành.
			if abs(delta_distance) < accuracy_distance:
				self.step_moveSimple = 3
				if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
					self.show_message_smart("----- MoveSimple Ahead: Completed STEP 1-----", 4)
				else:
					self.show_message_smart("----- MoveSimple Tail: Completed STEP 1-----", 4)

			# - Xoay
			if abs(delta_angle) > radians(0.4):
				vel_level.x = 0
				if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
					# -- Hien thi kieu di chuyen.
					self.navigationRespond.typeRun = 11
					# --
					self.show_message_smart("----- MoveSimple Ahead: Spining AngleFirst -----", 4)
				else:
					# -- Hien thi kieu di chuyen.
					self.navigationRespond.typeRun = -11
					# --
					self.show_message_smart("----- MoveSimple Tail: Spining AngleFirst -----", 4)
				# -
				coefficient_angle = delta_angle/radians(30)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.4
				# -
				if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
					if vel_level.r > 0:
						vel_level.r = self.rotation_min
					else:
						vel_level.r = -self.rotation_min
			else:
				vel_level.r = 0.0
				# -- add
				self.step_moveSimple = 6 # 2
				self.saveTime_waitStop = time.time()

			# - Bắt vật cản.
			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_level = Velocities()
				stopRun = 1

			elif self.safetyHC.zone_sick_ahead == 2 or self.safetyHC.zone_sick_behind == 2:
				if abs(vel_level.r) > 0.12:
					vel_level.r = vel_level.r*0.4
				stopRun = 0

			self.string_debug1 = "Goc Dau "
			self.string_debug2 = 'RTG: '+ str(round(angle_robotToGoal, 3)) + ' |R: ' + str(round(angle_robot, 3)) + ' |D: ' + str(round(delta_angle, 3)) + ' |V: ' + str(round(vel_level.r, 3))

		# -- Chờ dừng hẳn. # -- add
		if self.step_moveSimple == 6:
			delta_t = (time.time() - self.saveTime_waitStop)%60 
			if delta_t >= self.timeWait_stoped:
				self.step_moveSimple = 2
				self.velociate_increase = 0
				self.flag_increaseVel = 1

		# -- Đáp ứng khoảng cách.
		if self.step_moveSimple == 2:
			# -- Tính toán độ lệch góc.
			if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 12
				# --
				angle_robot = self.quaternion_to_euler(robotPose.orientation)
			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -12
				# --
				angle_robot = self.quaternion_to_euler(robotPose.orientation) + pi
				angle_robot = self.limitAngle(angle_robot)

			angle_robotToGoal = self.angleLine_AB(robotPose.position, goalSimple.pose.position)
			delta_angle =  angle_robotToGoal - angle_robot
			delta_angle = self.limitAngle(delta_angle)
			
			if abs(delta_distance) < distance_near:
				is_nearGoal = 1
			else:
				is_nearGoal = 0

			# -- Kiểm tra hoàn thành.
			if abs(delta_distance) < accuracy_distance:
				self.step_moveSimple = 3
				if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
					self.show_message_smart("----- MoveSimple Ahead: Completed -----", 4)
				else:
					self.show_message_smart("----- MoveSimple Tail: Completed -----", 4)

			# -- Di chuyển.
			coefficient_distance = abs(delta_distance)/0.7 # self.deceleration_distance
			coefficient_distance = self.constrain(coefficient_distance, 0, 1.0)

			# - Giam Nhanh Toc Do.
			coefficient_distance = coefficient_distance # *coefficient_distance #*coefficient_distance

			if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
				self.show_message_smart("----- MoveSimple Ahead: Go Strange -----", 4)
				# vel_level.x = coefficient_distance*speedMax
				# -- Tang dan toc do
				vel_now = coefficient_distance*speedMax
				if self.flag_increaseVel == 1:
					sts, vel_level.x = self.acceleration_velociate(0, vel_now)
					if sts == 1:
						self.flag_increaseVel = 0
				else:
					vel_level.x = vel_now
				# --
				vel_level.x = self.constrain(vel_level.x, self.linear_min, speedMax)
				# print ("S H: " + str(coefficient_distance) + " | Vx= " + str(vel_level.x) )
				# print ("--------------")
			else:
				self.show_message_smart("----- MoveSimple Tail: Go Strange -----", 4)
				# vel_level.x = coefficient_distance*speedMax*-1
				# -- Tang dan toc do
				vel_now = coefficient_distance*speedMax
				if self.flag_increaseVel == 1:
					sts, vel_level.x = self.acceleration_velociate(0, vel_now)
					vel_level.x = vel_level.x*-1
					if sts == 1:
						self.flag_increaseVel = 0
				else:
					vel_level.x = vel_now*-1
				# --
				vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)

			# -- R
			if abs(delta_angle) >= radians(0.3): # do lech du lon thi cho phep quay.
				coefficient_angle = delta_angle/radians(30) # self.deceleration_angle
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.4
			else:
				vel_level.r = 0.0

			# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
			if abs(delta_angle) > radians(60):
				if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
					self.show_message_smart("----- MoveSimple Ahead: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)
				else:
					# ang = self.quaternion_to_euler(robotPose.orientation)
					delta_angle__ = angle_robotToGoal - angle_robot
					self.show_message_smart("----- MoveSimple Tail: Delta_Angle larg -> Spining ----- delta_angle: "  + str( degrees(delta_angle)) + " | " + str( degrees(delta_angle__)) + " | R: " + str(degrees(angle_robot)) + " | " + str( degrees(angle_robotToGoal) ) , 4)
				self.step_moveSimple = 1

			# - Bắt vật cản.
			if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
				if self.safetyHC.zone_sick_ahead == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					stopRun = 1
				elif self.safetyHC.zone_sick_ahead == 2:
					if abs(vel_level.x) > 0.12:
						vel_level.x = vel_level.x*0.4
					stopRun = 0
			else:
				if self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					stopRun = 1
				elif self.safetyHC.zone_sick_behind == 2:
					if abs(vel_level.x) > 0.12:
						vel_level.x = vel_level.x*0.4
					stopRun = 0

			self.string_debug1 = 'Rx: '+ str(round(robotPose.position.x, 3)) + ' |Ry: ' + str(round(robotPose.position.y, 3)) + ' |D: ' + str(round(delta_distance, 3)) + ' |V: ' + str(round(vel_level.x, 3))
			self.string_debug2 = 'RTG: '+ str(round(angle_robotToGoal, 3)) + ' |R: ' + str(round(angle_robot, 3)) + ' |D: ' + str(round(delta_angle, 3)) + ' |V: ' + str(round(vel_level.r, 3))

		# -- Hoàn thành di chuyển khoảng cách -> Đáp ứng góc cuối.  ( Xoay góc cuối )
		if self.step_moveSimple == 3:
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = 14
			# --
			vel_level.x = 0
			if goalSimple.angleFinal >= 5:
				if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
					self.show_message_smart("----- MoveSimple Ahead: Spining AngleFinnal No -----" + str(goalSimple.angleFinal), 4)
				else:
					self.show_message_smart("----- MoveSimple Tail: Spining AngleFinnal No -----" + str(goalSimple.angleFinal), 4)
				self.step_moveSimple = 10

			else:
				angle_robot = self.quaternion_to_euler(robotPose.orientation)

				delta_angleFinal = goalSimple.angleFinal - angle_robot
				delta_angleFinal = self.limitAngle(delta_angleFinal)

				if abs(delta_angleFinal) > radians(0.4):
					if self.moveBy == self.moveBy_ahead: # - Di chuyển xuôi.
						self.show_message_smart("----- MoveSimple Ahead: Spining AngleFinnal Yes -----" + str(goalSimple.angleFinal), 4)
					else:
						self.show_message_smart("----- MoveSimple Tail: Spining AngleFinnal Yes -----" + str(goalSimple.angleFinal), 4)

					coefficient_angle = delta_angleFinal/radians(40)
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3

					if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
						if vel_level.r > 0:
							vel_level.r = self.rotation_min
						else:
							vel_level.r = -self.rotation_min
				else:
					vel_level.r = 0.0
					self.step_moveSimple = 10

				# - Bắt vật cản.
				if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					stopRun = 1
				elif self.safetyHC.zone_sick_ahead == 2 or self.safetyHC.zone_sick_behind == 2:
					if abs(vel_level.r) > 0.12:
						vel_level.r = vel_level.r*0.4
					stopRun = 0

				self.string_debug1 = "Goc Cuoi"
				self.string_debug2 = 'RGF: '+ str(round(goalSimple.angleFinal, 3)) + ' |R: ' + str(round(angle_robot, 3)) + ' |D: ' + str(round(delta_angleFinal, 3)) + ' |V: ' + str(round(vel_level.r, 3))

		# -- Hoàn thành.
		if self.step_moveSimple == 10:
			vel_level = Velocities()

		return self.step_moveSimple, vel_level, is_nearGoal, stopRun	

	# - OKK
	def moveSpecial_near(self, robotPose, goalSimple, speedMax, accuracy_distance, distance_near, moveRotationFirst):
		vel_level = Velocities()
		pose_extraGoal = Pose()
		stopRun = 0
		sts_run = 0
		is_nearGoal = 0
		poseGoal = Pose()

		# -
		poseGoal.position = goalSimple.point
		poseGoal.orientation = self.euler_to_quaternion(goalSimple.angleLine)
		# -- Chuyển đổi hệ tọa độ.
		poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoal(robotPose, goalSimple.pose) # poseGoal)

		# --------- Phát hiện lỗi --------- #
		angle_robotFake = self.quaternion_to_euler(poseRobot_fake.orientation)
		pose_extraGoal.orientation = poseGoal_fake.orientation

		# -- Kiểm tra Khoảng cách lệch an toàn.
		if abs(poseRobot_fake.position.y) > (goalSimple.roadWidth/2.):
			self.flag_errorSoFar_distance = 1
		else:
			self.flag_errorSoFar_distance = 0

		# -- Kiểm tra Góc lệch an toàn.
		if abs(angle_robotFake) < (goalSimple.deltaAngle/2.):
			self.flag_errorSoFar_angle = 0
		else:
			self.flag_errorSoFar_angle = 1
		
		# - Tạm thời loại bỏ lỗi.
		self.flag_errorSoFar_angle = 0
		self.flag_errorSoFar_distance = 0

		# -- Phát hiện lỗi
		if self.step_moveSpecial == 0:
			if self.flag_errorSoFar_distance != 0 or self.flag_errorSoFar_angle != 0:
				if self.flag_errorSoFar_distance != 0 and self.flag_errorSoFar_angle == 0:
					sts_run = -1

				elif self.flag_errorSoFar_distance == 0 and self.flag_errorSoFar_angle != 0:
					sts_run = -2
				else:
					sts_run = -3
			else:
				self.step_moveSpecial = 1

			# - Xác định hướng di chuyển. (Đi đầu, đi đuôi).
			if poseRobot_fake.position.x < 0: # - 
				self.moveBy = self.moveBy_ahead
			else:
				self.moveBy = self.moveBy_tail
		
			if moveRotationFirst == 1 or self.special_angleRight == 1:
				self.step_moveSpecial = 2
				print ("Bo - moveRotationFirst")

		# - Di chuyển - Đáp ứng góc đầu.
		if self.step_moveSpecial == 1:
			if self.moveBy == self.moveBy_ahead:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 21
				# --
				self.show_message_smart("----- MoveSpecial Ahead: Spining AngleFirst -----", 4)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x + 0.6
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				# -
				angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
				delta_angle = angle_robotToGoal - angle_robot_ahead

				# # -- Xác nhận hoàn thành.
				# delta_dis = self.calculate_distance(poseRobot_fake.position, Point() )
				# if abs(delta_dis) < accuracy_distance or poseRobot_fake.position.x > accuracy_distance:
				# # if poseRobot_fake.position.x > accuracy_distance:
				# 	self.step_moveSpecial = 3 #
				# 	vel_level = Velocities()
				# 	self.show_message_smart("----- MoveSpecial Ahead F: Go Completed -----", 4)
				# # print ("Ahead Delta: " + str(round(degrees(delta_angle), 2)) + " Dis_x: " + str(round(poseRobot_fake.position.x, 3)) + " Dis_y: " + str(round(poseRobot_fake.position.y, 3)) + " Dis: " + str(round(delta_dis, 3) ) )

			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -21
				# --
				self.show_message_smart("----- MoveSpecial Tail: Spining AngleFirst -----", 4)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x - 0.6
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				# -
				angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
				delta_angle = angle_robotToGoal - angle_robot_tail

				# # -- Xác nhận hoàn thành.
				# delta_dis = self.calculate_distance(poseRobot_fake.position, Point() )
				# if abs(delta_dis) < accuracy_distance or poseRobot_fake.position.x < accuracy_distance:
				# # if poseRobot_fake.position.x < accuracy_distance:
				# 	self.step_moveSpecial = 3 #
				# 	vel_level = Velocities()
				# 	self.show_message_smart("----- MoveSpecial Tail F: Go Completed -----", 4)
				# # print ("Ahead Delta: " + str(round(degrees(delta_angle), 2)) + " Dis_x: " + str(round(poseRobot_fake.position.x, 3)) + " Dis_y: " + str(round(poseRobot_fake.position.y, 3)) + " Dis: " + str(round(delta_dis, 3)) )
			# -
			delta_angle = self.limitAngle(delta_angle)
			vel_level.x = 0
			if abs(delta_angle) > radians(1):
				coefficient_angle = delta_angle/radians(32)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.3 # 0.4

				if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
					if vel_level.r > 0:
						vel_level.r = self.rotation_min
					else:
						vel_level.r = -self.rotation_min
			else:
				vel_level.r = 0.0
				self.special_angleRight = 1
				self.step_moveSpecial = 11 # 2
				# -- add 30/05/2023 -
				self.saveTime_waitStop = time.time()

			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_level.x = 0.0
				vel_level.r = 0.0
				stopRun = 1

		# -- Chờ dừng hẳn. # -- add 30/05/2023 -
		if self.step_moveSpecial == 11:
			delta_t = (time.time() - self.saveTime_waitStop)%60 
			if delta_t >= self.timeWait_stoped:
				self.step_moveSpecial = 2
				self.velociate_increase = 0
				self.flag_increaseVel = 1

		# -- Di chuyển - Bám đường.
		if self.step_moveSpecial == 2:
			if self.moveBy == self.moveBy_ahead:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 22
				# --
				self.show_message_smart("----- MoveSpecial Ahead: Go Strange -----", 4)
				distance_offset = 0.6 # 0.4
				if abs(poseRobot_fake.position.y) > distance_offset:
					dif_x = 0.0
				else:
					dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x + dif_x
				pose_extraGoal.position.y = 0
				delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
				# --
				deltaDistance_real = self.calculate_distance(poseRobot_fake.position, Point())
				
				# -- 
				angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				delta_angle = angle_robotToGoal - angle_robot_ahead
				delta_angle = self.limitAngle(delta_angle)
				# -
				speedMax_linear = speedMax
				if abs(deltaDistance_real) < 0.2:
					# print ("deltaDistance_real: ", deltaDistance_real)
					speedMax_linear = 0.04
				# -- X
				coefficient_distance = abs(delta_distance)/speedMax_linear
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)

				# vel_level.x = coefficient_distance*speedMax_linear
				# -- add 30/05/2023 - Tang dan toc do
				vel_now = coefficient_distance*speedMax_linear
				if self.flag_increaseVel == 1:
					sts, vel_level.x = self.acceleration_velociate2(0, vel_now)
					if sts == 1:
						self.flag_increaseVel = 0
				else:
					vel_level.x = vel_now

				vel_level.x = self.constrain(vel_level.x, self.linear_min, speedMax)
				# -- R
				if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
					coefficient_angle = delta_angle/radians(40) # self.deceleration_angle
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3
				else:
					vel_level.r = 0.0

				# --
				self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
				self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_ahead, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

				# - Bat vat can
				if self.safetyHC.zone_sick_ahead == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					# print ("VAT CAN TIEN")
					stopRun = 1

				# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
				if abs(delta_angle) > radians(80):
					self.step_moveSpecial = 1
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecail Ahead: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)
					
				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x > accuracy_distance:
					self.step_moveSpecial = 3 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Ahead: Completed -----", 4)
				# -
				delta_distance_2 = self.calculate_distance(robotPose.position, goalSimple.pose.position)
				# -- Khoảng cách gần tới đích.
				if abs(delta_distance_2) < distance_near:
					is_nearGoal = 1

			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -22
				# --
				self.show_message_smart("----- MoveSpecial Tail: Go Strange -----", 4)
				distance_offset = 0.6 # 0.4
				if abs(poseRobot_fake.position.y) > distance_offset:
					dif_x = 0.0
				else:
					dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x - dif_x
				pose_extraGoal.position.y = 0
				delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
				# --
				deltaDistance_real = self.calculate_distance(poseRobot_fake.position, Point())
				# -- 
				angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				delta_angle = angle_robotToGoal - angle_robot_tail
				delta_angle = self.limitAngle(delta_angle)
				# -
				speedMax_linear = speedMax
				if abs(deltaDistance_real) < 0.2:
					speedMax_linear = 0.04
				# -- X
				coefficient_distance = abs(delta_distance)/speedMax_linear
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)

				# vel_level.x = coefficient_distance*speedMax_linear*-1
				# -- add 30/05/2023 - Tang dan toc do
				vel_now = coefficient_distance*speedMax_linear
				if self.flag_increaseVel == 1:
					sts, vel_level.x = self.acceleration_velociate2(0, vel_now)
					vel_level.x = vel_level.x*-1
					if sts == 1:
						self.flag_increaseVel = 0
				else:
					vel_level.x = vel_now*-1

				vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)
				# -- R
				if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
					coefficient_angle = delta_angle/radians(30) # self.deceleration_angle
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3 # 0.4
				else:
					vel_level.r = 0.0

				# --
				self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
				self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_tail, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

				# - Bat vat can
				if self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					# print ("VAT CAN LUI")
					stopRun = 1

				# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
				if abs(delta_angle) > radians(80):
					self.step_moveSpecial = 1
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecail Tail: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)

				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x < accuracy_distance:
					self.step_moveSpecial = 3 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Tail: Completed -----", 4)
				# print ("SPS: ", poseRobot_fake.position.x)

				delta_distance_2 = self.calculate_distance(robotPose.position, goalSimple.pose.position)
				# -- Xác nhận gần đích. -> Để chuyển sang điểm tiếp theo.
				if abs(delta_distance_2) < distance_near:
					is_nearGoal = 1
					# print ("Gan toi Dich")

		# -- Xoay đáp ứng góc cuối.
		if self.step_moveSpecial == 3:
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = 24
			# --
			if goalSimple.angleFinal >= 5:
				self.step_moveSpecial = 10
			else:
				angle_robot = self.quaternion_to_euler(robotPose.orientation)
				delta_angleFinal = goalSimple.angleFinal - angle_robot
				delta_angleFinal = self.limitAngle(delta_angleFinal)

				if abs(delta_angleFinal) > radians(0.4):
					coefficient_angle = delta_angleFinal/radians(40)
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3

					if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
						if vel_level.r > 0:
							vel_level.r = self.rotation_min
						else:
							vel_level.r = -self.rotation_min
				else:
					vel_level.r = 0.0
					self.step_moveSpecial = 10

				# - Bat vat can
				if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					stopRun = 1

		# -- Hoàn thành.
		if self.step_moveSpecial == 10:
			vel_level = Velocities()

		return self.step_moveSpecial, vel_level, is_nearGoal, stopRun

	def moveSpecial_target(self, robotPose, goalSimple, speedMax, accuracy_distance, distance_slow, moveRotationFirst):
		vel_level = Velocities()
		pose_extraGoal = Pose()
		stopRun = 0
		sts_run = 0
		sts_error = 0
		is_nearGoal = 0
		poseGoal = Pose()
		# -
		poseGoal.position = goalSimple.point
		poseGoal.orientation = self.euler_to_quaternion(goalSimple.angleLine)
		# -- Chuyển đổi hệ tọa độ.
		poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoal(robotPose, poseGoal)

		# --------- Phát hiện lỗi --------- #
		angle_robotFake = self.quaternion_to_euler(poseRobot_fake.orientation)
		pose_extraGoal.orientation = poseGoal_fake.orientation

		# -- Kiểm tra Khoảng cách lệch an toàn.
		if abs(poseRobot_fake.position.y) > (goalSimple.roadWidth/2.):
			self.flag_errorSoFar_distance = 1
		else:
			self.flag_errorSoFar_distance = 0

		# -- Kiểm tra Góc lệch an toàn.
		if abs(angle_robotFake) < (goalSimple.deltaAngle/2.):
			self.flag_errorSoFar_angle = 0
		else:
			self.flag_errorSoFar_angle = 1
		
		# - Tạm thời loại bỏ lỗi.
		self.flag_errorSoFar_angle = 0
		self.flag_errorSoFar_distance = 0

		# -- Phát hiện lỗi
		if self.step_moveSpecial == 0:
			if self.flag_errorSoFar_distance != 0 or self.flag_errorSoFar_angle != 0:
				if self.flag_errorSoFar_distance != 0 and self.flag_errorSoFar_angle == 0:
					sts_error == -1
				elif self.flag_errorSoFar_distance == 0 and self.flag_errorSoFar_angle != 0:
					sts_error == -2
				else:
					sts_error == -3
			else:
				self.step_moveSpecial = 1

			# - Xác định hướng di chuyển. (Đi đầu, đi đuôi).
			if poseRobot_fake.position.x < 0: # - 
				self.moveBy = self.moveBy_ahead
				print ("------ moveBy_ahead ")
			else:
				self.moveBy = self.moveBy_tail
				print ("------ moveBy_tail ")
		
			if moveRotationFirst == 1 or self.special_angleRight == 1:
				self.step_moveSpecial = 2

			# - Check  
			print ("-- goalSimple ----------")
			print ("X = ", goalSimple.point.x)
			print ("Y = ", goalSimple.point.y)
			print ("angleLine = ", goalSimple.angleLine)
			print ("angleFinal = ", goalSimple.angleFinal)
			print ("--- poseRobot_fake ---------------------")
			print ("X = ", poseRobot_fake.position.x)
			print ("Y = ", poseRobot_fake.position.y)
			print ("------------------------")

		# -- Di chuyển - Đáp ứng góc đầu.
		if self.step_moveSpecial == 1:
			if self.moveBy == self.moveBy_ahead:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 21
				# --
				self.show_message_smart("----- MoveSpecial Target Ahead: Spining AngleFirst -----", 4)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x + 0.4
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				# -
				angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
				delta_angle = angle_robotToGoal - angle_robot_ahead
				
				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x > accuracy_distance:
					self.step_moveSpecial = 4 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Target Ahead F: Go Completed -----", 4)
					# print ("Delta: " + str(degrees(delta_angle)) + "Dis_x: " + str(poseRobot_fake.position.x) + "Dis_y: " + str(poseRobot_fake.position.y) )
			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -21
				# --
				self.show_message_smart("----- MoveSpecial Target Tail: Spining AngleFirst -----", 4)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x - 0.4
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				# -
				angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
				delta_angle = angle_robotToGoal - angle_robot_tail
				
				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x < accuracy_distance:
					self.step_moveSpecial = 4 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Target Tail F: Go Completed -----", 4)
					# print ("Delta: " + str(degrees(delta_angle)) + "Dis_x: " + str(poseRobot_fake.position.x) + "Dis_y: " + str(poseRobot_fake.position.y) )
			# -
			delta_angle = self.limitAngle(delta_angle)
			vel_level.x = 0
			if abs(delta_angle) > radians(1):
				coefficient_angle = delta_angle/radians(40)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.3 
				# -
				if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
					if vel_level.r > 0:
						vel_level.r = self.rotation_min
					else:
						vel_level.r = -self.rotation_min
			else:
				vel_level.r = 0.0
				self.step_moveSpecial = 2
				self.special_angleRight = 1

			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_level.x = 0.0
				vel_level.r = 0.0
				stopRun = 1

		# -- Di chuyển - Bám đường.
		if self.step_moveSpecial == 2:
			if self.moveBy == self.moveBy_ahead:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 22
				# --
				self.show_message_smart("----- MoveSpecial Target Ahead: Go Strange -----", 4)
				distance_offset = 0.64
				if abs(poseRobot_fake.position.y) > distance_offset:
					dif_x = 0.0
				else:
					dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
				pose_extraGoal.position.x = poseRobot_fake.position.x + dif_x
				pose_extraGoal.position.y = 0
				delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
				# -- 
				angle_robot_ahead = self.quaternion_to_euler(poseRobot_fake.orientation)
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				delta_angle = angle_robotToGoal - angle_robot_ahead
				delta_angle = self.limitAngle(delta_angle)

				# -- X
				coefficient_distance = abs(delta_distance)/0.3
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)
				vel_level.x = coefficient_distance*speedMax
				vel_level.x = self.constrain(vel_level.x, self.linear_min, speedMax)
				# -- R
				if abs(delta_angle) > radians(0.6): # do lech du lon thi cho phep quay.
					coefficient_angle = delta_angle/radians(40) # self.deceleration_angle
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3
				else:
					vel_level.r = 0.0

				# --
				self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
				self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_ahead, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

				# - Bat vat can
				if self.safetyHC.zone_sick_ahead == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					# print ("VAT CAN TIEN")
					stopRun = 1

				# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
				if abs(delta_angle) > radians(80):
					self.step_moveSpecial = 1
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecail Target Ahead: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)

				# -- Khoảng cách gần tới đích.
				if abs(poseRobot_fake.position.x) < distance_slow:
					self.step_moveSpecial = 3
					# print ("-------------------------------- Gan toi Dich -> Di chuyen Cham den Dich")

				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x > accuracy_distance:
					self.step_moveSpecial = 4 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Target Ahead: Go Completed -----", 4)
					print ("PPP: ", poseRobot_fake.position.x)

			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -22
				# --
				self.show_message_smart("----- MoveSpecial Target Tail: Go Strange -----", 4)
				distance_offset = 0.4
				if abs(poseRobot_fake.position.y) > distance_offset:
					dif_x = 0.0
				else:
					dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)
				# -
				pose_extraGoal.position.x = poseRobot_fake.position.x - dif_x
				pose_extraGoal.position.y = 0
				delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
				# -- 
				angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
				angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
				delta_angle = angle_robotToGoal - angle_robot_tail
				delta_angle = self.limitAngle(delta_angle)

				# -- X
				coefficient_distance = abs(delta_distance)/0.3
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)
				vel_level.x = coefficient_distance*speedMax*-1
				vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)
				# -- R
				if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
					coefficient_angle = delta_angle/radians(30) # self.deceleration_angle
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.3 # 0.4
				else:
					vel_level.r = 0.0

				# --
				self.string_debug1 = 'Rx: '+ str(round(poseRobot_fake.position.x, 3)) + ' |Ry: ' + str(round(poseRobot_fake.position.y, 3)) + ' |Gx: ' + str(round(pose_extraGoal.position.x, 3))
				self.string_debug2 = 'RG: '+ str(round(angle_robotToGoal, 2)) + ' |R: ' + str(round(angle_robot_tail, 2)) + ' |D: ' + str(round(delta_angle, 2)) + ' |V: ' + str(round(vel_level.r, 3))

				# - Bat vat can
				if self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level = Velocities()
					# print ("VAT CAN LUI")
					stopRun = 1

				# -- Góc lệch lớn quá. Phải dừng lại. Xoay lại góc.
				if abs(delta_angle) > radians(80):
					self.step_moveSpecial = 1
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecail Target Tail: Delta_Angle larg -> Spining ----- Dis: " + str(abs(delta_distance)), 4)

				# -- Xác nhận gần đích. -> Di chuyển chậm đến đích.
				if abs(poseRobot_fake.position.x) < distance_slow:
					self.step_moveSpecial = 3
					# print ("-------------------------------- Gan toi Dich -> Di chuyen Cham den Dich")

				# -- Xác nhận hoàn thành.
				if poseRobot_fake.position.x < accuracy_distance:
					self.step_moveSpecial = 4 #
					vel_level = Velocities()
					self.show_message_smart("----- MoveSpecial Target Tail: Go Completed -----", 4)
					print ("PPP: ", poseRobot_fake.position.x)

		# -- Di chuyen thang.
		if self.step_moveSpecial == 3:
			if self.moveBy == self.moveBy_ahead:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = 23
				# --
				pose_extraGoal.position.x = 0
				pose_extraGoal.position.y = 0
				delDis_near = poseRobot_fake.position.x
				vel_level.r = 0
				# -- X
				coefficient_distance = abs(delDis_near)/0.3
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)
				vel_level.x = coefficient_distance*speedMax
				vel_level.x = self.constrain(vel_level.x, self.linear_min, speedMax)

				if self.safetyHC.zone_sick_ahead == 1 or self.safetyNAV.data == 1:
					vel_level.x = 0.0
					# print ("VAT CAN TRUOC")
					stopRun = 1

				# print ("poseRobot_fake.position.x: " + str( round(poseRobot_fake.position.x, 3)) + " | " + str(vel_level.x) )
				if poseRobot_fake.position.x > accuracy_distance:
					print ("-------------------------------- Move Slow Ahead OK")
					self.step_moveSpecial = 4
					vel_level = Velocities()
			else:
				# -- Hien thi kieu di chuyen.
				self.navigationRespond.typeRun = -23
				# --
				pose_extraGoal.position.x = 0
				pose_extraGoal.position.y = 0
				delDis_near = poseRobot_fake.position.x
				vel_level.r = 0
				# -- X
				coefficient_distance = abs(delDis_near)/0.3
				coefficient_distance = self.constrain(coefficient_distance, 0, 1)
				vel_level.x = coefficient_distance*-speedMax
				vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)

				if self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
					vel_level.x = 0.0
					# print ("VAT CAN SAU")
					stopRun = 1

				# print ("poseRobot_fake.position.x: " + str( round(poseRobot_fake.position.x, 3)) + " | " + str(vel_level.x) )
				if poseRobot_fake.position.x < accuracy_distance:
					print ("-------------------------------- Move Slow Tail OK")
					self.step_moveSpecial = 4
					vel_level = Velocities()

		# -- Xoay đáp ứng góc cuối.
		if self.step_moveSpecial == 4:
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = 24
			# --
			if goalSimple.angleFinal >= 5:
				self.step_moveSpecial = 10
			else:
				self.show_message_smart("----- MoveSpecial Target: Rotation Final -----", 4)
				angle_robot = self.quaternion_to_euler(robotPose.orientation)
				delta_angleFinal = goalSimple.angleFinal - angle_robot
				delta_angleFinal = self.limitAngle(delta_angleFinal)

				if abs(delta_angleFinal) > radians(0.6):
					coefficient_angle = delta_angleFinal/radians(40)
					coefficient_angle = self.constrain(coefficient_angle, -1, 1)
					vel_level.r = coefficient_angle*0.2
					# -
					if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
						if vel_level.r > 0:
							vel_level.r = self.rotation_min
						else:
							vel_level.r = -self.rotation_min
				else:
					print ("goalSimple.angleFinal: ", degrees(goalSimple.angleFinal) )
					print ("----- MoveSpecial Target: Goc cuoi: " + str(degrees(angle_robot)) )
					vel_level.r = 0.0
					self.step_moveSpecial = 10

				# --
				self.string_debug1 = 'Goc cuoi'
				self.string_debug2 = 'RG: '+ str(round(goalSimple.angleFinal, 3)) + ' |R: ' + str(round(angle_robot, 3)) + ' |D: ' + str(round(delta_angleFinal, 3)) + ' |V: ' + str(round(vel_level.r, 3))

			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_level.x = 0.0
				vel_level.r = 0.0
				stopRun = 1
				
		# -- Hoàn thành.
		if self.step_moveSpecial == 10:
			vel_level = Velocities()

		return self.step_moveSpecial, vel_level, stopRun, sts_error

	def update_goalSimple(self):
		self.speed_runNow = 70
		self.pos_idFollow_now = 0
		self.idFollow_now = self.SCD_listID[self.pos_idFollow_now]
		# -
		self.goalFollow_now.id = self.idFollow_now
		self.goalFollow_now.pose.position.x = self.SCD_listX[self.pos_idFollow_now]
		self.goalFollow_now.pose.position.y = self.SCD_listY[self.pos_idFollow_now]

		self.goalFollow_now.angleLine = self.SCD_listAngleLine[self.pos_idFollow_now]
		self.goalFollow_now.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_now]
		
		self.goalFollow_now.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_now]

		if self.goalFollow_now.angleLine >= 5:
			self.goalFollow_now.pose.orientation = self.euler_to_quaternion(0)
		else:
			self.goalFollow_now.pose.orientation = self.euler_to_quaternion(self.goalFollow_now.angleLine)

		self.goalFollow_now.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_now]
		self.goalFollow_now.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_now.roadWidth)
		self.goalFollow_now.speed = self.SCD_listSpeed[self.pos_idFollow_now]

		print ("CP pos_idFollow_now: ", self.pos_idFollow_now)
		print ("CP ID_now: ", self.idFollow_now)

		print ("CP Goal x: ", self.goalFollow_now.pose.position.x)
		print ("CP Goal y: ", self.goalFollow_now.pose.position.y)
		print ("CP roadWidth : ", self.goalFollow_now.roadWidth)

		# -- Tìm điểm tiếp theo.
		self.pos_idFollow_next = self.pos_idFollow_now + 1
		if self.pos_idFollow_next > 4 or self.SCD_listID[self.pos_idFollow_next] <= 0:
			self.idFollow_next = 0
			self.goalFollow_next.id = self.idFollow_next
			print ("Not ID Goal Next")
		else:
			self.idFollow_next = self.SCD_listID[self.pos_idFollow_next]
			self.goalFollow_next.id = self.idFollow_next
			self.goalFollow_next.pose.position.x = self.SCD_listX[self.pos_idFollow_next]
			self.goalFollow_next.pose.position.y = self.SCD_listY[self.pos_idFollow_next]

			self.goalFollow_next.angleLine = self.SCD_listAngleLine[self.pos_idFollow_next]
			self.goalFollow_next.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_next]
			
			self.goalFollow_next.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_next]

			if (self.goalFollow_next.angleLine >= 5):
				self.goalFollow_next.pose.orientation = self.euler_to_quaternion(0)
			else:
				self.goalFollow_next.pose.orientation = self.euler_to_quaternion(self.goalFollow_next.angleLine)

			self.goalFollow_next.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_next]
			self.goalFollow_next.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_next.roadWidth)
			self.goalFollow_next.speed = self.SCD_listSpeed[self.pos_idFollow_next]

	# -- OKK
	def moveSpecial_charger(self, robotPose, goalSimple, speedMax, accuracy_distance, distance_slow, moveRotationFirst):
		vel_level = Velocities()
		pose_extraGoal = Pose()
		stopRun = 0
		sts_error = 0
		is_nearGoal = 0
		poseGoal = Pose()
		poseGoal.position = goalSimple.point
		poseGoal.orientation = self.euler_to_quaternion(goalSimple.angleLine)
		# -- Chuyển đổi hệ tọa độ.
		poseRobot_fake, poseGoal_fake = self.fakePose_robotFollowGoal(robotPose, poseGoal)

		# --------- Phát hiện lỗi --------- #
		angle_robotFake = self.quaternion_to_euler(poseRobot_fake.orientation)
		pose_extraGoal.orientation = poseGoal_fake.orientation

		# -- Kiểm tra Khoảng cách lệch an toàn.
		if abs(poseRobot_fake.position.y) > (goalSimple.roadWidth/2.):
			self.flag_errorSoFar_distance = 1
		else:
			self.flag_errorSoFar_distance = 0

		# -- Kiểm tra Góc lệch an toàn.
		if abs(angle_robotFake) < (goalSimple.deltaAngle/2.):
			self.flag_errorSoFar_angle = 0
		else:
			self.flag_errorSoFar_angle = 1
		
		# - Tạm thời loại bỏ lỗi.
		self.flag_errorSoFar_angle = 0
		self.flag_errorSoFar_distance = 0

		# - Kiểm tra vị trí Robot.
		if poseRobot_fake.position.x < 0:
			self.flag_errorSoFar_distance = 1
		else:
			self.flag_errorSoFar_distance = 0

		self.flag_errorSoFar_distance = 0
		# -- Phát hiện lỗi
		if self.step_moveSpecial == 0:
			if self.flag_errorSoFar_distance != 0 or self.flag_errorSoFar_angle != 0:
				if self.flag_errorSoFar_distance != 0 and self.flag_errorSoFar_angle == 0:
					sts_error = -1

				elif self.flag_errorSoFar_distance == 0 and self.flag_errorSoFar_angle != 0:
					sts_error = -2
				else:
					sts_error = -3
			else:
				self.step_moveSpecial = 1

			# - Xác định hướng di chuyển. (Đi đầu, đi đuôi).
			self.moveBy = self.moveBy_tail
			# -
			if moveRotationFirst == 1 or self.special_angleRight == 1:
				self.step_moveSpecial = 2

		# -- Di chuyển - Đáp ứng góc đầu.
		elif self.step_moveSpecial == 1:
			# --
			self.navigationRespond.typeRun = -21
			# --
			angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
			# -
			angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
			delta_angle = angle_robotToGoal - angle_robot_tail
			delta_angle = self.limitAngle(delta_angle)
			# -
			vel_level.x = 0
			if abs(delta_angle) > radians(1):
				coefficient_angle = delta_angle/radians(40)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.3 # 0.4	

				if abs(vel_level.r) < self.rotation_min and vel_level.r != 0:
					if vel_level.r > 0:
						vel_level.r = self.rotation_min
					else:
						vel_level.r = -self.rotation_min
			else:
				vel_level.r = 0.0
				self.step_moveSpecial = 2
				self.special_angleRight = 1
			# -
			if self.safetyHC.zone_sick_ahead == 1 or self.safetyHC.zone_sick_behind == 1 or self.safetyNAV.data == 1:
				vel_level.x = 0.0
				vel_level.r = 0.0
				stopRun = 1

		# -- Di chuyển - Bám đường.
		elif self.step_moveSpecial == 2:
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = -22
			# -- Tính toán độ lệch.
			distance_offset = 0.4 # 0.4
			if abs(poseRobot_fake.position.y) > distance_offset:
				dif_x = 0.0
			else:
				dif_x = distance_offset - pow(poseRobot_fake.position.y, 2)

			pose_extraGoal.position.x = poseRobot_fake.position.x - dif_x
			pose_extraGoal.position.y = 0

			delta_distance = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
			# -- 
			angle_robot_tail = self.quaternion_to_euler(poseRobot_fake.orientation) + pi
			angle_robotToGoal = self.angleLine_AB(poseRobot_fake.position, pose_extraGoal.position)
			delta_angle =  angle_robotToGoal - angle_robot_tail
			delta_angle = self.limitAngle(delta_angle)
			# -- X
			coefficient_distance = abs(delta_distance)/0.3
			coefficient_distance = self.constrain(coefficient_distance, 0, 1)
			vel_level.x = coefficient_distance*speedMax*-1
			vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)
			# -- R
			if abs(delta_angle) > radians(0.4): # do lech du lon thi cho phep quay.
				coefficient_angle = delta_angle/radians(30)
				coefficient_angle = self.constrain(coefficient_angle, -1, 1)
				vel_level.r = coefficient_angle*0.3 # 0.4
			else:
				vel_level.r = 0.0

			# -- Góc lệch quá lớn. Phải dừng lại. Xoay lại góc.
			if abs(delta_angle) > radians(80):
				self.step_moveSpecial = 1
				vel_level = Velocities()
				self.show_message_smart("----- MoveSpecail Charge Tail: Delta_Angle larg -> Spining -----", 4)
			
			# -- Xác nhận hoàn thành.
			# print ("poseRobot_fake.position.x: " + str( round(poseRobot_fake.position.x, 3)) + " | " + str(vel_level.x) )
			if poseRobot_fake.position.x < accuracy_distance:
				self.step_moveSpecial = 10

			delta_distance = self.calculate_distance(robotPose.position, goalSimple.point)
			# -- Xác nhận gần đích. -> Di chuyển chậm đến đích.
			if abs(delta_distance) < distance_slow:
				self.step_moveSpecial = 3
				# print ("-------------------------------- Gan toi Dich -> Di chuyen Cham den Dich")

		# -- Di chuyen thang.
		elif self.step_moveSpecial == 3:
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = -23
			# --
			pose_extraGoal.position.x = 0
			pose_extraGoal.position.y = 0
			# delDis_near = self.calculate_distance(poseRobot_fake.position, pose_extraGoal.position)
			delDis_near = poseRobot_fake.position.x
			vel_level.r = 0
			# -- X
			coefficient_distance = abs(delDis_near)/0.3
			coefficient_distance = self.constrain(coefficient_distance, 0, 1)
			vel_level.x = coefficient_distance*-speedMax
			vel_level.x = self.constrain(vel_level.x, speedMax*-1, self.linear_min*-1)

			# print ("poseRobot_fake.position.x: " + str( round(poseRobot_fake.position.x, 3)) + " | " + str(vel_level.x) )
			if poseRobot_fake.position.x < accuracy_distance:
				print ("-------------------------------- Move Slow Tail OK")
				self.step_moveSpecial = 10
				vel_level = Velocities()
			
		# -- Hoàn thành.
		elif self.step_moveSpecial == 10:
			vel_level = Velocities()
			# print ("poseRobot_fake.position.x: ", round(poseRobot_fake.position.x, 3) )

		return self.step_moveSpecial, vel_level, stopRun, sts_error

	# -- Di chuyen thang ra khoi sac - OKK
	def moveSimple_goStrange(self, robotPose, goalPose, speedMax, accuracy_distance):
		vel_level = Velocities()
		stopRun = 0
		if self.step_moveSimple == 0: # -- Xác định vị trí tương đối của Robot với điểm đích.
			self.step_moveSimple = 1
			self.poseRobot_save = robotPose
			self.distance_save = self.calculate_distance(robotPose.position, goalPose.position)
			if self.distance_save > 2:
				self.distance_save = 2

		elif self.step_moveSimple == 1: # - Di chuyển.
			# -- Hien thi kieu di chuyen.
			self.navigationRespond.typeRun = 13
			# --
			delta_distance = self.distance_save - self.calculate_distance(robotPose.position, self.poseRobot_save.position)
			# -
			coefficient_distance = abs(delta_distance)/0.2
			coefficient_distance = self.constrain(coefficient_distance, 0, 1)
			# -
			vel_level.x = coefficient_distance*speedMax
			vel_level.x = self.constrain(vel_level.x, 0.01, speedMax)
			# - 
			if delta_distance <= accuracy_distance:
				self.step_moveSimple = 10
				vel_level = Velocities()

			# - Bat vat can
			if self.safetyHC.zone_sick_ahead == 1 or self.safetyNAV.data == 1:
				vel_level = Velocities()
				stopRun = 1
				
		elif self.step_moveSimple == 10: # - Hoàn thành.
			vel_level = Velocities()

		return self.step_moveSimple, vel_level, stopRun

	def loopRun(self):
		isNear = 0
		stopRun = 0
		sts_run = 0
		# ---- Kiểm tra danh sách điểm hợp lệ không.
		if self.step_move == 0:
			length_1 = len(self.SCD_listID)
			length_2 = len(self.SCD_listX)
			length_3 = len(self.SCD_listY)
			length_4 = len(self.SCD_listAngleLine)
			length_5 = len(self.SCD_listAngleFinal)
			length_6 = len(self.SCD_listDirectionTravel)
			length_7 = len(self.SCD_listSpeed)
			length_8 = len(self.SCD_listRoadWidth)
			# -
			if length_1 == 5 and length_2 == 5 and length_3 == 5 and length_4 == 5 and length_5 == 5 and length_6 == 5 and length_7 == 5 and length_8 == 5 and self.SCD_GoalID != 0:
				self.step_move = 1
			else:
				self.step_move = -1
			
		# -- Kiểm tra thay đổi, cập nhật đích.
		if self.step_move == 1:
			if self.idGoal_now != self.SCD_GoalID:
				self.idGoal_now = self.SCD_GoalID
				self.flag_identifyInformation = 1
				self.show_message_smart('------ Update New Target ID: ' + str(self.idGoal_now), 3)
				# - Stop.
				self.stop_run()
				# -- Cập nhật lại dữ liệu vận hành.
				self.refresh_moveSimple()
				self.refresh_moveSpecial()
				self.completed_all = 0
				self.completed_angle = 0
				self.completed_move = 0

				# -- Lấy điểm di chuyển.
				self.idFollow_now = self.SCD_listID[0]
				self.angleGoal_finnal = self.SCD_GoalAngle
				
				# -- Lấy tọa độ điểm.
				self.pos_idFollow_now = self.find_pos(self.SCD_listID, self.idFollow_now)
				if self.pos_idFollow_now == -1:
					self.idFollow_now = self.SCD_listID[0]
					self.pos_idFollow_now = 0
				else:
					self.goalFollow_now.id = self.idFollow_now
					self.goalFollow_now.pose.position.x = self.SCD_listX[self.pos_idFollow_now]
					self.goalFollow_now.pose.position.y = self.SCD_listY[self.pos_idFollow_now]
					# -
					self.goalFollow_now.angleLine  = self.SCD_listAngleLine[self.pos_idFollow_now]
					self.goalFollow_now.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_now]
					self.goalFollow_now.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_now]
					# -
					if self.goalFollow_now.angleLine >= 5:
						self.goalFollow_now.pose.orientation = self.euler_to_quaternion(0)
					else:
						self.goalFollow_now.pose.orientation = self.euler_to_quaternion(self.goalFollow_now.angleLine)
					self.goalFollow_now.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_now]
					self.goalFollow_now.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_now.roadWidth)
					self.goalFollow_now.speed = self.SCD_listSpeed[self.pos_idFollow_now]

				# -- Tìm điểm tiếp theo.
				self.pos_idFollow_next = self.pos_idFollow_now + 1
				if self.pos_idFollow_next > 4 or self.SCD_listID[self.pos_idFollow_next] <= 0:
					self.idFollow_next = 0
					self.goalFollow_next.id = self.idFollow_next
					print ("> -- Khong co diem tiep theo --- <")
				else:
					# --
					self.idFollow_next = self.SCD_listID[self.pos_idFollow_next]
					self.goalFollow_next.id = self.idFollow_next
					self.goalFollow_next.pose.position.x = self.SCD_listX[self.pos_idFollow_next]
					self.goalFollow_next.pose.position.y = self.SCD_listY[self.pos_idFollow_next]
					# -
					self.goalFollow_next.angleLine = self.SCD_listAngleLine[self.pos_idFollow_next]
					self.goalFollow_next.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_next]
					self.goalFollow_next.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_next]

					if self.goalFollow_next.angleLine >= 5:
						self.goalFollow_next.pose.orientation = self.euler_to_quaternion(0)
					else:
						self.goalFollow_next.pose.orientation = self.euler_to_quaternion(self.goalFollow_next.angleLine)

					self.goalFollow_next.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_next]
					self.goalFollow_next.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_next.roadWidth)
					self.goalFollow_next.speed = self.SCD_listSpeed[self.pos_idFollow_next]
						
			self.step_move = 2

		# -- Kiểm tra lỗi ID.
		if self.step_move == 2:
			# - 18/05/2023
			if self.idFollow_now > 0:
				self.step_move = 3
			else:
				self.step_move = -4

		# -- Kiểm tra hoàn thành.
		if self.step_move == 3:
			if self.completed_move == 0:
				self.step_move = 4
			else:
				if self.completed_angle == 0:
					self.step_move = 5
				else:
					self.step_move = 10

		# -- Di chuyển.
		if self.step_move == 4:
			self.navigationRespond.error = 0
			self.show_message_smart("RUNING...!", 0)
			self.navigationRespond.message = self.message_status(6)
			sts_run = 0

			""" > Xác định cách di chuyển và cách hoàn thành 1 điểm < """
			# -- Xác định cách hoàn thành 1 điểm -- #
			if self.flag_identifyInformation == 1:
				self.flag_identifyInformation = 0
				# - Điểm hiện tại là điểm Đích hoặc Không có điểm tiếp theo => Di chuyển chính xác.
				if self.goalFollow_now.id == self.idGoal_now:
					self.typeCompleted = self.typeCompleted_absolute   # - Tuyệt đối - Tới đích - Sai số khoảng cách nhỏ.
					print ("---- 1 ---- self.typeCompleted = self.typeCompleted_absolute" )
				else:
					# -- Vị trí tương đối giữa (AGV - Đích hiện tại - Đích tiếp theo). Góc lớn -> Di chuyển chính xác.
					if self.goalFollow_next.id > 0:
						angle_ATB = self.angleThreePoint_ATB(self.robotPoseNAV.pose.position, self.goalFollow_now.pose.position, self.goalFollow_next.pose.position)
						angle_robotGoal = self.angleRobot_Goal(self.robotPoseNAV.pose, self.goalFollow_now.pose.position)

						print ("angleThreePoint_ATB: ", degrees(angle_ATB) )
						print ("angle_robotGoal: ", degrees(angle_robotGoal) )

						if abs(angle_ATB) > radians(150) and self.goalFollow_now.directionTravel == self.goalFollow_next.directionTravel:
							self.typeCompleted = self.typeCompleted_relatively # - Tương đối - Gần Tới đích - Sai số khoảng cách lớn.
						else:
							self.typeCompleted = self.typeCompleted_absolute   # - Tuyệt đối
					else:
						print ("---- 2 ---- self.typeCompleted = self.typeCompleted_absolute" )
						self.typeCompleted = self.typeCompleted_absolute       # - Tuyệt đối

				# - Xác định cách di chuyển.
				if self.goalFollow_now.angleLine < 5:
					self.typeMove = self.typeMove_special # - Di chuyển theo cách đặc biệt - Bám góc.
				else:
					self.typeMove = self.typeMove_simple

				# - Xác định khong cach hoan thanh Near.
				if self.typeCompleted == self.typeCompleted_relatively:  # - Tương đối
					self.distance_nearGoal = 0.2      # 0.5
					self.show_message_smart("distance_nearGoal: " + str(self.distance_nearGoal) + " ID Goal: " + str(self.goalFollow_now.id) , 3)
				else:
					self.distance_nearGoal = 0
			# -
			if self.typeMove == self.typeMove_special: # - Di chuyển theo cách đặc biệt - Bám góc.
				self.show_message_smart("----------------------- MOVE SPECAIL! Type: " + str(self.typeCompleted), 3)
				# -
				errorThreshold = 0.2
				speedMax = 0.15          #0.15
				if self.typeCompleted == self.typeCompleted_absolute:
					errorThreshold = 0.01
					speedMax = (self.speed_runNow/100.)*0.2 # 0.2
				else:
					errorThreshold = 0.02
					speedMax = (self.speed_runNow/100.)*0.34 # 0.34

				if speedMax < 0.12:
					speedMax = 0.12

				# -
				self.simple_angleRight = 0
				self.flag_removeRotationFirst = 0
				sts_run, vel_run, isNear, stopRun = self.moveSpecial_near(self.robotPoseNAV.pose, self.goalFollow_now, speedMax, errorThreshold, self.distance_nearGoal, self.flag_removeRotationFirst)
				self.cmd_vel.linear.x  = vel_run.x
				self.cmd_vel.angular.z = vel_run.r

			else:
				self.show_message_smart("----------------------- MOVE SIMPLE! " + str(self.distance_nearGoal), 3)
				# -
				speedMax = (self.speed_runNow/100.)*self.linear_max
				if speedMax < 0.12:
					speedMax = 0.12
				# -
				self.special_angleRight = 0
				sts_run, vel_run, isNear, stopRun = self.moveSimple_near(self.robotPoseNAV.pose, self.goalFollow_now, speedMax, 0.1, self.distance_nearGoal, self.flag_removeRotationFirst)
				self.cmd_vel.linear.x  = vel_run.x
				self.cmd_vel.angular.z = vel_run.r

			self.navigationRespond.process = sts_run # self.step_move
			self.navigationRespond.stop = stopRun
			# -
			nearOK = 0
			if self.typeCompleted == self.typeCompleted_relatively and isNear == 1: # and self.typeMove == self.typeMove_simple:
				self.flag_removeRotationFirst = 1
				# self.show_message_smart("---- Near Start!", 3)
				nearOK = 1
			# -
			if sts_run == 10 or nearOK == 1: # and self.typeCompleted == self.typeCompleted_absolute: # or (self.typeCompleted == self.typeCompleted_relatively and isNear == 1):
				self.flag_identifyInformation = 1
				# self.flag_removeRotationFirst = 0
				# - Kiểm tra hoàn thành đến đích.
				if self.idFollow_now == self.idGoal_now:
					self.completed_move = 1
					# - Stop.
					self.stop_run()
					self.step_move = 5
				else:
					# -- -- --  Cập nhật điểm di chuyển mới.
					# -- Tìm vị trí trong mảng.
					self.pos_idFollow_now = self.find_pos(self.SCD_listID, self.idFollow_now)
					if self.pos_idFollow_now == -1:
						# - Stop.
						self.stop_run()
						self.step_move = -3
					else:
						self.refresh_moveSimple()
						self.refresh_moveSpecial()
						if self.pos_idFollow_now >= 4:
							self.step_move = -2
						else:
							# - 
							self.SCD_listID_old = self.SCD_listID
							# -
							self.speed_runNow = self.SCD_listSpeed[self.pos_idFollow_now] # - Cập nhật tốc độ di chuyển.
							self.show_message_smart("----- OK ID_now: " + str(self.idFollow_now), 3)
							self.pos_idFollow_now += 1
							self.idFollow_now = self.SCD_listID[self.pos_idFollow_now]
							# -
							self.goalFollow_now.id = self.idFollow_now
							self.goalFollow_now.pose.position.x = self.SCD_listX[self.pos_idFollow_now]
							self.goalFollow_now.pose.position.y = self.SCD_listY[self.pos_idFollow_now]

							self.goalFollow_now.angleLine = self.SCD_listAngleLine[self.pos_idFollow_now]
							self.goalFollow_now.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_now]
							
							self.goalFollow_now.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_now]

							if self.goalFollow_now.angleLine >= 5:
								self.goalFollow_now.pose.orientation = self.euler_to_quaternion(0)
							else:
								self.goalFollow_now.pose.orientation = self.euler_to_quaternion(self.goalFollow_now.angleLine)

							self.goalFollow_now.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_now]
							self.goalFollow_now.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_now.roadWidth)
							self.goalFollow_now.speed = self.SCD_listSpeed[self.pos_idFollow_now]
							self.step_move = 0

							
							print ("CP pos_idFollow_now: ", self.pos_idFollow_now)
							print ("CP ID_now: ", self.idFollow_now)

							print ("CP Goal x: ", self.goalFollow_now.pose.position.x)
							print ("CP Goal y: ", self.goalFollow_now.pose.position.y)
							print ("CP roadWidth : ", self.goalFollow_now.roadWidth)

							# -- Tìm điểm tiếp theo.
							self.pos_idFollow_next = self.pos_idFollow_now + 1
							if self.pos_idFollow_next > 4 or self.SCD_listID[self.pos_idFollow_next] <= 0:
								self.idFollow_next = 0
								self.goalFollow_next.id = self.idFollow_next
							else:
								# --
								if nearOK == 1:
									self.flag_removeRotationFirst = 1
								# --
								self.idFollow_next = self.SCD_listID[self.pos_idFollow_next]
								self.goalFollow_next.id = self.idFollow_next
								self.goalFollow_next.pose.position.x = self.SCD_listX[self.pos_idFollow_next]
								self.goalFollow_next.pose.position.y = self.SCD_listY[self.pos_idFollow_next]

								self.goalFollow_next.angleLine = self.SCD_listAngleLine[self.pos_idFollow_next]
								self.goalFollow_next.angleFinal = self.SCD_listAngleFinal[self.pos_idFollow_next]
								
								self.goalFollow_next.directionTravel = self.SCD_listDirectionTravel[self.pos_idFollow_next]

								if (self.goalFollow_next.angleLine >= 5):
									self.goalFollow_next.pose.orientation = self.euler_to_quaternion(0)
								else:
									self.goalFollow_next.pose.orientation = self.euler_to_quaternion(self.goalFollow_next.angleLine)

								self.goalFollow_next.roadWidth = self.SCD_listRoadWidth[self.pos_idFollow_next]
								self.goalFollow_next.deltaAngle = self.angleFromRoadWidth(self.lengthRobot, self.goalFollow_next.roadWidth)
								self.goalFollow_next.speed = self.SCD_listSpeed[self.pos_idFollow_next]
							
							print ("flag_removeRotationFirst : ", self.flag_removeRotationFirst)
			else:				
				self.step_move = 0

		# -- Quay đúng góc.
		if self.step_move == 5:
			stopRun = 0
			self.navigationRespond.error = 0
			self.show_message_smart("---- Spinning Finnal!", 3)
			self.navigationRespond.message = self.message_status(5)
			angle = self.quaternion_to_euler(self.robotPoseNAV.pose.orientation)
			sts, vel_run, stopRun = self.rotation_run(self.robotPoseNAV.pose, self.angleGoal_finnal, radians(0.6) )
			self.cmd_vel.linear.x = vel_run.x
			self.cmd_vel.angular.z = vel_run.r
			self.navigationRespond.stop = stopRun
			self.navigationRespond.process = sts
			if sts == 1:
				self.completed_angle = 1

			self.step_move = 0

		# -- Hoàn thành - Đợi lệnh mới.
		if self.step_move == 10:
			self.navigationRespond.error = 0
			self.completed_all = 1
			# - Stop.
			self.stop_run()
			# - 
			self.step_move = 0
			# -
			self.poseSimple.x = self.robotPoseNAV.pose.position.x
			self.poseSimple.y = self.robotPoseNAV.pose.position.y
			angle = self.quaternion_to_euler(self.robotPoseNAV.pose.orientation)
			self.poseSimple.angle = angle
			# self.show_message_smart("Completed All: " + str( degrees(angle) ), 3)

			self.navigationRespond.message = self.message_status(0)
			# print (self.poseSimple.x)

		# -- Lỗi: .
		if self.step_move == -1:
			# self.show_message_smart("ERROR PATH" + str(-1), 3)
			# - Stop.
			self.stop_run()
			self.step_move = 0
			# self.navigationRespond.message = self.message_status(8)
			self.navigationRespond.message = 'Move Path: Error Frame Path!'
			self.navigationRespond.error = 1

		# -- Lỗi: .
		if self.step_move == -2:
			# self.show_message_smart("DI CHUYEN HET DIEM " + str(0), 3)
			# - Stop.
			self.stop_run()
			self.step_move = 0
			# self.navigationRespond.message = self.message_status(1)
			self.navigationRespond.message = 'Move Path: Error Move All Points!'
			self.navigationRespond.error = 2

		# -- Lỗi: .
		if self.step_move == -3:
			# self.show_message_smart("ERROR FIND POST " + str(0), 3)
			# - Stop.
			self.stop_run()
			self.step_move = 0
			self.navigationRespond.message = 'Move Path: Error Find ID Point!'
			self.navigationRespond.error = 3

		# -- Lỗi: .
		if self.step_move == -4:
			# self.show_message_smart("ERROR ID FOLLOW", 3)
			# - Stop.
			self.stop_run()
			self.step_move = 0
			self.navigationRespond.message = 'Move Path: Error ID FOLLOW!'
			self.navigationRespond.error = 4
			self.navigationRespond.stop = 0
			# -- 
			if self.SCD_listID_old != self.SCD_listID:
				self.update_goalSimple()
				self.SCD_listID_old = self.SCD_listID
			# print ("------MM-----")

		# -- Status Run
		self.navigationRespond.status = 1
		self.navigationRespond.modeMove = 4
		self.navigationRespond.completed = self.completed_all
		self.navigationRespond.id_goalFollow = self.idFollow_now
		
	def generalCoordinator(self):
		if self.modeMove_now != self.navigationQuery.modeMove:
			self.modeMove_now = self.navigationQuery.modeMove
			self.navigationRespond.message = "Update - Stop and Reset"
			self.navigationRespond.status = 0
			self.navigationRespond.modeMove = 0
			self.navigationRespond.stop = 0
			self.navigationRespond.id_goalFollow = 0
			self.navigationRespond.error = 0
			self.cmd_vel = Twist()
			# -
			self.idGoal_now = 0
			# -
			self.step_moveSpecial = 0
			self.step_moveSimple = 0
			self.flag_errorSoFar_distance = 0
			self.flag_errorSoFar_angle = 0
			self.flag_removeRotationFirst = 0 # - Bỏ bước xoay ban đầu.
			self.simple_angleRight = 0
			self.special_angleRight = 0
			self.simple_isNearGoal = 0
			self.special_isNearGoal = 0
			self.completed_move = 0
			self.step_move = 0
			self.completed_all = 0
			self.completed_angle = 0
			self.completed_move = 0
			
		if self.modeMove_now == 0: # -- Stop and Reset.
			self.navigationRespond.message = "Stop and Reset"
			self.navigationRespond.status = 0
			self.navigationRespond.modeMove = 0
			self.navigationRespond.stop = 0
			self.navigationRespond.id_goalFollow = 0
			self.navigationRespond.error = 0
			self.cmd_vel = Twist()
			# -
			self.idGoal_now = 0
			# -
			self.step_moveSpecial = 0
			self.step_moveSimple = 0
			self.flag_errorSoFar_distance = 0
			self.flag_errorSoFar_angle = 0
			self.flag_removeRotationFirst = 0 # - Bỏ bước xoay ban đầu.
			self.simple_angleRight = 0
			self.special_angleRight = 0
			self.simple_isNearGoal = 0
			self.special_isNearGoal = 0
			self.completed_move = 0
			self.step_move = 0

		elif self.modeMove_now == 1: # -- Tạm dừng.
			self.navigationRespond.message = "Pause!"
			self.navigationRespond.status = 0
			self.navigationRespond.modeMove = 1
			self.cmd_vel = Twist()

		elif self.modeMove_now == 2: # -- Parking - Sạc.
			self.idGoal_now = 0
			self.navigationRespond.modeMove = 2
			self.SCD_GoalID = self.navigationQuery.GoalID
			self.SCD_GoalX = self.navigationQuery.GoalX
			self.SCD_GoalY = self.navigationQuery.GoalY
			self.SCD_GoalAngle = self.navigationQuery.GoalAngle
			# - 
			self.goalSimple.id      = self.SCD_GoalID
			self.goalSimple.point.x = self.SCD_GoalX
			self.goalSimple.point.y = self.SCD_GoalY
			self.goalSimple.angleLine = self.SCD_GoalAngle
			sts_run, vel, stopRun, sts_error = self.moveSpecial_charger(self.robotPoseNAV.pose, self.goalSimple, 0.1, 0.001, 0.08, 0)
			# --
			self.navigationRespond.status = 1
			self.navigationRespond.process = sts_run
			self.navigationRespond.error = sts_error
			self.navigationRespond.modeMove = 2
			self.navigationRespond.id_goalFollow = self.SCD_GoalID

			if sts_run < 0:
				self.navigationRespond.stop = 0
				self.navigationRespond.completed = 0
				self.navigationRespond.message = "Move Charger: ERROR ..."

			elif sts_run >= 1 and sts_run <= 4:
				self.navigationRespond.completed = 0
				# --
				if stopRun == 0:
					self.navigationRespond.stop = 0
					self.navigationRespond.message = "Move Charger: Runing..."
				else:
					self.navigationRespond.stop = 1
					self.navigationRespond.message = "Move Charger: Stop - Vat Can"

			elif sts_run == 10:
				self.navigationRespond.stop = 0
				self.navigationRespond.completed = 1
				self.navigationRespond.message = "Move Charger: Completed"

			# --
			self.cmd_vel.linear.x = vel.x
			self.cmd_vel.linear.y = vel.y
			self.cmd_vel.angular.z = vel.r

		elif self.modeMove_now == 3: # -- Đi thẳng ra khỏi điểm - Điểm sạc.
			self.idGoal_now = 0
			self.SCD_GoalID = self.navigationQuery.GoalID
			self.SCD_GoalX = self.navigationQuery.GoalX
			self.SCD_GoalY = self.navigationQuery.GoalY

			self.navigationRespond.modeMove = 3
			self.navigationRespond.id_goalFollow = self.SCD_GoalID
			goalPose = Pose()
			goalPose.position.x = self.SCD_GoalX
			goalPose.position.y = self.SCD_GoalY
			goalPose.orientation = self.euler_to_quaternion(self.SCD_GoalAngle)

			sts, vel, stopRun = self.moveSimple_goStrange(self.robotPoseNAV.pose, goalPose, 0.15, 0.01)
			# --
			if sts == 1:
				self.navigationRespond.status = 1
				self.navigationRespond.modeMove = 3
				self.navigationRespond.completed = 0
				# --
				if stopRun == 0:
					self.navigationRespond.stop = 0
					self.navigationRespond.message = "Move go strange: Runing..."
				else:
					self.navigationRespond.stop = 1
					self.navigationRespond.message = "Move go strange: Stop - Vat Can"

			elif sts == 10:
				self.navigationRespond.status = 1
				self.navigationRespond.modeMove = 3
				self.navigationRespond.completed = 1
				self.navigationRespond.message = "Move go strange: Completed"

			# --
			self.cmd_vel.linear.x = vel.x
			self.cmd_vel.linear.y = vel.y
			self.cmd_vel.angular.z = vel.r

		elif self.modeMove_now == 4: # -- Di chuyển qua các điểm.
			self.SCD_GoalID = self.navigationQuery.GoalID
			self.SCD_GoalX  = self.navigationQuery.GoalX
			self.SCD_GoalY  = self.navigationQuery.GoalY
			self.SCD_GoalAngle = self.navigationQuery.GoalAngle
			# -
			self.SCD_listID = self.navigationQuery.listID
			self.SCD_listX  = self.navigationQuery.listX
			self.SCD_listY  = self.navigationQuery.listY
			self.SCD_listRoadWidth = self.navigationQuery.listRoadWidth
			self.SCD_listAngleLine = self.navigationQuery.listAngleLine
			self.SCD_listDirectionTravel = self.navigationQuery.listDirectionTravel
			self.SCD_listAngleFinal = self.navigationQuery.listAngleFinal
			self.SCD_listSpeed      = self.navigationQuery.listSpeed
			# - 
			if self.allow_path_plan == True and len(self.SCD_listX) != 0:
				# del self.path_plan.poses[:]
				self.path_plan.poses.append(self.point_path(self.robotPoseNAV.pose.position.x, self.robotPoseNAV.pose.position.y))

				for i in range(len(self.SCD_listX)):
					if self.SCD_listID[i] != 0.0:
						point = PoseStamped()
						# point.header.frame_id = 'frame_map_nav350'
						point.header.frame_id = 'world'
						point.header.stamp = rospy.Time.now()
						point.pose.position.x = self.SCD_listX[i]
						point.pose.position.y = self.SCD_listY[i]
						point.pose.position.z = -1.0
						point.pose.orientation.w = 1.0
						# print(point)
						self.path_plan.poses.append(point)
					else:
						break

				self.pub_path_global.publish(self.path_plan)
				# print(self.path_plan)
				self.allow_path_plan = False

			# - 
			self.loopRun()

		elif self.modeMove_now == 5: # -- Di chuyển tịnh tiến đến 1 điểm.
			self.idGoal_now = 0
			self.SCD_GoalID = self.navigationQuery.GoalID
			self.SCD_GoalX  = self.navigationQuery.GoalX
			self.SCD_GoalY  = self.navigationQuery.GoalY
			self.SCD_GoalAngle = self.navigationQuery.GoalAngle
			
			# - add 15/06/2023
			pos_id = -1
			pos_id = self.find_pos(self.navigationQuery.listID, self.navigationQuery.GoalID)
			if pos_id == -1:
				self.goalSimple.angleLine = self.navigationQuery.listAngleLine[0]
			else:
				self.goalSimple.angleLine = self.navigationQuery.listAngleLine[pos_id]

			# - 
			self.goalSimple.id      = self.navigationQuery.GoalID
			self.goalSimple.point.x = self.navigationQuery.GoalX
			self.goalSimple.point.y = self.navigationQuery.GoalY
			
			self.goalSimple.angleFinal = self.navigationQuery.GoalAngle
			self.goalSimple.speed = self.navigationQuery.listSpeed[0]
			self.goalSimple.roadWidth = self.navigationQuery.listRoadWidth[0]
			# -
			sts_run, vel, stopRun, sts_error = self.moveSpecial_target(self.robotPoseNAV.pose, self.goalSimple, 0.1, 0.002, 0.06, 0)
			# --
			self.cmd_vel.linear.x  = vel.x
			self.cmd_vel.linear.y  = vel.y
			self.cmd_vel.angular.z = vel.r
			# --
			self.navigationRespond.status  = 1
			self.navigationRespond.process = sts_run
			self.navigationRespond.error = sts_error
			self.navigationRespond.modeMove = 5
			self.navigationRespond.id_goalFollow = self.goalSimple.id
			# -
			if sts_run < 0:
				self.navigationRespond.stop = 0
				self.navigationRespond.completed = 0
				self.navigationRespond.message = "Move UP/DOWN: ERROR ..."

			elif sts_run >= 1 and sts_run <= 4:
				self.navigationRespond.completed = 0
				# --
				if stopRun == 0:
					self.navigationRespond.stop = 0
					self.navigationRespond.message = "Move UP/DOWN: Runing..."
				else:
					self.navigationRespond.stop = 1
					self.navigationRespond.message = "Move UP/DOWN: Stop - Vat Can"

			elif sts_run == 10:
				self.navigationRespond.stop = 0
				self.navigationRespond.completed = 1
				self.navigationRespond.message = "Move UP/DOWN: Completed"

		else:
			self.idGoal_now = 0
			self.navigationRespond.status  = 1
			self.navigationRespond.process = 0
			self.navigationRespond.modeMove = self.navigationQuery.modeMove
			self.cmd_vel = Twist()
			self.navigationRespond.message = "Move UNK"

		# -
		self.navigationRespond.debug1 = self.string_debug1
		self.navigationRespond.debug2 = self.string_debug2
		# -
		if self.modeMove_now != 0:
			self.publish_cmdVel(self.cmd_vel)
			self.flag_resetVel = 1
			self.count_resetVel = 0
		else:
			if self.flag_resetVel == 1:
				self.publish_cmdVel(Twist())
				self.count_resetVel += 1
				if self.count_resetVel > 10: 
					self.flag_resetVel = 0
		# -
		self.pub_navigationRespond.publish(self.navigationRespond)

	def run(self):
		while not rospy.is_shutdown():
			if self.is_robotPoseNAV == 1:
				self.generalCoordinator()
			self.rate.sleep()
			
		self.pub_cmdVel.publish(Twist())

def main():
	print('Starting main program')
	program = rickshaw_navigation()
	program.run()
	print('Exiting main program')
	program.stop_run()

if __name__ == '__main__':
    main()
