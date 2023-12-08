#!/usr/bin/env python
# Author : AnhTuan 25/8/2020
# update : 23/11/2020 : check or k check line tu pha cuoi
# update : 11/12/2020 : di vao bang line tu 
# update : 22/12/2020 : + TH ngoai line
# update : 31/12/2020 : + distance_target : la k/c robot dung truoc Tag , truoc khi parking
# update : 4/01/2021  : them launch apriltag --> tiet kiem cpu , them not rospy.is_shutdown() de kill clear
# update : 19/01/2021 : + check ke luc di vao ( status = -7 , zone = 2)
# update : 26/01/2021 : update one thread

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import *
import math
import time
import threading
import signal
import os

#global 
idTag_target = 0
tolerance = .0
distance_target = .0
tt_x = .0 # kc Tag <-> tam robot
tt_z = .0
tt_g = .0
is_marker_pose_received = False 
is_marker_pose_received2 = False 
tag_g = .0 # robot huong vao apriltag thi tag_g =0
find_id = 0 # if robot dang nhin thay
id_legal = False

class StiAutomaticParking():
	def __init__(self):
		
		rospy.init_node('parking_v10', anonymous=True)
		self.rate = rospy.Rate(100)

		#get param 
		self.dolech_x_CamVsRObot = rospy.get_param('~dolech_x_CamVsRObot',0)
		self.tolerance_max       = rospy.get_param('~tolerance_max',0.05)
		self.tolerance_min       = rospy.get_param('~tolerance_min',0.02)
		self.distance_max        = rospy.get_param('~distance_max',1)
		self.distance_min        = rospy.get_param('~distance_min',0.02)
		self.x_robot             = rospy.get_param('~x_robot',1)
		self.kc_tag_robot_setup  = rospy.get_param('~kc_tag_robot_setup',1.5)
		self.checkke             = rospy.get_param('~checkke',False)
		self.kc_checkke          = rospy.get_param('~kc_checkke',1)
		
		

		# self.rate = rospy.Rate(20)
		self.tf_listener = tf.TransformListener()
		self.tf_broadcaster = tf.TransformBroadcaster()

		self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 100)
		self.pub_cmd_vel    = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		self.pub_log        = rospy.Publisher('/data_log', Pose, queue_size=100)

		self.pub_status = rospy.Publisher('/parking_status', Parking_status, queue_size=100) 
		rospy.Subscriber("/parking_control", Parking_control, self.callSup)

		rospy.Subscriber("/magneticLine1", Magnetic_line, self.callSup1)
		rospy.Subscriber("/magneticLine2", Magnetic_line, self.callSup2)

		rospy.Subscriber("/zone_robot", Zone_lidar_2head, self.callSup3)


		self.is_odom_received = False  
		rospy.on_shutdown(self.fnShutDown)

		self.rb_x = .0
		self.rb_z = .0 
		self.rb_g = .0

		self.odom_x = .0
		self.odom_y = .0
		self.odom_g = .0

		self.enable_parking = False
		self.solanhoatdong  = 0
		self.v_robot = False

		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 10. 

		# time
		self.t1_ht = rospy.get_time()
		self.t1_tr = rospy.get_time()
		self.t2_ht = rospy.get_time()
		self.t2_tr = rospy.get_time()
		#zone
		self.zone_robot = Zone_lidar_2head()
		self.job = 0
		#lintu 
		self.line1 = 0.
		self.line2 = 0.
		self.line1_stt = False
		self.line2_stt = False
		self.vel_line = 0.15 
		self.sl_loc = 50 # 20 
		self.step_charge = 0 

		self.step_runros = 0
		self.step_runlintu = 0
		self.s_runros = 0.

		self.odom_x_ht = 0.
		self.odom_y_ht = 0.

		# thread 2
		#get param 
		self.kalman_r = rospy.get_param('~kalman_r',0.1)
		self.kalman_p = rospy.get_param('~kalman_p',1.0)
		self.kalman_q = rospy.get_param('~kalman_q',0.1)

		# rospy.init_node('tag_detections_filter')
		# self.rate = rospy.Rate(100)
		self.tf_listener = tf.TransformListener()
		self.tf_broadcaster = tf.TransformBroadcaster()

		self.sub_info_marker = rospy.Subscriber('/tag_detections_d435_parking', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
		self.pub_filter = rospy.Publisher('/tag_detections_filter', Point, queue_size=1)

		# kalman
		self.k1 = self.x_ht1 = self.x_truoc1 = .0
		self.f1 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
		self.k2 = self.x_ht2 = self.x_truoc2 = .0
		self.f2 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
		self.k3 = self.x_ht3 = self.x_truoc3 = .0
		self.f3 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q

		# check ke 
		self.dem_ke = 0 
		self.dem_time_checkke = 0

	def callSup1(self,line1):
		if self.line1_stt == False : self.line1_stt = True
		if line1.status == 1 : self.line1 = line1.value
		else : self.line1 = -2 # error magLine

	def callSup2(self,line2):
		if line2.status == 1 : self.line2 = line2.value
		else : self.line2 = -2 # error magLine

	def callSup3(self,data):
		self.zone_robot = data

	def write_log(self,solan,isTag,x,y,g):
		log = Pose()
		log.position.x = solan
		log.position.y = isTag # 0 : k nhin thay Tag , 1 : co Tag

		log.orientation.x = x
		log.orientation.y = y
		log.orientation.z = g
		self.pub_log.publish(log)

	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
			self.pub_cmd_vel.publish(twist)
		else :
			pass
			# rospy.loginfo("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )

		# self.pub_cmd_vel.publish(twist)

	def park_info(self, status,find_tag, ss_x, ss_y, ss_g):
		park = Parking_status()
		park.status  = status # 0: wait , 1-9 : running, 10 : done
		park.find_tag = find_tag
		park.saiso_x = ss_x
		park.saiso_y = ss_y
		park.saiso_g = ss_g
		self.pub_status.publish(park)

	def constrain(self,val, min_val, max_val):
		if val < min_val: return min_val
		if val > max_val: return max_val
		return val

	def callSup(self,data):
		global idTag_target,distance_target, tolerance
		idTag_target = data.idTag
		tolerance = self.constrain(data.tolerance, self.tolerance_min , self.tolerance_max)
		distance_target = self.constrain(data.distance, self.distance_min , self.distance_max)
		self.job = data.job
		# rospy.loginfo("distance_target : %s",distance_target )
		# rospy.spin()
		# rospy.loginfo("idTag_target : ",idTag_target )
		if idTag_target > 0 :
			self.enable_parking = True   
		else : self.enable_parking = False  

	def cbGetRobotOdom(self, robot_odom_msg):
		if self.is_odom_received == False:
			self.is_odom_received = True 
		quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,\
						robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
		theta = tf.transformations.euler_from_quaternion(quaternion)[2]
		self.odom_x = robot_odom_msg.pose.pose.position.x
		self.odom_y = robot_odom_msg.pose.pose.position.y
		# self.odom_g = (theta*180)/np.pi
		self.odom_g = theta
		
		# van toc robot 
		if((math.fabs(robot_odom_msg.twist.twist.linear.x ) < 0.01) and \
			(math.fabs(robot_odom_msg.twist.twist.angular.z ) < 0.01) ) :
			self.v_robot = True
		else :
			self.v_robot = False
		# http://docs.ros.org/jade/api/tf/html/python/transformations.html#references
		
	def fnGetDataTag(self,sl_loc): 
		global tt_x, tt_z, tt_g 
		x_ht = z_ht = g_ht = .0
		x = z = g = .0
		dem = 0
		for i in range(sl_loc):
			# print tt_x
			while x_ht == tt_x :
				dem = dem
			else:
				# print "dem :" ,dem
				if( sl_loc - dem <= 10):
					x = x + tt_x 
					z = z + tt_z 
					g = g + tt_g 
					# print "tt_x: ", tt_x
					
				x_ht = tt_x
				dem = dem + 1
		# lay trung binh 5 lan cuoi
		x = x / 10.
		z = z / 10.
		g = g / 10.
		return x, z, g

	def fnShutDown(self):
		rospy.loginfo("Shutting down. cmd_vel will be 0")
		self.pub_cmd_vel.publish(Twist()) 

	def fnCalcDistPoints(self, x1, x2, y1, y2):
		# print "fnCalcDistPoints"
		return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

	def run1_lineMag(self, vel ):
		twist = Twist()
		if self.line1_stt == True :
			self.line1_stt = False
			if self.line1 == -1 : 
				# print("ngoai line ")
				return 0
				
			if self.line1 > -1 and self.line1 < 16 : 
				err = self.line1 - 7.5 
				# print("err= {}".format(err))
				kp = 0.05
				twist.linear.x = vel # 0.15 
				twist.angular.z = -1 * err * kp
				self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				# print("twist= {}".format(twist))
				
			if self.line1 == 20 :
				# print("line ngang") 
				self.pub_cmd_vel.publish(Twist())
				# rospy.logwarn("ok")
				return 1
		else : return 0 

	def run1_lineMag_checkke(self, vel ):
		twist = Twist()
		if self.step_runlintu == 0 :
			self.odom_x_ht = self.odom_x
			self.odom_y_ht = self.odom_y
			self.step_runlintu = 1

		if self.step_runlintu == 1 :
			

			if self.line1_stt == True :
				self.line1_stt = False
				if self.line1 == -1 : 
					# print("ngoai line ")
					return 0
					
				if self.line1 > -1 and self.line1 < 16 : 
					s = self.fnCalcDistPoints(self.odom_x,self.odom_x_ht,self.odom_y,self.odom_y_ht)
					print("s: {}".format(s))

					if  self.checkke == True and s < self.kc_checkke :
						rospy.logwarn("check_ke , kc_checkke: {}".format(self.kc_checkke))
						if self.job == 2 and self.zone_robot.zone_ahead == 2 : # check ke khi job = 2 ( tra hang )
							self.pub_cmd_vel.publish(Twist())
							return 2
					
					err = self.line1 - 7.5 
					# print("err= {}".format(err))
					kp = 0.05
					twist.linear.x = vel # 0.15 
					twist.angular.z = -1 * err * kp
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					# print("twist= {}".format(twist))
					

				if self.line1 == 20 :
					# print("line ngang") 
					self.pub_cmd_vel.publish(Twist())
					# rospy.logwarn("ok")
					return 1
			else : return 0 

	def run1_lineMag_checkke_new(self, vel ):
		twist = Twist()
		 
		if self.step_runlintu == 0 :
			self.odom_x_ht = self.odom_x
			self.odom_y_ht = self.odom_y
			self.step_runlintu = 1

		if self.step_runlintu == 1 :
			

			if self.line1_stt == True :
				self.line1_stt = False
				if self.line1 == -1 : 
					# print("ngoai line ")
					return 0
					
				if self.line1 > -1 and self.line1 < 16 : 
					s = self.fnCalcDistPoints(self.odom_x,self.odom_x_ht,self.odom_y,self.odom_y_ht)
					print("s: {}".format(s))

					if  self.checkke == True and s < self.kc_checkke :
						rospy.logwarn("check_ke , kc_checkke: {}".format(self.kc_checkke))
						if self.job == 2 and self.zone_robot.zone_ahead == 2 : # check ke khi job = 2 ( tra hang )
							self.pub_cmd_vel.publish(Twist())
							# return 2
							rospy.sleep(0.1)
							self.dem_time_checkke = self.dem_time_checkke + 1 
					
					err = self.line1 - 7.5 
					# print("err= {}".format(err))
					kp = 0.05
					twist.linear.x = vel # 0.15 
					twist.angular.z = -1 * err * kp
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					# print("twist= {}".format(twist))
					

				if self.line1 == 20 :
					# print("line ngang") 
					self.pub_cmd_vel.publish(Twist())
					# rospy.logwarn("ok")
					return 1
			else : return 0

	def run2_lineMag(self, vel ):
		twist = Twist()
		if self.step_charge == 0 : 
			if self.line1 == -1 : 
				print("ngoai line ")
				return 0 
				
			if self.line1 > -1 and self.line1 < 16 : 
				err = self.line1 - 7.5 
				print("TIEN : err= {}".format(err))
				kp = 0.05
				twist.linear.x = vel
				twist.angular.z = -1 * err * kp
				self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				print("twist= {}".format(twist))

			if self.line1 == 20 :
				print("line ngang") 
				self.pub_cmd_vel.publish(Twist())
				rospy.logwarn("ok")
				rospy.sleep(1)
				self.step_charge = 1 

		if self.step_charge == 1 : 
			if self.line2 == -1 : 
				print("ngoai line ")
				return 0 
				
			if self.line2 > -1 and self.line2 < 16 : 
				err = self.line2 - 7.5 
				print("LUI : err= {}".format(err))
				kp = 0.05
				twist.linear.x = -vel
				twist.angular.z = -1 * err * kp
				self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				print("twist= {}".format(twist))

			if self.line2 == 20 :
				print("line ngang") 
				self.pub_cmd_vel.publish(Twist())
				rospy.logwarn("ok")
				rospy.sleep(1)
				self.step_charge = 2

		if self.step_charge == 2 : 
			if self.line1 == -1 : 
				print("ngoai line ")
				return 0 
				
			if self.line1 > -1 and self.line1 < 16 : 
				err = self.line1 - 7.5 
				print("TIEN : err= {}".format(err))
				kp = 0.05
				twist.linear.x = vel
				twist.angular.z = -1 * err * kp
				self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				print("twist= {}".format(twist))

			if self.line1 == 20 :
				print("line ngang") 
				self.pub_cmd_vel.publish(Twist())
				rospy.logwarn("ok")
				self.step_charge == 0
				return 1

	def run_ros(self, quangduong ):
		twist = Twist()
		if self.step_runros == 0 :
			self.odom_x_ht = self.odom_x
			self.odom_y_ht = self.odom_y
			self.step_runros = 1 

		if self.step_runros == 1 : 
			s = self.fnCalcDistPoints(self.odom_x,self.odom_x_ht,self.odom_y,self.odom_y_ht)
			if quangduong > 0 : # tien 
				rospy.loginfo("tien")
				if s < quangduong : 
					twist.linear.x = 0.15 
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				else :
					self.step_runros = 2 
			if quangduong < 0 : # lui
				rospy.loginfo("lui")
				if s < math.fabs(quangduong) : 
					twist.linear.x = -0.15 
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				else :
					self.step_runros = 2 

			if quangduong == 0 : # stop
				self.step_runros = 2 
		if self.step_runros == 2 :
			rospy.loginfo("stop")
			self.pub_cmd_vel.publish(Twist())
			return 1 

		#### thread 2 ######

	#THREAD 2
	def my_kalman1(self,input):
		self.k1 = self.f1[1]/(self.f1[1] + self.f1[0])
		self.x_ht1 = self.x_truoc1 + self.k1*(input-self.x_truoc1)
		self.f1[1] = (1-self.k1)*self.f1[1]+ math.fabs(self.x_truoc1-self.x_ht1)*self.f1[2]
		self.x_truoc1 = self.x_ht1    
		return self.x_ht1

	def my_kalman2(self,input):
		self.k2 = self.f2[1]/(self.f2[1] + self.f2[0])
		self.x_ht2 = self.x_truoc2 + self.k2*(input-self.x_truoc2)
		self.f2[1] = (1-self.k2)*self.f2[1]+ math.fabs(self.x_truoc2-self.x_ht2)*self.f2[2]
		self.x_truoc2 = self.x_ht2      
		return self.x_ht2

	def my_kalman3(self,input):
		self.k3 = self.f3[1]/(self.f3[1] + self.f3[0])
		self.x_ht3 = self.x_truoc3 + self.k3*(input-self.x_truoc3)
		self.f3[1] = (1-self.k3)*self.f3[1]+ math.fabs(self.x_truoc3-self.x_ht3)*self.f3[2]
		self.x_truoc3 = self.x_ht3    
		return self.x_ht3

	def cbGetMarkerOdom(self, markers_odom_msg): # 5Hz
		global is_marker_pose_received , tag_g , is_marker_pose_received2 , find_id ,id_legal
		global tt_x, tt_z, tt_g 

		if is_marker_pose_received2 == False:
			is_marker_pose_received2 = True

		sl_tag = len(markers_odom_msg.detections)
		if sl_tag == 0 : 
			find_id = 0 
			tag_g = .0 
		else :
			for sl_tag in range(sl_tag):            
				# check id
				id = markers_odom_msg.detections[sl_tag-1].id[0] 

				if is_marker_pose_received == False:
					is_marker_pose_received = True
				if id == idTag_target:
					find_id = id 
					marker_odom = markers_odom_msg.detections[sl_tag-1]

					tag_g = math.atan2(marker_odom.pose.pose.pose.position.x, marker_odom.pose.pose.pose.position.z)

		
		name_tag = "tag_" + str(int(idTag_target))
		if is_marker_pose_received == True :
			if find_id == idTag_target :
				id_legal = True
				self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1)) 
				position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
				quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
				theta = tf.transformations.euler_from_quaternion(quaternion)[1]

				tt_x = self.my_kalman1(position[0])
				# tt_z = position[0] # 
				tt_z = self.my_kalman2(position[2])
				tt_g = self.my_kalman3(theta) # radian
				is_marker_pose_received = False
				# print "tt_x= %s , tt_z= %s, tt_g= %s" %(tt_x, tt_z, tt_g)
				# print "tt_x=",position[0]
			else :
				id_legal = False

	def raa(self):
		global tag_g ,tt_x, tt_z, tt_g , find_id , id_legal , tolerance 
		global goc_taget, distance_target ,x_target , idTag_target 
		global is_marker_pose_received2
		# loop_rate = rospy.Rate(10) # 10hz
		buoc = -1
		odom_x_ht = 0.
		odom_y_ht = 0.
		check_linetu_false = False

		while not rospy.is_shutdown() :
			# reset
			if (self.enable_parking == False)  and (buoc != -1) :  
				# twist = Twist()2#140501105755904
				# self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
				for i in range(3):
					self.pub_cmd_vel.publish(Twist())
				rospy.loginfo("Reset 11")
				self.park_info(buoc,find_id,0,0,0)
				self.rate.sleep()
				buoc = -1

			# if (self.enable_parking == False)  and (buoc == -1) :
			#     self.park_info(buoc,find_id,0,0,0)
			#     rospy.loginfo("Reset 22")
			#     buoc = -1

			# pub status
			self.park_info(buoc,find_id,0,0,0) 
			#Test          
			if buoc == 0 :
				# self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(self.sl_loc)
				# print "odom_g = ", self.odom_g
				# self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag_notFilter(self.idTag_target) # rad
				# x_target_phu = self.rb_x - self.dolech_x_CamVsRObot
				# print "x_target_phu: ", x_target_phu
				# print "tt_x= %s , tt_z= %s, tt_g= %s" %(self.rb_x, self.rb_z, self.rb_g)
				# s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
				# print "s: ", s
				# print "tag_g: ", tag_g
				rospy.spin()

				# self.run1_lineMag()
				# self.run2_lineMag()

			#B0 Wait thong tin tu server
			if buoc == -1 :
				rospy.loginfo("buoc: %s",buoc)
				self.park_info(buoc,find_id,0,0,0) 
				if self.enable_parking == True:
					# self.enable_parking = False
					buoc = -3
		
			#Buoc -3 : Wait launch apriltag  
			if buoc == -3 : 
				self.park_info(buoc,find_id,0,0,0) 
				if is_marker_pose_received2 == True :
					if id_legal == True and find_id != 0:
						buoc = 1 
						rospy.loginfo("buoc: %s",buoc)
						rospy.loginfo("idTag_target: %s",idTag_target)

					elif find_id == 0:
						buoc = -4 
						# time
						self.t1_ht = rospy.get_time()
						self.t1_tr = rospy.get_time()
				else: 
					self.rate.sleep()
					buoc = -3 

			# Buoc -4 : quay trai 5s tim TAG
			if buoc == -4 :
				self.park_info(buoc,find_id,0,0,0)
				twist = Twist()
				self.t1_ht = rospy.get_time()

				if ( id_legal == False ) and (self.t1_ht - self.t1_tr < 5 ):
					twist.angular.z = 0.3
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())

				elif ( id_legal == False or find_id == 0 ) and (self.t1_ht - self.t1_tr > 5 ):
					self.pub_cmd_vel.publish(Twist())
					buoc = -5
					# time
					self.t1_ht = rospy.get_time()
					self.t1_tr = rospy.get_time()
					rospy.loginfo("buoc: %s",buoc)

				elif id_legal == True and find_id != 0 : 
					self.pub_cmd_vel.publish(Twist())
					buoc = 1

			# Buoc -5 : quay phai tim TAG
			if buoc == -5 :
				rospy.loginfo("quay phai tim TAG")
				self.park_info(buoc,find_id,0,0,0)
				twist = Twist()
				self.t1_ht = rospy.get_time()
				if (self.t1_ht - self.t1_tr < 30 ):
					if id_legal == False or find_id == 0 :
					
						twist.angular.z = -0.3
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
						
					elif id_legal == True and find_id != 0 : 
						self.pub_cmd_vel.publish(Twist())
						buoc  = 1
				else: 
					buoc = -6

			# Buoc -6 : quay tim TAG > 30s
			if buoc == -6 :
				self.pub_cmd_vel.publish(Twist())
				self.park_info(buoc,find_id,0,0,0) 
				rospy.loginfo("quay tim TAG > 30s")

			#B11: Huong camera ve Apriltag , NOTE : check yes/no Tag           
			if buoc == 1 :
				self.park_info(buoc,find_id,0,0,0) 
				twist = Twist()

				if find_id == 0 : 
					twist.angular.z = 0
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					buoc = -4
					# time
					self.t1_ht = rospy.get_time()
					self.t1_tr = rospy.get_time()
				else :
					if math.fabs(tag_g) > 0.1: #2do

						if tag_g > 0 : twist.angular.z = -0.15
						else :         twist.angular.z = 0.15
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:
						self.pub_cmd_vel.publish(Twist())
						if self.v_robot == True:
							buoc = 2 
							rospy.loginfo("buoc: %s",buoc)
			
			#B2: Tinh goc can quay, tt_x , tt_z
			if buoc == 2 : 
				self.park_info(buoc,find_id,0,0,0) 
				self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(self.sl_loc) # rad
				rospy.loginfo("idTag_target: %s",idTag_target)
				rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s" ,self.rb_x, self.rb_z, self.rb_g)
				rospy.loginfo("odom_g= %s ", self.odom_g)
				
				# tinh x can di
				x_target = self.rb_x - self.dolech_x_CamVsRObot 
				#--> self.rb_x la toa do robot voi tag
				#--> x_target: quang dg robot can di de cam thang tag

				if math.fabs(x_target) <= tolerance :
					odom_x_ht = self.odom_x
					odom_y_ht = self.odom_y
					buoc = 30 
					rospy.loginfo("buoc: %s",buoc)
					
				else :
					# tinh goc quay
					quayTrai = quayPhai = False
					goc_canquay = math.fabs(self.rb_g)
					if x_target > 0 : # quay trai ( + tang , - giam)
						rospy.loginfo("quay trai")
						quayTrai = True
						if self.odom_g > 0 : 
							goc_taget= self.odom_g + goc_canquay
							if goc_taget > np.pi : 
								goc_bu = goc_taget - np.pi 
								goc_taget = goc_bu - np.pi 
						else : 
							goc_taget= self.odom_g + goc_canquay

					else : # quay phai ( + giam , - tang)
						rospy.loginfo("quay phai")
						quayPhai = True
						if self.odom_g > 0 : 
							goc_taget= self.odom_g - goc_canquay
						else : 
							goc_taget= self.odom_g - goc_canquay
							if goc_taget < -np.pi :
								goc_taget = goc_taget + 2*np.pi

					rospy.loginfo("goc_taget= %s " , goc_taget)
					rospy.loginfo("x_target: %s", x_target)

					buoc = 30
					rospy.loginfo("buoc: %s",buoc)

			#B30: check line tu     
			# if buoc == 30 : 
			#     rb_z_temp = 0 
			# 	if distance_target < self.kc_tag_robot_setup :
			# 		distance_target = self.kc_tag_robot_setup

			# 	rb_z_temp = self.rb_z - self.x_robot/2
			# 	# self.rb_z += - self.x_robot/2
			# 	if self.rb_z > (distance_target + 0.05) : # robot dung xa dau linetu 
			# 		self.s_runros = self.rb_z - distance_target
			# 		self.step_runros = 0
			# 		buoc = 31

			# 	elif self.rb_z < (distance_target - 0.05) : # robot dung qua gan
			# 		self.s_runros = self.rb_z - distance_target 
			# 		self.step_runros = 0
			# 		buoc = 31
				
			# 	else : buoc = 32        

			if buoc == 30 : 
				rb_z_temp = 0. 
				if distance_target < self.kc_tag_robot_setup :
					distance_target = self.kc_tag_robot_setup
					
				rb_z_temp = self.rb_z - self.x_robot/2

				if rb_z_temp > (distance_target + 0.05) : # robot dung xa dau linetu 
					self.s_runros = rb_z_temp - distance_target
					self.step_runros = 0
					buoc = 31

				elif rb_z_temp < (distance_target - 0.05) : # robot dung qua gan
					self.s_runros = rb_z_temp - distance_target 
					self.step_runros = 0
					buoc = 31

				else : buoc = 32 
			
			#B31: tien khong vach du k/c    
			if buoc == 31 :
				self.park_info(buoc,find_id,0,0,0)  
				if self.run_ros(self.s_runros ) == 1 :
					buoc = 32

			#B32: check line tu lan 2    
			if buoc == 32 : 
				self.park_info(buoc,find_id,0,0,0) 
				if self.line1 > -1 and self.line1 < 16 and math.fabs(x_target) < 0.5:
					buoc = 9
				else : 
					buoc = 33

			#B33: Quay vuong goc           
			if buoc == 33 :
				self.park_info(buoc,find_id,0,0,0) 
				# rospy.loginfo("odom_g = %s", self.odom_g)
				# rospy.loginfo("goc_taget= %s" , goc_taget)
				# rospy.loginfo("x_target: %s", x_target)
				twist = Twist()
				
				#  A3 :odom_g= -0.105119969655 , goc_taget= 3.137853620 --> quay tron mai 

				if(goc_taget > 0):
					rospy.loginfo("A3 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
					if math.fabs(self.odom_g - goc_taget) > 0.04 :
						if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
						else :             twist.angular.z = math.fabs(self.odom_g - goc_taget)
						if twist.angular.z > 0.3 : twist.angular.z = 0.3
						if twist.angular.z < -0.3 : twist.angular.z = -0.3
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:

						self.pub_cmd_vel.publish(Twist())
						odom_x_ht = self.odom_x
						odom_y_ht = self.odom_y
						if self.v_robot == True:
							buoc = 4
							rospy.loginfo("buoc: %s",buoc)
				else:
					rospy.loginfo("B3 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)

					if math.fabs(self.odom_g - goc_taget) > 0.04 :
						if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
						else :            twist.angular.z = math.fabs(self.odom_g - goc_taget)
						if twist.angular.z > 0.3 : twist.angular.z = 0.3
						if twist.angular.z < -0.3 : twist.angular.z = -0.3
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:
						self.pub_cmd_vel.publish(Twist())
						odom_x_ht = self.odom_x
						odom_y_ht = self.odom_y
						if self.v_robot == True:
							buoc = 4
							rospy.loginfo("buoc: %s",buoc)

			#B4: Di chuyen chinh giua Apriltag         
			if buoc == 4 :
				self.park_info(buoc,find_id,0,0,0) 
				twist = Twist()
				s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
				rospy.loginfo("x_target: %s ", x_target)
				rospy.loginfo("s: %s", s )
				if math.fabs(s) < (math.fabs(x_target)) : 
					twist.linear.x = + ( math.fabs(x_target) - math.fabs(s) ) + 0.02
					if twist.linear.x > 0.2 : twist.linear.x = 0.2
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					
				else : 
					self.pub_cmd_vel.publish(Twist())
					buoc = 5
					rospy.loginfo("buoc: %s",buoc)

			#B5: Tinh goc quay lai
			if buoc == 5 :
				self.park_info(buoc,find_id,0,0,0) 
				rospy.loginfo("odom_g   %s= ", self.odom_g)
				rospy.loginfo("x_target: %s", x_target)
				if quayTrai == True:
					rospy.loginfo("quay phai")
					if self.odom_g > 0 : 
						goc_taget = self.odom_g - np.pi/2
					else : 
						goc_taget = self.odom_g - np.pi/2
						if goc_taget < -np.pi :
							goc_taget = goc_taget + 2*np.pi


				if quayPhai == True:    
					rospy.loginfo("quay trai")   
					if self.odom_g > 0 : 
						goc_taget= self.odom_g + np.pi/2
						if goc_taget > np.pi : 
							goc_bu = goc_taget - np.pi 
							goc_taget = goc_bu - np.pi 
					else : 
						goc_taget= self.odom_g + np.pi/2

				rospy.loginfo("goc_taget_quaylai= %s" , goc_taget  )
				buoc = 6
				rospy.loginfo("buoc : %s", buoc)

			#B6: quay nguoc lai     
			if buoc == 6 :
				self.park_info(buoc,find_id,0,0,0) 
				# rospy.loginfo("odom_g   %s= ", self.odom_g)
				# rospy.loginfo("goc_taget= %s" , goc_taget)
				# rospy.loginfo("x_target: %s", x_target)
				# rospy.loginfo("s: %s", s)
				twist = Twist()
				if(goc_taget > 0):
					rospy.loginfo("A6 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
					if math.fabs(self.odom_g - goc_taget) > 0.04 :
						if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
						if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
						if twist.angular.z > 0.4 : twist.angular.z = 0.4
						if twist.angular.z < -0.4 : twist.angular.z = -0.4
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:
						self.pub_cmd_vel.publish(Twist())
						odom_x_ht = self.odom_x
						odom_y_ht = self.odom_y
						buoc = 7 

				else:
					rospy.loginfo("B6 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
					if math.fabs(self.odom_g - goc_taget) > 0.04 :
						if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
						if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
						if twist.angular.z > 0.4 : twist.angular.z = 0.4
						if twist.angular.z < -0.4 : twist.angular.z = -0.4
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:
						self.pub_cmd_vel.publish(Twist())
						odom_x_ht = self.odom_x
						odom_y_ht = self.odom_y
						buoc = 7 
						rospy.loginfo("buoc : %s", buoc)

			#B7: tinh chinh goc  , NOTE : check yes/no Tag      
			if buoc == 7 : 
				self.park_info(buoc,find_id,0,0,0) 
				rospy.loginfo("tag_g: %s", tag_g)
				rospy.loginfo("x_target: %s", x_target)
				twist = Twist()

				if find_id == 0 : # --> TH : quay sai , k nhin thay Tah --> dung yen --> CAN KHAC PHUC CHO NAY 
					twist.angular.z = 0
					self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					buoc = -4
					# time
					self.t1_ht = rospy.get_time()
					self.t1_tr = rospy.get_time()
				else :
					if math.fabs(tag_g) > 0.05 : #2do
						if tag_g > 0 : twist.angular.z = -0.06
						else :         twist.angular.z = 0.06
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())

					else:
						self.pub_cmd_vel.publish(Twist())
						if self.v_robot == True:
							buoc = 8
							rospy.loginfo("buoc : %s", buoc)

			#B8: check line tu     
			if buoc == 8 : 
				self.park_info(buoc,find_id,0,0,0) 
				if self.line1 > -1 and self.line1 < 16 and math.fabs(x_target) < 0.5:
					buoc = 9
					self.step_runlintu = 0
				else : 
					buoc = 2 # da thu 77 , -5 deu k duoc 
				# note : TH : ngoai line , x_target < 0.5 --> vong lap 2,7,8,77
		
			#B9: tinh tien di vao  , NOTE : check yes/no Tag 
			      
			if buoc == 9 :
				self.park_info(buoc,find_id,0,0,0) 
				# rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s",self.rb_x, self.rb_z, self.rb_g)
				rospy.loginfo("tinh tien di vao")              
				if self.run1_lineMag_checkke(self.vel_line) == 1 : buoc = 10
				if self.run1_lineMag_checkke(self.vel_line) == 2 : 
					self.dem_ke = self.dem_ke + 1
					if self.dem_ke > 1 :
						buoc = -7
					else : 
						buoc = 30
				

			#B10: publish Log       
			if buoc == 10 :
				self.solanhoatdong = self.solanhoatdong + 1
				self.write_log((self.solanhoatdong / 3) ,idTag_target, self.line1,self.line2,99)

				buoc = 11   
			#B-7 : Bao dang co ke 
			if buoc == -7 : 
				self.dem_ke = 0
				self.park_info(buoc,find_id,0,0,0) 
				rospy.loginfo("CO KE --> khong tra duoc hang")
				if idTag_target == 0 :  
					rospy.loginfo("Reset")
					buoc = -1

			#B11: doi reset
			if buoc == 11 :
				# self.park_info(buoc,find_id,x_robot, y_robot, goc)
				rospy.loginfo("DONE!!!")
				if idTag_target == 0 :  
					rospy.loginfo("Reset")
					buoc = -1

			self.rate.sleep()


def main():
    
    try:
        m = StiAutomaticParking()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
