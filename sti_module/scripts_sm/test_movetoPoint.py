#!/usr/bin/env python3  
# -*- coding: utf-8 -*-
# Dev: Phùng Quý Dương STI 
# Latest Modify: 08/12/2023

"""
Nội dung code:
1. Sử dụng hàm control_navigation và convert tọa độ để điều khiển AGVSM di chuyển tới ĐIỂM

Kết quả: OK
"""

import time
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, atan
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import *
from agv_simulation.msg import * 
from nav_msgs.msg import Path

class robot_AGV():
	def __init__(self):
		# -- Khởi tạo ROS --
		rospy.init_node('test_movetoPoint', anonymous=False)
		self.rate_hz = 50
		self.rate = rospy.Rate(self.rate_hz)
		
		# -- mô tả vị trí ban đầu của AGV trên rviz -- 
		self.x_start = rospy.get_param('~x_start')
		self.y_start = rospy.get_param('~y_start')
		self.theta_start = rospy.get_param('~theta_start')

		# -- mô tả vị trí trước đó của agv -- 
		self.x_prepos = self.x_start
		self.y_prepos = self.y_start
		self.theta_prepos = self.theta_start
		
		# -- vị trí điểm đến tiếp theo 
		self.x_target = 5
		self.y_target = 5
		self.theta_target = 0
		self.index_target = 0
		
		# -- khoảng nhìn trước max --
		self.dist_ahead_max = rospy.get_param('~khoang_nhin_truoc_max')
		self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1')
		self.vel_rot_step1 = rospy.get_param('~vel_rot_step1')

		# -- Nhận vị trí của robot -- 
		rospy.Subscriber('/agv_pose', Pose, self.getPose, queue_size = 20)
		self.is_pose_robot = False
		self.poseRbMa = Pose()
		self.theta_rb_ht = 0.0

		# -- Nhận lộ trình từ traffic -- 
		rospy.Subscriber('/request_move', Move_request, self.move_callback ,queue_size = 20)                # lộ trình nhận được từ sti_control 
		self.req_move = Move_request()                                                 
		self.is_request_move = False

		# -- Nhận phản hồi điều khiển từ hệ thống(mode = 1: Manual mode, mode = 2: Auto mode) --
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()
		self.is_recv_NNinfoRespond = False

		# -- Gửi vận tốc robot -- 
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 20)
		self.pub_path_global = rospy.Publisher('/path_plan_global', Path, queue_size = 20)

		# -- Gửi trạng thái của robot 
		self.pub_status = rospy.Publisher('/robot_status', robot_status, queue_size = 20)
		self.robot_status = robot_status()
		self.define_var()

	def define_var(self):
		self.vel_x = 0.6
		self.rate_cmdvel = 30 
		self.time_tr = rospy.get_time()  		

	def getPose(self, data):
		self.is_pose_robot = True
		self.poseRbMa = data
		quata = (self.poseRbMa.orientation.x, self.poseRbMa.orientation.y, self.poseRbMa.orientation.z, self.poseRbMa.orientation.w)
		euler = euler_from_quaternion(quata)
		self.theta_rb_ht = euler[2]

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data
		self.is_recv_NNinfoRespond = True

	def move_callback(self, data):
		pass
		# self.req_move = data
		# self.is_request_move = True
		
		# if self.is_first_callback == True:
		# 	self.len_tranjectory = len(self.req_move.list_x)

		# 	# add first point
		# 	self.path_plan.header.frame_id = self.origin_frame
		# 	self.path_plan.header.stamp = rospy.Time.now()
		# 	self.path_plan.poses.append(self.point_path(self.agv_start_pose_x, self.agv_start_pose_y))
		# 	self.path_plan.poses.append(self.point_path(self.req_move.list_x[0], self.req_move.list_y[0]))

		# 	for i in range(self.nb_point_path + 1):
		# 		x, y = self.getPoint_curveFormula(self.path_t, self.req_move.list_x[0], self.req_move.list_y[0], \
		# 																					self.req_move.list_x[1], self.req_move.list_y[1],\
		# 																						self.req_move.list_x[2], self.req_move.list_y[2])
		# 		# print(self.path_t, x, y)
		# 		# self.point_path_x.append(x)
		# 		# self.point_path_y.append(y)               

		# 		self.path_plan.poses.append(self.point_path(x, y))
		# 		self.path_t = self.path_t + self.path_delta

		# 	# print(self.point_path_x)

		# 	# print(len(self.point_path_x))
		# 	self.pub_path_global.publish(self.path_plan)
		# 	self.is_first_callback = False 			

	# chuyển đổi tọa độ của 1 điểm bất kì theo tọa độ của AGV 							
	def convert_relative_coordinates(self, X_cv, Y_cv):
		angle = -self.theta_rb_ht
		_X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
		_Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
		return _X_cv, _Y_cv

	"""
	# input: Tọa độ điểm bất kì theo tọa độ của AGV + vận tốc dài mong muốn của AGV 
	# output: Vận tốc góc mong muốn của AGV 
	"""
	def control_navigation(self, X_point_goal, Y_point_goal, vel_x):
		vel_th = 0.0
		l = (X_point_goal*X_point_goal) + (Y_point_goal*Y_point_goal)
		if Y_point_goal == 0:
			print(Y_point_goal)
			Y_point_goal = 0.0001

		r = l/(2*fabs(Y_point_goal))
		vel = vel_x/r        # vel : omega 

		if Y_point_goal > 0:
			vel_th = vel
		else:
			vel_th = -vel

		return vel_th

	def pub_cmdVel(self, twist , rate):
		if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = rospy.get_time()
			self.pub_cmd_vel.publish(twist)
		
	def run(self):
		while not rospy.is_shutdown():
			twist = Twist()
			x_cv, y_cv = self.convert_relative_coordinates(self.x_target, self.y_target)
			omega = self.control_navigation(x_cv, y_cv, self.vel_x)
			twist.linear.x = self.vel_x
			twist.angular.z = omega
			self.pub_cmdVel(twist, self.rate_cmdvel)

			self.rate.sleep()
    
def main():
	print('Program starting')
	agv1 = robot_AGV()
	agv1.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()