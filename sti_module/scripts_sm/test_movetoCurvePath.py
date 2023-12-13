#!/usr/bin/env python3  
# -*- coding: utf-8 -*-
# Dev: Phùng Quý Dương STI 
# Latest Modify: 08/12/2023

"""
Nội dung code:
Input: Lộ trình đường cong + Vị trí của AGV 
Output: AGV di chuyển trên đường cong đó

Xác nhận kết quả: 

"""

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import sqrt, pow, atan, acos, modf, atan2, fabs, tan, degrees, radians
from math import pi as PI
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import *
from agv_simulation.msg import * 
from nav_msgs.msg import Path
import numpy as np

class Bezier_Curve():
	def __init__(self, _x1, _y1, _x2, _y2, _x3, _y3):
		# ft Berzier Curve 
		# Px = pow((1-t), 2) * _x1 + 2 * (1-t) * t * _x2 + t ** 2 * _x3
		# Py = pow((1-t), 2) * _y1 + 2 * (1-t) * t * _y2 + t ** 2 * _y3
		self.firstPoint_x = _x1
		self.firstPoint_y = _y1
		self.midPoint_x = _x2
		self.midPoint_y = _y2
		self.secondPoint_x = _x3
		self.secondPoint_y = _y3

	# Cập nhật lại thông số.
	def update_spec(self, _x1, _y1, _x2, _y2, _x3, _y3):
		self.firstPoint_x = _x1
		self.firstPoint_y = _y1
		self.midPoint_x = _x2
		self.midPoint_y = _y2
		self.secondPoint_x = _x3
		self.secondPoint_y = _y3

	# Nhận tọa độ của các điểm trên đường cong: 0 -> 1
	def getPoint_curveFormula(self, t):
		Px = (1-t)**2 * self.firstPoint_x + 2 * (1-t) * t * self.midPoint_x + t ** 2 * self.secondPoint_x
		Py = (1-t)**2 * self.firstPoint_y + 2 * (1-t) * t * self.midPoint_y + t ** 2 * self.secondPoint_y

		return (Px, Py)	
	
	# Nhận góc của đường thẳng tiếp tuyến với đường cong tại giá trị t: 0 -> 1
	def getAngle_TangentLine(self, t):
		Pxdot = 2 * (1-t) * self.firstPoint_x + (2 - 4 * t) * self.midPoint_x + 2 * t * self.secondPoint_x
		Pydot = 2 * (1-t) * self.firstPoint_y + (2 - 4 * t) * self.midPoint_y + 2 * t * self.secondPoint_y

		return atan2(Pydot, Pxdot)

	# Nhận chiều dài của đường cong
	def getLength_curveFormula(self, t):
		Pxdot = 2 * (1-t) * self.firstPoint_x + (2 - 4 * t) * self.midPoint_x + 2 * t * self.secondPoint_x
		Pydot = 2 * (1-t) * self.firstPoint_y + (2 - 4 * t) * self.midPoint_y + 2 * t * self.secondPoint_y

		return sqrt(Pxdot**2 + Pydot**2)

	# Nhận bán kính của đường cong tại thời điểm t
	def getRadius_curveFormula(self, t):	
		# ft Berzier Curve Derive level 1
		Pxdot = 2 * (1-t) * self.firstPoint_x + (2 - 4 * t) * self.midPoint_x + 2 * t * self.secondPoint_x
		Pydot = 2 * (1-t) * self.firstPoint_y + (2 - 4 * t) * self.midPoint_y + 2 * t * self.secondPoint_y

		# ft Berzier Curve Derive level 2
		Px2dot = -2 * self.firstPoint_x - 4 * self.midPoint_x + 2 * self.secondPoint_x
		Py2dot = -2 * self.firstPoint_y - 4 * self.midPoint_y + 2 * self.secondPoint_y

		R = pow((Pxdot ** 2 + Pydot ** 2), 3/2) / (Pxdot * Py2dot - Pydot * Px2dot)
		# print(R)
		return R

class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y
    
class QuadraticBezierCurves():
    def __init__(self, _pointOne=Point(), _pointSecond=Point(), _midpoint=Point(), _typeDefine=0,_angleOne=0., _angleSecond=0., _numberPts=0 ):

        self.pointOne = _pointOne
        self.pointSecond = _pointSecond
        self.midPoint = _midpoint
        if _typeDefine == 1:
            a1, b1, c1 = self.findStraightLineByAngleAndPoint(self.pointOne, _angleOne)
            a2, b2, c2 = self.findStraightLineByAngleAndPoint(self.pointSecond, _angleSecond)
            if b1 == 0:
                x = -c1/a1
                y = (-c2-a2*x)/b2

            else:
                x = ((b2*c1)/b1 - c2)/(a2 - (b2*a1)/b1)
                y = (-c1-a1*x)/b1

            # x = (-c1+b2)/(a1-a2)
            # y = a1*x + c1
            self.midPoint = Point(x,y)
            print(self.midPoint.x ,self.midPoint.y)
        # self.numberPts = _numberPts

        self.numberPts = self.findNumberPts()

        self.t = np.array([i*1/self.numberPts for i in range(0,self.numberPts+1)])

        print("Make a Quadratic Bezier Curves ")

    def findStraightLineByAngleAndPoint(self, _point, _angle):
        if fabs(_angle) == PI/2.:
            return 1, 0, -_point.x
        else:
            k = tan(_angle)
            return k, -1., (-1)*k*_point.x + _point.y


    def findNumberPts(self):
        dis1 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.midPoint.x, self.midPoint.y)
        dis2 = self.calculate_distance(self.midPoint.x, self.midPoint.y, self.pointSecond.x, self.pointSecond.y)
        dis3 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.pointSecond.x, self.pointSecond.y)

        return int(max(dis1, dis2, dis3)/0.01)


    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def slip(self):
        listPoint = np.empty((0,2), dtype=float)
        for i in self.t:
            _x = (1-i)*((1-i)*self.pointOne.x + i*self.midPoint.x) + i*((1-i)*self.midPoint.x + i*self.pointSecond.x)
            _y = (1-i)*((1-i)*self.pointOne.y + i*self.midPoint.y) + i*((1-i)*self.midPoint.y + i*self.pointSecond.y)

            listPoint = np.append(listPoint, np.array([[_x, _y]]), axis=0)

        return listPoint
    
class robot_AGV():
	def __init__(self):
		# -- Khởi tạo ROS --
		rospy.init_node('MovetoCurvePath', anonymous=False)
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
		self.x_target = 0
		self.y_target = 0
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

		# -- Gửi thông số của path to rviz --
		self.pub_path_global = rospy.Publisher('/path_plan_global', Path, queue_size = 20)
		self.path_plan = Path()
	
		# -- Gửi trạng thái của robot 
		self.pub_status = rospy.Publisher('/robot_status', robot_status, queue_size = 20)
		self.robot_status = robot_status()
		self.define_AGVvar()
		self.define_Pathvar()
		self.define_Systemvar()

	def define_AGVvar(self):
		self.vel_x = 1.2
		self.rate_cmdvel = 30 
		self.time_tr = rospy.get_time()  

		self.secondPoint_x = 0.0
		self.secondPoint_y = 0.0

		self.firstPoint_x = 0.0
		self.firstPoint_y = 0.0

		self.X_n = 0.0      # hc x cua agv tren lo trinh
		self.Y_n = 0.0		# hc y cua agv tren lo trinh
		self.a_qd = 0.0     
		self.b_qd = 0.0
		self.c_qd = 0.0

		self.theta = 0.0    # góc lệch giữa AGV và lộ trình

		self.min_vel_x_gh = 0.3
		self.angle_find_vel = 30.0*PI/180.0

		self.distance_goal = 0.0

		self.is_need_turn_step1 = 0
		self.x_td_goal = 0.0 
		self.y_td_goal = 0.0
		self.dis_hc = 0.0

		self.kc_con_lai = 0.0

		self.v_th_send = 0.0
		
		self.angle_giam_toc = 45.0*PI/180.0

		self.dis_gt = 0.3
		self.kc_qd = 0.0

		self.R_move = 0.5
	
	def define_Pathvar(self):
		self.path_index = 0
		self.agv_frame =  "agv1"
		self.origin_frame = "world"

		self.tol_simple = rospy.get_param('~tol_simple')

		self.curve_path = Bezier_Curve(0, 0, 0, 0, 0, 0)
		self.nb_curvePart = 5

		# for screen on rviz
		self.nb_pointPath = 25
		self.path_delta = 0.04
		self.path_t = 0

	def define_Systemvar(self):
		self.process = 1
		self.is_first_callback = True

	def getPose(self, data):
		self.is_pose_robot = True
		self.poseRbMa = data
		quata = (self.poseRbMa.orientation.x, self.poseRbMa.orientation.y, self.poseRbMa.orientation.z, self.poseRbMa.orientation.w)
		euler = euler_from_quaternion(quata)
		self.theta_rb_ht = euler[2]

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data
		self.is_recv_NNinfoRespond = True

	def point_path(self, x, y):
		point = PoseStamped()
		point.pose.position.x = x
		point.pose.position.y = y
		point.pose.position.z = 0
		point.pose.orientation.x = 0 ###
		point.pose.orientation.y = 0
		point.pose.orientation.z = 0.
		point.pose.orientation.w = 1.0
		return point

	def move_callback(self, data):
		self.req_move = data
		self.is_request_move = True

		if self.is_first_callback == True:
			self.curve_path.update_spec(self.req_move.list_x[0], self.req_move.list_y[0], \
			       							self.req_move.list_x[1], self.req_move.list_y[1], \
												self.req_move.list_x[2], self.req_move.list_y[2])
			
			# self.len_tranjectory = len(self.req_move.list_x)
			self.path_plan.header.frame_id = self.origin_frame
			self.path_plan.header.stamp = rospy.Time.now()
			self.path_plan.poses.append(self.point_path(self.curve_path.firstPoint_x, self.curve_path.firstPoint_y))
			self.path_t = self.path_t + self.path_delta	

			while self.path_t <= 1:
				x, y = self.curve_path.getPoint_curveFormula(self.path_t)               
				self.path_plan.poses.append(self.point_path(x, y))
				self.path_t = self.path_t + self.path_delta	
		
			self.pub_path_global.publish(self.path_plan)
			self.is_first_callback = False

	# tìm điểm giao giữa đường cong lộ trình và vùng di chuyển của AGV.
	# def PointBetweenAGVandPath(self, agv_x, agv_y):
	# 	px, py = self.curve_path.getPoint_curveFormula()
	# 	return x_intersection, y_intersection
		
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

	# -- hàm pub vận tốc robot 
	def pub_cmdVel(self, twist , rate):
		if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = rospy.get_time()
			self.pub_cmd_vel.publish(twist)

	# -- góc giữa đường lộ trình và góc robot
	def find_angle_between(self, a, b, angle_rb):
		angle_bt = 0.0
		angle_fn = 0.0
		if b == 0:
			if a < 0:
				angle_bt = PI/2.0
			elif a > 0:
				angle_bt = -PI/2.0
		elif a == 0:
			if -b < 0:
				angle_bt = 0.0
			elif -b > 0:
				angle_bt = PI
		else:

			angle_bt = acos(b/sqrt(b*b + a*a))
			if -a/b > 0:
				if fabs(angle_bt) > PI/2:
					angle_bt = -angle_bt
				else:
					angle_bt = angle_bt
			else:
				if fabs(angle_bt) > PI/2:
					angle_bt = angle_bt
				else:
					angle_bt = -angle_bt

		angle_fn = angle_bt - angle_rb
		# print(angle_bt, angle_fn)
		if fabs(angle_fn) >= PI:
			angle_fnt = (2*PI - fabs(angle_fn))
			if angle_fn > 0:
				angle_fn = -angle_fnt
			else:
				angle_fn = angle_fnt

		# print(angle_fn)
		return angle_fn  

	# -- hàm quay robot 1 góc 
	def turn_ar(self, theta, tol_theta, vel_rot):
		if fabs(theta) > tol_theta: # +- 10 do
			if theta > 0: #quay trai
				# print "b"
				if fabs(theta) <= self.angle_giam_toc:
					# print('hhhhhhhhhhh')
					vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
				else:
					vel_th = vel_rot

				if vel_th < 0.1:
					vel_th = 0.1

				# vel_th = fabs(theta) + 0.1
				# if vel_th > vel_rot : vel_th = vel_rot
				return vel_th

			if theta < 0: #quay phai , vel_z < 0
				# print "a"
				if fabs(theta) <= self.angle_giam_toc:
					# print('hhhhhhhhhhhh')
					vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
				else:
					vel_th = -vel_rot

				if vel_th > -0.1:
					vel_th = -0.1

				# vel_th = -fabs(theta) - 0.1
				# if vel_th < -vel_rot : vel_th = -vel_rot
				return vel_th
				# buoc = 1

		else : 
			return -10 

	# -- robot dừng 
	def stop(self):
		for i in range(2):
			self.pub_cmd_vel.publish(Twist())

    # -- Hàm tìm tọa độ hình chiếu của AGV trên lộ trình     
	def find_hc(self, Xt, Yt, Xs, Ys):
		X_n = Y_n = 0.0
		kc_hinh_chieu = 0.0
		# pt duong thang quy dao
		a_qd = Ys - Yt
		b_qd = Xt - Xs 
		c_qd = Xs*Yt - Xt*Ys
		
		# pt duong thang hinh chieu
		a_hc = b_qd
		b_hc = -a_qd
		c_hc = -self.poseRbMa.position.x*a_hc - self.poseRbMa.position.y*b_hc

		# -- tính toán vị trí hình chiếu trên lộ trình --
		if a_qd == 0 and b_qd != 0:             # lộ trình của AGV là đường thẳng nằm ngang có dạng by + c = 0
			X_n = self.poseRbMa.position.x
			Y_n = -c_qd/b_qd
		
		elif a_qd != 0 and b_qd == 0:           # lộ trình của AGV là đường thẳng thẳng đứng có dạng ax + c = 0
			X_n = -c_qd/a_qd
			Y_n = self.poseRbMa.position.y
		
		elif a_qd != 0 and b_qd != 0:           # lộ trình của AGV là đường thẳng có dạng: ax + by + c = 0
			Y_n = (a_hc*c_qd - c_hc*a_qd)/(b_hc*a_qd - a_hc*b_qd)
			X_n = -c_qd/a_qd - b_qd*Y_n/a_qd

		elif a_qd == 0 and b_qd == 0:              # ko thay đổi lộ trình 
			pass     

		kc_hinh_chieu = sqrt((X_n - self.poseRbMa.position.x)**2 + (Y_n - self.poseRbMa.position.y)**2)

		return X_n, Y_n, a_qd, b_qd, c_qd, kc_hinh_chieu

    # -- Hàm tìm vị trí điểm đích trên lộ trình -- 
	def find_point_goal(self, Xt, Yt, Xs, Ys, a_qd, b_qd, c_qd, X_n, Y_n, dis_ahead):

		X_g = Y_g = X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
		x_cv = y_cv = 0.0

		kc_ns = sqrt((X_n - Xs)**2 + (Y_n - Ys)**2)
		if a_qd == 0 and b_qd != 0:               # lộ trình của AGV là đường thẳng nằm ngang có dạng by + c = 0
			Y_g1 = Y_g2 = -c_qd/b_qd
			X_g1 = -sqrt(dis_ahead*dis_ahead - (Y_g1 - Y_n)*(Y_g1 - Y_n)) + X_n
			X_g2 = sqrt(dis_ahead*dis_ahead - (Y_g2 - Y_n)*(Y_g2 - Y_n)) + X_n
			
		elif a_qd !=0 and b_qd == 0:              # lộ trình của AGV là đường thẳng thẳng đứng có dạng ax + c = 0
			X_g1 = X_g2 = -c_qd/a_qd
			Y_g1 = -sqrt(dis_ahead*dis_ahead - (X_g1 - X_n)*(X_g1 - X_n)) + Y_n
			Y_g2 = sqrt(dis_ahead*dis_ahead - (X_g2 - X_n)*(X_g2 - X_n)) + Y_n

		elif a_qd != 0 and b_qd !=0:              # lộ trình của AGV là đường thẳng có dạng: ax + by + c = 0
			la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
			lb = -2.0*(X_n - (a_qd/b_qd)*((c_qd/b_qd) + Y_n))
			lc = X_n*X_n + ((c_qd/b_qd) + Y_n)*((c_qd/b_qd) + Y_n) - dis_ahead*dis_ahead
			denlta = lb*lb - 4.0*la*lc
			# print(la,lb,lc,denlta)

			X_g1 = (-lb + sqrt(denlta))/(2.0*la)
			X_g2 = (-lb - sqrt(denlta))/(2.0*la)

			Y_g1 = (-c_qd - a_qd*X_g1)/b_qd
			Y_g2 = (-c_qd - a_qd*X_g2)/b_qd

		elif a_qd == 0 and b_qd == 0:              # ko thay đổi lộ trình 
			pass 
		
		# print(a_qd, b_qd, c_qd, X_n, Y_n)

		# -- Chọn vị trí goal mà có hướng cùng chiều với hướng di chuyển -- 
		# loai nghiem bang vector
		vector_qd_x = Xt - Xs
		vector_qd_y = Yt - Ys

		vector_point1_x = X_n - X_g1
		vector_point1_y = Y_n - Y_g1

		if vector_qd_x == 0.0:
			if vector_qd_y*vector_point1_y > 0.0:
				X_g = X_g1
				Y_g = Y_g1
			else:
				X_g = X_g2
				Y_g = Y_g2
		elif vector_qd_y == 0.0:
			if vector_qd_x*vector_point1_x > 0.0:
				X_g = X_g1
				Y_g = Y_g1
			else:
				X_g = X_g2
				Y_g = Y_g2

		else:
			v_a = vector_qd_x/vector_point1_x
			v_b = vector_qd_y/vector_point1_y
			if v_a*v_b > 0.0 and v_a > 0.0:
				X_g = X_g1
				Y_g = Y_g1
			else:
				X_g = X_g2
				Y_g = Y_g2

		# print(X_g, Y_g)

		x_cv, y_cv = self.convert_relative_coordinates(X_g, Y_g)              
		return x_cv, y_cv, kc_ns

	# tính khoảng cách giữa 2 điểm trên hệ trục tọa độ 
	def fnCalcDistPoints(self, x1, x2, y1, y2):                     
		return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)	
	
	def run(self):
		while not rospy.is_shutdown():
            # kiểm tra dữ liệu đầu vào
			if self.process == 1:
				c_k = 0
				if self.is_request_move == True:
					c_k = c_k + 1

				if self.is_pose_robot == True:
					c_k = c_k + 1

				if c_k == 2:
					rospy.loginfo("Completed wakeup ('_')")
					self.process = 2

			elif self.process == 2:
				self.firstPoint_x = self.poseRbMa.position.x
				self.firstPoint_y = self.poseRbMa.position.y
				self.secondPoint_x = self.req_move.list_x[self.path_index + 1]
				self.secondPoint_y = self.req_move.list_y[self.path_index + 1]

				self.is_need_turn_step1 = 1
				self.process = 3

			# -- Quay AGV về đường mục tiêu -- 
			elif self.process == 3: 
				if self.is_need_turn_step1 == 1:
					a = self.poseRbMa.position.y - self.secondPoint_y                   # khoảng cách giữa điểm mục tiêu và vị trí hiện tại của AGV. 
					b = self.secondPoint_x - self.poseRbMa.position.x
					theta = self.find_angle_between(a, b, self.theta_rb_ht)          # trả về góc lệch giữa AGV và đường lộ trình 

					gt = self.turn_ar(theta, self.tolerance_rot_step1, self.vel_rot_step1)
					
					# print("Góc lệch giữa AGV và lộ trình là:", theta)
					# print("Tốc độ để robot quay về hướng lộ trình là", gt)

					if gt == -10:
						# print("Không thực hiện quay")
						self.stop()
						# self.stt_agv = 1
						self.is_need_turn_step1 = 0
						# self.time_start_navi = rospy.Time.now().to_sec()

					else:
						# print("Đang thực hiện quay")
						twist = Twist()
						twist.angular.z = gt
						self.pub_cmdVel(twist, self.rate_cmdvel)
				else:
					self.stop()
					self.process = 4		

			elif self.process == 4:
				self.X_n, self.Y_n, self.a_qd, self.b_qd, self.c_qd, self.dis_hc = self.find_hc(self.firstPoint_x,\
																								self.firstPoint_y,\
																								self.secondPoint_x,\
																								self.secondPoint_y)

				self.theta = self.find_angle_between(self.a_qd, self.b_qd, self.theta_rb_ht)

				dist_ahead = self.dist_ahead_max
				self.x_td_goal, self.y_td_goal, self.kc_con_lai = self.find_point_goal(self.firstPoint_x,\
																		self.firstPoint_y,\
																		self.secondPoint_x,\
																		self.secondPoint_y,\
																		self.a_qd,self.b_qd,self.c_qd,\
																		self.X_n,self.Y_n,\
																		dist_ahead)

				self.distance_goal = self.fnCalcDistPoints(self.poseRbMa.position.x,\
																	self.secondPoint_x,\
																	self.poseRbMa.position.y,\
																	self.secondPoint_y)

				# print("dis_hc= %s , x_now= %s, y_now= %s, distance_goal= %s, kc_conlai= %s" %(self.dis_hc, self.poseRbMa.position.x, self.poseRbMa.position.y, self.distance_goal, self.kc_con_lai))       

				v_x = 0.0                                                      
				if fabs(self.theta) > self.angle_find_vel:                      # nếu self.theta > PI/6, v_x = 0.3
					v_x = self.min_vel_x_gh                                   

				elif round(fabs(self.theta),2) == 0.0:                          # nếu theta = 0, v_x = self.vel_x =0.6  trong trường hơp điện áp đầy 
					v_x = self.vel_x

				else:
					v_x = self.min_vel_x_gh + ((self.angle_find_vel - fabs(self.theta))/self.angle_find_vel)*(self.vel_x - self.min_vel_x_gh)

				if self.distance_goal <= self.tol_simple or self.kc_con_lai <= self.tol_simple:    # kiểm tra AGV gần tới điểm cuối và chuyển lộ trình tiếp theo 
					self.stop()
					self.process = 50                    # AGV hoàn thành di chuyển 
				else:
					self.v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal, v_x)
				            
				twist = Twist()
				twist.linear.x = v_x
				twist.angular.z = self.v_th_send
				self.pub_cmdVel(twist, self.rate_cmdvel)	

			# AGV đã hoàn thành lộ trình cần di chuyển 
			elif self.process == 50:    
				self.stop()
		    
			self.rate.sleep()
    
def main():
	print('Program starting')
	agv1 = robot_AGV()
	agv1.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()