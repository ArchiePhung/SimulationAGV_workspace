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

from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from sti_msgs.msg import *
from agv_simulation.msg import * 
from nav_msgs.msg import Path
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin, acos, degrees, radians, atan2
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import threading
import signal
import time

class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y

class InfoPathFollowing():
    def __init__(self):
        self.pathID = 0.
        self.typePath = 0.
        self.direction = 0
        self.indexInListpath = 0.
        self.velocity = 0.
        self.radius = 0.
        self.movableZone = 0.
        self.X = 0.
        self.Y = 0.

class StraightLine():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond
        
        self.a = self.pointOne.y - self.pointSecond.y
        self.b = self.pointSecond.x - self.pointOne.x
        self.c = -self.pointOne.x*self.a - self.pointOne.y*self.b
        # print(self.a,self.b,self.c)
        self.dis = sqrt(self.a*self.a + self.b*self.b)

    def getQuaternion(self):
        euler = 0.0
        if self.b == 0:
            if self.a < 0:
                euler = PI/2.0
            elif self.a > 0:
                euler = -PI/2.0
        elif self.a == 0:
            if -self.b < 0:
                euler = 0.0
            elif -self.b > 0:
                euler = PI

        else:
            euler = acos(self.b/sqrt(self.b*self.b + self.a*self.a))
            if -self.a/self.b > 0:
                if fabs(euler) > PI/2:
                    euler = -euler
                else:
                    euler = euler
            else:
                if fabs(euler) > PI/2:
                    euler = euler
                else:
                    euler = -euler
                    
        return quaternion_from_euler(0.0, 0.0, euler)

    def checkPointInLine(self, _pointX, _pointY):
        if _pointX == self.pointSecond.x and _pointY == self.pointSecond.y:
            return True
        
        if _pointX == self.pointOne.x and _pointY == self.pointOne.y:
            return True
        
        # loai nghiem bang vector
        vector_qd_x = self.pointOne.x - self.pointSecond.x
        vector_qd_y = self.pointOne.y - self.pointSecond.y

        vector_point1_x = self.pointOne.x - _pointX
        vector_point1_y = self.pointOne.y - _pointY

        if vector_qd_x == 0.0:
            if vector_qd_y*vector_point1_y > 0.0:
                return True
            
        elif vector_qd_y == 0.0:
            if vector_qd_x*vector_point1_x > 0.0:
                return True

        else:
            v_a = vector_qd_x/vector_point1_x
            v_b = vector_qd_y/vector_point1_y
            if v_a*v_b > 0.0 and v_a > 0.0:
                return True

        return False
    
    def find_HC(self, _poseX, _poseY):
        X_n = Y_n = 0.
        if _poseX == self.pointOne.x and _poseY == self.pointOne.y :
            X_n = self.pointOne.x
            Y_n = self.pointOne.y

        else:
            a_hc = self.b
            b_hc = -self.a
            c_hc = -_poseX*a_hc - _poseY*b_hc

            X_n = ((c_hc*self.b)-(self.c*b_hc))/((self.a*b_hc)-(self.b*a_hc))
            Y_n = ((c_hc*self.a)-(self.c*a_hc))/((a_hc*self.b)-(b_hc*self.a))

        return X_n, Y_n
    
    def findGoalWithLookAhead(self, _poseX, _poseY, _L):
        X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        Xhc, Yhc = self.find_HC(_poseX, _poseY)

        if  round(self.b, 5) == 0.0:
            X_g1 = X_g2 = -self.c/self.a
            Y_g1 = -sqrt(_L*_L - (X_g1 - Xhc)*(X_g1 - Xhc)) + Yhc
            Y_g2 = sqrt(_L*_L - (X_g2 - Xhc)*(X_g2 - Xhc)) + Yhc
            
        else:
            la = (1.0 + (self.a/self.b)*(self.a/self.b))
            lb = -2.0*(Xhc - (self.a/self.b)*((self.c/self.b) + Yhc))
            lc = Xhc*Xhc + ((self.c/self.b) + Yhc)*((self.c/self.b) + Yhc) - _L*_L
            denlta = lb*lb - 4.0*la*lc

            X_g1 = (-lb + sqrt(denlta))/(2.0*la)
            X_g2 = (-lb - sqrt(denlta))/(2.0*la)

            Y_g1 = (-self.c - self.a*X_g1)/self.b
            Y_g2 = (-self.c - self.a*X_g2)/self.b

        if self.checkPointInLine(X_g1, Y_g1):
            return X_g1, Y_g1
        else:
            return X_g2, Y_g2
        
class robot_AGV(threading.Thread):
	def __init__(self, threadID):
		# -- Khởi tạo ROS --
		# rospy.init_node('MovetoCurvePath', anonymous=False)
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		self.saveTimeMainTheard = rospy.get_time()
		self.startLoop = False
	
		self.rate_hz = 50
		self.rate = rospy.Rate(self.rate_hz)
		
		# -- mô tả vị trí ban đầu của AGV trên rviz -- 
		self.x_start = rospy.get_param('x_start')
		self.y_start = rospy.get_param('y_start')
		self.theta_start = rospy.get_param('theta_start')

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
		self.poseStampedAGV = PoseStamped()
		self.theta_robotNow = 0.0

		# -- Nhận lộ trình từ traffic -- 
		rospy.Subscriber('/list_pointRequestMove', ListPointRequestMove, self.callback_listPointRequestMove, queue_size = 1)
		self.datalistPointRequestMove = ListPointRequestMove() 
		self.startPoint = InfoPathFollowing()
		self.finishPoint = InfoPathFollowing()
		self.is_listPointRequestMove = False

		# -- Nhận phản hồi điều khiển từ hệ thống(mode = 1: Manual mode, mode = 2: Auto mode) --
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()
		self.is_recv_NNinfoRespond = False

		# -- Gửi vận tốc robot -- 
		self.pub_cmd_vel = rospy.Publisher('/cmd_fvel', Velocities, queue_size=20)
		self.time_tr = rospy.get_time()
		self.rate_pubVel = 30

		# -- Gửi trạng thái của node 
		self.pubMoveRespond = rospy.Publisher('/move_respond', Status_goal_control, queue_size=10) 

		# self.robot_status = robot_status()
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

		self.curr_velocity = 0.
	
	def define_Pathvar(self):
		self.path_index = 0
		self.agv_frame =  "agv1"
		self.origin_frame = "world"

		self.tol_simple = rospy.get_param('~tol_simple')

		# for screen on rviz
		self.nb_pointPath = 25
		self.path_delta = 0.04
		self.path_t = 0

	def define_Systemvar(self):
		self.process = 0
		self.is_first_callback = True

		self.completed_all = False
		self.completed_simple = 0
		self.completed_backward = 0
		self.completed_reset = False
		self.warnAGV = 0
		self.nextPointIsStopGoal = False
		self.flagVelChange = True
		self.flagFollowPointStop = False
		self.flagCheckAngle = False
		self.isGoalNearest = False
		self.stepMoveToPoint = 0
		self.stepCheckStart = 0
		self.stepFollowGoalStop = 0
		self.stepMoveForward = 0
		self.stepTurnAR = 0

		self.poseGoalStop = Pose()
		self.idGoalStop = 0
		self.infoGoalNearest = InfoPathFollowing()
		self.disRbToGoalNearest = 0.

		self.rate_pubVel = 30

		self.offsetWheelwithMainAxis = rospy.get_param("offsetWheelwithMainAxis", 0.03)
		self.distanceBetwenOriginAndWheels = rospy.get_param("distanceBetwenOriginAndWheels", 1.285)
		self.radiusWheels = rospy.get_param("radiusWheels", 0.125)
	
		self.maxRPM_MainMotor = 3300
		self.maxAngleStreering = int(degrees(atan(self.distanceBetwenOriginAndWheels/self.offsetWheelwithMainAxis))*100)
		self.minAngleStreering = int(-9000 - degrees(atan(self.offsetWheelwithMainAxis/self.distanceBetwenOriginAndWheels))*100)
		self.minRPM_MainMotor = 440

        # gioi han van toc
		self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
		self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

		self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 0.8) #0.1
		self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.06) # 0.008	

		self.listVel = [0.06, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
		self.numTable = len(self.listVel)
		self.listLookAhead = [0.4, 0.5, 0.54, 0.6, 0.68, 0.78, 0.9, 0.92, 0.95, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]	

        # sai so
		self.toleranceByX_PointFinish = rospy.get_param('~toleranceByX_PointFinish', 0.015)
		self.toleranceByY_PointFinish = rospy.get_param('~toleranceByY_PointFinish', 0.015)
		self.toleranceByX_PointStop = rospy.get_param('~toleranceByX_PointStop', 0.015)
		self.toleranceByY_PointStop = rospy.get_param('~toleranceByY_PointStop', 0.015)
		
		self.decelerationRatio = 5.5
		self.disDeceleration = 0.

		self.directFollow = 0
		self.directForward = 1
		self.directReverse = 2

		# gioi han lookahead
		self.max_lookahead = rospy.get_param('~max_lookahead', 1.49)
		self.min_lookahead = rospy.get_param('~min_lookahead', 0.4)
		self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)
		self.distanceError = rospy.get_param('~distanceError', 1.2)			

		self.infoPathFollow = InfoPathFollowing()
		self.velFollowOutOfRange = 0.15

		self.theta_robotNow = 0.0	 

		self.coordinate_unknown = 500.0
		self.target_x = self.coordinate_unknown
		self.target_y = self.coordinate_unknown
		self.target_z = 0.0

		self.cmdVel = 0.
		self.flagVelChange = True
		self.statusVel = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
		self.velSt = 0.

		self.saveAngle = 0.
		
	def constrain(self, value_in, value_min, value_max):
		value_out = 0.0
		if value_in < value_min:
			value_out = value_min
		elif value_in > value_max:
			value_out = value_max
		else:
			value_out = value_in
		return value_out
		
	def getPose(self, data):
		self.is_pose_robot = True
		self.poseRbMa = data
		quata = (self.poseRbMa.orientation.x, self.poseRbMa.orientation.y, self.poseRbMa.orientation.z, self.poseRbMa.orientation.w)
		euler = euler_from_quaternion(quata)
		self.theta_robotNow = euler[2]

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data
		self.is_recv_NNinfoRespond = True

	def callback_listPointRequestMove(self, data):
		self.datalistPointRequestMove = data
		try:
			self.startPoint.X = self.datalistPointRequestMove.infoPoint[0].pose.position.x
			self.startPoint.Y = self.datalistPointRequestMove.infoPoint[0].pose.position.y
			self.startPoint.indexInListpath = 0
			self.startPoint.movableZone = self.datalistPointRequestMove.infoPoint[0].movableZone
			self.startPoint.pathID = self.datalistPointRequestMove.infoPoint[0].pathID
			self.startPoint.typePath = self.datalistPointRequestMove.infoPoint[0].typePath
			self.startPoint.velocity = self.datalistPointRequestMove.infoPoint[0].velocity

			self.finishPoint.X = self.datalistPointRequestMove.infoPoint[-1].pose.position.x
			self.finishPoint.Y = self.datalistPointRequestMove.infoPoint[-1].pose.position.y
			self.finishPoint.indexInListpath = len(self.datalistPointRequestMove.infoPoint) - 1
			self.finishPoint.movableZone = self.datalistPointRequestMove.infoPoint[-1].movableZone
			self.finishPoint.pathID = self.datalistPointRequestMove.infoPoint[-1].pathID
			self.finishPoint.typePath = self.datalistPointRequestMove.infoPoint[-1].typePath
			self.finishPoint.velocity = self.datalistPointRequestMove.infoPoint[-1].velocity
		except:
			pass
		self.is_listPointRequestMove = True

	# tìm điểm giao giữa đường cong lộ trình và vùng di chuyển của AGV.
	# def PointBetweenAGVandPath(self, agv_x, agv_y):
	# 	px, py = self.curve_path.getPoint_curveFormula()
	# 	return x_intersection, y_intersection
		
	# chuyển đổi tọa độ của 1 điểm bất kì theo tọa độ của AGV 							
	def convert_relative_coordinates(self, X_cv, Y_cv):
		angle = -self.theta_robotNow
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
		self.curr_velocity = 0.
		for i in range(2):
			self.pub_cmd_vel.publish(Velocities())

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
	def calculate_distance(self, x1, y1, x2, y2):
		x = x2 - x1
		y = y2 - y1
		return sqrt(x*x + y*y)

	def pub_cmdVelIndividual(self, _mode, _angleRequest, _velRequest, rate): # 1 cho tung kenh, 2 cho song kenh
		vel = Velocities()
		vel.selectMode = _mode # 0: dk dong co chinh | 1: dk dong co lai 
		vel.angleRequest = int(_angleRequest)
		vel.velRequest = int(_velRequest)
		
		if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = rospy.get_time()
			self.pub_cmd_vel.publish(vel)
		else:
			pass

	def pub_cmdVelNavigation(self, twist, rate):
		vel = Velocities()
		vel.selectMode = 2 # 2: dieu khien song song
		vel.twist = twist
		
		if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = rospy.get_time()
			self.pub_cmd_vel.publish(vel)
		else:
			pass

	def Meaning(self, stt, war):
		mess_step = {
			0: 'Step: Wait All data',
			1: 'Step: Select Mode',
			12: 'Kiem tra vi tri',
			13: 'Di chuyen den vi tri finish',
			14: 'Di chuyen den vi tri stop point',
			15: 'Di chuyen nhap vao duong dan',
			16: 'Follow duong dan',
			17: 'Dap ung goc cuoi',
			18: 'Hoan thanh - Doi reset :v',
			19: 'Cap nhat target',
			20: 'Phan chia proces'
		}
		mess_warn = {
			0: 'AGV di chuyen binh thuong :)',
			1: 'AGV gap vat can :(',
			2: 'AGV gap loi chuong trinh',
			3: 'AGV ra khoi duong dan'
		}
		mess = mess_step.get(stt, 'UNK') + ' | ' + mess_warn.get(war, 'UNK')
		return mess 

	def pub_status(self, misson, process_now, error, safety, complete_misson , id):
		status = Status_goal_control()
		status.misson = misson
		status.status_now = process_now
		status.ID_follow = int(id)
		status.error = error
		status.safety = safety
		status.meaning = self.Meaning(process_now, error)
		status.complete_misson = complete_misson
		self.pubMoveRespond.publish(status)

	def updateAll(self):
		self.completed_all = False
		self.completed_simple = 0
		self.completed_backward = 0
		self.completed_reset = False
		self.warnAGV = 0
		self.nextPointIsStopGoal = False
		self.flagVelChange = True
		self.flagFollowPointStop = False
		self.flagCheckAngle = False
		self.isGoalNearest = False
		self.nextPointIsStopGoal = False
		self.stepMoveToPoint = 0
		self.stepCheckStart = 0
		self.stepFollowGoalStop = 0
		self.stepMoveForward = 0
		self.stepTurnAR = 0

	def findStopGoal(self, idFollow):
		num = len(self.datalistPointRequestMove.infoStopPoint)
		if num:
			for i in range(num):
				pathID = self.datalistPointRequestMove.infoStopPoint[i].pathID
				if idFollow == pathID:
					self.poseGoalStop = self.datalistPointRequestMove.infoStopPoint[i].point
					self.idGoalStop = pathID
					return True
				
		return False         

	def checkPathOutOfRange(self):
		self.warnAGV = 0
		if self.isGoalNearest: # tim diem goal gan nhat
			goalNearNow = InfoPathFollowing()
			goalNearNow = self.infoGoalNearest
			disNear = self.disRbToGoalNearest

			if disNear > goalNearNow.movableZone:
				self.warnAGV = 3
				return -1 # bao loi qua khoang cachs -> yeu cau di chuyen gan lai
			
			if goalNearNow.X == self.finishPoint.X and goalNearNow.Y == self.finishPoint.Y: # la diem cuoi
				return 2
			
			if goalNearNow.pathID == self.finishPoint.pathID: # la diem cuoi
				a = 0.84
				b = 1.491
				dis = self.calculate_distance(goalNearNow.X, goalNearNow.Y, self.finishPoint.X, self.finishPoint.Y)
				if dis < a*disNear + b:
					return 2
				
			if self.findStopGoal(goalNearNow.pathID): # la diem stop point
				a = 0.84
				b = 1.491
				dis = self.calculate_distance(goalNearNow.X, goalNearNow.Y, self.poseGoalStop.position.x, self.poseGoalStop.position.y)
				if dis < a*disNear + b:
					return 3
				
			return 1  

	def checkResetOrTargetChange(self):
		targetX = round(self.datalistPointRequestMove.target_x, 3)
		targetY = round(self.datalistPointRequestMove.target_y, 3)
		targetZ = round(self.datalistPointRequestMove.target_z, 3)
		if self.datalistPointRequestMove.enable == 0 or self.target_x != targetX or self.target_y != targetY:
			self.stop()
			return 1
		
		return 0
	
	def angleStraightLine(self, XpointA, YpointA, XpointB, YpointB): # -- Angle Line Point A to Point B. | Point()
		a = YpointA - YpointB
		b = XpointB - XpointA
		ang = 0.0
		if b == 0:
			if a < 0:
				ang = PI/2.0
			elif a > 0:
				ang = -PI/2.0

		elif a == 0:
			if -b < 0:
				ang = 0.0
			elif -b > 0:
				ang = PI

		else:
			ang = acos(b/sqrt(b*b + a*a))
			if -a/b > 0:
				if fabs(ang) > PI/2:
					ang = -ang
				else:
					ang = ang
			else:
				if fabs(ang) > PI/2:
					ang = ang
				else:
					ang = -ang
		return ang

	def angleBetweenRobotAndPath2(self, _angle):
		dtAngle = _angle - self.theta_robotNow
		if fabs(dtAngle) >= PI:
			dtAngleTG = (2*PI - fabs(dtAngle))
			if dtAngle > 0:
				dtAngle = -dtAngleTG
			else:
				dtAngle = dtAngleTG

		return dtAngle
	
	def checkTurn(self, _Angle):          # ???????????????????????????? cần check lại vì ko dùng tới động cơ 
		ss = _Angle - self.respMotor.angleNow
		if fabs(ss) <= 100:
			return 1
		else:
			self.pub_cmdVelIndividual(1, _Angle, 0, self.rate_pubVel)
			return 0
	
	def turnAroundFGV(self, theta, tol_theta, velMax):
		if fabs(theta) > tol_theta:
			if self.stepTurnAR == 0: # quay goc streering
				if theta > 0:
					var = self.checkTurn(self.maxAngleStreering)
					self.checkDirect = 1
					if var == 1:
						self.stepTurnAR = 1 # da hoan thanh quay goc streering 
				else:
					var = self.checkTurn(self.minAngleStreering)
					self.checkDirect = -1
					if var == 1:  
						self.stepTurnAR = 1 # da hoan thanh quay goc streering 

			if self.stepTurnAR == 1: 
				temp = self.SIGN(theta)*self.checkDirect
				if temp > 0:
					if fabs(theta) <= radians(30.):
						_vel = (fabs(theta)/radians(30.))*velMax
						
					else:
						_vel = velMax
						
					if _vel < self.minRPM_MainMotor:
						_vel = self.minRPM_MainMotor
						
				else:
					if fabs(theta) <= radians(30.):
						_vel = (fabs(theta)/radians(30.))*(-velMax)
					else:
						_vel = -velMax
						
					if _vel > (-1)*self.minRPM_MainMotor:
						_vel = (-1)*self.minRPM_MainMotor

				self.pub_cmdVelIndividual(0, 0, _vel, self.rate_pubVel)
				
		else:
			self.stop()
			self.stepTurnAR = 0
			return 1 #done
		
		return 0
	
	def findLookAheadByVel(self, curr_velocity):
		a = 0.
		b = 0.
		if curr_velocity >= self.max_linearVelocity:
			curr_velocity = self.max_linearVelocity
		elif curr_velocity <= self.min_linearVelocity:
			curr_velocity = self.min_linearVelocity

		for i in range(self.numTable -1 ):
			if curr_velocity >= self.listVel[i] and curr_velocity <= self.listVel[i+1]:
				a = (self.listLookAhead[i] - self.listLookAhead[i+1])/(self.listVel[i] - self.listVel[i+1])
				b = self.listLookAhead[i] - a*self.listVel[i]

		return a*curr_velocity + b

	def moveToPoint(self, _x, _y, _ssX, _ssY, _vel):
		poseX = self.poseRbMa.position.x
		poseY = self.poseRbMa.position.y
		self.warnAGV = 0
		# quay toi Point
		if self.stepMoveToPoint == 0: #tinh goc can toi
			self.saveAngle = self.angleStraightLine(poseX, poseY, _x, _y)
			self.stepMoveToPoint = 1
		
		if self.stepMoveToPoint == 1: # quay goc
			angle = self.angleBetweenRobotAndPath2(self.saveAngle)
			gt = self.turn_ar(angle, self.tolerance_rot_step1, self.vel_rot_step1)
			if gt == -10:
				self.stop()
				self.stepMoveToPoint = 2

			else:
				# print("Đang thực hiện quay")
				twist = Twist()
				twist.angular.z = gt
				self.pub_cmdVelNavigation(twist, self.rate_pubVel)

			# if self.turnAroundFGV(angle, radians(0.5), 2000): # kiem tra goc AGV voi quy dao
			# 	if self.checkTurn(0.):
			# 		self.stepMoveToPoint = 2

		elif self.stepMoveToPoint == 2: # tim quy dao di chuyen
			self.saveSTL = StraightLine(Point(poseX, poseY), Point(_x, _y))
			self.saveTimeVel = rospy.get_time()
			self.stepMoveToPoint = 3
		
		elif self.stepMoveToPoint == 3:
			twist = Twist()
			ss_x = poseX - _x
			ss_y = poseY - _y
			self.curr_velocity = _vel
			lookAhead = self.findLookAheadByVel(self.curr_velocity)

			dis = sqrt(ss_x*ss_x + ss_y*ss_y)
			# print(ss_x, ss_y, dis, self.stepMoveToPoint)
			if (fabs(ss_x) <= _ssX and fabs(ss_y) <= _ssY) or dis <= sqrt(_ssX*_ssX + _ssY*_ssY):
				self.stepMoveToPoint = 0
				self.stop()
				print("Done Move To Point!")
				return 1
			
			else:
				# if self.respSafety.zone_ahead == 1 or self.lostSafety :
				# 	self.warnAGV = 1
				# 	self.stop()
				# 	print('not safety')

				# else:
				xFollow, yFollow = self.saveSTL.findGoalWithLookAhead(poseX, poseY, lookAhead)
				xCVFollow , yCVFollow = self.convert_relative_coordinates(xFollow, yFollow)
				velX = self.curr_velocity
				velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
				twist.linear.x = velX
				twist.angular.z = velAng
				self.pub_cmdVelNavigation(twist, self.rate_pubVel)

		return 0

	def angleBetweenRobotAndPath(self, _quat):
		roll, pitch, yaw = euler_from_quaternion(_quat)
		dtAngle = yaw - self.theta_robotNow
		if fabs(dtAngle) >= PI:
			dtAngleTG = (2*PI - fabs(dtAngle))
			if dtAngle > 0:
				dtAngle = -dtAngleTG
			else:
				dtAngle = dtAngleTG

		return dtAngle

	def getWaitPoint(self, xRb, yRb, curr_velocity):
		numSearch = 400
		longest_distance = 0.
		indexSelect = -1
		lookahead = self.findLookAheadByVel(curr_velocity)
		numpoint = len(self.datalistPointRequestMove.infoPoint)
		if self.startPoint.X == self.finishPoint.X and self.startPoint.Y == self.finishPoint.Y: # check loop
			# find Goal next numSearch step
			numStart = int(self.infoGoalNearest.indexInListpath)
			numEnd = int((self.infoGoalNearest.indexInListpath + numSearch) % numpoint)
			if numEnd <= numStart:
				for i in range(numStart, numpoint, 1):
					x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
					y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
					dis = self.calculate_distance(x, y, xRb, yRb)
					if dis <= lookahead and dis >= longest_distance:
						longest_distance = dis
						indexSelect = i
				
				for i in range(0, numEnd, 1):
					x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
					y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
					dis = self.calculate_distance(x, y, xRb, yRb)
					if dis <= lookahead and dis >= longest_distance:
						longest_distance = dis
						indexSelect = i

			else:
				for i in range(numStart, numEnd, 1):
					x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
					y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
					dis = self.calculate_distance(x, y, xRb, yRb)
					if dis <= lookahead and dis >= longest_distance:
						longest_distance = dis
						indexSelect = i

			if indexSelect == -1:
				for i in range(0, numpoint, 1):
					x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
					y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
					dis = self.calculate_distance(x, y, xRb, yRb)
					if dis >= lookahead:
						longest_distance = dis
						indexSelect = i
						break

		else:
			# find goal to goal finish
			numStart = int(self.infoGoalNearest.indexInListpath)
			for i in range(numStart,  numpoint, 1):
				x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
				y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
				dis = self.calculate_distance(x, y, xRb, yRb)
				if dis >= lookahead or (i == numpoint - 1 and dis <= lookahead):
					longest_distance = dis
					indexSelect = i
					break

		if indexSelect == -1:
			return False
		
		else:
			self.infoPathFollow.indexInListpath = indexSelect
			self.infoPathFollow.X = self.datalistPointRequestMove.infoPoint[indexSelect].pose.position.x
			self.infoPathFollow.Y = self.datalistPointRequestMove.infoPoint[indexSelect].pose.position.y

			self.infoPathFollow.movableZone = self.datalistPointRequestMove.infoPoint[indexSelect].movableZone
			self.infoPathFollow.direction = self.datalistPointRequestMove.infoPoint[indexSelect].direction

			pathID = self.datalistPointRequestMove.infoPoint[indexSelect].pathID
			typePath = self.datalistPointRequestMove.infoPoint[indexSelect].typePath
			if pathID != self.infoPathFollow.pathID and typePath == 1:
				self.flagCheckAngle = True

			self.infoPathFollow.pathID = pathID
			self.infoPathFollow.typePath = typePath
			# kiem tra id goal tiep theo co phai Stop khong
			if self.nextPointIsStopGoal == False and self.findStopGoal(self.infoPathFollow.pathID):
				self.nextPointIsStopGoal = True

			if self.flagCheckAngle:
				quat = (self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.x, \
						self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.y, \
						self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.z, \
						self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.w)

				angle = self.angleBetweenRobotAndPath(quat)
				if fabs(angle) <= radians(1.15) and self.disRbToGoalNearest <= 0.017:
					self.flagCheckAngle  = False

			else:         
				if self.statusVel != 2:
					vel = self.datalistPointRequestMove.infoPoint[indexSelect].velocity
					self.infoPathFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

			return True
	
	def moveToTheStopPoint(self, _dis, _poseX, _poseY, _XStop, _YStop):
		twist = Twist()
		self.warnAGV = 0
		if self.directFollow == self.directForward:
			self.stop()
			self.warnAGV = 1
			print('not safety - move to the stop Point')

		else:
			if _dis <= self.min_lookahead:
				velX = self.min_linearVelocity
				infoPointNear = InfoPathFollowing()
				infoPointNear = self.infoGoalNearest

				if self.stepFollowGoalStop == 0:
					self.saveSTL = StraightLine(Point(infoPointNear.X, infoPointNear.Y), Point(_XStop, _YStop))
					self.stepFollowGoalStop = 1

				if self.stepFollowGoalStop == 1:
					xFollow, yFollow = self.saveSTL.findGoalWithLookAhead(_poseX, _poseY, self.min_lookahead)
					xCVFollow , yCVFollow = self.convert_relative_coordinates(xFollow, yFollow)
					directVel = velX if self.directFollow == self.directForward else -velX
					velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
					twist.linear.x = directVel
					twist.angular.z = velAng
					self.pub_cmdVelNavigation(twist, self.rate_pubVel)
				
				# if self.stepFollowGoalStop == 0:
				#     # pt duong thang follow
				#     a, b, c = self.funcalptduongthang(infoPointNear.X, infoPointNear.Y,  self.infoPathFollow.X, self.infoPathFollow.Y)
				#     self.STTarget[0] = a
				#     self.STTarget[1] = b
				#     self.STTarget[2] = c
				#     self.stepFollowGoalStop = 1
				
				# if self.stepFollowGoalStop == 1:
				#     XFollow, YFollow = self.findGoalAheadType1(self.STTarget[0], self.STTarget[1], self.STTarget[2], infoPointNear.X, infoPointNear.Y, self.min_lookahead)
				#     self.infoPathFollow.X = XFollow
				#     self.infoPathFollow.Y = YFollow
				#     xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
				#     velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
				#     twist.linear.x = velX
				#     twist.angular.z = velAng
				#     self.pub_cmdVelNavigation(twist, self.rate_pubVel)

			else:
				self.curr_velocity = self.velSt*(_dis/self.disDeceleration)
				self.curr_velocity  = self.constrain(self.curr_velocity, self.min_linearVelocity, self.max_linearVelocity)
				print(self.curr_velocity, self.disDeceleration, self.infoPathFollow.velocity, "Follow Target Normal!")
				if self.getWaitPoint(_poseX, _poseY, self.curr_velocity):
					velX = self.curr_velocity
					xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
					directVel = velX if self.directFollow == self.directForward else -velX
					velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
					twist.linear.x = directVel
					twist.angular.z = velAng
					self.pub_cmdVelNavigation(twist, self.rate_pubVel)

				else:
					self.stop()
					print('Error, Ko tim dc waypoint mode follow point stop')
					return -1
			
		return 1	

	def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
		denlta_time_now = rospy.get_time() - time_s
		v_re = v_s + a*denlta_time_now
		if a > 0.:
			if v_re >= v_f:
				v_re = v_f
		else:
			if v_re <= v_f:
				v_re = v_f

		return v_re

	def getVeloctity(self, _safety, _statusVel):
		if self.directFollow == self.directForward:
			self.statusVel = 0
			print('not safety - get velocity')
			return 0.
			
		else:      
			if _statusVel == 0:
				return self.infoPathFollow.velocity
			elif _statusVel == 1:
				return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, 0.12)
			elif _statusVel == 2:
				return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, -0.4)
			else:
				return 0.
	    
	def followPath(self, _ssXNormal, _ssYNormal, _ssXFinish, _ssYFinish):
		twist = Twist()
		poseX = self.poseRbMa.position.x
		poseY = self.poseRbMa.position.y
		# angleAGV = self.theta_robotParking
		XStop = 0.
		YStop = 0.
		ss_x = 0.
		ss_y = 0.
		dis = 0.
		disGT = 0.
		mode = ''
		toleranceX = 0.
		toleranceY = 0.
		tolerance_radius = 0.
		self.warnAGV = 0

		if self.nextPointIsStopGoal:
			XStop = self.poseGoalStop.position.x
			YStop = self.poseGoalStop.position.y
			ss_x = poseX - self.poseGoalStop.position.x
			ss_y = poseY - self.poseGoalStop.position.y
			toleranceX = _ssXNormal
			toleranceY = _ssYNormal
			mode = 'follow Stop Goal'

		else:
			XStop = self.finishPoint.X
			YStop = self.finishPoint.Y
			ss_x = poseX - self.finishPoint.X
			ss_y = poseY - self.finishPoint.Y
			toleranceX = _ssXFinish
			toleranceY = _ssYFinish
			mode = 'follow Finish Goal'

		dis = sqrt(ss_x*ss_x + ss_y*ss_y)
		tolerance_radius = sqrt(toleranceX*toleranceX + toleranceY*toleranceY)
		disGT = self.curr_velocity*self.decelerationRatio

		if self.flagFollowPointStop == False and dis <= disGT:
			self.velSt = self.curr_velocity
			self.disDeceleration = disGT
			self.flagFollowPointStop = True

		# xet them dk den target - tranh truong hop di qua
		xcv, ycv = self.convert_relative_coordinates(XStop, YStop)
		isFinish = False
		if self.directFollow == self.directForward:
			if xcv <= toleranceX and fabs(ycv) < 0.15:
				isFinish = True
		elif self.directFollow == self.directReverse: 
			if xcv >= -toleranceX and fabs(ycv) < 0.15:
				isFinish = True

		print(ss_x, ss_y, dis, self.disDeceleration, mode)
		if (fabs(ss_x) <= toleranceX and fabs(ss_y) <= toleranceY) or dis <= tolerance_radius or isFinish:
			self.stop()
			self.stepFollowGoalStop = 0. # chung
			self.flagFollowPointStop = False # chung
			if self.nextPointIsStopGoal:
				self.nextPointIsStopGoal = False
				print("Done Achieve Stop Goal! next Step is continue to follow the next path (^-^)")
				return 2 # den diem Stop Done
			
			else:
				print(self.finishPoint.X, self.finishPoint.Y, self.target_x, self.target_y)
				if self.finishPoint.X == self.target_x and self.finishPoint.Y == self.target_y: # hoan thanh diem target
					print("Done Achieve Target Goal! next Step is rotate target angle (^-^)")
					return 5
				
				else:
					print("Done Achieve Finish Goal! next Step is waiting request Traffic(^-^)")
					return 3 # het duong dan, cho duong dan tiep theo
		
		else:
			if self.flagFollowPointStop:
				stt = self.moveToTheStopPoint(dis, poseX, poseY, XStop, YStop)
				if stt == -1:
					self.warnAGV = 2
					self.stop()
					print("Something went wrong (T-T)")
					return -2

			else:
				# kiem tra dang follow point cuoi chua
				# self.checkFollowPointFinish()
				# them phuong trinh giam toc
				if self.curr_velocity < self.infoPathFollow.velocity and self.statusVel != 1:
					self.statusVel = 1
					self.velSt = self.curr_velocity
					self.saveTimeVel = rospy.get_time()

				elif self.curr_velocity > self.infoPathFollow.velocity and self.statusVel != 2:
					self.statusVel = 2
					self.velSt = self.curr_velocity
					self.saveTimeVel = rospy.get_time()

				elif self.curr_velocity == self.infoPathFollow.velocity:
					self.statusVel = 0

				self.curr_velocity = self.getVeloctity(0, self.statusVel)
				print(self.curr_velocity, self.statusVel, self.infoPathFollow.velocity)

				if self.curr_velocity != 0. :
					if self.getWaitPoint(poseX, poseY, self.curr_velocity):
						velX = self.curr_velocity
						xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
						directVel = velX if self.directFollow == self.directForward else -velX
						velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
						twist.linear.x = directVel
						twist.angular.z = velAng

						self.pub_cmdVelNavigation(twist, self.rate_pubVel)
					
					else:
						self.stop()
						self.warnAGV = 2
						print("Something went wrong (T-T)")
						return -1
				else:
					self.warnAGV = 1
					self.stop()
					
		return 1

	def infoPointPathByIndex(self, _index):
		infoPoint = InfoPathFollowing()
		infoPoint.indexInListpath = _index
		infoPoint.pathID = self.datalistPointRequestMove.infoPoint[_index].pathID
		infoPoint.velocity = self.datalistPointRequestMove.infoPoint[_index].velocity
		infoPoint.typePath = self.datalistPointRequestMove.infoPoint[_index].typePath
		infoPoint.direction = self.datalistPointRequestMove.infoPoint[_index].direction
		infoPoint.movableZone = self.datalistPointRequestMove.infoPoint[_index].movableZone
		infoPoint.X = self.datalistPointRequestMove.infoPoint[_index].pose.position.x
		infoPoint.Y = self.datalistPointRequestMove.infoPoint[_index].pose.position.y

		return infoPoint

	def moveIntoPath(self):
		indexSelect = -1
		lookahead = self.findLookAheadByVel(self.velFollowOutOfRange)
		numpoint = len(self.datalistPointRequestMove.infoPoint)
		numStart = self.infoGoalNearest.indexInListpath
		numStop = numpoint
		self.warnAGV = 0
		for i in range(numStart, numStop, 1):  # can kiem tra lai ------------------------------#####--------------------------------------
			x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
			y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
			pathID = self.datalistPointRequestMove.infoPoint[i].pathID
			# vel = self.dataPath.info[i].velocity
			dis = self.calculate_distance(x, y, self.infoGoalNearest.X, self.infoGoalNearest.Y)
			if dis >= lookahead:
				indexSelect = i
				break

		self.infoPathFollow = self.infoPointPathByIndex(indexSelect)
		self.directFollow = self.infoPathFollow.direction

		angleLineStart = 0.
		if self.directFollow == self.directForward:
			angleLineStart = self.angleStraightLine(self.infoGoalNearest.X, self.infoGoalNearest.Y, self.infoPathFollow.X, self.infoPathFollow.Y)
		elif self.directFollow == self.directReverse: 
			angleLineStart = self.angleStraightLine(self.infoPathFollow.X, self.infoPathFollow.Y, self.infoGoalNearest.X, self.infoGoalNearest.Y)

		angleBW = self.angleBetweenRobotAndPath2(angleLineStart)

		if self.stepCheckStart == 0:
			if self.directFollow == self.directForward:
				self.saveAngle = self.angleStraightLine(self.poseRbMa.position.x, self.poseRbMa.position.y, self.infoPathFollow.X, self.infoPathFollow.Y)
			elif self.directFollow == self.directReverse: 
				self.saveAngle = self.angleStraightLine(self.infoPathFollow.X, self.infoPathFollow.Y, self.poseRbMa.position.x, self.poseRbMa.position.y)
			self.stepCheckStart = 1

		elif self.stepCheckStart == 1:
			angle = self.angleBetweenRobotAndPath2(self.saveAngle)
			gt = self.turn_ar(angle, self.tolerance_rot_step1, self.vel_rot_step1)
			if gt == -10:
				self.stop()
				self.stepCheckStart = 2

			else:
				# print("Đang thực hiện quay")
				twist = Twist()
				twist.angular.z = gt
				self.pub_cmdVelNavigation(twist, self.rate_pubVel)

			# if self.turnAroundFGV(angle, radians(5.), 2000): # kiem tra goc AGV voi quy dao
			# 	if self.checkTurn(0.):
			# 		self.stepCheckStart = 2

		elif self.stepCheckStart == 2:
			if self.disRbToGoalNearest <= 0.02  and fabs(angleBW) <= 10.*PI/180.: # dieu kien de xac nhan da vao duong dan
				self.stepCheckStart = 0
				return 1
			
			else:
				if self.directFollow == self.directForward:
					self.stop()
					self.warnAGV = 1
					print('not safety - move into Path')

				else:
					twist = Twist()
					velX = self.velFollowOutOfRange
					self.curr_velocity = velX
					xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X,self.infoPathFollow.Y)
					directVel = velX if self.directFollow == self.directForward else -velX
					velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
					twist.linear.x = directVel
					twist.angular.z = velAng

					self.pub_cmdVelNavigation(twist, self.rate_pubVel)

		return 0	

	def findGoalNearest(self, xRb, yRb):
		index = -1
		status = False
		min = float('inf')
		numpoint = len(self.datalistPointRequestMove.infoPoint)
		for i in range(numpoint - 1, -1, -1):
			x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
			y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
			# vel = self.dataPath.info[i].velocity
			dis = self.calculate_distance(x, y, xRb, yRb)

			if dis <= min:
				min = dis
				index = i
		infoPoint = InfoPathFollowing()
		if index != -1:  
			infoPoint.indexInListpath = index
			infoPoint.pathID = self.datalistPointRequestMove.infoPoint[index].pathID
			infoPoint.velocity = self.datalistPointRequestMove.infoPoint[index].velocity
			infoPoint.typePath = self.datalistPointRequestMove.infoPoint[index].typePath
			infoPoint.movableZone = self.datalistPointRequestMove.infoPoint[index].movableZone
			infoPoint.X = self.datalistPointRequestMove.infoPoint[index].pose.position.x
			infoPoint.Y = self.datalistPointRequestMove.infoPoint[index].pose.position.y
			# print('dis_rb_to_path: ', min)

		return infoPoint, min, status

	def run(self):
		while not self.shutdown_flag.is_set():
            # kiểm tra dữ liệu đầu vào
			if self.process == 0:
				c_k = 0
				if self.is_listPointRequestMove == True:
					c_k = c_k + 1

				if self.is_pose_robot == True:
					c_k = c_k + 1

				if c_k == 2:
					rospy.loginfo("Completed wakeup ('_')")
					self.process = 1

			elif self.process == 1:
				if self.is_listPointRequestMove:
					if self.datalistPointRequestMove.enable == 1: # di chuyen theo line
						self.process = 19
			
			elif self.process == 19:
				enable = self.datalistPointRequestMove.enable
				targetX = round(self.datalistPointRequestMove.target_x, 3)
				targetY = round(self.datalistPointRequestMove.target_y, 3)
				targetZ = round(self.datalistPointRequestMove.target_z, 3)
				if enable == 1:
					print("Before!", self.target_x, self.target_y, targetX, targetY)
					if targetX < 500.0 and targetY < 500.0:
						if self.target_x != targetX or self.target_y != targetY:
							self.stop()
							self.updateAll()
							self.target_x = targetX
							self.target_y = targetY
							self.target_z = targetZ
							print("Recive new Target and Plan!", self.target_x, self.target_y)

						self.process = 20

					else:
						self.process = 1

				else:
					self.process = 1		

			# Buoc kiem tra xu ly process 
			elif self.process == 20:
				if self.completed_all:
					self.process = 1
				else:
					self.process = 12

			elif self.process == 12:
				stt = self.checkPathOutOfRange()
				if stt == -1:
					self.process = 1
					print("OUT OF PATH")
				if stt == 1: # di chuyen bam duong dan
					self.process = 15
					print("di chuyen bam duong", self.process)
				elif stt == 2: # di chuyen huong den diem finish
					self.process = 13
					print("di chuyen huong den diem finish", self.process)
				elif stt == 3: # di chuyen huong den diem Stop
					self.process = 14
					print("di chuyen huong den diem Stop", self.process)		    

			elif self.process == 13: # di chuyen den vi tri finish
				isChangeOrReset = self.checkResetOrTargetChange()
				if isChangeOrReset == 0:
					stt = self.moveToPoint(self.finishPoint.X, self.finishPoint.Y, self.toleranceByX_PointFinish, self.toleranceByY_PointFinish, self.min_linearVelocity)
					if stt == 1: # done
						if self.finishPoint.X == self.target_x and self.finishPoint.Y == self.target_y:
							self.process = 17 # quay dap ung goc cuoi
						else:
							self.process = 100 # di chuyen het duong dan.. wait cap nhat
							print("waiting update!")
					elif stt == -1: # error
						print("error", self.process)
				else:
					self.process = 1
		    
			elif self.process == 14: # di chuyen den vi tri Stop Point
				isChangeOrReset = self.checkResetOrTargetChange()
				if isChangeOrReset== 0:
					stt = self.moveToPoint(self.poseGoalStop.position.x, self.poseGoalStop.position.y, self.toleranceByX_PointStop, self.toleranceByY_PointStop, self.min_linearVelocity)
					if stt == 1: # done
						self.process = 15
					elif stt == -1: # error
						print("error", self.process)
				else:
					self.process = 1

			elif self.process == 15: # kiem tra vi tri AGV voi duong dan 
				isChangeOrReset = self.checkResetOrTargetChange()
				if isChangeOrReset== 0:
					stt = self.moveIntoPath()
					if stt == 1:
						print("Done move into path")
						self.process = 16
				else:
					self.process = 1

			elif self.process == 16: # follow theo duong dan
				isChangeOrReset = self.checkResetOrTargetChange()
				if isChangeOrReset== 0:
					stt = self.followPath(self.toleranceByX_PointStop, self.toleranceByY_PointStop, self.toleranceByX_PointFinish, self.toleranceByY_PointFinish)
					if stt == 2: # den diem stop point
						self.process = 15
					elif stt == 3: # den diem cuoi trong list
						self.process = 100 # di chuyen het duong dan.. wait cap nhat
					elif stt == 5: # den diem target
						self.process = 17 # quay dap ung goc cuoi
					elif stt == -1: # loi
						self.process = -2
					elif stt == -2: # loi
						self.process = -3
				else:
					self.process = 1		

			elif self.process == 17:
				isChangeOrReset = self.checkResetOrTargetChange()
				if isChangeOrReset== 0:
					theta = self.target_z - self.theta_robotNow
					if fabs(theta) >= PI:
						theta_t = (2*PI - fabs(theta))
						if theta > 0:
							theta = -theta_t
						else:
							theta = theta_t
					
					gt = self.turn_ar(theta, self.tolerance_rot_step1, self.vel_rot_step1)
					if gt == -10:
						self.completed_all = True
						self.process = 1
						print("DONE ROTATY, WAIT NEW TARGET! (^-^)")

					else:
						# print("Đang thực hiện quay")
						twist = Twist()
						twist.angular.z = gt
						self.pub_cmdVelNavigation(twist, self.rate_pubVel)

					# if self.turnAroundFGV(theta, radians(1.), 2000): # kiem tra goc AGV voi quy dao
					# 	if self.checkTurn(0.):
					# 		self.completed_all = True
					# 		self.process = 1
					# 		print("DONE ROTATY, WAIT NEW TARGET! (^-^)")

				else:
					self.process = 1

			elif self.process == -2:
				print("Error, Ko tim dc waypoint mode navi")

			elif self.process == -3:
				# print("Error, Ko tim dc waypoint mode follow point stop")
				pass
		    
			if self.datalistPointRequestMove.enable == 0:
				self.pub_status(self.datalistPointRequestMove.enable,0,0,0,0,0)

			elif self.datalistPointRequestMove.enable == 1 or self.datalistPointRequestMove.enable == 3:
				self.pub_status(self.datalistPointRequestMove.enable, self.process, 0, self.warnAGV, self.completed_all, self.infoPathFollow.pathID)   

			elif self.datalistPointRequestMove.enable == 2:
				self.pub_status(self.datalistPointRequestMove.enable,self.process,0,self.warnAGV,self.completed_backward,0) 						    		    
			
			self.rate.sleep()
    
class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass
 
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit
 
def main():

    rospy.init_node('goal_control', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        br = robot_AGV(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            # find goal nearest
            if br.is_pose_robot and br.is_listPointRequestMove:
                if br.datalistPointRequestMove.enable == 1:
                    br.infoGoalNearest, br.disRbToGoalNearest, br.isGoalNearest = br.findGoalNearest(br.poseRbMa.position.x, br.poseRbMa.position.y)
                    br.isGoalNearest = True
                else:
                    br.isGoalNearest = False

            time.sleep(0.05)
 
    except ServiceExit:
        
        br.shutdown_flag.set()
        # Wait for the threads to close...
        br.join()
    print('Exiting main program')

if __name__ == '__main__':
    main()
