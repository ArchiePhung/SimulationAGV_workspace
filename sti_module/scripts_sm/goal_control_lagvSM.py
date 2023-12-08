#!/usr/bin/env python3  
# -*- coding: utf-8 -*-
# Dev: Phùng Quý Dương STI 19/5/2023

"""
Nội dung code mới:
1. Chương trình 1: Sử dụng hàm control_navigation và convert tọa độ để điều khiển AGVSM di chuyển tới điểm

2. Chương trình 2: AGV đi theo lộ trình
   * Nhận lộ trình từ traffic
   * Generate các đường lộ trình thành nhiều điểm nhỏ.
   * Không tăng các khoảng thời gian cố định mà đưa nó vào 1 vòng lặp
   * Khai báo vùng hoạt động của AGV là hình tròn với bán kính R
   * Vòng tròn này sẽ cắt đường lộ trình tại 1 điểm, AGV follow theo điểm đó
   * Thay vì nhận hình chiếu của AGV, lúc này việc cần làm là xác định vị trí của AGV với điểm gần nhất với lộ trình.

"""
import roslib
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
from agvball_simulation.msg import * 
from nav_msgs.msg import Path
import numpy as np
from scipy.integrate import quad

from matplotlib import pyplot as plt
from itertools import count 
# import pandas as pd
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')
import threading
import signal
import os
import sys

class goalControl():
    def __init__(self):

        # threading.Thread.__init__(self)
        # self.threadID = threadID
        # self.shutdown_flag = threading.Event()

        rospy.init_node('goal_control', anonymous=False)
    
        # -- mô tả vị trí ban đầu của AGV trên rviz -- 
        self.agv_start_pose_x = rospy.get_param('~agv_start_x')
        self.agv_start_pose_y = rospy.get_param('~agv_start_y')
        self.agv_start_angle = rospy.get_param('~agv_start_angle')

        # -- mô tả vị trí trước đó của agv -- 
        self.agv_pre_pose_x = self.agv_start_pose_x
        self.agv_pre_pose_y = self.agv_start_pose_y
        self.agv_pre_angle = self.agv_start_angle

        # -- vị trí điểm đích tiếp theo của agv theo lộ trình gửi xuống  
        self.agv_target_pose_x = 0
        self.agv_target_pose_y = 0
        self.agv_target_angle = 0
        self.agv_target_index = 0

        # -- khoảng nhìn trước max --
        self.dist_ahead_max = rospy.get_param('~khoang_nhin_truoc_max')
        self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1')
        self.vel_rot_step1 = rospy.get_param('~vel_rot_step1')

        # -- node subcribe -- 
        rospy.Subscriber('/agv_pose', Pose, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb_ht = 0.0

        rospy.Subscriber('/request_move_fake', Move_request, self.move_callback ,queue_size = 20)                # lộ trình nhận được từ sti_control 
        self.req_move = Move_request()                                                 
        self.is_request_move = False

        rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
        self.NN_infoRespond = NN_infoRespond()
        self.is_recv_NNinfoRespond = False                                                  
        
        # -- node publish -- 
        self.rate_hz = 50
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 20)
        self.pub_path_global = rospy.Publisher('/path_plan_global', Path, queue_size = 20)

        self.rate = rospy.Rate(self.rate_hz)

        self.pub_status = rospy.Publisher('/ball_status', ball_status, queue_size = 20)
        self.ballstate = ball_status()

        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)
        self.auto_reset = 0

    # -- biến từ node goal_Control mẫu, khi hoàn thành sẽ xóa những biến thừa đi
        self.process = 1
        self.pre_mess = ""      # print just one time.
        self.war_agv = 0 # 0: agv di chuyen binh thuong | 1: agv gap vat can

        self.coordinate_unknown = 500.0
        self.is_target_change = False

        self.target_x = self.coordinate_unknown
        self.target_y = self.coordinate_unknown
        self.target_z = 0.0
        self.tag = 0
        self.offset = 0.0
        self.list_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.list_y = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.list_id = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.mission = 0

        self.completed_simple = 0      # bao da den dich.
        self.completed_backward = 0     # bao da den aruco.
        self.completed_all = 0
        self.completed_list = 0
        self.completed_reset = 0
        self.stt_agv = 0
        self.error = 0

        self.end_of_list = False

        self.cur_goal_x = 0.0
        self.cur_goal_y = 0.0

        self.point_goal_start_x = 0.0
        self.point_goal_start_y = 0.0

        self.is_need_turn_step1 = 0
        self.is_need_pttt = 1
        self.is_pre_pttt = 0
        self.x_td_goal = 0.0 
        self.y_td_goal = 0.0
        self.dis_hc = 0.0

        self.cur_goal_is = 0 # 1: diem thuong, 2: diem dac biet, 3: diem dich

        self.distance_goal = 0.0

        # self.tol_simple = 1.0      # do chinh xac theo truc x,y
        self.tol_simple = rospy.get_param('~tol_simple')
        self.tol_target = 0.02
        self.ss_luivsnextGoal = 0.25

        self.vel_x_now = 0.0

        self.vel_x = 0.0            # toc do theo truc x
        self.theta_max = 0.6

        self.vel_x1 = 0.4  		    # level 1 slowless
        self.theta_max_1 = 0.4 

        self.vel_x2 = 0.5  		    # level 2 
        self.theta_max_2 = 0.5

        self.vel_x3 = 0.2    		# level 3 fastless
        self.theta_max_3 = 0.6

        self.min_vel_x = 0.04


        self.odom_x_ht = 0.0
        self.odom_y_ht = 0.0
        self.kc_backward = 0.0

        self.XRobotStart = 0.0
        self.YRobotStart = 0.0

        self.min_vel_x_gh = 0.3
        self.theta = 0.0

        self.angle_find_vel = 30.0*PI/180.0
        self.time_start_navi = rospy.Time.now().to_sec()

        self.angle_giam_toc = 45.0*PI/180.0

        self.dis_gt = 0.3
        self.kc_con_lai = 0.0
        self.kc_qd = 0.0
        self.is_over_goal = False

        self.X_n = 0.0
        self.Y_n = 0.0
        self.a_qd = 0.0
        self.b_qd = 0.0
        self.c_qd = 0.0

        self.id_fl = 0.0

        self.rate_cmdvel = 30
        self.time_tr = rospy.get_time()

        # self.path_plan = Path()
        # self.path_plan.header.frame_id = 'frame_map_nav350'
        # self.path_plan.header.stamp = rospy.Time.now()

        # -- journey info 
        self.agv_frame =  "agv1"
        self.origin_frame = "world"

        self.path_plan = Path()

        self.timeRecieveNAV = time.time()
        self.timeRecieveTIM = time.time()

        self.timeZone3TIM = 0
        self.timeZone2TIM = 0

        self.timeWaitTIM = 0.1
        self.timeWaitNAV = 0.5

        self.oldzone = 0

        # -- Duowng add
        self.path_index = 0
        self.v_th_send = 0
        self.len_tranjectory = 0

        self.is_first_callback = True

        self.move_straight = 1
        self.move_curve = 2
        self.type_run = self.move_straight 

        # for case distance range up is fixed
        self.Padlength = 0
        self.time_up = 0
        # self.delta_up = 0.2
        self.delta_up = rospy.get_param('~curve_deltaup')
        self.avg_delta_up = self.delta_up / 2
        self.first_delta = 0
        self.second_delta = self.delta_up 
        self.numberofdelta = 1/self.delta_up   # = 5 khoang = 5 times update v, r
        self.time_run = time.time()
        self.t = 0
        self.r = 0
        self.timeWait = time.time()
        self.lactime = 63.0   # độ trễ của hệ thống 
        self.d_org = 0.
        self.d_now = 0.
        self.ratio_d_now = 0.
        self.err_ratio_d = 0.
        self.first_target_x = 0.
        self.first_target_y = 0.
        self.second_target_x = 0.
        self.second_target_y = 0.

        # for pub path tranjectory of agv
        self.nb_point_path = 25
        # self.nb_point_path = rospy.get_param('~nb_point_path')
        self.point_path_x = []
        self.point_path_y = []
        self.path_delta = 0.04
        self.path_t = 0

        self.arr_timeup = []
        self.arr_lactime = []

        self.arr_dnow = []
        self.arr_dorg = []
        self.arr_ratio = []
        self.arr_t = []
        # self.index = count()

        self.time_getdata_ticks = time.time()
        self.time_getdata = rospy.get_param('~time_getdata')
        # (self.fig, self.subplot) = plt.subplots(2,2)
        self.i = 0.05

        # for vị trí gốc đường cong ko phải là vị trí bắt đầu của AGV. 
        self.is_agv_onCurve = 0
#-------------------------------------------------------------------------------------------------
    def fnShutDown(self):
        # rospy.loginfo("Shutting down. cmd_vel will be 0")
        # self.pub_cmd_vel.publish(Twist()) 
        pass

    def getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

    def infoAGV_callback(self, data):
        self.NN_infoRespond = data
        self.is_recv_NNinfoRespond = True

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)
    
    def getRadius_curveFormula(self, t, a0, b0, a1, b1, a2, b2):
        # ft Berzier Curve 
        Px = pow((1-t), 2) * a0 + 2 * (1-t) * t * a1 + t ** 2 * a2
        Py = pow((1-t), 2) * b0 + 2 * (1-t) * t * b1 + t ** 2 * b2
        
        # ft Berzier Curve Derive level 1
        Pxdot = 2 * (1-t) * a0 + (2 - 4 * t) * a1 + 2 * t * a2
        Pydot = 2 * (1-t) * b0 + (2 - 4 * t) * b1 + 2 * t * b2

        # ft Berzier Curve Derive level 2
        Px2dot = -2 * a0 - 4 * a1 + 2 * a2
        Py2dot = -2 * b0 - 4 * b1 + 2 * b2

        R = pow((Pxdot ** 2 + Pydot ** 2), 3/2) / (Pxdot * Py2dot - Pydot * Px2dot)
        # print(R)

        return R

    def getLength_curveFormula(self, x, a0, b0, a1, b1, a2, b2):
        # ft Berzier Curve Derive level 1
        Pxdot = 2 * (1-x) * a0 + (2 - 4 * x) * a1 + 2 * x * a2
        Pydot = 2 * (1-x) * b0 + (2 - 4 * x) * b1 + 2 * x * b2

        return sqrt(Pxdot**2 + Pydot**2)
        # L = quad(sqrt(Pxdot**2 + Pydot**2), 0, 1)
        # print(L)

        # return L
    
    def getAngle_TangentLine(self, t, a0, b0, a1, b1, a2, b2):
        Pxdot = 2 * (1-t) * a0 + (2 - 4 * t) * a1 + 2 * t * a2
        Pydot = 2 * (1-t) * b0 + (2 - 4 * t) * b1 + 2 * t * b2

        return atan2(Pydot, Pxdot)
    
    def getPoint_curveFormula(self, t, a0, b0, a1, b1, a2, b2):
        Px = (1-t)**2 * a0 + 2 * (1-t) * t * a1 + t ** 2 * a2
        Py = (1-t)**2 * b0 + 2 * (1-t) * t * b1 + t ** 2 * b2
        # x = 4, 8, 8
        # y = 0, 0, -4
        # PDx = a0 + t*(a1-a0)
        # PDy = b0 + t*(b1-b0)

        # PEx = a1 + t*(a2-a1) 
        # PEy = b1 + t*(b2-b1)

        # Px = PDx + t*(PEx - PDx)
        # Py = PDy + t*(PEy - PDy)
        # print (t, PDx, PDy, PEx, PEy, Px, Py)

        return (Px, Py)

    def euler_to_quaternion(self, euler):
        quat = Quaternion()
        odom_quat = quaternion_from_euler(0, 0, euler)
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        return quat

    def getAngle_2nearbyPad(self, a0, b0, a1, b1, a2, b2):
        V1x = a1 - a0
        V1y = b1 - b0
        L1 = sqrt(V1x**2 + V1y**2)
        V2x = a2 - a1
        V2y = b2 - b1
        L2 = sqrt(V2x**2 + V2y**2)

        costheta = (V1x * V2x + V1y * V2y) / (L1 * L2)
        theta = acos(costheta)

        return theta

    def move_callback(self, data):       # get tranjectory, then generate curve equation 
        self.req_move = data
        self.is_request_move = True
        
        if self.is_first_callback == True:
            self.len_tranjectory = len(self.req_move.list_x)

            # add first point
            self.path_plan.header.frame_id = self.origin_frame
            self.path_plan.header.stamp = rospy.Time.now()
            self.path_plan.poses.append(self.point_path(self.agv_start_pose_x, self.agv_start_pose_y))
            self.path_plan.poses.append(self.point_path(self.req_move.list_x[0], self.req_move.list_y[0]))

            for i in range(self.nb_point_path + 1):
                x, y = self.getPoint_curveFormula(self.path_t, self.req_move.list_x[0], self.req_move.list_y[0], \
                                                                                            self.req_move.list_x[1], self.req_move.list_y[1],\
                                                                                                self.req_move.list_x[2], self.req_move.list_y[2])
                # print(self.path_t, x, y)
                # self.point_path_x.append(x)
                # self.point_path_y.append(y)               

                self.path_plan.poses.append(self.point_path(x, y))
                self.path_t = self.path_t + self.path_delta

            # print(self.point_path_x)

            # print(len(self.point_path_x))
            self.pub_path_global.publish(self.path_plan)
            self.is_first_callback = False 

    def log_mess(self, typ, mess, val):
        if self.pre_mess != mess:
            if typ == "info":
                rospy.loginfo (mess + ": %s", val)
            elif typ == "warn":
                rospy.logwarn (mess + ": %s", val)
            else:
                rospy.logerr (mess + ": %s", val)
        self.pre_mess = mess

    def point_path(self, x, y):
        point = PoseStamped()
        # point.header.frame_id = self.origin_frame
        # point.header.stamp = rospy.Time.now()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0
        point.pose.orientation.x = 0
        point.pose.orientation.y = 0
        point.pose.orientation.z = 0.
        point.pose.orientation.w = 1.0
        return point
    
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
    
    # -- Hàm chuyển đổi hệ tọa độ của AGV làm gốc: 
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_rb_ht
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
        return _X_cv, _Y_cv
    
    # -- Hàm đưa ra  vận tốc góc của AGV (by myself)
    # def control_navigation(self, X_point_goal, Y_point_goal, vel_x):
    #     omega_th = 0.0
    #     l = sqrt(X_point_goal**2 + Y_point_goal**2)
    #     if Y_point_goal == 0:
    #         # print(Y_point_goal)
    #         Y_point_goal = 0.0001

    #     phi = atan2(X_point_goal,Y_point_goal)
    #     rest_phi = PI - 2*phi
    #     r = sin(phi)*l/sin(rest_phi)
    #     omega = vel_x/r        # vel : omega 

    #     if Y_point_goal > 0:
    #         omega_th = omega
    #     else:
    #         omega_th = -omega

    #     return omega_th

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
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):                     # tính khoảng cách giữa 2 điểm trên hệ trục tọa độ 
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)
    
    def ptgt(self, denlta_time, time_s, v_s, v_f):
        v_re = 0.0
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        if denlta_time_now <= denlta_time :
            v_re = v_s + (v_f-v_s)*denlta_time_now

        else:
            v_re = v_f

        return v_re
    
    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass
    
    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())

    # def animate_time(self, i):
    #     self.arr_t.append(next(self.index))
    #     self.arr_timeup.append(self.time_up)
    #     self.arr_lactime.append(self.lactime)

    #     plt.plot(self.arr_t, self.arr_timeup, label='Timeup')
    #     plt.plot(self.arr_t, self.arr_lactime, label='Lactime')

    #     plt.legend(loc='upper left')
    #     plt.tight_layout()
    def reset_all(self):
        self.arr_t.clear()
        self.arr_timeup.clear()
        self.arr_lactime.clear()
        self.arr_dnow.clear()
        self.arr_dorg.clear()
        self.arr_ratio.clear()
        self.i = 0.05
        self.first_delta = 0
        self.second_delta = self.delta_up
# -------------------------------------------------------------------------------------------------------------------------------
    
    def run(self):
        while not rospy.is_shutdown():
            # kiểm tra dữ liệu đầu vào
            if self.process == 1:
                c_k = 0
                if self.is_request_move == True:
                    c_k = c_k + 1

                else:
                    pass
                    # self.log_mess("warn","Wait command from STI_Control", c_k)

                if self.is_pose_robot == True:
                    c_k = c_k + 1
                else:
                    pass
                    # self.log_mess("warn","Wait data from STI_Getpose", c_k)
                
                if self.is_recv_NNinfoRespond == True:
                    c_k = c_k + 1
                else:
                    pass

                if c_k == 3:
                    rospy.loginfo("Completed wakeup ('_')")
                    self.process = 2
                                               
            elif self.process == 2:                       # kiểm tra lộ trình di chuyển

                if self.NN_infoRespond.mode == 1:
                    self.process = 2
                    # self.stop()

                elif self.NN_infoRespond.mode == 2:
                    if (self.poseRbMa.position.x != self.req_move.list_x[0] or self.poseRbMa.position.y != self.req_move.list_y[0]):
                        self.point_goal_start_x = self.poseRbMa.position.x
                        self.point_goal_start_y = self.poseRbMa.position.y
                        self.cur_goal_x = self.req_move.list_x[self.path_index]
                        self.cur_goal_y = self.req_move.list_y[self.path_index]
                        self.type_run = self.move_straight
                        self.process = 3
                        self.is_need_turn_step1 = 1
                    else:
                        self.process = 10

                # if self.path_index == 0:
                #     self.point_goal_start_x = self.poseRbMa.position.x
                #     self.point_goal_start_y = self.poseRbMa.position.y
                #     self.cur_goal_x = self.req_move.list_x[self.path_index]
                #     self.cur_goal_y = self.req_move.list_y[self.path_index]
                # else:
                #     self.point_goal_start_x = self.cur_goal_x
                #     self.point_goal_start_y = self.cur_goal_y
                #     self.cur_goal_x = self.req_move.list_x[self.path_index]
                #     self.cur_goal_y = self.req_move.list_y[self.path_index]

                    # print("Tao đã vô đây rồi nè !")
                # self.is_need_turn_step1 = 1
                # self.process = 3    

            # - quyết định AGV di chuyển theo đường cong hay đường thẳng. 
            elif self.process == 10:

                theta = self.getAngle_2nearbyPad(self.req_move.list_x[self.path_index], self.req_move.list_x[self.path_index],\
                                                    self.req_move.list_x[self.path_index+1], self.req_move.list_y[self.path_index+1], \
                                                        self.req_move.list_x[self.path_index+2], self.req_move.list_y[self.path_index+2])
                theta = theta*180/PI
                print(theta)
                
                if -5 < theta and theta < 5: # 2 đường này đang gần như thẳng với nhau >> cho phép di chuyển thẳng
                    self.type_run = self.move_straight
                    self.process = 2

                else:                        # 2 đường thẳng này đang vuông góc >> cho phép di chuyển cong
                    self.type_run = self.move_curve
                    self.process = 3
                    self.is_need_turn_step1 = 1
                    self.point_goal_start_x = self.poseRbMa.position.x
                    self.point_goal_start_y = self.poseRbMa.position.y
                    self.cur_goal_x = self.req_move.list_x[self.path_index + 1]
                    self.cur_goal_y = self.req_move.list_y[self.path_index + 1]

                    self.vel_x = self.vel_x3

                    self.first_target_x, self.first_target_y = self.getPoint_curveFormula(self.first_delta, self.point_goal_start_x,self.point_goal_start_y, \
                                                                                                self.req_move.list_x[1], self.req_move.list_y[1],\
                                                                                                    self.req_move.list_x[2], self.req_move.list_y[2])
                    self.second_target_x, self.second_target_y = self.getPoint_curveFormula(self.second_delta, self.point_goal_start_x,self.point_goal_start_y, \
                                                                                                self.req_move.list_x[1], self.req_move.list_y[1],\
                                                                                                    self.req_move.list_x[2], self.req_move.list_y[2])
                    self.d_org = self.calculate_distance(self.first_target_x, self.first_target_y, self.second_target_x, self.second_target_y)
                    print(self.d_org)
                    self.d_now = self.calculate_distance(self.poseRbMa.position.x, self.poseRbMa.position.y, self.second_target_x, self.second_target_y)
                    print(self.d_now)

            # -- Quay AGV về đường mục tiêu -- 
            elif self.process == 3: 
                if self.is_need_turn_step1 == 1:
                    a = self.poseRbMa.position.y - self.cur_goal_y                   # khoảng cách giữa điểm mục tiêu và vị trí hiện tại của AGV. 
                    b = self.cur_goal_x - self.poseRbMa.position.x
                    theta = self.find_angle_between(a, b, self.theta_rb_ht)          # trả về góc lệch giữa AGV và đường lộ trình 

                    gt = self.turn_ar(theta,self.tolerance_rot_step1,self.vel_rot_step1)
                    
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
                    if self.type_run == self.move_straight:
                        self.process = 4
                    elif self.type_run == self.move_curve:
                        self.process = 51
                        self.time_run = time.time()

            # -- Điều hướng AGV -- 
            elif self.process == 4:

                self.X_n, self.Y_n, self.a_qd, self.b_qd, self.c_qd, self.dis_hc = self.find_hc(self.point_goal_start_x,\
                                                                                                self.point_goal_start_y,\
                                                                                                self.cur_goal_x,\
                                                                                                self.cur_goal_y)
                
                self.theta = self.find_angle_between(self.a_qd, self.b_qd, self.theta_rb_ht)

                dist_ahead = self.dist_ahead_max
                self.x_td_goal, self.y_td_goal, self.kc_con_lai = self.find_point_goal(self.point_goal_start_x,\
                                                                        self.point_goal_start_y,\
                                                                        self.cur_goal_x,\
                                                                        self.cur_goal_y,\
                                                                        self.a_qd,self.b_qd,self.c_qd,\
                                                                        self.X_n,self.Y_n,\
                                                                        dist_ahead)

                self.distance_goal = self.fnCalcDistPoints(self.poseRbMa.position.x,\
                                                                    self.cur_goal_x,\
                                                                    self.poseRbMa.position.y,\
                                                                    self.cur_goal_y)
                
                # print("dis_hc= %s , x_now= %s, y_now= %s, distance_goal= %s, kc_conlai= %s" %(self.dis_hc, self.poseRbMa.position.x, self.poseRbMa.position.y, self.distance_goal, self.kc_con_lai))

                self.vel_x = self.vel_x2        # = 0.5

                v_x = 0.0                                                      
                if fabs(self.theta) > self.angle_find_vel:                      # nếu self.theta > PI/6, v_x = 0.3
                    v_x = self.min_vel_x_gh                                   

                elif round(fabs(self.theta),2) == 0.0:                          # nếu theta = 0, v_x = self.vel_x =0.6  trong trường hơp điện áp đầy 
                    v_x = self.vel_x

                else:
                    v_x = self.min_vel_x_gh + ((self.angle_find_vel - fabs(self.theta))/self.angle_find_vel)*(self.vel_x - self.min_vel_x_gh)

                if self.distance_goal <= self.tol_simple or self.kc_con_lai <= self.tol_simple:    # kiểm tra AGV gần tới điểm cuối và chuyển lộ trình tiếp theo 
                    self.stop()
                    self.process = 10                    # chuyển về process 2 để kiểm tra lộ trình cần di chuyển.
                    # self.path_index += 1
                    # del self.path_plan.poses[0]                     # xóa lộ trình cũ đi 
                    # self.pub_path_global.publish(self.path_plan)
                else:
                    self.v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal, v_x)
                #            
                # if self.path_index < self.len_tranjectory - 1:
                #     if self.distance_goal <= self.tol_simple or self.kc_con_lai <= self.tol_simple:    # kiểm tra AGV gần tới điểm cuối và chuyển lộ trình tiếp theo 
                #         self.stop()
                #         self.process = 2                    # chuyển về process 2 để kiểm tra lộ trình cần di chuyển.
                #         # self.path_index += 1
                #         del self.path_plan.poses[0]                     # xóa lộ trình cũ đi 
                #         self.pub_path_global.publish(self.path_plan)
                #     else:
                #         self.v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal, v_x) 

                # elif self.path_index == self.len_tranjectory - 1:
                #     if self.distance_goal <= self.tol_target or self.kc_con_lai <= self.tol_target:    # agv chay het diem
                #         self.stop()
                #         self.process = 50                    # 
                #         self.path_index = 0
                #         del self.path_plan.poses[0]                     
                #         self.pub_path_global.publish(self.path_plan)
                #     else:
                #         self.v_th_send = self.control_navigation(self.x_td_goal, self.y_td_goal, v_x)    

                if self.NN_infoRespond.mode == 1:       
                    self.process = 2
                    self.reset_all()

                twist = Twist()
                twist.linear.x = v_x
                twist.angular.z = self.v_th_send
                self.pub_cmdVel(twist,self.rate_cmdvel)

            # -- for distance range up is fixed 
            elif self.process == 51:
                self.Padlength = quad(self.getLength_curveFormula, self.first_delta, self.second_delta, args=(self.point_goal_start_x, self.point_goal_start_y, \
                                                                                    self.req_move.list_x[self.path_index+1], self.req_move.list_y[self.path_index+1], \
                                                                                        self.req_move.list_x[self.path_index+2], self.req_move.list_y[self.path_index+2]))
                # print(self.Padlength)
                self.time_up = self.Padlength[0] / self.vel_x
                self.time_up = self.time_up*1000 #+ self.lactime
                # print("thời gian tăng gốc là: ", self.time_up)

                self.r = self.getRadius_curveFormula(self.avg_delta_up, self.point_goal_start_x, self.point_goal_start_y,\
                                        self.req_move.list_x[self.path_index+1], self.req_move.list_y[self.path_index+1], \
                                            self.req_move.list_x[self.path_index+2], self.req_move.list_y[self.path_index+2])
                
                self.time_run = time.time()
                self.process = 5
                
            elif self.process == 5:          # di chuyển theo đường cong
                
                twist = Twist()
                twist.linear.x = self.vel_x
                twist.angular.z = self.vel_x / self.r
                self.pub_cmdVel(twist, self.rate_cmdvel)
                
                self.time_up = self.Padlength[0] / self.vel_x
                self.time_up = self.time_up*1000
                
                if (time.time() - self.time_getdata_ticks)*1000 > self.time_getdata:
                    self.arr_t.append(self.i)
                    self.arr_timeup.append(self.time_up)
                    self.arr_lactime.append(self.lactime)
                    self.arr_dnow.append(self.d_now)
                    self.arr_dorg.append(self.d_org)
                    self.arr_ratio.append(self.ratio_d_now)
                    self.time_getdata_ticks = time.time()
                    self.i = self.i + 0.05
                    # print(self.d_now)

                self.d_now = self.calculate_distance(self.poseRbMa.position.x, self.poseRbMa.position.y, self.second_target_x, self.second_target_y)
                # print(self.d_now)
                self.ratio_d_now = self.d_now/self.d_org
                self.err_ratio_d = 1 - self.ratio_d_now
                # print(self.ratio_d_now)
                kp = 80
                self.lactime = kp*self.err_ratio_d
                # self.lactime = 65
                self.time_up = self.time_up + self.lactime
                # print("thời gian tăng là: ", self.time_up)

                # -- for pub ball data
                self.ballstate.pathLength = self.Padlength[0]
                self.ballstate.timeup = self.time_up
                self.ballstate.radius = self.r
                self.ballstate.d_curr = self.d_now
                self.ballstate.d_org = self.d_org
                self.ballstate.d_ratio = self.ratio_d_now
                self.ballstate.lactime = self.lactime
                self.ballstate.v_linear = twist.angular.x
                self.ballstate.v_angular = twist.angular.z
                
                t = (time.time() - self.time_run)*1000
                # print(t)
                
                print(self.second_delta)
                if t > self.time_up:    # update parameter
                    # print("IM here -------------------------------------------------------------")
                    self.stop()
                    self.first_delta = self.second_delta
                    self.avg_delta_up = self.first_delta #+ self.delta_up/2
                    self.second_delta = self.second_delta + self.delta_up

                    self.first_target_x = self.second_target_x
                    self.first_target_y = self.second_target_y

                    self.second_target_x, self.second_target_y = self.getPoint_curveFormula(self.second_delta, self.point_goal_start_x, self.point_goal_start_y, \
                                                                                                self.req_move.list_x[1], self.req_move.list_y[1],\
                                                                                                    self.req_move.list_x[2], self.req_move.list_y[2])
                    self.d_org = self.calculate_distance(self.first_target_x, self.first_target_y, self.second_target_x, self.second_target_y)
                    self.time_run = time.time()
                    
                    self.process = 51
                
                if self.second_delta >= 1:     # robot run all of tranjectory
                    print("IM here -----------------------------------robot run done--------------------------")
                    self.stop()
                    self.process = 50
                    # self.subplot[0,0].plot(self.arr_t, self.arr_timeup, color='green', label='timeup')
                    # self.subplot[0,0].plot(self.arr_t, self.arr_lactime, color='orange', label='lactime')
                    # self.subplot[0,0].legend(loc='upper left')

                    # self.subplot[0,1].plot(self.arr_t, self.arr_dnow, color='green', label='dnow')
                    # self.subplot[0,1].plot(self.arr_t, self.arr_dorg, color='orange', label='dorg')
                    # self.subplot[0,1].legend(loc='upper left')

                    # self.subplot[1,1].plot(self.arr_t, self.arr_ratio, color='blue', label='ratio')
                    # self.subplot[1,1].legend(loc='upper left')

                    # plt.show()
                    self.reset_all()
                    # self.arr_dnow.clear()
                    # self.arr_t.clear()
                    # self.arr_timeup.clear()
                    # self.arr_lactime.clear()
                    # self.arr_dorg.clear()
                    # self.arr_ratio.clear()

                if self.NN_infoRespond.mode == 1:       
                    self.process = 2
                    self.reset_all()

            # AGV đã hoàn thành lộ trình cần di chuyển 
            elif self.process == 50:    
                self.stop()
                self.path_index = 0

                if self.NN_infoRespond.mode == 1:       
                    self.process = 2
                    self.reset_all()

                # print(self.agv_name,"đã hoàn thành !")

            self.ballstate.process = self.process
            self.pub_status.publish(self.ballstate)
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
	print('Program starting')
	program = goalControl()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()
