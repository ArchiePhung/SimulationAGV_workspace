#!/usr/bin/python3
# Author : PhucHoang 19/05/2022
# Chinh sua cho phu hop voi FGV 

import os 
import time
import rospy
import tf
from std_msgs.msg import String, Bool

import sys
import struct
import string
import roslib
import serial
import signal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, atan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import *
from message_pkg.msg import Parking_request, Parking_respond
import threading

class ParkingNAV(threading.Thread):
    def __init__(self, threadID):
    
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        # get param
        self.rate = rospy.get_param('~rate',40) 
        self.rate = rospy.Rate(self.rate)

        # Sub topic
        rospy.Subscriber('/parking_request', Parking_request, self.request_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/odometry', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.is_odom_rb = False
        self.odom_rb = Odometry()
        
        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_fvel', Velocities, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 15

        self.pub_ParkingRespond = rospy.Publisher('/parking_respond', Parking_respond, queue_size=20)
        # self.data_PubRespond = Parking_respond()
        self.timePubRespond = rospy.get_time()
        self.rate_pubRespond = 30

        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)
        self.auto_reset = 0

        # Tf
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        #avera
        self.odom_x_ht = 0.
        self.odom_y_ht = 0.
        self.odom_g = 0.

        self.step_moveForward = 0
        self.x_odom_start = 0.0
        self.y_odom_start = 0.0

        self.step_Rotary = 0
        self.angle_odom_start = 0.0

        self.min_vel = 0.03
        self.min_velFinish = 0.06
        self.min_rol = 0.1
        self.min_rolFinish = 0.15

        self.max_vel = 0.1
        self.max_rol = 0.3

        self.pubTransform = False
        self.process = 0

        self.ss_x = 0.0
        self.ss_y = 0.0
        self.ss_a = 0.0
        
        self.warn_agv = 0


        # self.poseThenTransform = Pose()

        self.time_waitTransfrom = time.time()
        self.time_startTransform = rospy.Time.now()
        self.is_transform = False

        self.stepGoWaitPoint = 0
        self.disWait = 1.45
        self.XCircle = 0.
        self.YCircle = 0.
        self.RadiusFollow = 0.
        
        self.A_Circle = 0.
        self.B_Circle = 0.
        self.C_Circle = 0.
        
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Velocities()) 

    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True


    def getPose(self, data):
        self.poseStampedAGV = data
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

        self.is_pose_robot = True

    def cbGetRobotOdom(self, robot_odom_msg):
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,\
                        robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.odom_x_ht = robot_odom_msg.pose.pose.position.x
        self.odom_y_ht = robot_odom_msg.pose.pose.position.y

        self.odom_g = theta
        self.is_odom_rb = True

    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass
        
    def pub_cmdVelParking(self, twist, rate):
        vel = Velocities()
        vel.selectMode = 2
        vel.twist = twist
        
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(vel)
        else :
            pass
        
    def zone_callback(self, data):
        self.zone_lidar = data
        self.is_check_zone = True

    def pub_Stop(self):
        for i in range(0,3,1):
            self.pub_cmd_vel.publish(Velocities())
    
    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def Meaning(self, stt, war):
        mess_step = {
            0: 'Step: Wait All data',
            1: 'Step: Select Mode',
            21: 'Tinh quang duong di chuyen',
            -21: 'Di chuyen vao duong thang target',
            -210: 'Quay goc truoc khi tinh chinh khoang cach neu goc di chuyen lon',
            31: 'Tinh goc can quay',
            -31: 'Quay vao duong thang target',
            40: 'Den diem cho',
            41: 'Parking vao ke',
            51: 'Doi reset',
        }
        mess_warn = {
            0: 'AGV di chuyen binh thuong :)',
            1: 'AGV gap vat can :(',
            2: 'AGV gap loi chuong trinh',
        }
        mess = mess_step[stt] + ' | ' + mess_warn[war]
        return mess 

    def pub_Status(self, stt, modeRun, pose, offset, ss_x, ss_y, ss_a, meaning, warn):
        mess = Parking_respond()
        mess.status = stt
        mess.warning = warn
        mess.modeRun = modeRun
        mess.poseTarget = pose
        mess.offset = offset
        mess.ss_x = ss_x
        mess.ss_y = ss_y
        mess.ss_a = ss_a
        mess.message = meaning

        if rospy.get_time() - self.timePubRespond > float(1/self.rate_pubRespond) : # < 20hz 
            self.timePubRespond = rospy.get_time()
            self.pub_ParkingRespond.publish(mess)
        else :
            pass


    def sendTransform(self, frame_world, frame_id_pointTarget, point_target):
        self.time_startTransform = rospy.Time.now()
        self.tf_broadcaster.sendTransform((point_target.position.x, point_target.position.y, -1.0),
                                        (0.0, 0.0, point_target.orientation.z, point_target.orientation.w),
                                        self.time_startTransform,
                                        frame_id_pointTarget,
                                        frame_world)

    def transformPoseNAV(self, frame_id_pointTarget, frame_agvNAV):
        poseThenTransform = Pose()
        try:
            self.tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, rospy.Time(), rospy.Duration(10))
            pos, orien = self.tf_listener.lookupTransform(frame_id_pointTarget, frame_agvNAV, rospy.Time())
            poseThenTransform.position.x = pos[0]
            poseThenTransform.position.y = pos[1]
            poseThenTransform.position.z = pos[2]
            poseThenTransform.orientation.x = orien[0]
            poseThenTransform.orientation.y = orien[1]
            poseThenTransform.orientation.z = orien[2]
            poseThenTransform.orientation.w = orien[3]
            self.ss_x = pos[0]
            self.ss_y = pos[1]
            quaternion = (orien[0], orien[1], orien[2], orien[3])
            self.ss_a = 180 - fabs((tf.transformations.euler_from_quaternion(quaternion)[2]))*(180/PI)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass

        return poseThenTransform


    def move_forward(self, S, direct, velocity_min, permission_tolerance):
        twist = Twist()
        if self.step_moveForward == 0:
            self.x_odom_start = self.odom_x_ht
            self.y_odom_start = self.odom_y_ht
            self.step_moveForward = 1

        elif self.step_moveForward == 1:
            if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
                S_moved = self.calculate_distance(self.odom_x_ht, self.odom_y_ht, self.x_odom_start, self.y_odom_start)
                if fabs(S_moved - S) >= permission_tolerance and S_moved - S <= permission_tolerance:
                    twist.angular.z = 0
                    # vel = velocity_max*(fabs(S_moved - S)/S)
                    vel = velocity_min
                    # if vel <= velocity_min:
                    #     vel = velocity_min
                    if direct == 1: # tien
                        twist.linear.x = vel
                    elif direct == 2: # lui
                        twist.linear.x = vel*(-1)
                    print("S= %s , S_moved= %s, direct= %s" %(S, S_moved, direct))
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                    return 0

                else:
                    self.pub_Stop()
                    self.step_moveForward = 0
                    return 1
            else:
                if self.req_parking.modeRun == 3:
                    # print("Recieve data Stop!")
                    self.pub_cmdVelParking(Twist(), self.rate_pubVel)
                elif self.req_parking.modeRun == 0:
                    # print("Recieve data Reset!")
                    self.resetAll()
                    self.pub_Stop()
                return 0

        else:
            return 0

    def rotary_around(self, Angle, direct, velocity_min, velocity_max, Angle_StartDecel, permission_tolerance):
        twist = Twist()
        if self.step_Rotary == 0:
            self.angle_odom_start = self.odom_g
            self.step_Rotary = 1
        elif self.step_Rotary == 1:
            if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
                Angle_turned = self.odom_g - self.angle_odom_start
                if fabs(Angle_turned) > PI:
                    Angle_turned = 2*PI - fabs(Angle_turned)
                else:
                    Angle_turned = fabs(Angle_turned)
                if fabs(Angle_turned - Angle) >= permission_tolerance and Angle_turned - Angle <= permission_tolerance:
                    twist.linear.x = 0
                    vel = 0.0
                    if Angle >= PI/9: # PI/6
                        if fabs(Angle_turned - Angle) <= Angle_StartDecel:
                            vel = velocity_max*(fabs(Angle_turned - Angle)/Angle)
                            if vel <= velocity_min:
                                vel = velocity_min
                        else:
                            vel = velocity_max

                    else:
                        vel = velocity_min
                            
                    if direct == 1: # quay trai
                        twist.angular.z = vel
                    elif direct == 2: # quay phai
                        twist.angular.z = vel*(-1)

                    print("Angle= %s , Angle_turned= %s, direct= %s" %(Angle, Angle_turned, direct))
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                    return 0  

                else:
                    self.pub_Stop()
                    self.step_Rotary = 0
                    return 1
            else:
                if self.req_parking.modeRun == 3:
                    # print("Recieve data Stop!")
                    self.pub_cmdVelParking(Twist(), self.rate_pubVel)
                elif self.req_parking.modeRun == 0:
                    # print("Recieve data Reset!")
                    self.resetAll()
                    self.pub_Stop()
                return 0

        else:
            return 0
        
    def checkN(self, temp):
        if temp > 0:
            return 1
        elif temp < 0:
            return -1
        else:
            return 0
        
    def calptCircle(self, _X_c, _Y_c, _R):
        a_c = -2*_X_c 
        b_c =-2*_Y_c 
        c_c =_X_c*_X_c + _Y_c*_Y_c - _R*_R
        
        return a_c, b_c, c_c
    
    def calAngleThreePoint(self, x1, y1, x2, y2, x3, y3):
        dx1 = x1 - x2
        dy1 = y1 - y2
        dx2 = x3 - x2
        dy2 = y3 - y2
        c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
        goc = acos(c_goc)
        
        return goc
    
    def convert_relative_coordinates(self, X_cv, Y_cv, X_rb, Y_rb, Theta_rb):
        angle = -Theta_rb
        _X_cv = (X_cv - X_rb)*cos(angle) - (Y_cv - Y_rb)*sin(angle)
        _Y_cv = (X_cv - X_rb)*sin(angle) + (Y_cv - Y_rb)*cos(angle)
        
        return _X_cv, _Y_cv
    
    def find_point_goal_modeCircle(self, a_r, b_r, c_r, a_c, b_c, c_c, x_circle, y_circle, x_finish, y_finish, x_rb, y_rb, theta_rb):
        # print('ar = %f, br = %f, cr = %f, ac = %lf, bc = %f, cc = %f' %(a_r, b_r, c_r, a_c, b_c, c_c))
        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        # tinh goal ahead nam tren duong tron quy dao
        la = 1+((a_r-a_c)/(b_r-b_c))*((a_r-a_c)/(b_r-b_c))
        lb = -2*((c_c-c_r)/(b_r-b_c))*((a_r-a_c)/(b_r-b_c)) + a_c - b_c*((a_r-a_c)/(b_r-b_c))
        lc = ((c_c-c_r)/(b_r-b_c))*((c_c-c_r)/(b_r-b_c)) + b_c*((c_c-c_r)/(b_r-b_c)) + c_c
        denlta=lb*lb-4*la*lc
        # print('la = %f, lb = %f, lc = %f, denlta = %lf' %(la,lb,lc,denlta))

        X_g1=(-lb+sqrt(denlta))/(2*la)
        Y_g1=((c_c-c_r)/(b_r-b_c)) - X_g1*((a_r-a_c)/(b_r-b_c))
    
        X_g2=(-lb-sqrt(denlta))/(2*la)
        Y_g2=((c_c-c_r)/(b_r-b_c)) - X_g2*((a_r-a_c)/(b_r-b_c))
        
        _theta1 = self.calAngleThreePoint(X_g1, Y_g1, x_circle, y_circle, x_finish, y_finish)
        _theta2 = self.calAngleThreePoint(X_g2, Y_g2, x_circle, x_circle, x_finish, y_finish)
        
        if _theta1 < _theta2:
            X_g = X_g1
            Y_g = Y_g1
            
        else:
            X_g = X_g2
            Y_g = Y_g2
        
        x_cv, y_cv = self.convert_relative_coordinates(X_g, Y_g, x_rb, y_rb, theta_rb)
        return x_cv, y_cv
    
    def control_navigation(self, X_point_goal, Y_point_goal, vel_x):
        vel_th = 0.0
        l = (X_point_goal*X_point_goal) + (Y_point_goal*Y_point_goal)
        if Y_point_goal == 0:
            Y_point_goal = 0.0001

        r = l/(2*fabs(Y_point_goal))
        vel = vel_x/r

        if Y_point_goal < 0:
            vel_th = vel
        else:
            vel_th = -vel

        return vel_th
        
    def gotoWaitPoint(self, dis_tolerance, ang_tolerance, velocity_max, vel_rotMax, d_endCricle):
        twist = Twist()
        x_WaitPoint = -self.disWait
        y_WaitPoint = 0
        distance_ahead = 0.2
        poseThenTransform = Pose()
        poseThenTransform = self.transformPoseNAV('frame_target', 'frame_robot')
        
        if poseThenTransform.position.x == 0.0 and poseThenTransform.position.y == 0.0 and poseThenTransform.orientation.z == 0.0 and poseThenTransform.orientation.w == 0.0 :
            # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
            # self.warn_agv = 2
            return 2
        else:
            euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                            poseThenTransform.orientation.y,\
                                            poseThenTransform.orientation.z,\
                                            poseThenTransform.orientation.w))
            angleAGV = euler[2]
            
            distance = self.calculate_distance(poseThenTransform.position.x, poseThenTransform.position.y, x_WaitPoint, y_WaitPoint)
            _dis = sqrt(poseThenTransform.position.y*poseThenTransform.position.y + distance_ahead*distance_ahead)
            d1 = fabs(poseThenTransform.position.x - x_WaitPoint)
            if poseThenTransform.position.x - x_WaitPoint > 0:
                _stop = True
            else:
                _stop = False
                
            if fabs(angleAGV) >  ang_tolerance and _stop == False:
                kpd1 = 0.4
                vel_x = kpd1*_dis
                if vel_x > velocity_max:
                    vel_x = velocity_max
                
                twist.linear.x = 0.1*(-1)
                
                # angle = PI - fabs(angleAGV)
                # kpd2 = 0.7
                # kpg = 0.5   # 0.7
                # vel_rot = kpg*angle + kpd2*fabs(poseThenTransform.position.y)
                # # vel_rot = kpg*angle
                # # angle = fabs(atan2(poseThenTransform.position.y-y_WaitPoint,poseThenTransform.position.x-x_WaitPoint))
                # # vel_rot = kpg*angle
                
                angle_folow = atan(fabs(poseThenTransform.position.y)/d1)
                if poseThenTransform.position.y > 0 and angleAGV > 0:
                    if angleAGV > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 1
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 2
                elif poseThenTransform.position.y > 0 and angleAGV < 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 2
                elif poseThenTransform.position.y < 0 and angleAGV < 0:
                    if fabs(angleAGV) > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 2
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 1
                elif poseThenTransform.position.y < 0 and angleAGV > 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 1
                    
                kpg = 0.7  # 2. #1.5 #1.55
                kpd = 0.5
                vel_rot = kpg*angle + kpd*fabs(poseThenTransform.position.y)
                
                if vel_rot >= vel_rotMax:
                    vel_rot = vel_rotMax
                
                if directMode1 == 1:
                    twist.angular.z = vel_rot
                elif directMode1 == 2:
                    twist.angular.z = vel_rot*(-1)
                    
                # if angleAGV > 0:
                #     twist.angular.z = vel_rot
                # else:
                #     twist.angular.z = vel_rot*(-1)
                    
                print("distance= %s , angleAGV= %s" %(distance, angle))                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                self.pub_cmdVelParking(twist, self.rate_pubVel)
                return 0
            
            else:
                self.pub_Stop()
                return 1
            
    def gotoWaitPoint_vs2(self, dis_tolerance, ang_tolerance, velocity_max, vel_rotMax, d_endCricle):
        twist = Twist()
        x_WaitPoint = -self.disWait
        y_WaitPoint = 0
        x_endCircle = -d_endCricle
        y_endCircle = 0
        distance_ahead = 0.2
        poseThenTransform = Pose()
        poseThenTransform = self.transformPoseNAV('frame_target', 'frame_robot')
        
        if poseThenTransform.position.x == 0.0 and poseThenTransform.position.y == 0.0 and poseThenTransform.orientation.z == 0.0 and poseThenTransform.orientation.w == 0.0 :
            # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
            # self.warn_agv = 2
            return 2
        else:
            
            euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                            poseThenTransform.orientation.y,\
                                            poseThenTransform.orientation.z,\
                                            poseThenTransform.orientation.w))
            angleAGV = euler[2]
            x_rb = poseThenTransform.position.x
            y_rb = poseThenTransform.position.y
            
            if self.stepGoWaitPoint == 0:
                    # quay goc tai cho
                self.stepGoWaitPoint = 1
                
            elif self.stepGoWaitPoint == 2:
                # tim duong tron follow
                _k = tan(angleAGV)*(-1)
                b = y_rb - _k*x_rb
                self.XCircle = x_endCircle
                self.YCircle = _k* x_circle + b
                self.RadiusFollow = fabs(Y_Circle)
                rospy.logwarn("XCircle = %s |YCircle = %s |RadiusFollow = %s", self.XCircle, self.YCircle, self.RadiusFollow)
                self.A_Circle, self.B_Circle, self.C_Circle = self.calptCircle(self.XCircle, self.YCircle, self.RadiusFollow)
                self.stepGoWaitPoint = 3
                
            elif self.stepGoWaitPoint == 3:
                #tim goal follow   
                disOne = self.calculate_distance(x_rb, y_rb, x_endCircle, y_endCircle)
                if distance_ahead >= disOne:
                    self.stepGoWaitPoint = 4
                else:
                    A_CRobot, B_CRobot, C_CRobot = self.calptCircle(x_rb, y_rb, distance_ahead)
                    x_fl, y_fl = self.find_point_goal_modeCircle(A_CRobot, B_CRobot, C_CRobot, self.A_Circle, self.B_Circle, self.C_Circle, self.XCircle, self.YCircle, x_endCircle, y_endCircle, x_rb, y_rb, angleAGV)
                    vel_x = 0.15
                    vel_th = self.control_navigation(x_fl, y_fl, vel_x)
                    
                    twist.linear.x = (-1)*vel_x
                    twist.angular.z = vel_th
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                    
            if self.stepGoWaitPoint == 4:
                if poseThenTransform.position.x - x_WaitPoint > 0:
                    _stop = True
                else:
                    _stop = False
                    
                if fabs(angleAGV) >  ang_tolerance and _stop == False:
                    twist.linear.x = 0.1*(-1)
                    
                    angle_folow = atan(fabs(poseThenTransform.position.y)/d1)
                    if poseThenTransform.position.y > 0 and angleAGV > 0:
                        if angleAGV > angle_folow:
                            angle = PI - angle_folow - fabs(angleAGV)
                            directMode1 = 1
                        else:
                            angle = angle_folow + fabs(angleAGV) - PI
                            directMode1 = 2
                    elif poseThenTransform.position.y > 0 and angleAGV < 0:
                        angle = PI + angle_folow - fabs(angleAGV)
                        directMode1 = 2
                    elif poseThenTransform.position.y < 0 and angleAGV < 0:
                        if fabs(angleAGV) > angle_folow:
                            angle = PI - angle_folow - fabs(angleAGV)
                            directMode1 = 2
                        else:
                            angle = angle_folow + fabs(angleAGV) - PI
                            directMode1 = 1
                    elif poseThenTransform.position.y < 0 and angleAGV > 0:
                        angle = PI + angle_folow - fabs(angleAGV)
                        directMode1 = 1
                        
                    kpg = 0.7  # 2. #1.5 #1.55
                    kpd = 0.5
                    vel_rot = kpg*angle + kpd*fabs(poseThenTransform.position.y)
                    
                    if vel_rot >= vel_rotMax:
                        vel_rot = vel_rotMax
                    
                    if directMode1 == 1:
                        twist.angular.z = vel_rot
                    elif directMode1 == 2:
                        twist.angular.z = vel_rot*(-1)
                        
                    print("distance= %s , angleAGV= %s" %(distance, angle))                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                
                else:
                    self.pub_Stop()
                    return 1
                
            return 0                  
                
    def follow_target_vs2(self, velocity_min, velocity_max, vel_rotMax, distance_decel, distance_ahead, permission_tolerance):
        vel_x = 0.0
        vel_rot = 0.0
        twist = Twist()
        selectAngle = 0
        directMode1 = 0 # dung voi truong hop selectAngle = 1 | 1 quay trai, 2 quay phai
        poseThenTransform = Pose()
        poseThenTransform = self.transformPoseNAV('frame_target', 'frame_robot')
        print(poseThenTransform)
        if poseThenTransform.position.x == 0.0 and poseThenTransform.position.y == 0.0 and poseThenTransform.orientation.z == 0.0 and poseThenTransform.orientation.w == 0.0 :
            # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
            self.warn_agv = 2
            return 2
        else:

            euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                            poseThenTransform.orientation.y,\
                                            poseThenTransform.orientation.z,\
                                            poseThenTransform.orientation.w))
            angleAGV = euler[2]
            distance = sqrt(poseThenTransform.position.x*poseThenTransform.position.x + poseThenTransform.position.y*poseThenTransform.position.y)
            angle = 0.0
            if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
                if distance >= permission_tolerance and poseThenTransform.position.x < permission_tolerance: # dieu kien den target
                    # if self.zone_lidar.zone_sick_behind == 1 and self.req_parking.modeRun == 1:
                    #     self.warn_agv = 1
                    #     # print("co vat can")
                    #     self.pub_Stop()
                        
                    # else:
                    self.warn_agv = 0
                    if distance <= distance_decel: # tinh van toc dai theo khoang cach
                        vel_x = velocity_max*(distance/distance_decel)
                        if vel_x >= velocity_max:
                            vel_x = velocity_max
                        if vel_x <= velocity_min:
                            vel_x = velocity_min

                    else:
                        vel_x = velocity_max

                    twist.linear.x = vel_x*(-1)
                    angle_folow = atan(fabs(poseThenTransform.position.y)/distance_ahead)
                    if fabs(poseThenTransform.position.x) <= distance_decel: # tinh van toc goc
                        angle = PI - fabs(angleAGV)
                        selectAngle = 1

                        kg = 0.4   # 0.7
                        vel_rot = kg*angle
                        if vel_rot >= vel_rotMax:
                            vel_rot = vel_rotMax

                    else:
                        angle_folow = atan(fabs(poseThenTransform.position.y)/distance_ahead)
                        if poseThenTransform.position.y > 0 and angleAGV > 0:
                            if angleAGV > angle_folow:
                                angle = PI - angle_folow - fabs(angleAGV)
                                directMode1 = 1
                            else:
                                angle = angle_folow + fabs(angleAGV) - PI
                                directMode1 = 2
                        elif poseThenTransform.position.y > 0 and angleAGV < 0:
                            angle = PI + angle_folow - fabs(angleAGV)
                            directMode1 = 2
                        elif poseThenTransform.position.y < 0 and angleAGV < 0:
                            if fabs(angleAGV) > angle_folow:
                                angle = PI - angle_folow - fabs(angleAGV)
                                directMode1 = 2
                            else:
                                angle = angle_folow + fabs(angleAGV) - PI
                                directMode1 = 1
                        elif poseThenTransform.position.y < 0 and angleAGV > 0:
                            angle = PI + angle_folow - fabs(angleAGV)
                            directMode1 = 1
                        selectAngle = 2

                        kpg = 0.7  # 2. #1.5 #1.55
                        kpd = 0.5
                        vel_rot = kpg*angle + kpd*fabs(poseThenTransform.position.y)
                        if vel_rot >= vel_rotMax:
                            vel_rot = vel_rotMax

                    if selectAngle == 1:
                        if angleAGV > 0:
                            twist.angular.z = vel_rot
                        else:
                            twist.angular.z = vel_rot*(-1)

                    else:
                        if directMode1 == 1:
                            twist.angular.z = vel_rot
                        elif directMode1 == 2:
                            twist.angular.z = vel_rot*(-1)
                    lech = PI - fabs(angleAGV)
                    # print("MODE = %s ,Distan= %s, X= %s , goc_lech= %s, goc_follow= %s" %(selectAngle, poseThenTransform.position.y, poseThenTransform.position.x, lech, angle))
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                    return 0

                else:
                    self.pub_Stop()
                    return 1

            else:
                if self.req_parking.modeRun == 3:
                    # print("Recieve data Stop!")
                    self.pub_cmdVelParking(Twist(), self.rate_pubVel)
                elif self.req_parking.modeRun == 0:
                    # print("Recieve data Reset!")
                    self.resetAll()
                    self.pub_Stop()
                return 0

    def resetAll(self):
        self.process = 1
        self.pubTransform = False
        self.is_request_parking = False
        self.time_waitTransfrom = time.time()
        self.step_moveForward = 0
        self.step_Rotary = 0
        self.warn_agv = 0

    def run(self):
        SGo_forward = 0.0
        AGO_rotary = 0.0
        direct_forward = 0 # 1 tien, 2 lui
        direct_rotary = 0 # 1 quay trai, 2 quay phai
        poseThenTransform = Pose()
        while not self.shutdown_flag.is_set():

            if self.process == 0:
                # print("wait receive all need data......")
                c_k = 0
                if self.is_pose_robot == True:
                    c_k = c_k + 1
                # if self.is_odom_rb == True:
                #     c_k = c_k + 1
                # if self.is_check_zone == True:
                #     c_k = c_k + 1
                if c_k == 1:
                    self.process = 1
                    # print("Done receive all need data!")

            elif self.process == 1: # cho tin hieu Parking
                if self.is_request_parking == True:
                    if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2: #Run
                        print("Received data request Parking, Start Process!")
                        self.time_waitTransfrom = time.time()
                        self.pubTransform = True
                        self.process = 40

            elif self.process == 21: # tinh quang duong di chuyen
                
                if time.time() - self.time_waitTransfrom <= 1.5:
                    poseThenTransform = self.transformPoseNAV('frame_target', 'frame_robot')

                else:
                    if poseThenTransform.position.x == 0.0 and poseThenTransform.position.y == 0.0 and poseThenTransform.orientation.z == 0.0 and poseThenTransform.orientation.w == 0.0 :
                        # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
                        self.process = 52
                        self.pub_Stop()

                    else:
                        if fabs(poseThenTransform.position.y) <= 0.02:
                            self.time_waitTransfrom = time.time()
                            self.process = 31 # chuyen sang buoc tinh goc quay de lui vao ke
                        else:
                            print(poseThenTransform)
                            euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                                            poseThenTransform.orientation.y,\
                                                            poseThenTransform.orientation.z,\
                                                            poseThenTransform.orientation.w))
                            angle = euler[2]

                            if fabs(angle) < PI/4 or fabs(angle) > (3*PI)/4:   # truong hop AGV lech goc lon
                                AGO_rotary = fabs((PI/2) - fabs(angle))
                                if (angle > 0 and angle < PI/2) or (angle < 0 and fabs(angle) > PI/2):
                                    direct_rotary = 1
                                else:
                                    direct_rotary = 2

                                self.step_moveForward = 0
                                self.process = -210

                            else:
                                if fabs(angle) > PI/2:
                                    SGo_forward = fabs(poseThenTransform.position.y)/sin(PI - fabs(angle))
                                else:
                                    SGo_forward = fabs(poseThenTransform.position.y)/sin(fabs(angle))

                                if (poseThenTransform.position.y > 0 and angle > 0) or (poseThenTransform.position.y < 0 and angle < 0):
                                    direct_forward = 2
                                else:
                                    direct_forward = 1

                                self.step_moveForward = 0
                                self.process = -21

            elif self.process == -21: # di chuyen vao duong thang target
                if self.move_forward(SGo_forward, direct_forward, self.min_vel, 0) == 1:
                    self.time_waitTransfrom = time.time()
                    self.process = 21 # kiem tra lai xem da chinh xac chua

            elif self.process == -210: # quay goc truoc khi tinh chinh khoang cach neu goc di chuyen lon
                if self.rotary_around(AGO_rotary, direct_rotary, self.min_rol, self.max_rol, PI/5.0, 0.015) == 1:
                    self.time_waitTransfrom = time.time()
                    self.process = 21 # kiem tra lai xem da chinh xac chua

            elif self.process == 31: # tinh goc can quay
                if time.time() - self.time_waitTransfrom <= 1.5:
                    poseThenTransform = self.transformPoseNAV('frame_target', 'frame_robot')

                else:
                    print(poseThenTransform)
                    if poseThenTransform.position.x == 0.0 and poseThenTransform.position.y == 0.0 and poseThenTransform.orientation.z == 0.0 and poseThenTransform.orientation.w == 0.0 :
                        # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
                        self.process = 52
                        self.pub_Stop()
                    else:

                        euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                                        poseThenTransform.orientation.y,\
                                                        poseThenTransform.orientation.z,\
                                                        poseThenTransform.orientation.w))
                        angle = euler[2]
                        print(PI - fabs(angle))
                        if (PI - fabs(angle)) <= 0.04:
                            self.process = 41 # chuyen sang buoc parking

                        else:
                            AGO_rotary = PI - fabs(angle)
                            if angle > 0:
                                direct_rotary = 1
                            else:
                                direct_rotary = 2

                            self.step_Rotary = 0
                            self.process = -31
                            
            elif self.process == -31: # quay vao duong thang target
                if self.rotary_around(AGO_rotary, direct_rotary, self.min_rol, self.max_rol, PI/6.0, 0.015) == 1:
                    self.time_waitTransfrom = time.time()
                    self.process = 31 # kiem tra lai xem da chinh xac chua
                    
            elif self.process == 40: # di chuyen den diem cho
                stt = self.gotoWaitPoint(0.0, 0.015, 0.1, 0.15)
                if stt == 1:
                    time.sleep(5.)
                    self.process = 41
                    
                elif stt == 2:
                    self.process = 52
                    self.pub_Stop()
                    
            elif self.process == 41: # parking vao ke
                # print("Start Parking!")
                # if self.follow_target(self.min_vel, self.max_vel, self.min_vel, 0.3, 0) == 1:
                # if self.follow_target(self.min_vel, self.max_vel, self.min_rol, 0.6, 0.4, 0) == 1:  # 0.5 | 0.35
                stt = self.follow_target_vs2(self.min_velFinish, self.max_vel, self.min_rolFinish, 0.55, 0.4, 0) #  0.5, 0.35, 0
                if stt == 1:  # 0.5 | 0.35
                    # print("Done Parking!")
                    time.sleep(0.5)
                    self.process = 51
                elif stt == 2:
                    # print("Value poseThenTransform is 0! Stop process, Wait for restart!")
                    self.process = 52
                    self.pub_Stop()

            elif self.process == 51: # wait for reset transform
                # print("Done!")
                if self.req_parking.modeRun == 0: #Reset
                    # self.req_parking = Parking_request()
                    # print("Recieve data Reset!")
                    self.resetAll()

            elif self.process == 52: # loi pose transform error
                if self.req_parking.modeRun == 0: #Reset
                    # self.req_parking = Parking_request()
                    # print("Recieve data Reset!")
                    self.resetAll()
                    
            mess_pub = self.Meaning(self.process, self.warn_agv)

            self.pub_Status(self.process, self.req_parking.modeRun, self.req_parking.poseTarget, self.req_parking.offset, self.ss_x, self.ss_y, self.ss_a, mess_pub, self.warn_agv)

            # print(process)
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

    rospy.init_node('Parking_NAV', anonymous=False)

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        j1 = ParkingNAV(1)
        j1.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            if j1.pubTransform == True and (j1.req_parking.modeRun == 1 or j1.req_parking.modeRun == 2):
                j1.sendTransform('frame_map_nav350', 'frame_target', j1.req_parking.poseTarget) # -- edit 03/03/2022: frame_map_nav350 frame_global_map
                j1.is_transform = True
            else:
                j1.is_transform = False
            time.sleep(0.01)
 
    except ServiceExit:
        
        j1.shutdown_flag.set()
        # Wait for the threads to close...
        j1.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()
