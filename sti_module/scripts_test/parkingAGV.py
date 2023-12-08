#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sti_msgs.msg import APathParking, PointOfPath, HC_info
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ParkingAGV():
    def __init__(self):
        rospy.init_node('parking_agv', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
        self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

        self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 0.1)
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.008)

        self.max_lookahead = rospy.get_param('~min_lookahead', 0.3)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.05)
        self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/parking_request', Parking_request, self.parkingRequest_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False

        rospy.Subscriber('/poseParking', PoseStamped, self.poseParking_callback)
        self.is_poseParking = False
        self.poseParking = Pose()
        self.poseParkingStamped = PoseStamped()

        rospy.Subscriber("/path_parking", APathParking, self.path_callback)
        self.is_path = False
        self.dataPath = APathParking()
        self.theta_AngentFisrtPoint = 0.

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 15

        rospy.on_shutdown(self.fnShutDown)

        self.process = 0

        self.min_vel = 0.03
        self.min_velFinish = 0.04
        self.min_rol = 0.1
        self.min_rolFinish = 0.15

        self.max_vel = 0.07
        self.max_rol = 0.3

        self.xFollow = 0.
        self.yFollow = 0.
        self.velFollow = 0.

        self.distDeceleration = 0.3
        self.disgetLocation = 0.15

        self.stepContourFollow = 0
        self.curr_velocity = 0.

    # function callback
    def callback_poseRobot(self, data):
        self.poseStampedAGV = data
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotNow = euler[2]

        self.is_pose_robot = True

    def parkingRequest_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True

    def poseParking_callback(self, data):
        self.poseParkingStamped = data
        self.poseParking = data.pose
        quata = ( self.poseParking.orientation.x,\
                self.poseParking.orientation.y,\
                self.poseParking.orientation.z,\
                self.poseParking.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotParking = euler[2]
        self.is_poseParking = True

    def path_callback(self, data):
        self.dataPath = data
        quata = ( self.dataPath.poseStart.orientation.x,\
                self.dataPath.poseStart.orientation.y,\
                self.dataPath.poseStart.orientation.z,\
                self.dataPath.poseStart.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_AngentFisrtPoint = euler[2]
        self.is_path = True

    def zone_callback(self, data):
        self.zone_lidar = data
        self.is_check_zone = True

    # function pub
    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def pub_Stop(self):
        for i in range(0,3,1):
            self.pub_cmd_vel.publish(Twist())

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    # function 
    def constrain(self, value_in, value_min, value_max):
        value_out = 0.0
        if value_in < value_min:
            value_out = value_min
        elif value_in > value_max:
            value_out = value_max
        else:
            value_out = value_in
        return value_out

    def rotary_around(self, thetaTarget, angular_velocity, permission_tolerance):
        twist = Twist()
        ss_theta = thetaTarget - self.theta_robotParking
        if fabs(ss_theta) > PI:
            newss_theta = 2*PI - fabs(ss_theta)
            if ss_theta > 0.:
                ss_theta = - newss_theta
            else:
                ss_theta = newss_theta

        if fabs(ss_theta) > permission_tolerance:
            if ss_theta > 0.: # quay trai
                twist.angular.z = angular_velocity
            else:
                twist.angular.z = -angular_velocity
            
            self.pub_cmdVel(twist, self.rate_pubVel)
            return 0
        
        self.pub_Stop()
        return 1
    
    def funcDecelerationByTime(self, denlta_time, time_s, v_s, v_f):
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        a = (v_f-v_s)/denlta_time

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = rospy.Time.now().to_sec() - time_s
        v_re = v_s + a*denlta_time_now
    
    def findLookAheadByVel(self, curr_velocity):
        lookahead = self.constrain(self.max_lookahead*curr_velocity/self.lookahead_ratio, self.min_lookahead, self.max_lookahead)
        return lookahead
    
    def getWaitPoint(self, xRb, yRb, curr_velocity):
        lookahead = self.constrain(self.max_lookahead*curr_velocity/self.lookahead_ratio, self.min_lookahead, self.max_lookahead)
        lookahead = 0.3
        kcToFakeGoal = self.calculate_distance(-0.1, 0., xRb, yRb)
        kc = self.calculate_distance(0., 0., xRb, yRb)
        if kcToFakeGoal <= lookahead:
            self.xFollow = -0.1
            self.yFollow = 0.
            self.velFollow = self.min_linearVelocity
            return True

        else:
            numpoint = len(self.dataPath.info)
            # print(numpoint)
            for i in range(numpoint - 1, -1, -1):
                x = self.dataPath.info[i].pose.position.x
                y = self.dataPath.info[i].pose.position.y
                # vel = self.dataPath.info[i].velocity
                d = self.calculate_distance(x, y, xRb, yRb)
                if d <= lookahead:
                    self.xFollow = x
                    self.yFollow = y

                self.velFollow = self.max_linearVelocity
                return True
            
        return False
    
    def followIntermediaryTarget(self):
        twist = Twist()
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        # angleAGV = self.theta_robotParking
        
        if self.getWaitPoint(poseX, poseY, self.curr_velocity):
            # kc = self.calculate_distance(self.xFollow, self.yFollow, poseX, poseY)
            ss_x = poseX
            ss_y = poseY
            print(ss_x, ss_y, self.velFollow)
            if (fabs(ss_x) <= 0.01 and fabs(ss_y) <= 0.01) or poseX <= 0.:
                self.pub_Stop()
                return 1
            
            else:
                velX = -self.velFollow
                xCVFollow , yCVFollow = self.convert_relative_coordinates(self.xFollow, self.yFollow)
                velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                twist.linear.x = velX
                twist.angular.z = velAng

                self.pub_cmdVel(twist, self.rate_pubVel)
                return 0
        
        else:
            return -1


    def SIGN(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        return 0
    
    def moveIntoPallet(self, velocity_min, velocity_max, vel_rotMax, distance_decel, distance_ahead, permission_tolerance):
        vel_x = 0.0
        vel_rot = 0.0
        twist = Twist()
        selectAngle = 0
        directMode1 = 0 # dung voi truong hop selectAngle = 1 | 1 quay trai, 2 quay phai
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        angleAGV = self.theta_robotParking
        distance = sqrt(poseX*poseX + poseY*poseY)
        angle = 0.0
        if distance >= permission_tolerance and poseX < permission_tolerance: # dieu kien den target
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
            angle_folow = atan(fabs(poseY)/distance_ahead)
            if fabs(poseY) <= 0.01 or fabs(poseX) <= distance_decel: # tinh van toc goc
                angle = PI - fabs(angleAGV)
                selectAngle = 1

                kg = 0.25   # 0.7
                vel_rot = kg*angle
                if vel_rot >= vel_rotMax:
                    vel_rot = vel_rotMax

            else:
                angle_folow = atan(fabs(poseY)/distance_ahead)
                if poseY > 0 and angleAGV > 0:
                    if angleAGV > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 1
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 2
                elif poseY > 0 and angleAGV < 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 2
                elif poseY < 0 and angleAGV < 0:
                    if fabs(angleAGV) > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 2
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 1
                elif poseY < 0 and angleAGV > 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 1
                selectAngle = 2

                kg = 0.7
                kd = 0.5
                vel_rot = kg*angle + kd*fabs(poseY)
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
            self.pub_cmdVel(twist, self.rate_pubVel)
            return 0
        
        else:
            self.pub_Stop()
            return 1
        
    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

        
    # function Navigation 
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_robotParking
        _X_cv = (X_cv - self.poseParking.position.x)*cos(angle) - (Y_cv - self.poseParking.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseParking.position.x)*sin(angle) + (Y_cv - self.poseParking.position.y)*cos(angle)
        return _X_cv, _Y_cv

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x): # 1 goal phia truoc | -1 goal phia sau
        vel_th = 0.0
        xGoal = X_point_goal
        yGoal = Y_point_goal

        if fabs(yGoal) <= 0.005:
            vel_th = 0.

        else:
            l = (xGoal*xGoal) + (yGoal*yGoal)
            r = l/(2*fabs(yGoal))
            # print(vel_x, r, l)
            vel = fabs(vel_x)/r
            if yGoal > 0:
                vel_th = vel*self.SIGN(vel_x)
            else:
                vel_th = -vel*self.SIGN(vel_x)

        return vel_th

    def control_naviTarget(self):
        pass



    def run(self):
        self.process = 1
        while not rospy.is_shutdown():    
            if self.process == 0:
                ck = 0
                if self.is_pose_robot:
                    ck += 1
                if self.is_check_zone:
                    ck += 1
                if ck == 2:
                    self.process = 1
                    rospy.loginfo("Done receive all need data <(^-^)> ")

            elif self.process == 1:
                if self.is_request_parking == True and (self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2):
                    print("recieve data Run Parking")
                    if  self.is_poseParking:
                        print("recieve data transfrom pose target")
                        if self.is_path:
                            print("recieve data local goal")
                            self.process = 2

            elif self.process == 2: # Quay dap ung goc
                if self.rotary_around(self.theta_AngentFisrtPoint, 0.1, 0.015):
                    print("DONE Rotary!")
                    rospy.sleep(0.5)
                    self.process = 3

            elif self.process == 3: # follow den diem target trung gian
                if self.followIntermediaryTarget():
                    print("Arrived at the Intermediary Target location")
                    rospy.sleep(0.5)
                    self.process = 4
            
            elif self.process == 4: # quay vuong goc vao lay hang
                if self.rotary_around(0. , 0.1, 0.015):
                    print("DONE Rotary!")
                    self.process = 5

            elif self.process == 5: # cho nang ha cang neu co
                rospy.sleep(2.)
                self.process = 7

            elif self.process == 6: # di chuyen vao pallet
                stt = self.moveIntoPallet(self.min_velFinish, self.max_vel, self.min_rolFinish, 0.5, 0.5, 0) #  0.5, 0.35, 0
                if stt:
                    self.process = 7

            elif self.process == 7:
                print("Done!")

                
            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = ParkingAGV()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()