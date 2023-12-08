#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sti_msgs.msg import APathParking, PointOfPath, HC_info, MotorDrive_respond, Velocities
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin, degrees, radians, acos
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


class InfoPathFollowing():
    def __init__(self):
        self.indexInListpath = 0.
        self.velocity = 0.
        self.radius = 0.
        self.X = 0.
        self.Y = 0.


class ParkingAGV():
    def __init__(self):
        rospy.init_node('parking_agv', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
        self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

        self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 0.8)
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.06)

        self.max_lookahead = rospy.get_param('~min_lookahead', 1.49)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.45)
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
        self.theta_robotParking = 0.

        rospy.Subscriber("/path_parking", APathParking, self.path_callback)
        self.is_path = False
        self.startPoint = Pose()
        self.intermediaryPoint = Pose()
        self.dataPath = APathParking()
        self.theta_AngentFisrtPoint = 0.

        rospy.Subscriber("/motorDrive_respond", MotorDrive_respond, self.motorDriveRespond_callback)
        self.respMotor = MotorDrive_respond() 
        self.is_MotorDrive_respond = False

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_fvel', Velocities, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 30

        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)
        rospy.on_shutdown(self.fnShutDown)

        self.pub_ParkingRespond = rospy.Publisher('/parking_respond', Parking_respond, queue_size=20)
        self.timePubRespond = rospy.get_time()
        self.rate_pubRespond = 30

        # param forklift
        self.offsetWheelwithMainAxis = rospy.get_param("offsetWheelwithMainAxis", 0.03)
        self.distanceBetwenOriginAndWheels = rospy.get_param("distanceBetwenOriginAndWheels", 1.285)
        self.radiusWheels = rospy.get_param("radiusWheels", 0.125)

        self.maxRPM_MainMotor = 3300
        self.maxAngleStreering = int(degrees(atan(self.distanceBetwenOriginAndWheels/self.offsetWheelwithMainAxis))*100)
        self.minAngleStreering = int(-9000 - degrees(atan(self.offsetWheelwithMainAxis/self.distanceBetwenOriginAndWheels))*100)
        self.minRPM_MainMotor = 440

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

        self.flagFollowPointFinish = False

        # self.listVel = [0.012, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
        # self.numTable = len(self.listVel)
        # self.listLookAhead = [0.12, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]

        self.listVel = [0.06, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
        self.numTable = len(self.listVel)
        self.listLookAhead = [0.45, 0.5, 0.54, 0.6, 0.65, 0.73, 0.75, 0.8, 0.9, 1.05, 1.28, 1.34, 1.39, 1.44, 1.49]

        self.infoPathFollow = InfoPathFollowing()
        self.saveTimeVel = rospy.get_time()
        self.statusVel = 0
        self.velSt = 0.

        self.stepTurnAR = 0
        self.ratioDeceleration = 8.
        self.isDeceleration = False

    # function callback
    def motorDriveRespond_callback(self, data):
        self.respMotor = data
        self.is_MotorDrive_respond = True

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
        self.startPoint = self.dataPath.poseStart
        self.intermediaryPoint = self.dataPath.poseIntermediary
        self.is_path = True

    def zone_callback(self, data):
        self.zone_lidar = data
        self.is_check_zone = True

    # function pub
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Velocities()) 

    def stop(self):
        self.curr_velocity = 0.
        for i in range(2):
            self.pub_cmd_vel.publish(Velocities())
        
    def pub_cmdVelIndividual(self, _mode, _angleRequest, _velRequest, rate): # 1 cho tung kenh, 2 cho song kenh
        vel = Velocities()
        vel.selectMode = _mode # 0: dk dong co chinh | 1: dk dong co lai 
        vel.angleRequest = int(_angleRequest)
        vel.velRequest = int(_velRequest)
        
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(vel)
        else :
            pass
    
    def pub_cmdVelParking(self, twist, rate):
        vel = Velocities()
        vel.selectMode = 2 # 2: dieu khien song song
        vel.twist = twist
        
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(vel)
        else :
            pass

    def pubMakerPointFollow(self, x, y):
        # Create rviz marker message
        marker = Marker()
        marker.header.frame_id = "frame_target"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.
        marker.pose.orientation.w = 1.
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        self.pubMarker.publish(marker)

    def pub_Status(self, stt, modeRun, pose, offset, ss_x, ss_y, ss_a, meaning, warn):
        mess = Parking_respond()
        mess.status = stt
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

    # forklift -------------------------------------------------------------
    def checkTurn(self, _Angle):
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
                    if fabs(theta) <= 30.:
                        _vel = (fabs(theta)/30.)*velMax
                        
                    else:
                        _vel = velMax
                        
                    if _vel < self.minRPM_MainMotor:
                        _vel = self.minRPM_MainMotor
                        
                else:
                    if fabs(theta) <= 30.:
                        _vel = (fabs(theta)/30.)*(-velMax)
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
    
    def funcDecelerationByTime(self, denlta_time, time_s, v_s, v_f):
        denlta_time_now = rospy.get_time() - time_s
        a = (v_f-v_s)/denlta_time

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
        if _safety != 0: # trang thai giam toc do vat can
            if _safety == 1:
                self.statusVel = 0
                self.stop()
                return 0.
            elif _safety == 2: 
                self.statusVel = 0
                return self.infoPathFollow.velocity*(1./3.)
            else:
                self.statusVel = 0
                return self.infoPathFollow.velocity*(2./3.)
            
        else:      
            if _statusVel == 0:
                return self.infoPathFollow.velocity
            elif _statusVel == 1:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, 0.12)
            elif _statusVel == 2:
                return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, self.infoPathFollow.velocity, -0.4)
            else:
                return 0.
            
    def findVelByLookAhead(self, l):
        a = 0.
        b = 0.
        if l >= self.max_lookahead:
            l = self.max_lookahead
        elif l <= self.min_lookahead:
            l = self.min_lookahead

        for i in range(self.numTable -1 ):
            if l >= self.listLookAhead[i] and l <= self.listLookAhead[i+1]:
                a = (self.listVel[i] - self.listVel[i+1])/(self.listLookAhead[i] - self.listLookAhead[i+1])
                b = self.listVel[i] - a*self.listLookAhead[i]

        return a*l + b
    
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
    
    def findGoalNearest(self, xRb, yRb):
        index = 0
        min = float('inf')
        numpoint = len(self.dataPath.info)
        for i in range(numpoint - 1, -1, -1):
            x = self.dataPath.info[i].pose.position.x
            y = self.dataPath.info[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)

            if dis <= min:
                min = dis
                index = i

        infoPoint = InfoPathFollowing()
        infoPoint.indexInListpath = index
        infoPoint.velocity = self.dataPath.info[index].velocity
        infoPoint.X = self.dataPath.info[index].pose.position.x
        infoPoint.Y = self.dataPath.info[index].pose.position.y

        return infoPoint, min
    
    def checkPathOutOfRange(self):
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        self.infoPathFollow, dmin = self.findGoalNearest(poseX, poseY)
        return True
    
    def getWaitPoint(self, xRb, yRb, curr_velocity):
        indexSelect = -1
        lookahead = self.findLookAheadByVel(curr_velocity)

        # distoTarget = fabs(xRb)
        # if self.flagFollowPointFinish == False and  distoTarget < 1.1:
        #     self.flagFollowPointFinish = True
        #     self.distDeceleration = distoTarget
        #     self.velSt = curr_velocity

        numpoint = len(self.dataPath.info)

        # find goal to goal finish
        for i in range(numpoint - 1,  -1, -1):
            x = self.dataPath.info[i].pose.position.x
            y = self.dataPath.info[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)
            if dis <= lookahead:
                indexSelect = i
                break

        if indexSelect == -1:
            return False
        
        else:
            self.infoPathFollow.indexInListpath = indexSelect

            self.infoPathFollow.X = self.dataPath.info[indexSelect].pose.position.x
            self.infoPathFollow.Y = self.dataPath.info[indexSelect].pose.position.y
        
            if self.statusVel != 2:
                vel = self.dataPath.info[indexSelect].velocity
                self.infoPathFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

            return True
    
    def followIntermediaryTarget(self):
        twist = Twist()
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        # angleAGV = self.theta_robotParking
        ss_x = poseX - self.intermediaryPoint.position.x
        ss_y = poseY - self.intermediaryPoint.position.y

        if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
            dis = sqrt(ss_x*ss_x + ss_y*ss_y)
            print(ss_x, ss_y, dis)
            if ((fabs(ss_x) <= 0.01 and fabs(ss_y) <= 0.01) or dis <= 0.015 or ss_x <= 0.) and self.flagFollowPointFinish:
                self.flagFollowPointFinish = False
                self.stop()
                return 2
            
            else:
                if self.flagFollowPointFinish:
                    self.curr_velocity = self.velSt*(dis/self.distDeceleration)
                    self.curr_velocity  = self.constrain(self.curr_velocity, self.min_linearVelocity, self.velSt)
                    print(self.curr_velocity, self.distDeceleration, self.infoPathFollow.velocity, "Follow Target Intermediary!")
                    if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                        velX = -self.curr_velocity
                        xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                        velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                        twist.linear.x = velX
                        twist.angular.z = velAng

                        self.pub_cmdVelParking(twist, self.rate_pubVel)
                        return 1
                    
                    else:
                        self.stop()
                        return -1

                else:
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
                            velX = -self.curr_velocity
                            xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                            velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                            twist.linear.x = velX
                            twist.angular.z = velAng

                            self.pub_cmdVelParking(twist, self.rate_pubVel)
                            return 1
                        
                        else:
                            self.stop()
                            return -1
                        
                    else:
                        return 0
                    
        else:
            if self.req_parking.modeRun == 3:
                # print("Recieve data Stop!")
                self.pub_cmdVelParking(Twist(), self.rate_pubVel)
            elif self.req_parking.modeRun == 0:
                # print("Recieve data Reset!")
                self.stop()
                self.resetAll()
            return 0
                           
    def moveIntoPallet(self):
        twist = Twist()
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        angleAGV = self.theta_robotParking

        dis = sqrt(poseX*poseX + poseY*poseY)

        if poseX < 0.005:
            self.stop()
            self.isDeceleration = False
            self.statusVel = 0
            return 2
        
        else:
            # ham chon van toc
            velX = 0
            distDeceleration = self.curr_velocity*self.ratioDeceleration
            if dis <= distDeceleration and self.isDeceleration == False:
                self.distDeceleration = dis
                self.isDeceleration = True
                self.velSt = self.curr_velocity

            if self.isDeceleration:
                self.curr_velocity = self.velSt*(dis/self.distDeceleration)
                self.curr_velocity  = self.constrain(self.curr_velocity, self.min_linearVelocity, self.velSt)

            else:
                if self.statusVel == 0:
                    self.saveTimeVel = rospy.get_time()
                    self.statusVel = 1
                
                if self.statusVel == 1: # tang toc
                    if self.curr_velocity < self.infoPathFollow.velocity:
                        self.curr_velocity = self.funcDecelerationByAcc(self.saveTimeVel, self.min_linearVelocity, self.infoPathFollow.velocity, 0.12)
                    else:
                        self.curr_velocity = self.infoPathFollow.velocity
                        self.statusVel = 2

                if self.statusVel == 2:
                    self.curr_velocity = self.infoPathFollow.velocity

            # ham xu ly
            if (fabs(poseY) <= 0.01 and fabs(angleAGV) <= radians(1.)) or fabs(poseX) <= 1.:
                if fabs(poseX) > 0.6:
                    print("Mode follow Angle 1")
                    velX = -self.curr_velocity
                    angleLine = self.angleStraightLine(poseX - 0.35, 0, poseX, poseY)
                    angleBW = self.angleBetweenRobotAndPath2(angleLine)
                    velX = -self.curr_velocity
                    kp = 0.4
                    kd = 0.
                    dentaAngle = angleBW
                    velAng = kp*fabs(dentaAngle)
                    # if velAng > 0.15:
                    #     velAng = 0.15
                    # velAng = kp*fabs(dentaAngle)
                    if dentaAngle > 0:
                        velAng = velAng
                    else:
                        velAng = -velAng

                    twist.linear.x = velX
                    twist.angular.z = velAng
                    self.pub_cmdVelParking(twist, self.rate_pubVel)

                else:
                    print("Mode follow Angle 2")
                    velX = -self.curr_velocity
                    kp = 0.4
                    kd = 0.
                    dentaAngle = -angleAGV
                    velAng = kp*fabs(dentaAngle) + kd*atan(fabs(poseY))
                    # if velAng > 0.15:
                    #     velAng = 0.15
                    # velAng = kp*fabs(dentaAngle)
                    if dentaAngle > 0:
                        velAng = velAng
                    else:
                        velAng = -velAng

                    twist.linear.x = velX
                    twist.angular.z = velAng
                    self.pub_cmdVelParking(twist, self.rate_pubVel)

            else:
                if self.getWaitPoint(poseX, poseY, self.curr_velocity):
                    print("Mode follow Point")
                    velX = -self.curr_velocity
                    xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                    velAng = self.control_navigation(xCVFollow, yCVFollow, velX)

                    twist.linear.x = velX
                    twist.angular.z = velAng
                    self.pub_cmdVelParking(twist, self.rate_pubVel)
                
                else:
                    self.stop()
                    return -1
                
            print(poseX, poseY, dis, angleAGV, self.curr_velocity)
                
        return 1
    
    def moveIntoPalletVS3(self):
        twist = Twist()
        selectAngle = 0
        directMode1 = 0 # dung voi truong hop selectAngle = 1 | 1 quay trai, 2 quay phai
        poseX = self.poseParking.position.x
        poseY = self.poseParking.position.y
        angleAGV = self.theta_robotParking
        ss_x = poseX
        ss_y = poseY
        angle = 0.0

        dis = sqrt(ss_x*ss_x + ss_y*ss_y)
        print(ss_x, ss_y, dis)
        if ((fabs(ss_x) <= 0.005 and fabs(ss_y) <= 0.005) or ss_x <= 0.) and self.flagFollowPointFinish:
            self.flagFollowPointFinish = False
            self.stop()
            return 2
        
        else:
            if self.flagFollowPointFinish:
                self.curr_velocity = self.velSt*(dis/self.distDeceleration)
                self.curr_velocity  = self.constrain(self.curr_velocity, self.min_linearVelocity, self.velSt)
                velX = -self.curr_velocity
                kp = 0.4
                dentaAngle = -self.theta_robotParking
                velAng = kp*fabs(dentaAngle)
                if dentaAngle > 0:
                    velAng = velAng
                else:
                    velAng = -velAng

                print(velX, "Follow Target Finish!")
                twist.linear.x = velX
                twist.angular.z = velAng
                self.pub_cmdVelParking(twist, self.rate_pubVel)
                return 1

            else:
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
                        velX = -self.curr_velocity
                        xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                        velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                        twist.linear.x = velX
                        twist.angular.z = velAng

                        self.pub_cmdVelParking(twist, self.rate_pubVel)
                        return 1
                    
                    else:
                        self.stop()
                        return -1
                    
                else:
                    return 0

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def SIGN(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        return 0
        
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
    
    def angleBetweenRobotAndPath2(self, _angle):
        dtAngle = _angle - self.theta_robotParking
        if fabs(dtAngle) >= PI:
            dtAngleTG = (2*PI - fabs(dtAngle))
            if dtAngle > 0:
                dtAngle = -dtAngleTG
            else:
                dtAngle = dtAngleTG

        return dtAngle

    def control_naviTarget(self, Xrb, Yrb, Xtarget, Ytarget, _maxAngular):
        kpg = 0.7  # 2. #1.5 #1.55
        kpd = 0.5
        angleLine = self.angleStraightLine(Xtarget, Ytarget, Xrb, Yrb)
        angleBW = self.angleBetweenRobotAndPath2(angleLine)
        vel_rot = kpg*fabs(angleBW) + kpd*fabs(Ytarget)
        if vel_rot > _maxAngular:
            vel_rot = _maxAngular

        if angleBW > 0:
            return vel_rot
        else:
            return -vel_rot
        
    def resetAll(self):
        self.process = 1
        self.is_request_parking = False
        self.is_poseParking = False
        self.is_path = False
        self.isDeceleration = False
        self.statusVel = 0
        
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
                if self.is_path:
                    print("recieve data Run Parking")
                    if self.dataPath.modeRun == 1 or self.dataPath.modeRun == 2:
                        print("recieve data Mode Parking")
                        if self.is_poseParking:
                            self.is_poseParking = False
                            print("recieve data Pose Parking")
                            if self.dataPath.enable:
                                print("Start Parking")
                                self.process = 2

                    elif self.dataPath.modeRun == 0:
                        self.resetAll()

            elif self.process == 2: # tim goal gan nhat
                if self.dataPath.modeRun == 1 or self.dataPath.modeRun == 2:
                    if self.checkPathOutOfRange():
                        print("Done find Goal Nearest")
                        self.process = 4
                else:
                    self.resetAll()
                    print("Reset parking!")

            elif self.process == 3: # Quay dap ung goc
                if self.turnAroundFGV(self.theta_AngentFisrtPoint, radians(1.), 3000): # kiem tra goc AGV voi quy dao
                    if self.checkTurn(0.):
                        print("DONE Rotary!")
                        self.process = 4

            elif self.process == 4: # di chuyen vao pallet
                if self.dataPath.modeRun == 1 or self.dataPath.modeRun == 2:
                    stt = self.moveIntoPallet() #  0.5, 0.35, 0
                    if stt == 2:
                        self.process = 8
                        print("Done!")
                    elif stt == -1:
                        self.process = 11
                        print("Something went wrong (T-T)")

                else:
                    self.resetAll()
                    print("Reset parking!")

            elif self.process == 8:
                if self.dataPath.modeRun == 0: # wait reset!
                    self.resetAll()
                    print("Reset parking!")
                    self.process = 1

            elif self.process == 11:
                if self.dataPath.modeRun == 0: # wait reset!
                    self.resetAll()
                    print("Reset parking because something went wrong!")
                    self.process = 1

            self.pubMakerPointFollow(self.infoPathFollow.X, self.infoPathFollow.Y) 
            self.pub_Status(self.process, self.dataPath.modeRun, self.req_parking.poseTarget, self.dataPath.offset, self.poseParking.position.x, self.poseParking.position.y, self.theta_robotParking, "mess", 0)   
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
        

# x: -3.645
# y: -6.258
# z: -0.713
# w: 0.700