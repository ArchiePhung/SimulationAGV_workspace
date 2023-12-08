#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove, HC_info
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class InfoPathFollowing():
    def __init__(self):
        self.pathID = 0.
        self.typePath = 0.
        self.indexInListpath = 0.
        self.velocity = 0.
        self.radius = 0.
        self.X = 0.
        self.Y = 0.

class GoalControl():
    def __init__(self):
        rospy.init_node('goal_control', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
        self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

        self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 0.8) #0.1
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.012) # 0.008

        self.max_lookahead = rospy.get_param('~max_lookahead', 1.48)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.12)
        self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/move_request', LineRequestMove, self.callback_moveRequest)
        self.dataRequestMove = LineRequestMove() 
        self.is_moveRequest = False

        rospy.Subscriber('/list_pointRequestMove', ListPointRequestMove, self.callback_listPointRequestMove)
        self.datalistPointRequestMove = ListPointRequestMove() 
        self.startPoint = Pose()
        self.finishPoint = Pose()
        self.is_listPointRequestMove = False

        rospy.Subscriber("/HC_info", HC_info, self.zone_callback)
        self.zone_lidar = HC_info()
        self.is_check_zone = False	

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 15

        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)

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
        self.lineIDFollow = 0
        self.typeLineFollow = 0

        self.distDeceleration = 0.3
        self.disgetLocation = 0.15

        self.stepContourFollow = 0
        self.curr_velocity = 0.

        self.listVel = [0.012, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
        self.numTable = len(self.listVel)
        self.listLookAhead = [0.12, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1.05, 1.28, 1.33, 1.38, 1.43, 1.48]

        self.infoPathFollow = InfoPathFollowing()
        self.velFollowOutOfRange = 0.2

        self.cmdVel = 0.
        self.flagVelChange = True
        self.statusVel = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
        self.velSt = 0.

        self.saveTimeVel = rospy.get_time()
        self.flagFollowPointFinish = False
        self.flagCheckAngle = False
  
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

    def callback_moveRequest(self, data):
        self.dataRequestMove = data
        self.is_moveRequest = True

    def callback_listPointRequestMove(self, data):
        self.datalistPointRequestMove = data
        try:
            self.startPoint = self.datalistPointRequestMove.infoPoint[0].pose
            self.finishPoint = self.datalistPointRequestMove.infoPoint[-1].pose
        except:
            pass
        self.is_listPointRequestMove = True

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
        self.curr_velocity = 0.
        for i in range(0,3,1):
            self.pub_cmd_vel.publish(Twist())

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def pubMakerPointFollow(self, x, y):
        # Create rviz marker message
        marker = Marker()
        marker.header.frame_id = "frame_map_nav350"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -1.
        marker.pose.orientation.w = 1.
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        self.pubMarker.publish(marker)

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
    
    def findGoalNearest(self, xRb, yRb):
        index = 0
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
        infoPoint.indexInListpath = index
        infoPoint.pathID = self.datalistPointRequestMove.infoPoint[index].pathID
        infoPoint.velocity = self.datalistPointRequestMove.infoPoint[index].velocity
        infoPoint.typePath = self.datalistPointRequestMove.infoPoint[index].typePath
        infoPoint.X = self.datalistPointRequestMove.infoPoint[index].pose.position.x
        infoPoint.Y = self.datalistPointRequestMove.infoPoint[index].pose.position.y

        return infoPoint, min
    
    def checkPathOutOfRange(self):
        poseX = self.poseRbMa.position.x
        poseY = self.poseRbMa.position.y
        self.infoPathFollow, dmin = self.findGoalNearest(poseX, poseY)
        return 1
    
        if dmin >= self.max_lookahead:
            twist = Twist()
            velX = self.velFollowOutOfRange
            self.curr_velocity = velX
            xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.x,self.infoPathFollow.y)
            velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
            twist.linear.x = velX
            twist.angular.z = velAng

            self.pub_cmdVel(twist, self.rate_pubVel)
            return 0

        else:
            return 1

        
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
                self.pub_Stop()
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
        # lookahead = self.constrain(self.max_lookahead*curr_velocity/self.lookahead_ratio, self.min_lookahead, self.max_lookahead)
        # return lookaheadflagVelChange

    def checkFollowPointFinish(self):
        if self.infoPathFollow.X == self.finishPoint.position.x and self.infoPathFollow.Y == self.finishPoint.position.y:
            self.flagFollowPointFinish = True

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
        longest_distance = 0.
        indexSelect = -1
        lookahead = self.findLookAheadByVel(curr_velocity)
        numpoint = len(self.datalistPointRequestMove.infoPoint)
        for i in range(numpoint - 1,  - 1, -1):
            x = self.datalistPointRequestMove.infoPoint[i].pose.position.x
            y = self.datalistPointRequestMove.infoPoint[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)
            if dis <= lookahead:
                longest_distance = dis
                indexSelect = i

                break

        if indexSelect == -1:
            return False
        
        else:
            self.infoPathFollow.indexInListpath = indexSelect

            self.infoPathFollow.X = self.datalistPointRequestMove.infoPoint[indexSelect].pose.position.x
            self.infoPathFollow.Y = self.datalistPointRequestMove.infoPoint[indexSelect].pose.position.y

            pathID = self.datalistPointRequestMove.infoPoint[indexSelect].pathID
            typePath = self.datalistPointRequestMove.infoPoint[indexSelect].typePath
            if pathID != self.infoPathFollow.pathID and typePath == 1:
                self.infoPathFollow.pathID = pathID
                self.infoPathFollow.typePath = typePath
                self.flagCheckAngle = True

            if self.flagCheckAngle:
                quat = (self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.x, \
                        self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.y, \
                        self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.z, \
                        self.datalistPointRequestMove.infoPoint[indexSelect].pose.orientation.w)

                angle = self.angleBetweenRobotAndPath(quat)
                if fabs(angle) <= 0.02:
                    self.flagCheckAngle  = False

            else:         
                if self.statusVel != 2:
                    vel = self.datalistPointRequestMove.infoPoint[indexSelect].velocity
                    self.infoPathFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

            return True

    """
        -1: loi di chuyen 
        0: dung
        1: dang thuc hien
        2. hoan thanh
    """  

    def followPath(self):
        twist = Twist()
        poseX = self.poseRbMa.position.x
        poseY = self.poseRbMa.position.y
        # angleAGV = self.theta_robotParking
        ss_x = poseX - self.finishPoint.position.x
        ss_y = poseY - self.finishPoint.position.y

        dis = sqrt(ss_x*ss_x + ss_y*ss_y)
        print(ss_x, ss_y, dis)
        if (fabs(ss_x) <= 0.01 and fabs(ss_y) <= 0.01) or dis <= 0.02:
            self.pub_Stop()
            return 2
        
        else:
            if self.flagFollowPointFinish:
                self.curr_velocity = self.findVelByLookAhead(dis)
                print(self.curr_velocity, self.infoPathFollow.velocity)
                velX = self.curr_velocity
                xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                twist.linear.x = velX
                twist.angular.z = velAng
                self.pub_cmdVel(twist, self.rate_pubVel)
                return 1

            else:
                # kiem tra dang follow point cuoi chua
                self.checkFollowPointFinish()
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
                        # kc = self.calculate_distance(self.xFollow, self.yFollow, poseX, poseY)
                        ss_x = poseX - self.finishPoint.position.x
                        ss_y = poseY - self.finishPoint.position.y
                        # print(ss_x, ss_y, self.curr_velocity, self.infoPathFollow.X, self.infoPathFollow.Y)

                        velX = self.curr_velocity
                        xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                        velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                        twist.linear.x = velX
                        twist.angular.z = velAng

                        self.pub_cmdVel(twist, self.rate_pubVel)
                        return 1
                    
                    else:
                        self.pub_Stop()
                        return -1
                    
                else:
                    return 0


    def SIGN(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        return 0
    
    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)
        
    # function Navigation 
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_robotNow
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
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
                if self.is_moveRequest and self.dataRequestMove.enable == 1:
                    print("recieve data Navigation")
                    if self.is_listPointRequestMove:
                        print("recieve data local goal")
                        self.process = 2

            elif self.process == 2: # kiem tra vi tri AGV voi duong dan
                if self.checkPathOutOfRange():
                    print("Done Find Nearest Goal")
                    self.process = 3

            elif self.process == 3: # follow theo duong dan
                stt = self.followPath()
                if stt == 2:
                    print("Arrived at the Intermediary Target location")
                    rospy.sleep(0.5)
                    self.process = 4

                elif stt == -1:
                    self.process = -2

            elif self.process == 4:
                # print("Done!")
                pass

            elif self.process == -2:
                print("Error")

            self.pubMakerPointFollow(self.infoPathFollow.X, self.infoPathFollow.Y)
            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = GoalControl()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()