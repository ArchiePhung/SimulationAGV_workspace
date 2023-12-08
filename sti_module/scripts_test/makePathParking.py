#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import APathParking, PointOfPath
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, tan, fabs
from math import pi as PI
# import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y

class Paraboll():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        #khoi tao 2 bien point
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond

        b = np.array([self.pointOne.y, self.pointSecond.y, 0])
        b = b[:, np.newaxis]
        A = np.array([[pow(self.pointOne.x, 2), self.pointOne.x, 1], [pow(self.pointSecond.x, 2), self.pointSecond.x, 1], [2*self.pointSecond.x, 1, 0]])
        C = np.dot(np.linalg.inv(A), b)
        print(C)
        self.a = C[0,0]
        self.b = C[1,0]
        self.c = C[2,0]

    def calc(self, _x):
        y = self.a*pow(_x, 2) + self.b*_x + self.c
        return y
    
    def tangent(self, _point):
        a = 2*self.a*_point.x + self.b
        if _point.y > 0:
            return -atan(a)
        else:
            return atan(a)
        
    def slip(self, _step):
        listPoint = np.empty((0,2), dtype=float)
        rx = np.arange(self.pointOne.x, self.pointSecond.x, -_step, float)
        for i in rx:
            listPoint = np.append(listPoint, np.array([[i, self.calc(i)]]), axis=0)
        return listPoint
        
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

class SUB():
    def __init__(self):
        rospy.init_node('make_lineParking', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(15)

        # param
        self.distanceStop = rospy.get_param('~distanceStop', 1.3) 
        self.distanceAlign = rospy.get_param('~distanceAlign', 0.1) 
        self.velocityMAX = rospy.get_param('~velocityMAX', 0.1) 
        self.velocityMIN = rospy.get_param('~velocityMAX', 0.03) 

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/parking_request', Parking_request, self.request_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False

        rospy.Subscriber('/poseParking', PoseStamped, self.poseParking_callback)
        self.is_poseParking = False
        self.poseParking = PoseStamped()

        self.pubPathParking = rospy.Publisher("/path_parking", APathParking, queue_size= 20)
        self.msgPathParking = APathParking()

        self.pubPath = rospy.Publisher("/path_global", Path, queue_size= 20)
        self.msgPath = Path()
        self.msgPath.header.frame_id = 'frame_target'
        self.msgPath.header.stamp = rospy.Time.now()

        self.pointOne = Point(2.5, 1.)
        self.pointSecond = Point(1.2, 0)

        # point Path Benze
        self.pointSecondCurvel = Point(self.distanceStop + self.distanceAlign, 0.)
        self.pointFinsishStepOne = Point(self.distanceStop, 0.)

        self.makedParaboll = False

        self.poseRobotX = 0.
        self.poseRobotY = 0.

        self.xFollow = 0.
        self.yFollow = 0.

        self.process = 0

        self.stepPoint = 0.01

    
    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True

    def poseParking_callback(self, data):
        self.poseParking = data
        self.is_poseParking = True

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def callback_poseRobot(self, msg):
        # if self.makedParaboll:

        pass
 
    def makePath(self, pointOne, pointSecond): #pose local
        # save = rospy.get_time()
        # print(save)
        # self.myPath = Paraboll(pointOne, pointSecond)
        self.myPath = QuadraticBezierCurves(pointOne, pointSecond, Point(pointOne.x, 0.))
        # listPoint = self.myPath.slip(self.stepPoint)
        listPoint = self.myPath.slip()

        self.msgPathParking.poseStart.position.x = pointOne.x
        self.msgPathParking.poseStart.position.y = pointOne.y
        # quaternion = quaternion_from_euler(0., 0., self.myPath.tangent(pointOne))
        # self.msgPathParking.poseStart.orientation.x = quaternion[0]
        # self.msgPathParking.poseStart.orientation.y = quaternion[1]
        # self.msgPathParking.poseStart.orientation.z = quaternion[2]
        # self.msgPathParking.poseStart.orientation.w = quaternion[3]

        self.msgPathParking.poseIntermediary.position.x = self.pointFinsishStepOne.x
        self.msgPathParking.poseIntermediary.position.y = self.pointFinsishStepOne.y

        rx_strline = np.arange(pointSecond.x - self.stepPoint, -1., -self.stepPoint, float)
        for i in rx_strline:
            listPoint = np.append(listPoint, np.array([[i, 0.]]), axis=0)

        # arrXConcate = np.concatenate((self.rx, self.rx_strline))
        # arrYConcate = np.concatenate((self.ry, self.ry_strline))

        # self.listPoint = np.stack((arrXConcate, arrYConcate), axis=1)

        # pub Path
        del self.msgPath.poses[:]
        del self.msgPathParking.info[:]
        for i in listPoint:
            # pub path rviz
            point = PoseStamped()
            point.header.frame_id = 'frame_target'
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = i[0]
            point.pose.position.y = i[1]
            point.pose.orientation.w = 1.0
            # print(point)
            self.msgPath.poses.append(point)

            # pub path
            pointPath = PointOfPath()
            pointPath.pose.position.x = i[0]
            pointPath.pose.position.y = i[1]
            pointPath.pose.orientation.w = 1.0

            pointPath.velocity = 0.1

            self.msgPathParking.info.append(pointPath)

        # print(self.findGoalNearRobot(2.))
        # print(self.xFollow, self.yFollow)
        # dt = rospy.get_time() - save
        # print(dt)
        # In ra số chiều (trục)
        # print("Số chiều: ", self.listPoint.ndim)
        
        # # In ra hình dạng của mảng 
        # print("Dạng của mảng: ", self.listPoint.shape)
        
        # # In ra tổng số phần tử
        # print("Tổng số phần tử: ", self.listPoint.size)
        # plt.plot(self.rx, self.ry, "-r")
        # plt.grid(True)
        # plt.axis("equal")
        # plt.show()
    
    def test(self):
        self.makePath(self.pointOne, self.pointSecond)
        while not rospy.is_shutdown():    
            self.pubPath.publish(self.msgPath)
            self.rate.sleep()

    def run(self):
        step = 0
        while not rospy.is_shutdown():    
            if step == 0: # cho thong tin parking
                if self.is_poseParking == True and (self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2):
                    pointStartCurvel = Point(self.poseParking.pose.position.x, self.poseParking.pose.position.y)
                    self.makePath(pointStartCurvel, self.pointSecondCurvel)
                    step = 1
                    print("Make path done!")

            elif step == 1: # publish path
                if self.req_parking.modeRun == 0:
                    step = 0

                else:
                    self.pubPath.publish(self.msgPath)
                    self.pubPathParking.publish(self.msgPathParking)

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = SUB()
    class_1.run()
    # class_1.test()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()