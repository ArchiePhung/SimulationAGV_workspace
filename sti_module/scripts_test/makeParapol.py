#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import localGoalParking
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan
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
        if a >= 0:
            return atan(a)
        else:
            return -atan(a)

class SUB():
    def __init__(self):
        rospy.init_node('make_lineParking', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        # param
        self.distanceStop = rospy.get_param('~distanceStop', 0.5) 

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

        # rospy.Subscriber('/parking_request', Parking_request, self.request_callback)
        # self.req_parking = Parking_request() 
        # self.is_request_parking = False

        self.pubLocalGoal = rospy.Publisher("/point_goal_local", localGoalParking, queue_size= 20)
        self.msgLocalGoal = localGoalParking()

        self.pubPath = rospy.Publisher("/path_global", Path, queue_size= 20)
        self.msgPath = Path()
        self.msgPath.header.frame_id = 'frame_target'
        self.msgPath.header.stamp = rospy.Time.now()

        self.pointOne = Point(2.5, 1)
        self.pointSecond = Point(1.2, 0)

        self.makedParaboll = False

        self.poseRobotX = 0.
        self.poseRobotY = 0.

        self.xFollow = 0.
        self.yFollow = 0.

        self.process = 0
    
    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True

    def poseParking_callback(self, data):
        self.poseParking = data
        self.is_poseParking = True

        if self.makedParaboll == True:
            self.pubPath.publish(self.msgPath)
            if self.findGoalNearRobot(1.0, self.poseParking.pose.position.x, self.poseParking.pose.position.y):
                self.msgLocalGoal.poseFollow.position.x = self.xFollow
                self.msgLocalGoal.poseFollow.position.y = self.yFollow

                self.pubLocalGoal.publish(self.msgLocalGoal)


    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def callback_poseRobot(self, msg):
        # if self.makedParaboll:

        pass
 
    def makeParaboll(self, pointOne, pointSecond): #pose local
        # save = rospy.get_time()
        # print(save)
        self.paraboll = Paraboll(pointOne, pointSecond)
        # self.msgLocalGoal.poseStart.position.x = 
        # self.msgLocalGoal.poseStart.position.y = 
        # self.msgLocalGoal.poseStart.orientation = 
        self.rx = np.arange(pointOne.x, pointSecond.x, -0.01, float)
        self.ry = [self.paraboll.calc(i) for i in self.rx]
        self.listPoint = np.stack((self.rx, self.ry), axis=1)

        # pub Path
        del self.msgPath.poses[:]
        for i in self.listPoint:
            point = PoseStamped()
            point.header.frame_id = 'frame_target'
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = i[0]
            point.pose.position.y = i[1]
            point.pose.orientation.w = 1.0
            # print(point)
            self.msgPath.poses.append(point)

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

    def findGoalNearRobot(self, l, xRb, yRb):
        numpoint = self.listPoint.shape[0]
        # print(numpoint)
        for i in range(numpoint - 1, -1, -1):
            point = self.listPoint[i]
            d = self.calculate_distance(point[0], point[1], xRb, yRb)
            if d <= l:
                self.xFollow = point[0]
                self.yFollow = point[1]
                return True
        
        return False
    
    def test(self):
        self.makeParaboll(self.pointOne, self.pointSecond)

    def run(self):
        while not rospy.is_shutdown():    
            if self.makedParaboll == False:
                if self.is_poseParking == True and (self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2):
                    pointStart = Point(self.poseParking.pose.position.x, self.poseParking.pose.position.y)
                    pointFinish = Point(self.distanceStop, 0.)
                    self.makeParaboll( pointStart, pointFinish)

                    self.msgLocalGoal.poseStart.position.x = pointStart.x
                    self.msgLocalGoal.poseStart.position.y = pointStart.y

                    self.msgLocalGoal.poseFinish.position.x = pointFinish.x
                    self.msgLocalGoal.poseFinish.position.y = pointFinish.y

                    self.makedParaboll = True
                    print("Make path done!")

                elif self.req_parking.modeRun == 0:
                    self.makedParaboll = False
                    print("receive cmd RESET!")

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = SUB()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()