#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import signal
import tf
import time
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import APathParking, PointOfPath 
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, tan, fabs
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

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


class BroadcasterParking(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        
        self.rate = rospy.Rate(30)

        self.distanceAlign = rospy.get_param('~distanceAlign', 0.1) 

        # Tf
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()


        rospy.Subscriber('/parking_request', Parking_request, self.request_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False
        
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.cbPose, queue_size = 20)
        self.is_pose_robot = False
        
        self.pubPoseParking = rospy.Publisher("/poseParking", PoseStamped, queue_size= 20)

        self.pubPathParking = rospy.Publisher("/path_parking", APathParking, queue_size= 20)
        self.msgPathParking = APathParking()

        self.pubPath = rospy.Publisher("/path_parking_visualization", Path, queue_size= 20)

        self.pointOne = Point(2.5, 1.)
        self.pointSecond = Point(1.2, 0)

        self.process = 0
        self.pubTransform = False
        self.time_waitTransfrom = time.time()
        self.is_transform = False

        self.poseRobotParking = Pose()
        self.isTransFormPose = False

        self.stepPoint = 0.01

    def cbPose(self, data):
        self.is_pose_robot = True
        if self.is_transform:
            if self.transformPoseNAV('frame_target', 'frame_robot'):
                self.isTransFormPose = True
            else:
                self.isTransFormPose = False

    def sendTransform(self, frame_world, frame_id_pointTarget, point_target):
        self.time_startTransform = rospy.Time.now()
        self.tf_broadcaster.sendTransform((point_target.position.x, point_target.position.y, -2.45),
                                        (0.0, 0.0, point_target.orientation.z, point_target.orientation.w),
                                        self.time_startTransform,
                                        frame_id_pointTarget,
                                        frame_world)
        
    def transformPoseNAV(self, frame_id_pointTarget, frame_agvNAV):
        msgPoseParking = PoseStamped()
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, now, rospy.Duration(10))
            pos, orien = self.tf_listener.lookupTransform(frame_id_pointTarget, frame_agvNAV, now)

            if pos[0] == 0.0 and pos[1] == 0.0 and orien[2] == 0.0 and orien[3] == 0.0:
                print("tranform false")
                return False
            
            else:
                msgPoseParking.header.stamp = now
                msgPoseParking.header.frame_id = frame_id_pointTarget

                self.poseRobotParking.position.x = pos[0]
                self.poseRobotParking.position.y = pos[1]
                self.poseRobotParking.position.z = pos[2]

                self.poseRobotParking.orientation.x = orien[0]
                self.poseRobotParking.orientation.y = orien[1]
                self.poseRobotParking.orientation.z = orien[2]
                self.poseRobotParking.orientation.w = orien[3]

                msgPoseParking.pose = self.poseRobotParking

                # msgPoseParking.pose.position.x = pos[0]
                # msgPoseParking.pose.position.y = pos[1]
                # msgPoseParking.pose.position.z = pos[2]
                # msgPoseParking.pose.orientation.x = orien[0]
                # msgPoseParking.pose.orientation.y = orien[1]
                # msgPoseParking.pose.orientation.z = orien[2]
                # msgPoseParking.pose.orientation.w = orien[3]

                self.pubPoseParking.publish(msgPoseParking)
                return True

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return False
        
    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True


    def makePath(self): #pose local
        # save = rospy.get_time()
        # print(save)
        # self.myPath = Paraboll(pointOne, pointSecond)
        poseRobotInParkingNow = Pose()
        poseRobotInParkingNow = self.poseRobotParking
        # print(self.poseRobotParking, "get")
        pointOne = Point(poseRobotInParkingNow.position.x, poseRobotInParkingNow.position.y) # vi tri agv hien tai
        pointSecond = Point(self.req_parking.offset + self.distanceAlign, 0.)
        myPath = QuadraticBezierCurves(pointOne, pointSecond, Point(pointOne.x, 0.))
        listPoint = myPath.slip()

        self.msgPathParking.poseStart.position.x = poseRobotInParkingNow.position.x
        self.msgPathParking.poseStart.position.y = poseRobotInParkingNow.position.y
        self.msgPathParking.poseStart.orientation.z = poseRobotInParkingNow.orientation.z
        self.msgPathParking.poseStart.orientation.w = poseRobotInParkingNow.orientation.w

        self.msgPathParking.poseIntermediary.position.x = self.req_parking.offset
        self.msgPathParking.poseIntermediary.position.y = 0.
        self.msgPathParking.poseIntermediary.orientation.z = 0.
        self.msgPathParking.poseIntermediary.orientation.w = 1.

        rx_strline = np.arange(pointSecond.x - self.stepPoint, -1., -self.stepPoint, float)
        for i in rx_strline:
            listPoint = np.append(listPoint, np.array([[i, 0.]]), axis=0)

        self.msgPath = Path()
        self.msgPath.header.frame_id = 'frame_target'
        self.msgPath.header.stamp = rospy.Time.now()

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

        return True
    
    def resetALl(self):
        self.is_transform = False
        self.isTransFormPose = False


    def run(self):
        ck = 0
        while not self.shutdown_flag.is_set(): 
            if self.process == 0: # cho du tin hieu
                if self.is_pose_robot:
                    ck = ck + 1
                if ck == 1:
                    self.process = 2
                    print("Recieve All Data Needed (^-^)")
            
            elif self.process == 2:
                if self.req_parking:
                    if self.req_parking.modeRun == 1 or self.req_parking.modeRun == 2:
                        print("Mode Run != 0")
                        if self.is_transform and self.isTransFormPose:
                            self.resetALl()
                            self.process = 3
                            print("Done Transform!")

            elif self.process == 3: # make path
                if self.makePath():
                    self.process = 4
                    print("make Path Done! Start Publish data")

            elif self.process == 4:
                if self.req_parking.modeRun == 0:
                    self.resetALl()
                    self.process = 2
                    print("Reset!")

                else:
                    self.pubPath.publish(self.msgPath)
                    self.pubPathParking.publish(self.msgPathParking)

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

    rospy.init_node('broadcaster_parking', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
    # Start the job threads
    try:

        br = BroadcasterParking(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            if br.is_request_parking:
                if br.req_parking.modeRun == 1 or br.req_parking.modeRun == 2:
                    br.sendTransform('frame_map_nav350', 'frame_target', br.req_parking.poseTarget) # -- edit 03/03/2022: frame_map_nav350 frame_global_map
                    br.is_transform = True

                elif br.req_parking.modeRun == 0:
                    br.resetALl()
            time.sleep(0.01)
 
    except ServiceExit:
        
        br.shutdown_flag.set()
        # Wait for the threads to close...
        br.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()
