#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Gửi lộ trình cho AGV 
    - Dừng AGV nếu va chạm với AGV khác
"""

import roslib
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
from agv_simulation.msg import *
import math
import time 

"""
###  Line Request Move msg ####3
int32 enable
int32 target_id
float64 target_x
float64 target_y
float64 target_z
float64 offset
sti_msgs/PathInfo[] pathInfo
int32 enableStop
sti_msgs/StopPoint pointStopAvoid
int32 mission
string mess

### Path info #####
int16 pathID
int16 typePath
int8 direction
geometry_msgs/Pose pointOne
geometry_msgs/Pose pointSecond
float64 velocity
float64 radius
geometry_msgs/Pose pointCenter
int64 numberPts
geometry_msgs/Pose pointMid
float64  movableZone

"""

class Fake_stiClient():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiClient_fake', anonymous=False)
        
        # -- node publish -- 
        self.pub_requestMove = rospy.Publisher('/move_request', LineRequestMove, queue_size = 10)
        self.dataRequestMove = LineRequestMove()
        self.dataRequestMove.pathInfo = [PathInfo()]
        self.rate = rospy.Rate(10.0)
        
        # -- biến lộ trình cho AGV 
        self.agv1_list_index = 0            # agv1 
        self.agv1_list_pub_x = []
        self.agv1_list_pub_y = []
        self.agv1_list_len = 5

        self.enable = 0
        self.target_id = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0 
        self.offset = 0.0

        self.pathInfo = PathInfo()
        self.pathID = 0
        self.typePath = 0
        self.direction = 0
        self.pointOne = Pose()
        self.pointSecond = Pose()
        self.velocity = 0.0
        self.radius = 0.0
        self.pointCenter = Pose()
        self.numberPts = 0
        self.pointMid = Pose()
        self.movableZone = 0  

        self.enableStop = 0
        self.pointStopAvoid = StopPoint()
        self.mission = 0
        self.mess = ""

        # -- biến hệ thống --
        self.DieTime_pub = time.time()           # thời gian chờ pub
        self.mode = 0                            # chế độ hoạt động 

        self.stepPoint = 0.01
        self.straightLineMode = 1
        self.circleMode = 2
        self.quadraticBezierCurves = 3
        self.curverLineMode = 4
    
    def handle_ProcessCollision(self, data):
        self.msg_processCollision = data

    def handle_DetectErr(self, data):
        self.msg_detectErr = data

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0:                                           # khi các AGV chưa chạy 
                if time.time() - self.DieTime_pub < 3:                   # chờ 2s rồi pub lộ trình tiếp theo
                    self.enable = 1
                    self.target_id = 1
                    self.target_x = 5.0
                    self.target_y = 5.0
                    self.target_z = 0.0 
                    self.offset = 1.0
                    self.mission = 1
                    self.mess = ""

                    self.pathID = 1
                    self.typePath = self.quadraticBezierCurves
                    self.direction = 0
                    self.pointOne.position.x = 0.0
                    self.pointOne.position.y = 0.0
                    self.pointSecond.position.x = 5.0
                    self.pointSecond.position.y = 5.0
                    self.velocity = 0.6
                    self.radius = 0.0
                    self.pointCenter = Pose()
                    self.numberPts = 100
                    self.pointMid.position.x = 0.0
                    self.pointMid.position.y = 5.0
                    self.movableZone = 0

                else:
                    self.dataRequestMove.enable = self.enable
                    self.dataRequestMove.target_x = self.target_x
                    self.dataRequestMove.target_y = self.target_y
                    self.dataRequestMove.target_z = self.target_z
                    self.dataRequestMove.offset = self.offset
                    self.dataRequestMove.mission = self.mission
                    self.dataRequestMove.mess = self.mess

                    self.dataRequestMove.pathInfo[0].pathID = self.pathID
                    self.dataRequestMove.pathInfo[0].typePath = self.typePath
                    self.dataRequestMove.pathInfo[0].direction = self.direction

                    self.dataRequestMove.pathInfo[0].pointOne = self.pointOne
                    self.dataRequestMove.pathInfo[0].pointSecond = self.pointSecond
                    self.dataRequestMove.pathInfo[0].velocity = self.velocity
                    self.dataRequestMove.pathInfo[0].radius = self.radius

                    self.dataRequestMove.pathInfo[0].pointCenter = self.pointCenter
                    self.dataRequestMove.pathInfo[0].numberPts = self.numberPts
                    self.dataRequestMove.pathInfo[0].pointMid = self.pointMid
                    self.dataRequestMove.pathInfo[0].movableZone = self.movableZone                   
                              
                    self.pub_requestMove.publish(self.dataRequestMove)

                    self.DieTime_pub = time.time()
                    self.mode = 1

            elif self.mode == 1:
                self.pub_requestMove.publish(self.dataRequestMove)

            self.rate.sleep()

def main():
	print('Program starting')
	program = Fake_stiClient()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()