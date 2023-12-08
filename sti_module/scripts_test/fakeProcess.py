#!/usr/bin/env python
# license removed for brevity
import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove, Status_goal_control, OC_ForkLift_request, OC_ForkLift_respond
from message_pkg.msg import Parking_request, Parking_respond

from math import sin, cos, asin, tan, atan, degrees, radians, sqrt, fabs, acos, atan2
from math import pi as PI


class FakeProcess():
    def __init__(self):
        rospy.init_node('make_process', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(10)

        # sub process
        rospy.Subscriber("/move_respond", Status_goal_control, self.respondMove_callback)
        self.respondMove = Status_goal_control() 
        self.sttGoalControl = 0
        self.is_respondMove = False

        rospy.Subscriber("/parking_respond", Parking_respond, self.respondParking_callback)
        self.respondParking = Status_goal_control() 
        self.sttParking = 0
        self.is_respondParking = False

        rospy.Subscriber("/OC_respond", OC_ForkLift_respond, self.respondOC_callback)
        self.respondOC = OC_ForkLift_respond() 
        self.sttOC = 0
        self.is_respondOC = False

        # pub process
        self.pubMoveRequest = rospy.Publisher('/move_request', LineRequestMove, queue_size=10)
        self.msg = LineRequestMove()
        self.plan1 = LineRequestMove()
        self.plan2 = LineRequestMove()

        self.plan3 = LineRequestMove()
        self.plan4 = LineRequestMove()

        self.movePoint1 = LineRequestMove()
        self.movePoint2 = LineRequestMove()

        self.pubParkingRequest = rospy.Publisher('/parking_request', Parking_request, queue_size=10)
        self.msgRequestParking = Parking_request()
        self.msgRequestParking1 = Parking_request()
        self.msgRequestParking2 = Parking_request()

        self.pubRequestOC = rospy.Publisher('/OC_request', OC_ForkLift_request, queue_size=10)
        self.msgRequestOC = OC_ForkLift_request()
        self.msgRequestOC1 = OC_ForkLift_request()
        self.msgRequestOC2 = OC_ForkLift_request()

        self.process = 0
        self.saveTime = rospy.get_time()

        self.bit = False

        self.phaseStart = True

        self.numberStep = 0

        # define path
        #------------------------- Path 1 ---------------------------------------------------
        self.plan1.enable = 1
        self.plan1.target_x = -6.832
        self.plan1.target_y = -6.84
        self.plan1.target_z = 0.

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 1
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = 0.667
        path.pointOne.position.y = -1.496
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.956
        path.pointSecond.position.y = -2.951
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.956
        path.pointMid.position.y = -1.496
        self.plan1.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.956
        path.pointOne.position.y = -2.951
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.956
        path.pointSecond.position.y = -5.26
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan1.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.956
        path.pointOne.position.y = -5.26
        path.pointSecond = Pose()
        path.pointSecond.position.x = -2.973
        path.pointSecond.position.y = -6.84
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.956
        path.pointMid.position.y = -6.84
        self.plan1.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 4
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -2.973
        path.pointOne.position.y = -6.84
        path.pointSecond = Pose()
        path.pointSecond.position.x = -4.832
        path.pointSecond.position.y = -8.497
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -4.832
        path.pointMid.position.y = -6.84
        self.plan1.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 5
        path.typePath = 3
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -4.832
        path.pointOne.position.y = -8.497
        path.pointSecond = Pose()
        path.pointSecond.position.x = -6.332
        path.pointSecond.position.y = -6.84
        path.velocity = 0.15
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -4.832
        path.pointMid.position.y = -6.84
        self.plan1.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -6.332
        path.pointOne.position.y = -6.84
        path.pointSecond = Pose()
        path.pointSecond.position.x = -6.832
        path.pointSecond.position.y = -6.84
        path.velocity = 0.15
        path.movableZone = 1.2
        self.plan1.pathInfo.append(path)

        # self.movePoint1.enable = 2
        # self.movePoint1.target_x = -6.832
        # self.movePoint1.target_y = -6.84

        # define path
        #------------------------- Path 2 ---------------------------------------------------
        self.plan2.enable = 1
        self.plan2.target_x = 0.667
        self.plan2.target_y = -1.496
        self.plan2.target_z = 3.13

        # path 1
        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -6.832
        path.pointOne.position.y = -6.84
        path.pointSecond = Pose()
        path.pointSecond.position.x = -6.332
        path.pointSecond.position.y = -6.84
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -6.332
        path.pointOne.position.y = -6.84
        path.pointSecond = Pose()
        path.pointSecond.position.x = -2.973
        path.pointSecond.position.y = -6.84
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)


        # path 3 Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -2.973
        path.pointOne.position.y = -6.84
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.956
        path.pointSecond.position.y = -5.26
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.956
        path.pointMid.position.y = -6.84
        self.plan2.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.956
        path.pointOne.position.y = -5.26
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.956
        path.pointSecond.position.y = -2.951
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 5
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.956
        path.pointOne.position.y = -2.951
        path.pointSecond = Pose()
        path.pointSecond.position.x = -2.556
        path.pointSecond.position.y = -1.496
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.956
        path.pointMid.position.y = -1.496
        self.plan2.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 2
        path.pointOne = Pose() 
        path.pointOne.position.x = -2.556
        path.pointOne.position.y = -1.496
        path.pointSecond = Pose()
        path.pointSecond.position.x = 0.667
        path.pointSecond.position.y = -1.496
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)

        self.movePoint2.enable = 2
        self.movePoint2.target_x = 0.667
        self.movePoint2.target_y = -1.496


        # ---------------------------------------------------------
        # path3-----------
        self.plan3.enable = 1
        self.plan3.target_x = -18.118
        self.plan3.target_y = -6.885
        self.plan3.target_z = 3.13

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 1
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -26.468
        path.pointOne.position.y = -1.635
        path.pointSecond = Pose()
        path.pointSecond.position.x = -24.968
        path.pointSecond.position.y = -3.135
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -24.968
        path.pointMid.position.y = -1.635
        self.plan3.pathInfo.append(path)

        # path 2
        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose() 
        path.pointOne.position.x = -24.968
        path.pointOne.position.y = -3.135
        path.pointSecond = Pose()
        path.pointSecond.position.x = -24.968
        path.pointSecond.position.y = -5.385
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan3.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 4
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -24.968
        path.pointOne.position.y = -5.385
        path.pointSecond = Pose()
        path.pointSecond.position.x = -23.468
        path.pointSecond.position.y = -6.885
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -24.968
        path.pointMid.position.y = -6.885
        self.plan3.pathInfo.append(path)

        # path 2
        path = PathInfo()
        path.pathID = 5
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose() 
        path.pointOne.position.x = -23.468
        path.pointOne.position.y = -6.885
        path.pointSecond = Pose()
        path.pointSecond.position.x = -21.118
        path.pointSecond.position.y = -6.885
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan3.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 6
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -21.118
        path.pointOne.position.y = -6.885
        path.pointSecond = Pose()
        path.pointSecond.position.x = -19.618
        path.pointSecond.position.y = -5.385
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -19.618
        path.pointMid.position.y = -6.885
        self.plan3.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 7
        path.typePath = 3
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -19.618
        path.pointOne.position.y = -5.385
        path.pointSecond = Pose()
        path.pointSecond.position.x = -18.118
        path.pointSecond.position.y = -6.885
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -19.618
        path.pointMid.position.y = -6.885
        self.plan3.pathInfo.append(path)

        self.movePoint1.enable = 2
        self.movePoint1.target_x = -18.118
        self.movePoint1.target_y = -6.885

        # ---------------------------------------------------------
        # path4-----------
        self.plan4.enable = 1
        self.plan4.target_x = -26.468
        self.plan4.target_y = -1.635
        self.plan4.target_z = 0.

        # path 2
        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose() 
        path.pointOne.position.x = -18.118
        path.pointOne.position.y = -6.885
        path.pointSecond = Pose()
        path.pointSecond.position.x = -23.468
        path.pointSecond.position.y = -6.885
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan4.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 2
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -23.468
        path.pointOne.position.y = -6.885
        path.pointSecond = Pose()
        path.pointSecond.position.x = -24.968
        path.pointSecond.position.y = -5.385
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -24.968
        path.pointMid.position.y = -6.885
        self.plan4.pathInfo.append(path)

        # path 2
        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose() 
        path.pointOne.position.x = -24.968
        path.pointOne.position.y = -5.385
        path.pointSecond = Pose()
        path.pointSecond.position.x = -24.968
        path.pointSecond.position.y = -3.135
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan4.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 5
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -24.968
        path.pointOne.position.y = -3.135
        path.pointSecond = Pose()
        path.pointSecond.position.x = -23.468
        path.pointSecond.position.y = -1.635
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -24.968
        path.pointMid.position.y = -1.635
        self.plan4.pathInfo.append(path)

        # path 2
        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 2
        path.pointOne = Pose() 
        path.pointOne.position.x = -23.468
        path.pointOne.position.y = -1.635
        path.pointSecond = Pose()
        path.pointSecond.position.x = -26.468
        path.pointSecond.position.y = -1.635
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan4.pathInfo.append(path)

        self.movePoint2.enable = 2
        self.movePoint2.target_x = -26.468
        self.movePoint2.target_y = -1.635

        #------------------------------------------------------
        # pub request parking
        # self.msgRequestParking1.modeRun = 1
        # self.msgRequestParking1.poseTarget.position.x = -8.332
        # self.msgRequestParking1.poseTarget.position.y = -6.84
        # self.msgRequestParking1.poseTarget.orientation.z = 0
        # self.msgRequestParking1.poseTarget.orientation.w = 1
        # self.msgRequestParking1.offset = 1.3

        # self.msgRequestParking2.modeRun = 1
        # self.msgRequestParking2.poseTarget.position.x = 2.167
        # self.msgRequestParking2.poseTarget.position.y = -1.496
        # self.msgRequestParking2.poseTarget.orientation.z = 0.999
        # self.msgRequestParking2.poseTarget.orientation.w = 0.0087265
        # self.msgRequestParking2.offset = 1.3

        self.msgRequestParking1.modeRun = 1
        self.msgRequestParking1.poseTarget.position.x = -16.718
        self.msgRequestParking1.poseTarget.position.y = -6.885
        self.msgRequestParking1.poseTarget.orientation.z = 0.999
        self.msgRequestParking1.poseTarget.orientation.w = 0.0087265
        self.msgRequestParking1.offset = 1.4

        self.msgRequestParking2.modeRun = 1
        self.msgRequestParking2.poseTarget.position.x = -27.868
        self.msgRequestParking2.poseTarget.position.y = -1.635
        self.msgRequestParking2.poseTarget.orientation.z = 0
        self.msgRequestParking2.poseTarget.orientation.w = 1
        self.msgRequestParking2.offset = 1.4

        #------------------------------------------------------
        # pub request OC
        # nang
        self.msgRequestOC1.mode = 1
        self.msgRequestOC1.mission = 1
        self.msgRequestOC1.reqHeightLift = 10

        # ha
        self.msgRequestOC2.mode = 1
        self.msgRequestOC2.mission = 1
        self.msgRequestOC2.reqHeightLift = 0


    def respondMove_callback(self, data):
        self.sttGoalControl = data.complete_misson
        self.is_respondMove = True

    def respondParking_callback(self, data):
        self.sttParking = data.status
        self.is_respondParking = True

    def respondOC_callback(self, data):
        self.sttOC = data.status
        self.is_respondOC = True

    def scriptMove(self):
        if self.process == 0:
            if self.is_respondMove:
                print("recieve data")
                self.process = 100

        elif self.process == 100:
            if self.sttGoalControl == 0:
                self.msg = self.plan3
                print("pub plan 3")
                self.process = 1

        elif self.process == 1:
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 2
                self.saveTime = rospy.get_time()

        elif self.process == 2:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 3

        elif self.process == 3: # parking
            if self.is_respondParking:
                self.process = 4
        
        elif self.process == 4:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParking1
                print("pub paking 1")
                self.process = 5

        elif self.process == 5:
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 6
                self.saveTime = rospy.get_time()

        elif self.process == 6:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 7

        elif self.process == 7: # doi tin hieu
            self.process = 8
            if self.is_respondOC:
                self.process = 8

        elif self.process == 8: 
            self.numberStep = self.numberStep + 1
            if self.numberStep % 3 != 0:
                if self.sttOC == 0:
                    if self.bit == False:
                        self.msgRequestOC = self.msgRequestOC1
                    else:
                        self.msgRequestOC = self.msgRequestOC2
                    self.bit = not self.bit
                    print("pub OC ")
                    self.process = 9
            else:
                self.numberStep = 0
                self.process = 14

        elif self.process == 9:
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 10
                self.saveTime = rospy.get_time()

        elif self.process == 10:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 14

        # elif self.process == 11:
        #     if self.phaseStart:
        #         self.phaseStart = False
        #         self.process = 14
        #     else:
        #         if self.sttOC == 0:
        #             if self.bit == False:
        #                 self.msgRequestOC = self.msgRequestOC1
        #             else:
        #                 self.msgRequestOC = self.msgRequestOC2
        #             self.bit = not self.bit
        #             print("pub OC ")
        #             self.process = 12
        
        # elif self.process == 12:
        #     if self.sttOC == 3:
        #         self.msgRequestOC = OC_ForkLift_request()
        #         self.process = 13
        #         self.saveTime = rospy.get_time()

        # elif self.process == 13:
        #     dentaT = rospy.get_time() - self.saveTime
        #     if dentaT > 3.:
        #         print("wait done")
        #         self.process = 14

        elif self.process == 14: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePoint1
                print("pub plan 1")
                self.process = 15

        elif self.process == 15:
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 16
                self.saveTime = rospy.get_time()

        elif self.process == 16:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 17

        elif self.process == 17:
            if self.sttGoalControl == 0:
                self.msg = self.plan4
                print("pub plan 1")
                self.process = 18

        elif self.process == 18:
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 19
                self.saveTime = rospy.get_time()

        elif self.process == 19:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                self.process = 20
                print("wait done")

        elif self.process == 20: # parking
            if self.is_respondParking:
                self.process = 21
        
        elif self.process == 21:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParking2
                print("pub paking 2")
                self.process = 22

        elif self.process == 22:
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 23
                self.saveTime = rospy.get_time()

        elif self.process == 23:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 24

        elif self.process == 24: #
            self.numberStep = self.numberStep + 1
            if self.numberStep % 3 != 0:
                if self.sttOC == 0:
                    if self.bit == False:
                        self.msgRequestOC = self.msgRequestOC1
                    else:
                        self.msgRequestOC = self.msgRequestOC2
                    self.bit = not self.bit
                    print("pub OC ")
                    self.process = 25

            else:
                self.numberStep = 0
                self.process = 30

        elif self.process == 25:
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 26
                self.saveTime = rospy.get_time()

        elif self.process == 26:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 30

        # elif self.process == 27:
        #     if self.phaseStart:
        #         self.phaseStart = False
        #         self.process = 30
        #     else:
        #         if self.sttOC == 0:
        #             if self.bit == False:
        #                 self.msgRequestOC = self.msgRequestOC1
        #             else:
        #                 self.msgRequestOC = self.msgRequestOC2
        #             self.bit = not self.bit
        #             print("pub OC ")
        #             self.process = 28
        
        # elif self.process == 28:
        #     if self.sttOC == 3:
        #         self.msgRequestOC = OC_ForkLift_request()
        #         self.process = 29
        #         self.saveTime = rospy.get_time()

        # elif self.process == 29:
        #     dentaT = rospy.get_time() - self.saveTime
        #     if dentaT > 3.:
        #         print("wait done")
        #         self.process = 30

        elif self.process == 30: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePoint2
                print("pub plan 1")
                self.process = 31

        elif self.process == 31:
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 32
                self.saveTime = rospy.get_time()

        elif self.process == 32:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 3.:
                print("wait done")
                self.process = 100

        if self.msg.enable == 1 and self.msg.target_x == 0.:
            print("loi ow day")

        print(self.process)
            
        self.pubMoveRequest.publish(self.msg)
        self.pubParkingRequest.publish(self.msgRequestParking)
        self.pubRequestOC.publish(self.msgRequestOC)

    def run(self):
        while not rospy.is_shutdown():    
            self.scriptMove()
            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = FakeProcess()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()
