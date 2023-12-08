#!/usr/bin/env python
# license removed for brevity
import roslib
import sys
import time
import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import *
from message_pkg.msg import *

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

        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callback_getPose, queue_size = 100)
        self.robotPose_nav = PoseStamped()

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

        self.pub_infoRespond = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size=100)
        self.NN_infoRespond = NN_infoRespond()

        # -- App
        rospy.Subscriber("/app_button", App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
        self.app_button = App_button()


        self.process = 0
        self.processmain = 0
        self.saveTime = rospy.get_time()

        self.bit = False

        self.phaseStart = True
        self.numberStep = 0

        self.listError = []
        self.mode_by_hand = 1
        self.mode_auto = 2
        self.mode_operate = self.mode_by_hand    # Lưu chế độ hoạt động.
        self.flag_Auto_to_Byhand = 0

        self.savePlanning = 0
        self.planning1 = 1
        self.planning2 = 2
        self.planning3 = 3
        self.savePlanning = self.planning1

        # define path
        #------------------------- Path 1 ---------------------------------------------------
        self.plan1.enable = 1
        self.plan1.target_x = 1.083
        self.plan1.target_y = -4.482
        self.plan1.target_z = 3.13

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 1
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -4.956
        path.pointOne.position.y = -10.5
        path.pointSecond = Pose()
        path.pointSecond.position.x = -3.456
        path.pointSecond.position.y = -9.036
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -4.956
        path.pointMid.position.y = -9.036
        self.plan1.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -3.456
        path.pointOne.position.y = -9.036
        path.pointSecond = Pose()
        path.pointSecond.position.x = -1.367
        path.pointSecond.position.y = -9.036
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan1.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -1.367
        path.pointOne.position.y = -9.036
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -7.836
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -9.036
        self.plan1.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -7.836
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -2.982
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan1.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 7
        path.typePath = 3
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -2.982
        path.pointSecond = Pose()
        path.pointSecond.position.x = 1.083
        path.pointSecond.position.y = -4.482
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -4.482
        self.plan1.pathInfo.append(path)

        self.movePoint1.enable = 2
        self.movePoint1.target_x = 1.083
        self.movePoint1.target_y = -4.482

        # define path
        #------------------------- Path 2 ---------------------------------------------------
        self.plan2.enable = 1
        self.plan2.target_x = -4.956
        self.plan2.target_y = -10.5
        self.plan2.target_z = 1.57

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 1
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = 1.083
        path.pointOne.position.y = -4.482
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -5.982
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -4.482
        self.plan2.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -5.982
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -7.836
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)


        # path 3 Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -7.836
        path.pointSecond = Pose()
        path.pointSecond.position.x = -1.367
        path.pointSecond.position.y = -9.036
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -9.036
        self.plan2.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -1.367
        path.pointOne.position.y = -9.036
        path.pointSecond = Pose()
        path.pointSecond.position.x = -6.456
        path.pointSecond.position.y = -9.036
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan2.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 5
        path.typePath = 3
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -6.456
        path.pointOne.position.y = -9.036
        path.pointSecond = Pose()
        path.pointSecond.position.x = -4.956
        path.pointSecond.position.y = -10.5
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -4.956
        path.pointMid.position.y = -9.036
        self.plan2.pathInfo.append(path)

        self.movePoint2.enable = 2
        self.movePoint2.target_x = -4.956
        self.movePoint2.target_y = -10.5


        # parking
        self.msgRequestParking1.modeRun = 1
        self.msgRequestParking1.poseTarget.position.x = 2.4
        self.msgRequestParking1.poseTarget.position.y = -4.482
        self.msgRequestParking1.poseTarget.orientation.z = 0.999
        self.msgRequestParking1.poseTarget.orientation.w = 0.0087265
        self.msgRequestParking1.offset = 1.2

        self.msgRequestParking2.modeRun = 1
        self.msgRequestParking2.poseTarget.position.x = -4.956
        self.msgRequestParking2.poseTarget.position.y = -11.849
        self.msgRequestParking2.poseTarget.orientation.z = 0.707
        self.msgRequestParking2.poseTarget.orientation.w = 0.707
        self.msgRequestParking2.offset = 1.2

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
        self.respondMove = data
        self.sttGoalControl = data.complete_misson
        self.is_respondMove = True

    def callback_appButton(self, dat):
        self.app_button = dat

    def respondParking_callback(self, data):
        self.sttParking = data.status
        self.is_respondParking = True

    def respondOC_callback(self, data):
        self.sttOC = data.status
        self.is_respondOC = True

    def callback_getPose(self, dat):
        self.robotPose_nav = dat
        # doi quaternion -> rad    
        quaternion1 = (dat.pose.orientation.x, dat.pose.orientation.y,\
                    dat.pose.orientation.z, dat.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion1)

        self.NN_infoRespond.x = round(dat.pose.position.x, 3)
        self.NN_infoRespond.y = round(dat.pose.position.y, 3)
        self.NN_infoRespond.z = round(euler[2], 3)


    def synthetic_error(self):
        listError_now = []
        # -- Co vat can khi di chuyen giua cac diem
        if (self.respondMove.safety == 1):
            listError_now.append(411)

        # -- loi khoang cach
        if (self.respondMove.safety == 3):
            listError_now.append(500)

        return listError_now

    def scriptMove(self):
        if self.processmain == 0:
            self.listError = self.synthetic_error()
            self.processmain = 1

        elif self.processmain == 1:
            if self.app_button.bt_passHand == 1:
                if self.mode_operate == self.mode_auto: # keo co bao dang o tu dong -> chuyen sang bang tay.
                    self.flag_Auto_to_Byhand = 1
                self.mode_operate = self.mode_by_hand

            if self.app_button.bt_passAuto == 1:
                self.mode_operate = self.mode_auto
        
            if self.mode_operate == self.mode_by_hand:
                self.processmain = 10

            elif self.mode_operate == self.mode_auto:
                self.processmain = 30
        
        # -- BY HAND
        elif self.processmain == 10:
            self.msg.enable = 0
            self.msgRequestParking.modeRun = 0
            self.msgRequestOC.mode = 0

            self.processmain = 0

        # -- AUTO
        elif self.processmain == 30:
        # send status
            if self.process == 0:
                if self.is_respondMove:
                    print("recieve data")
                    self.process = 100

            elif self.process == 100:
                if self.sttGoalControl == 0:
                    self.msg = self.plan1
                    print("pub plan 3")
                    self.process = 1

            elif self.process == 1:
                self.msg.enable = 1
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
                self.msgRequestParking.modeRun = 1
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
                self.msgRequestOC.mode = 1
                if self.sttOC == 3:
                    self.msgRequestOC = OC_ForkLift_request()
                    self.process = 10
                    self.saveTime = rospy.get_time()

            elif self.process == 10:
                dentaT = rospy.get_time() - self.saveTime
                if dentaT > 3.:
                    print("wait done")
                    self.process = 14

            elif self.process == 14: # di ra
                if self.sttGoalControl == 0:
                    self.msg = self.movePoint1
                    print("pub plan 1")
                    self.process = 15

            elif self.process == 15:
                self.movePoint1.enable = 2
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
                    self.msg = self.plan2
                    print("pub plan 1")
                    self.process = 18

            elif self.process == 18:
                self.msg.enable = 1
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
                self.msgRequestParking.modeRun = 1
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
                self.msgRequestOC.mode = 1
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
                self.movePoint2.enable = 2
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

            self.processmain = 0


        if self.mode_operate == self.mode_by_hand:         # Che do by Hand
            self.NN_infoRespond.mode = 1

        elif self.mode_operate == self.mode_auto:          # Che do Auto
            self.NN_infoRespond.mode = 2
                                
        statusAGV = 0 if len(self.listError) == 0  else 1
        # send status
        self.NN_infoRespond.process = self.process
        self.NN_infoRespond.status = statusAGV
        self.NN_infoRespond.listError = self.synthetic_error()
        self.NN_infoRespond.task_status = 0
        self.NN_infoRespond.tag = 0
        self.NN_infoRespond.offset = 0

        print(self.process)
            
        self.pubMoveRequest.publish(self.msg)
        self.pubParkingRequest.publish(self.msgRequestParking)
        self.pubRequestOC.publish(self.msgRequestOC)
        self.pub_infoRespond.publish(self.NN_infoRespond)

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
