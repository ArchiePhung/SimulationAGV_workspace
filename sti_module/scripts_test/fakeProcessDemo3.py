#!/usr/bin/env python
# license removed for brevity
import roslib
import sys
import time
import rospy
import tf
from playsound import playsound
import threading
import signal
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import *
from message_pkg.msg import *

from math import sin, cos, asin, tan, atan, degrees, radians, sqrt, fabs, acos, atan2
from math import pi as PI


class FakeProcess(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(10)

        # sound
        self.typeMute = 0
        self.typeSoundMoving = 1
        self.typeSoundParking = 2
        self.typeSoundHaveObstacle = 3
        self.typeSoundMoveHand = 4
        self.pathSoundMoving = '/home/stivietnam/catkin_ws/src/sti_module/sound/movingAGV.mp3'
        self.pathSoundParking = '/home/stivietnam/catkin_ws/src/sti_module/sound/parkingAGV.mp3'
        self.pathSoundHaveObstacle = '/home/stivietnam/catkin_ws/src/sti_module/sound/vatcan.mp3'
        self.pathSoundModeHand = '/home/stivietnam/catkin_ws/src/sti_module/sound/piano.mp3'
        self.typeSoundNow = self.typeMute
        self.requestSound = self.typeMute
        self.enableSound = False
        self.saveTimeSound = rospy.get_time()

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
        self.plan5 = LineRequestMove()
        self.plan6 = LineRequestMove()

        self.movePoint = LineRequestMove()
        self.movePointSAC = LineRequestMove()
        self.movePointVT1 = LineRequestMove()
        self.movePointVT2 = LineRequestMove()

        self.pubParkingRequest = rospy.Publisher('/parking_request', Parking_request, queue_size=10)
        self.msgRequestParking = Parking_request()
        self.msgRequestParkingSAC = Parking_request()
        self.msgRequestParkingVT1 = Parking_request()
        self.msgRequestParkingVT2 = Parking_request()

        self.pubRequestOC = rospy.Publisher('/OC_request', OC_ForkLift_request, queue_size=10)
        self.msgRequestOC = OC_ForkLift_request()
        self.msgRequestOCNANG = OC_ForkLift_request()
        self.msgRequestOCHA = OC_ForkLift_request()

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

        self.codeStart = 0
        self.codeQT1 = 1
        self.codeQT2 = 2
        self.codeQTOut = 3
        self.saveQT = self.codeStart
        self.saveNextQT = self.codeQT1


        #---- sac to vt1 ----------------------------------------------------------------------
        self.plan1.enable = 1
        self.plan1.target_x = -4.956
        self.plan1.target_y = -10.5
        self.plan1.target_z = 1.57

        # Benze 
        # path  Benze 
        path = PathInfo()
        path.pathID = 1
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -10.5
        path.pointSecond = Pose()
        path.pointSecond.position.x = -1.367
        path.pointSecond.position.y = -9.036
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -9.036
        self.plan1.pathInfo.append(path)

        # path 1
        path = PathInfo()
        path.pathID = 2
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
        self.plan1.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 3
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
        self.plan1.pathInfo.append(path)


        #-- VT1 to VT2 -----------------------------------------------------------
        self.plan2.enable = 1
        self.plan2.target_x = 1.083
        self.plan2.target_y = -4.482
        self.plan2.target_z = 3.13

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
        self.plan2.pathInfo.append(path)

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
        self.plan2.pathInfo.append(path)

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
        self.plan2.pathInfo.append(path)

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
        self.plan2.pathInfo.append(path)

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
        self.plan2.pathInfo.append(path)

        #-- VT2 - SAC ---------------------------------------------------
        self.plan3.enable = 1
        self.plan3.target_x = -0.167
        self.plan3.target_y = -10.5
        self.plan3.target_z = 1.57

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
        self.plan3.pathInfo.append(path)

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
        self.plan3.pathInfo.append(path)

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
        self.plan3.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 4
        path.typePath = 3
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -1.367
        path.pointOne.position.y = -9.036
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -10.5
        path.velocity = 0.25
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.167
        path.pointMid.position.y = -9.036
        self.plan3.pathInfo.append(path)

        # quy trinh 2
        #-- sac to VT2 ---------------------------------------------------
        self.plan4.enable = 1
        self.plan4.target_x = 1.083
        self.plan4.target_y = -4.482
        self.plan4.target_z = 3.13

        # path 1
        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -10.5
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -2.982
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan4.pathInfo.append(path)

        # path 3 Benze 
        path = PathInfo()
        path.pathID = 2
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
        self.plan4.pathInfo.append(path)

        # -- VT2 to VT1 ------------------------------------------------------
        self.plan5.enable = 1
        self.plan5.target_x = -4.956
        self.plan5.target_y = -10.5
        self.plan5.target_z = 1.57

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
        self.plan5.pathInfo.append(path)

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
        self.plan5.pathInfo.append(path)


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
        self.plan5.pathInfo.append(path)

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
        self.plan5.pathInfo.append(path)

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
        self.plan5.pathInfo.append(path)


        # -- VT1 to SAC ----------------------------------------------------
        self.plan6.enable = 1
        self.plan6.target_x = -0.167
        self.plan6.target_y = -10.5
        self.plan6.target_z = 1.57

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
        self.plan6.pathInfo.append(path)

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
        self.plan6.pathInfo.append(path)

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
        self.plan6.pathInfo.append(path)


        # path 1
        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 2
        path.pointOne = Pose()
        path.pointOne.position.x = -0.167
        path.pointOne.position.y = -7.836
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.167
        path.pointSecond.position.y = -10.5
        path.velocity = 0.3
        path.movableZone = 1.2
        self.plan6.pathInfo.append(path)



        # ------------------------------------------------------------------------------
        # di chuyen ra
        self.movePointSAC.enable = 2
        self.movePointSAC.target_x = -0.167
        self.movePointSAC.target_y = -10.5

        self.movePointVT1.enable = 2
        self.movePointVT1.target_x = -4.956
        self.movePointVT1.target_y = -10.5

        self.movePointVT2.enable = 2
        self.movePointVT2.target_x = 1.083
        self.movePointVT2.target_y = -4.482


        # parking
        self.msgRequestParkingVT2.modeRun = 1
        self.msgRequestParkingVT2.poseTarget.position.x = 2.4
        self.msgRequestParkingVT2.poseTarget.position.y = -4.482
        self.msgRequestParkingVT2.poseTarget.orientation.z = 0.999
        self.msgRequestParkingVT2.poseTarget.orientation.w = 0.0087265
        self.msgRequestParkingVT2.offset = 1.2

        self.msgRequestParkingVT1.modeRun = 1
        self.msgRequestParkingVT1.poseTarget.position.x = -4.956
        self.msgRequestParkingVT1.poseTarget.position.y = -11.849
        self.msgRequestParkingVT1.poseTarget.orientation.z = 0.707
        self.msgRequestParkingVT1.poseTarget.orientation.w = 0.707
        self.msgRequestParkingVT1.offset = 1.2

        self.msgRequestParkingSAC.modeRun = 1
        self.msgRequestParkingSAC.poseTarget.position.x = -0.167
        self.msgRequestParkingSAC.poseTarget.position.y = -11.849
        self.msgRequestParkingSAC.poseTarget.orientation.z = 0.707
        self.msgRequestParkingSAC.poseTarget.orientation.w = 0.707
        self.msgRequestParkingSAC.offset = 1.2

        #------------------------------------------------------
        # pub request OC
        # nang
        self.msgRequestOCNANG.mode = 1
        self.msgRequestOCNANG.mission = 1
        self.msgRequestOCNANG.reqHeightLift = 10

        # ha
        self.msgRequestOCHA.mode = 1
        self.msgRequestOCHA.mission = 1
        self.msgRequestOCHA.reqHeightLift = 0

    def respondMove_callback(self, data):
        self.respondMove = data
        self.sttGoalControl = data.complete_misson
        if self.respondMove.misson != 0:
            if self.respondMove.safety == 1:
                self.requestSound = self.typeSoundHaveObstacle
            else:
                self.requestSound = self.typeSoundMoving

        self.is_respondMove = True

    def callback_appButton(self, dat):
        self.app_button = dat

    def respondParking_callback(self, data):
        self.sttParking = data.status
        if data.modeRun != 0:
            self.requestSound = self.typeSoundParking
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

    def pSound(self, type):
        if self.app_button.bt_spk_on:
            if self.typeSoundNow != type:
                self.typeSoundNow = type
                self.saveTimeSound = rospy.get_time()

            if self.typeSoundNow == self.typeSoundMoving:
                dentaTime = rospy.get_time() - self.saveTimeSound
                if dentaTime < 2.:
                    playsound(self.pathSoundMoving, block=True)

                elif dentaTime > 4.:
                    self.saveTimeSound = rospy.get_time()

            elif self.typeSoundNow == self.typeSoundParking:
                dentaTime = rospy.get_time() - self.saveTimeSound
                if dentaTime < 2.:
                    playsound(self.pathSoundParking, block=True)

                elif dentaTime > 4.:
                    self.saveTimeSound = rospy.get_time()

            elif self.typeSoundNow == self.typeSoundHaveObstacle:
                dentaTime = rospy.get_time() - self.saveTimeSound
                if dentaTime < 2.:
                    playsound(self.pathSoundHaveObstacle, block=True)

                elif dentaTime > 4.:
                    self.saveTimeSound = rospy.get_time()

    def synthetic_error(self):
        listError_now = []
        # -- Co vat can khi di chuyen giua cac diem
        if (self.respondMove.safety == 1):
            listError_now.append(411)

        # -- loi khoang cach
        if (self.respondMove.safety == 3):
            listError_now.append(500)

        return listError_now
    
    def moveOUTSAC(self):
        if self.process == 0:
            if self.is_respondMove:
                print("recieve data")
                self.process = 1

        elif self.process == 1:
            if self.sttGoalControl == 0:
                self.msg = self.movePointSAC
                print("pub move out sac")
                self.process = 2

        elif self.process == 2:
            self.msg.enable = 2
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 3
                self.saveTime = rospy.get_time()

        elif self.process == 3:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 5

        elif self.process == 5:
            print("done")

        return self.process
    
    def moveQT1(self):
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
            if dentaT > 1.:
                print("wait done")
                self.process = 3

        elif self.process == 3: # parking
            if self.is_respondParking:
                self.process = 4
        
        elif self.process == 4:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingVT1
                print("pub paking VT1")
                self.process = 5

        elif self.process == 5:
            self.msgRequestParking.modeRun = 1
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 6
                self.saveTime = rospy.get_time()

        elif self.process == 6:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 7

        elif self.process == 7: # doi tin hieu
            self.process = 8
            if self.is_respondOC:
                self.process = 8

        elif self.process == 8: 
            if self.sttOC == 0:
                self.msgRequestOC = self.msgRequestOCNANG
                print("pub OC ")
                self.process = 9

        elif self.process == 9:
            self.msgRequestOC.mode = 1
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 10
                self.saveTime = rospy.get_time()

        elif self.process == 10:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 14

        elif self.process == 14: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePointVT1
                print("pub plan 1")
                self.process = 15

        elif self.process == 15:
            self.msg.enable = 2
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 16
                self.saveTime = rospy.get_time()

        elif self.process == 16:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
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
            if dentaT > 1.:
                self.process = 20
                print("wait done")

        elif self.process == 20: # parking
            if self.is_respondParking:
                self.process = 21
        
        elif self.process == 21:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingVT2
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
            if dentaT > 1.:
                print("wait done")
                self.process = 24

        elif self.process == 24: #
            if self.sttOC == 0:
                self.msgRequestOC = self.msgRequestOCHA
                print("pub OC ")
                self.process = 25

        elif self.process == 25:
            self.msgRequestOC.mode = 1
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 26
                self.saveTime = rospy.get_time()

        elif self.process == 26:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 30

        elif self.process == 30: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePointVT2
                print("pub plan 1")
                self.process = 31

        elif self.process == 31:
            self.msg.enable = 2
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 32
                self.saveTime = rospy.get_time()

        elif self.process == 32:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 33

        elif self.process == 33:
            if self.sttGoalControl == 0:
                self.msg = self.plan3
                print("pub plan 3")
                self.process = 34

        elif self.process == 34:
            self.msg.enable = 1
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 35
                self.saveTime = rospy.get_time()

        elif self.process == 35:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                self.process = 36
                print("wait done")

        elif self.process == 36: # parking
            if self.is_respondParking:
                self.process = 37
        
        elif self.process == 37:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingSAC
                print("pub paking SAC")
                self.process = 38

        elif self.process == 38:
            self.msgRequestParking.modeRun = 1
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 39
                self.saveTime = rospy.get_time()

        elif self.process == 39:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 40

        elif self.process == 40:
            print("DONE QT1")

        return self.process

    def moveQT2(self):
        if self.process == 0:
            if self.is_respondMove:
                print("recieve data")
                self.process = 100

        elif self.process == 100:
            if self.sttGoalControl == 0:
                self.msg = self.plan4
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
            if dentaT > 1.:
                print("wait done")
                self.process = 3

        elif self.process == 3: # parking
            if self.is_respondParking:
                self.process = 4
        
        elif self.process == 4:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingVT2
                print("pub paking VT1")
                self.process = 5

        elif self.process == 5:
            self.msgRequestParking.modeRun = 1
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 6
                self.saveTime = rospy.get_time()

        elif self.process == 6:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 7

        elif self.process == 7: # doi tin hieu
            self.process = 8
            if self.is_respondOC:
                self.process = 8

        elif self.process == 8: 
            if self.sttOC == 0:
                self.msgRequestOC = self.msgRequestOCNANG
                print("pub OC ")
                self.process = 9

        elif self.process == 9:
            self.msgRequestOC.mode = 1
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 10
                self.saveTime = rospy.get_time()

        elif self.process == 10:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 14

        elif self.process == 14: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePointVT2
                print("pub plan 1")
                self.process = 15

        elif self.process == 15:
            self.msg.enable = 2
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 16
                self.saveTime = rospy.get_time()

        elif self.process == 16:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 17

        elif self.process == 17:
            if self.sttGoalControl == 0:
                self.msg = self.plan5
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
            if dentaT > 1.:
                self.process = 20
                print("wait done")

        elif self.process == 20: # parking
            if self.is_respondParking:
                self.process = 21
        
        elif self.process == 21:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingVT1
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
            if dentaT > 1.:
                print("wait done")
                self.process = 24

        elif self.process == 24: #
            if self.sttOC == 0:
                self.msgRequestOC = self.msgRequestOCHA
                print("pub OC ")
                self.process = 25

        elif self.process == 25:
            self.msgRequestOC.mode = 1
            if self.sttOC == 3:
                self.msgRequestOC = OC_ForkLift_request()
                self.process = 26
                self.saveTime = rospy.get_time()

        elif self.process == 26:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 30

        elif self.process == 30: # di ra
            if self.sttGoalControl == 0:
                self.msg = self.movePointVT1
                print("pub plan 1")
                self.process = 31

        elif self.process == 31:
            self.msg.enable = 2
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 32
                self.saveTime = rospy.get_time()

        elif self.process == 32:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 33

        elif self.process == 33:
            if self.sttGoalControl == 0:
                self.msg = self.plan6
                print("pub plan 3")
                self.process = 34

        elif self.process == 34:
            self.msg.enable = 1
            if self.sttGoalControl == 1:
                print("recive data Done")
                self.msg = LineRequestMove()
                self.process = 35
                self.saveTime = rospy.get_time()

        elif self.process == 35:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                self.process = 36
                print("wait done")

        elif self.process == 36: # parking
            if self.is_respondParking:
                self.process = 37
        
        elif self.process == 37:
            if self.sttParking == 1:
                self.msgRequestParking = self.msgRequestParkingSAC
                print("pub paking SAC")
                self.process = 38

        elif self.process == 38:
            self.msgRequestParking.modeRun = 1
            if self.sttParking == 8:
                self.msgRequestParking = Parking_request()
                self.process = 39
                self.saveTime = rospy.get_time()

        elif self.process == 39:
            dentaT = rospy.get_time() - self.saveTime
            if dentaT > 1.:
                print("wait done")
                self.process = 40

        elif self.process == 40:
            print("DONE QT2")

        return self.process

    def scripMove2(self):
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
            self.requestSound = self.typeMute
            self.msg.enable = 0
            self.msgRequestParking.modeRun = 0
            self.msgRequestOC.mode = 0

            self.processmain = 0

        # -- AUTO
        elif self.processmain == 30: # kiem tra agv hien tai
        # send status
            if self.saveNextQT == self.codeQTOut:
                self.processmain = 33

            else:
                if self.saveQT != self.saveNextQT:
                    self.saveQT = self.saveNextQT

                if self.saveQT == self.codeQT1:
                    self.processmain = 31 # 
                elif self.saveQT == self.codeQT2:
                    self.processmain = 32

        elif self.processmain == 31: # di chuyen QT1
            if self.moveQT1() == 40: # hoan thanh di chuyen
                self.process = 0
                self.saveNextQT = self.codeQTOut

            self.processmain = 0

        elif self.processmain == 32: # di chuyen QT2
            if self.moveQT2() == 40: # hoan thanh di chuyen
                self.process = 0
                self.saveNextQT = self.codeQTOut
            
            self.processmain = 0

        elif self.processmain == 33: # di chuyen ra
            if self.moveOUTSAC() == 5:
                self.process = 0
                if self.saveQT == self.codeQT1:
                    self.saveNextQT = self.codeQT2

                elif self.saveQT == self.codeQT2:
                    self.saveNextQT = self.codeQT1

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
        while not self.shutdown_flag.is_set():    
            self.scripMove2()
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

    rospy.init_node('make_process', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        br = FakeProcess(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            br.pSound(br.requestSound)
 
    except ServiceExit:
        
        br.shutdown_flag.set()
        # Wait for the threads to close...
        br.join()
    print('Exiting main program')

if __name__ == '__main__':
    main()

