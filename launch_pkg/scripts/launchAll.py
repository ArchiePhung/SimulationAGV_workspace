#!/usr/bin/env python3
# Author: HOANG VAN QUANG - BEE
# DATE  : 18/10/2021

from sensor_msgs.msg import LaserScan , Image , PointCloud2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int8

from sti_msgs.msg import * # Status_port Driver_respond, Driver_respond
from message_pkg.msg import *

import roslaunch
import rospy
import string
import time
import os

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # -- variable
        self.process = 0
        self.time_pre = time.time()

    def start(self):
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1  

    def start_and_wait(self, timeWait): # second - Dung cho cac node ko pub.
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1
            self.time_pre = time.time()
            return 0

        elif (self.process == 1): # - Wait
            t = (time.time() - self.time_pre)%60
            if (t > timeWait):
                self.process = 2
            return 0

        elif (self.process == 2): # - Wait
            return 1

class launcher():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('launcher', anonymous=False)
        self.rate = rospy.Rate(10)

        self.count_node = 0
        self.notification = ''
        self.step = 0
        self.timeWait = 0.4 # s

        self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
        self.stausLaunch = Status_launch()

        # -- module - firstWork.
        self.path_firstWork = rospy.get_param('path_firstWork', '')
        self.launch_firstWork = Launch(self.path_firstWork)
        rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
        self.is_firstWork = 0
        self.count_node += 1

        # -- module - checkPort.
        self.path_checkPort = rospy.get_param('path_checkPort', '')
        self.launch_checkPort = Launch(self.path_checkPort)
        rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
        self.is_checkPort = 0
        self.count_node += 1

        # -- module - reconnectBase.
        self.path_reconnectBase = rospy.get_param('path_reconnectBase', '')
        self.launch_reconnectBase = Launch(self.path_reconnectBase)
        rospy.Subscriber('/status_reconnectBase', Status_reconnect, self.callBack_reconnectBase)
        self.is_reconnectBase = 1
        self.count_node += 1

        # -- module - reconnectDriver.
        self.path_reconnectDriver = rospy.get_param('path_reconnectDriver', '')
        self.launch_reconnectDriver = Launch(self.path_reconnectDriver)
        rospy.Subscriber('/status_reconnectDriver', Status_reconnect, self.callBack_reconnectDriver)
        self.is_reconnectDriver = 1
        self.count_node += 1

        # -- module - Main.
        self.path_main = rospy.get_param('path_main', '')
        self.launch_main = Launch(self.path_main)
        rospy.Subscriber('/POWER_info', POWER_info, self.callBack_main)
        self.is_main = 0
        self.count_node += 1

        # -- module - driverAll.
        self.path_driverAll = rospy.get_param('path_driverAll', '')
        self.launch_driverAll = Launch(self.path_driverAll)
        rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
        rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
        self.is_driverLeft = 0
        self.is_driverRight = 0
        self.count_node += 1

        # -- module - nav350.
        self.path_nav350 = rospy.get_param('path_nav350', '')
        self.launch_nav350 = Launch(self.path_nav350)
        rospy.Subscriber('/scan', LaserScan, self.callBack_nav350)
        self.is_nav350 = 0
        self.count_node += 1

        # -- module - HC.
        self.path_hc = rospy.get_param('path_hc', '')
        self.launch_hc = Launch(self.path_hc)
        rospy.Subscriber('/HC_info', HC_info, self.callBack_hc)
        self.is_hc = 0
        self.count_node += 1

        # -- module - OC.
        self.path_oc = rospy.get_param('path_oc', '')
        self.launch_oc = Launch(self.path_oc)
        rospy.Subscriber('/lift_status', Lift_status, self.callBack_oc)
        self.is_oc = 0
        self.count_node += 1

        # -- module - imu.
        self.path_imu = rospy.get_param('path_imu', '')
        self.launch_imu = Launch(self.path_imu)
        rospy.Subscriber('/imu/data', Imu, self.callBack_imu)
        self.is_imu = 0
        self.count_node += 1

        # -- module - loadcell
        self.path_loadcell = rospy.get_param('path_loadcell', '')
        self.launch_loadcell = Launch(self.path_loadcell)
        rospy.Subscriber('/loadcell_respond', Loadcell_respond, self.callBack_loadcell)
        self.is_loadcell = 0
        self.count_node += 1

        # -- module - imuFilter.
        self.path_imuFilter = rospy.get_param('path_imuFilter', '')
        self.launch_imuFilter = Launch(self.path_imuFilter)
        rospy.Subscriber('/imu_filter', Imu, self.callBack_imuFilter)
        self.is_imuFilter = 0
        self.count_node += 1

        # -- module - kinematic.
        self.path_kinematic = rospy.get_param('path_kinematic', '')
        self.launch_kinematic = Launch(self.path_kinematic)
        rospy.Subscriber('/driver1_query', Driver_query, self.callBack_kinematic)
        self.is_kinematic = 0
        self.count_node += 1

        # -- get pose robot from nav.
        self.path_robotPoseNav = rospy.get_param('path_robotPoseNav', '')
        self.launch_robotPoseNav = Launch(self.path_robotPoseNav)
        rospy.Subscriber('/robotPose_nav', PoseStamped, self.callBack_robotPoseNav)
        self.is_robotPoseNav = 0
        self.count_node += 1

        # -- module - ekf.
        self.path_ekf = rospy.get_param('path_ekf', '')
        self.launch_ekf = Launch(self.path_ekf)
        rospy.Subscriber('/odometry', Odometry, self.callBack_ekf)
        self.is_ekf = 0
        self.count_node += 1

        # -- module - safety zone Nav350.
        self.path_safetyNav350 = rospy.get_param('path_safetyNav350', '')
        self.launch_safetyNav350 = Launch(self.path_safetyNav350)
        rospy.Subscriber('/safety_NAV', Int8, self.callBack_safetyNav350)
        self.is_safetyNav350 = 0
        self.count_node += 1

        # -- module - goalControl.
        self.path_goalControl = rospy.get_param('path_goalControl', '')
        self.launch_goalControl = Launch(self.path_goalControl)
        rospy.Subscriber('/status_goal_control', Status_goal_control, self.callBack_goalControl)
        self.is_goalControl = 0
        self.count_node += 1

        # -- module - parkingControl.
        self.path_parkingControl = rospy.get_param('path_parkingControl', '')
        self.launch_parkingControl = Launch(self.path_parkingControl)
        rospy.Subscriber('/parking_respond', Parking_respond, self.callBack_parkingControl)
        self.is_parkingControl = 0
        self.count_node += 1

        # -- module - stiClient.
        self.path_client = rospy.get_param('path_client', '')
        self.launch_client = Launch(self.path_client)
        rospy.Subscriber('/NN_infoRequest', NN_infoRequest, self.callBack_client)
        self.is_client = 0
        self.count_node += 1

        # -- module - stiControl.
        self.path_control = rospy.get_param('path_control', '')
        self.launch_control = Launch(self.path_control)
        rospy.Subscriber('/NN_infoRespond', NN_infoRespond, self.callBack_control)
        self.is_control = 0
        self.count_node += 1

        # -- module - data App.
        self.path_dataApp = rospy.get_param('path_dataApp', '')
        self.launch_dataApp = Launch(self.path_dataApp)
        self.count_node += 1

        # -- module - posePublisher.
        self.path_posePublisher = rospy.get_param('path_posePublisher', '')
        self.launch_posePublisher = Launch(self.path_posePublisher)
        rospy.Subscriber('/robot_pose', Pose, self.callBack_posePublisher)
        self.is_posePublisher = 0
        self.count_node += 1
        # -- ko dc xoa
        self.count_node += 1
        
    def callBack_firstWork(self, data):
        self.is_firstWork = 1

    def callBack_checkPort(self, data):
        self.is_checkPort = 1

    def callBack_reconnectBase(self, data):
        self.is_reconnectBase = 1

    def callBack_reconnectDriver(self, data):
        self.is_reconnectDriver = 1

    def callBack_main(self, data):
        self.is_main = 1

    def callBack_driverLeft(self, data):
        self.is_driverLeft = 1

    def callBack_driverRight(self, data):
        self.is_driverRight = 1

    def callBack_imu(self, data):
        self.is_imu = 1

    def callBack_loadcell(self, data):
        self.is_loadcell = 1

    def callBack_imuFilter(self, data):
        self.is_imuFilter = 1

    def callBack_nav350(self, data):
        self.is_nav350 = 1

    def callBack_robotPoseNav(self, data):
        self.is_robotPoseNav = 1

    def callBack_hc(self, data):
        self.is_hc = 1

    def callBack_oc(self, data):
        self.is_oc = 1

    def callBack_kinematic(self, data):
        self.is_kinematic = 1

    def callBack_rawOdom(self, data):
        self.is_rawOdom = 1

    def callBack_ekf(self, data):
        self.is_ekf = 1

    def callBack_safetyNav350(self, data):
        self.is_safetyNav350 = 1

    def callBack_goalControl(self, data):
        self.is_goalControl = 1

    def callBack_parkingControl(self, data):
        self.is_parkingControl = 1

    def callBack_client(self, data):
        self.is_client = 1

    def callBack_control(self, data):
        self.is_control = 1

    def callBack_posePublisher(self, data):
        self.is_posePublisher = 1

    def run(self):
        while not rospy.is_shutdown():
            # print "runing"
            # -- firstWork
            if (self.step == 0):
                self.notification = 'launch_firstWork'
                self.launch_firstWork.start()
                if (self.is_firstWork == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- checkPort
            elif (self.step == 1):
                self.notification = 'launch_checkPort'
                self.launch_checkPort.start()
                if (self.is_checkPort == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- reconnectBase
            elif (self.step == 2):
                self.notification = 'launch_reconnectBase'
                self.launch_reconnectBase.start()
                if (self.is_reconnectBase == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- reconnectDriver
            elif (self.step == 3):
                self.notification = 'launch_reconnectDriver'
                self.launch_reconnectDriver.start()
                if (self.is_reconnectDriver == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- data APP
            elif (self.step == 4):
                self.notification = 'launch_dataApp'
                sts = self.launch_dataApp.start_and_wait(3.)
                if (sts == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- main
            elif (self.step == 5):
                self.notification = 'launch_main'
                self.launch_main.start()
                if (self.is_main == 1):
                    # self.step += 1
                    self.step = 9
                    time.sleep(self.timeWait)

            # -- hc
            # elif (self.step == 6):
            #     self.notification = 'launch_hc'
            #     self.launch_hc.start()
            #     if (self.is_hc == 1):
            #         self.step += 1
            #         time.sleep(self.timeWait)

            # -- oc
            # elif (self.step == 7):
            #     self.notification = 'launch_oc'
            #     self.launch_oc.start()
            #     if (self.is_oc == 1):
            #         self.step += 1
            #         time.sleep(self.timeWait)

            # -- imu
            # elif (self.step == 8):
            #     self.notification = 'launch_imu'
            #     self.launch_imu.start()
            #     if (self.is_imu == 1):
            #         self.step += 1
            #         time.sleep(self.timeWait)

            # -- loadcell
            elif (self.step == 9):
                 self.notification = 'launch_loadcell'
                 self.launch_loadcell.start()
                 if (self.is_loadcell == 1):
                     self.step += 1
                     time.sleep(self.timeWait)

            # -- driverAll
            elif (self.step == 10):
                self.notification = 'launch_driverAll'
                self.launch_driverAll.start()
                if (self.is_driverLeft == 1 and self.is_driverRight == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- nav350
            elif (self.step == 11):
                self.notification = 'launch_nav350'
                self.launch_nav350.start()
                if (self.is_nav350 == 1):
                    self.step = 13
                    time.sleep(self.timeWait)

            # -- imuFilter
            # elif (self.step == 12):
            #     self.notification = 'launch_imuFilter'
            #     self.launch_imuFilter.start()
            #     if (self.is_imuFilter == 1):
            #         self.step += 1
            #         time.sleep(self.timeWait)

            # -- kinematic
            elif (self.step == 13):
                self.notification = 'launch_kinematic'
                self.launch_kinematic.start()
                if (self.is_kinematic == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- get pose robot from nav.
            elif (self.step == 14):
                self.notification = 'launch_robotPoseNav'
                self.launch_robotPoseNav.start()
                if (self.is_robotPoseNav == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- ekf
            elif (self.step == 15):
                self.notification = 'launch_ekf'
                self.launch_ekf.start()
                if (self.is_ekf == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- safety zone Nav350
            elif (self.step == 16):
                self.notification = 'launch_safetyNav350'
                self.launch_safetyNav350.start()
                if (self.is_safetyNav350 == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- goal control
            elif (self.step == 17):
                self.notification = 'launch_goalControl'
                self.launch_goalControl.start()
                if (self.is_goalControl == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- parking control
            elif (self.step == 18):
                self.notification = 'launch_parkingControl'
                self.launch_parkingControl.start()
                if (self.is_parkingControl == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- client
            elif (self.step == 19):
                self.notification = 'launch_StiClient'
                sts = self.launch_client.start_and_wait(3.)
                if (self.is_client == 1 or sts == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- control
            elif (self.step == 20):
                self.notification = 'launch_StiControl'
                self.launch_control.start()
                if (self.is_control == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # # -- posePublisher
            # elif (self.step == 18):
            # 	self.notification = 'launch_posePublisher'
            # 	self.launch_posePublisher.start()
            # 	if (self.is_posePublisher == 1):
            # 		self.step += 1
            # 		time.sleep(self.timeWait)

            # -- Completed
            elif (self.step == 21):
                self.notification = 'Completed!'

            # -- -- PUBLISH STATUS
            # self.stausLaunch.persent = int((self.step/self.count_node)*100.)
            self.stausLaunch.persent = int((self.step/21)*100.)
            self.stausLaunch.position = self.step
            self.stausLaunch.notification = self.notification
            self.pub_stausLaunch.publish(self.stausLaunch)
            # time.sleep(0.1)
            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = launcher()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()
