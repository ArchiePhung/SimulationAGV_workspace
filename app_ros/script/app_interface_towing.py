#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Ho Phuc Hoang
Company: STI Viet Nam
Date: 5/1/2024
"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal
import json

import random

import os
import re  
import subprocess
import argparse
from datetime import datetime

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPaintEvent
from PyQt5.QtWidgets import QWidget

sys.path.append('/home/tiger/simulation_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy

from sti_tow12demo_msgs.msg import *
from message_tow12demo_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from sick_lidar_localization.srv import LocGetLocalizationStatusSrv, LocInitializeAtPoseSrv, LocAutoStartSavePoseSrv
from sick_lidar_localization.msg import LocalizationControllerResultMessage0502

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from enum import Enum

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * # QDialog, QApplication, QWidget
from PyQt5.QtGui import * # QPixmap
from PyQt5.QtCore import * # QTimer, QDateTime, Qt

import sqlite3

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class statusButton:
    def __init__(self):
        # button left side and righ side
        self.bt_passAuto = 0
        self.bt_passHand = 0
        self.bt_cancelMission = 0
        self.bt_getInfo = 0
        self.bt_setting = 0
        self.bt_clearError = 0
        self.bt_exit = 0
        
        # button hand mode move page 1
        self.bt_changePage2 = 0
        self.bt_forwards = 0
        self.bt_backwards = 0
        self.bt_rotation_left = 0
        self.bt_rotation_right = 0
        self.bt_stop = 0
        self.bt_moveHand = 0
        
        # button hand mode move page 2
        self.bt_changePage1 = 0
        self.bt_chg_on = 0
        self.bt_chg_off = 0
        self.bt_spk_on = 0
        self.bt_spk_off = 0
        self.bt_speaker = 0
        self.bt_charger = 0
        self.bt_disableBrake = 0
        self.bt_setpose = 0

        # button auto mode
        self.bt_doNextPlan = 0

        self.ck_remote = 0

        self.vs_speed = 50

class statusColor:
    def __init__(self):
    # --
        self.lbc_safety_ahead = 0
        self.lbc_safety_behind = 0
        # --
        self.cb_status = 0
        self.lbc_battery = 0
        self.lbc_setpose = 0

        # --
        self.lbc_button_clearError = 0
        self.lbc_button_power = 0
        self.lbc_blsock = 0
        self.lbc_emg = 0

        self.lbc_limit_up = 0
        self.lbc_limit_down = 0
        self.lbc_detect_lifter = 0

        self.lbc_port_rtc = 0
        self.lbc_port_driverLeft = 0
        self.lbc_port_driverRight = 0
        self.lbc_port_lidar = 0

        

        # self. = 0
        # self. = 0
        # self. = 0
        # self. = 0
        # self. = 0

class valueLable:
    def __init__(self):
        self.modeRuning = 1

        self.lbv_name_agv = ''
        self.lbv_mac = ''
        self.lbv_ip = ''
        self.lbv_battery = ''
        self.lbv_date = ''

        self.lbv_coordinates_x = 0.
        self.lbv_coordinates_y = 0.
        self.lbv_coordinates_r = 0.
        self.lbv_pingServer = ''

        self.lbv_map_match_status = ''
        self.lbv_loc_status = ''

        self.lbv_errorPath = ''

        self.lbv_goalFollow_id = ''
        self.lbv_route_message = ''
        self.lbv_jobRuning = ''

        self.lbv_mac = ''
        self.lbv_namePc = ''

        self.lbv_launhing = ''
        self.lbv_numberLaunch = ''
        self.percentLaunch = 0
        self.listError = ['A', 'B', 'C']
        self.listError_pre = []

        self.listIDSetpose = []
        self.isUpdateListIdSetpose = True

        self.listPath = []
        self.listPath_pre = []

        self.lbv_qualityWifi = 0
        # -
        self.list_logError = []
        self.list_logError_pre = []
        # -
        self.lbv_idPathFollow = ''
        self.lbv_typeLineFollow = ''
        self.lbv_idtarget = ''
        self.lbv_xTarget = ''
        self.lbv_ytarget = ''
        self.lbv_rTarget = ''


class TypeLine(Enum):
    STRAIGHTLINE = 1
    CIRCLELINE = 2
    BEZIERLINE = 3

class infoPath:
    def __init__(self):
        self.typeLine = TypeLine
        self.startPoint = Point()
        self.endPoint = Point()
        self.midPoint = Point()
        self.isTarget = False

class CanvasWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.listPath = []
        self.xAGV = 0.
        self.yAGV = 0.
        self.rAGV = 0.

        self.isUpdatePath = False
        self.isUpdateAGV = False

        self.offset_width = 20
        self.offset_height = 20
        self.dentaX = 0
        self.dentaY = 0
        self.max_X = 0.
        self.max_Y = 0.
        self.min_X = 0.
        self.min_Y = 0.

        # path image
        self.pathIconTarget = '/home/tiger/simulation_ws/src/app_ros/interface/toado.png'
        self.pathIconAGV = '/home/tiger/simulation_ws/src/app_ros/interface/iconmuiten _resize.png'

    def makePath(self, listPath):
        self.listPath = []
        max_X = 0.
        max_Y = 0.
        min_X = 1000.
        min_Y = 1000.
        for p in listPath:
            if max_X < p.startPoint.x:
                max_X = p.startPoint.x

            if min_X > p.startPoint.x:
                min_X = p.startPoint.x

            if max_Y < p.startPoint.y:
                max_Y = p.startPoint.y
                
            if min_Y > p.startPoint.y:
                min_Y = p.startPoint.y

            if max_X < p.endPoint.x:
                max_X = p.endPoint.x

            if min_X > p.endPoint.x:
                min_X = p.endPoint.x

            if max_Y < p.endPoint.y:
                max_Y = p.endPoint.y
                
            if min_Y > p.endPoint.y:
                min_Y = p.endPoint.y

            if p.typeLine == TypeLine.BEZIERLINE:
                if max_X < p.midPoint.x:
                    max_X = p.midPoint.x

                if min_X > p.midPoint.x:
                    min_X = p.midPoint.x

                if max_Y < p.midPoint.y:
                    max_Y = p.midPoint.y
                    
                if min_Y > p.midPoint.y:
                    min_Y = p.midPoint.y

        self.max_X = max_X
        self.min_X = min_X
        self.max_Y = max_Y
        self.min_Y = min_Y
        self.dentaX = max_X - min_X
        self.dentaY = max_Y - min_Y

        for p in listPath:
            cvPath = infoPath()
            cvPath.typeLine = p.typeLine
            cvPath.isTarget = p.isTarget
            cvPath.startPoint.x = self.offset_width + int(((p.startPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
            cvPath.startPoint.y = self.offset_height + int(((p.startPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

            cvPath.endPoint.x = self.offset_width + int(((p.endPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
            cvPath.endPoint.y = self.offset_height + int(((p.endPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

            if p.typeLine == TypeLine.BEZIERLINE:
                cvPath.midPoint.x = self.offset_width + int(((p.midPoint.x - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
                cvPath.midPoint.y = self.offset_height + int(((p.midPoint.y - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))

            # print("start Point: ", cvPath.startPoint)
            # print("end Point: ", cvPath.endPoint)
            # print("mid Point: ", cvPath.midPoint)
            # print('-------------------------------')

            self.listPath.append(cvPath)

    def makeAGV(self, x_rb, y_rb, r_rb):
        self.xAGV = self.offset_width + int(((x_rb - self.min_X)/self.dentaX)*(self.geometry().width() - 2*self.offset_width))
        self.yAGV = self.offset_height + int(((y_rb - self.min_Y)/self.dentaY)*(self.geometry().height() - 2*self.offset_height))
        # euler = euler_from_quaternion([_poseA.orientation.x, _poseA.orientation.y, _poseA.orientation.z, _poseA.orientation.w])
        # self.rAGV = euler[2]
        self.rAGV = r_rb + 90.

    def paintEvent(self, event):
        # Create a painting object
        ThePainter = QPainter()
        # Do the painting
        ThePainter.begin(self) # everything between here and the end() call will be drawn into the Widget
        if self.isUpdatePath:
            self.isUpdatePath = False
            ThePen=QPen(Qt.green) # set a color for a pen.  Pens draw lines and outlines
            for path in self.listPath:
                if path.isTarget:
                    pixmap = QPixmap(self.pathIconTarget)
                    ThePainter.drawPixmap(path.endPoint.x - 15, path.endPoint.y - 30, pixmap)
                else:
                    ThePen.setWidth(10)
                    ThePainter.setPen(ThePen) # set the pen into the painter
                    ThePainter.drawPoint(path.endPoint.x, path.endPoint.y)

                ThePen.setWidth(2)
                ThePainter.setPen(ThePen) # set the pen into the painter
                if path.typeLine == TypeLine.STRAIGHTLINE:
                    ThePainter.drawLine(path.startPoint.x, path.startPoint.y, path.endPoint.x, path.endPoint.y)

                elif path.typeLine == TypeLine.BEZIERLINE:
                    pathCubic = QPainterPath()
                    pathCubic.moveTo(path.startPoint.x, path.startPoint.y)
                    pathCubic.cubicTo(path.startPoint.x, path.startPoint.y, path.midPoint.x, path.midPoint.y, path.endPoint.x, path.endPoint.y)
                    ThePainter.drawPath(pathCubic)

        if self.isUpdateAGV:
            self.isUpdateAGV = False
            pixmap = QPixmap(self.pathIconAGV)
            transform = QTransform().rotate(self.rAGV)
            pixmap = pixmap.transformed(transform)
            ThePainter.drawPixmap(self.xAGV - 12, self.yAGV - 12, pixmap)

        ThePainter.end() # finishing drawing

class WelcomeScreen(QDialog):
    def __init__(self):
        super(WelcomeScreen, self).__init__()
        loadUi("/home/tiger/simulation_ws/src/app_ros/interface/app_towing.ui", self)
        # --
        self.statusButton = statusButton()
        self.statusColor  = statusColor()
        self.valueLable   = valueLable()

        # -- 
        # Add a box layout into the frame
        SettingsLayout = QVBoxLayout()
        self.fr_pathPlan.setLayout(SettingsLayout)
        self.TheCanvas = CanvasWidget() # create the canvas widget that will do the painting
        SettingsLayout.addWidget(self.TheCanvas)  # add the canvas to the layout within the frame
        SettingsLayout.setContentsMargins(0, 0, 0, 0)
        # -
        # -- frame left side ---------------------------------------
        self.bt_exit.pressed.connect(self.out)
        # -
        self.bt_cancelMission.pressed.connect(self.pressed_cancelMission)
        self.bt_cancelMission.released.connect(self.released_cancelMission)
        self.status_cancelMission = 0
        self.timeSave_cancelMisson = rospy.Time.now()
        # -
        self.bt_passHand.pressed.connect(self.pressed_passHand)
        self.bt_passHand.released.connect(self.released_passHand)
        # -
        self.bt_passAuto.pressed.connect(self.pressed_passAuto)
        self.bt_passAuto.released.connect(self.released_passAuto)
        # -
        # -- frame right side ---------------------------------------
        self.bt_getInfo.pressed.connect(self.pressed_getInfo)
        self.bt_getInfo.released.connect(self.released_getInfo)
        # -- 
        self.timeSave_getInfo = rospy.Time.now()
        self.status_getInfo = 0
        # -
        self.bt_clearError.pressed.connect(self.pressed_clearError)
        self.bt_clearError.released.connect(self.released_clearError)

        # -- Setting devices
        self.bt_setting.pressed.connect(self.pressed_setting)
        self.bt_setting.released.connect(self.released_setting)
        self.status_setting = 0
        self.timeSave_setting = rospy.Time.now()
        # -
        # self.bt_hideSetting.clicked.connect(self.clicked_hideSetting)

        # -- page manual 1 ------------------------------------
        self.bt_forwards.clicked.connect(self.clicked_forwards)
        self.bt_backwards.clicked.connect(self.clicked_backwards)
        # -
        self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
        self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
        # -
        self.bt_stop.clicked.connect(self.clicked_stop)
        self.bt_changePage2.pressed.connect(self.pressed_changePage2)
        self.bt_changePage2.released.connect(self.released_changePage2)

        # -- page manual 2 -------------------------------------
        self.statusButton.bt_speaker = 1
        self.bt_speaker_on.clicked.connect(self.clicked_speaker_on)
        self.bt_speaker_off.clicked.connect(self.clicked_speaker_off)
        # -
        self.bt_charger_on.clicked.connect(self.clicked_charger_on)
        self.bt_charger_off.clicked.connect(self.clicked_charger_off)
        # -- 
        self.bt_disableBrake_on.clicked.connect(self.clicked_brakeOn)
        self.bt_disableBrake_off.clicked.connect(self.clicked_brakeOff)
        self.bt_changePage1.pressed.connect(self.pressed_changePage1)
        self.bt_changePage1.released.connect(self.released_changePage1)

        self.bt_setpose.pressed.connect(self.pressed_setpose)
        self.bt_setpose.released.connect(self.released_setpose)

        # -- frame auto
        self.bt_doNextPlan.pressed.connect(self.pressed_doNextPlan)
        self.bt_doNextPlan.released.connect(self.released_doNextPlan)

        # -
        self.bt_pw_cancel.clicked.connect(self.clicked_password_cancel)
        self.bt_pw_agree.clicked.connect(self.clicked_password_agree)
        self.password_data = ""
        self.password_right = "1412"
        self.bt_pw_0.clicked.connect(self.clicked_password_n0)
        self.bt_pw_1.clicked.connect(self.clicked_password_n1)
        self.bt_pw_2.clicked.connect(self.clicked_password_n2)
        self.bt_pw_3.clicked.connect(self.clicked_password_n3)
        self.bt_pw_4.clicked.connect(self.clicked_password_n4)
        self.bt_pw_5.clicked.connect(self.clicked_password_n5)
        self.bt_pw_6.clicked.connect(self.clicked_password_n6)
        self.bt_pw_7.clicked.connect(self.clicked_password_n7)
        self.bt_pw_8.clicked.connect(self.clicked_password_n8)
        self.bt_pw_9.clicked.connect(self.clicked_password_n9)
        self.bt_pw_clear.clicked.connect(self.clicked_password_clear)
        self.bt_pw_delete.clicked.connect(self.clicked_password_delete)

        self.ck_remote.stateChanged.connect(self.stateChanged_ck_remote)

        # -
        self.bt_exit_fr_infoDevice.clicked.connect(self.clicked_exit_fr_info)
        self.bt_exit_fr_infoModeAuto.clicked.connect(self.clicked_exit_fr_info)

        # -- Set speed manual
        self.bt_upSpeed.pressed.connect(self.pressed_upSpeed)
        self.bt_reduceSpeed.pressed.connect(self.pressed_reduceSpeed)

        # -- -- -- Timer updata data
        # -- Fast
        timer_fast = QTimer(self)
        timer_fast.timeout.connect(self.process_fast)
        timer_fast.start(50)
        # -- normal
        timer_normal = QTimer(self)
        timer_normal.timeout.connect(self.process_normal)
        timer_normal.start(996)
        # -- Slow
        timer_slow = QTimer(self)
        timer_slow.timeout.connect(self.process_slow)
        timer_slow.start(3000)
        # --
        self.modeRuning = 0
        self.modeRun_launch = 0
        self.modeRun_byhand = 1
        self.modeRun_auto = 2
        self.modeRun_cancelMission = 5

        self.modeRuning = self.modeRun_launch
        self.saveModeRuning = self.modeRun_launch
        # --

        self.password_data = ""
        # - 
        self.handMode1 = 1
        self.handMode2 = 2
        self.handMode = self.handMode1

    def out(self):
        QApplication.quit()
        print('out')

    def stateChanged_ck_remote(self):
        self.clicked_stop()

    def pressed_cancelMission(self):
        self.bt_cancelMission.setStyleSheet("background-color: blue;")
        self.statusButton.bt_cancelMission = 1

    def released_cancelMission(self):
        self.bt_cancelMission.setStyleSheet("background-color: white;")
        self.statusButton.bt_cancelMission = 0
        self.status_setting = 0

    def pressed_passHand(self):
        self.statusButton.bt_passHand = 1
        self.bt_passHand.setStyleSheet("background-color: blue;")

    def released_passHand(self):
        self.statusButton.bt_passHand = 0
        self.bt_passHand.setStyleSheet("background-color: white;")
        self.status_setting = 0

    def pressed_passAuto(self):
        self.statusButton.bt_passAuto = 1
        self.clicked_stop()
        # --
        self.statusButton.bt_disableBrake = 0
        self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_on.setStyleSheet("background-color: white;")

    def released_passAuto(self):
        self.statusButton.bt_passAuto = 0
        self.status_setting = 0

    def pressed_getInfo(self):
        self.statusButton.bt_getInfo = 1
        self.clicked_stop()
        # --
        self.bt_getInfo.setStyleSheet("background-color: blue;")

    def released_getInfo(self):
        self.statusButton.bt_getInfo = 0
        self.bt_getInfo.setStyleSheet("background-color: white;")

    def pressed_clearError(self):
        self.statusButton.bt_clearError = 1
        self.bt_clearError.setStyleSheet("background-color: blue;")
        self.clicked_stop()

    def released_clearError(self):
        self.statusButton.bt_clearError = 0
        self.bt_clearError.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_speaker_on(self):
        # self.statusButton.bt_spk_on = 1
        # self.statusButton.bt_spk_off = 0
        self.statusButton.bt_speaker = 1
        self.bt_speaker_on.setStyleSheet("background-color: blue;")
        self.bt_speaker_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_speaker_off(self):
        # self.statusButton.bt_spk_on = 0
        # self.statusButton.bt_spk_off = 1
        self.statusButton.bt_speaker = 0
        self.bt_speaker_off.setStyleSheet("background-color: blue;")
        self.bt_speaker_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_charger_on(self):
        # self.statusButton.bt_chg_on = 1
        # self.statusButton.bt_chg_off = 0
        self.statusButton.bt_charger = 1
        self.bt_charger_on.setStyleSheet("background-color: blue;")
        self.bt_charger_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_charger_off(self):
        # self.statusButton.bt_chg_on = 0
        # self.statusButton.bt_chg_off = 1
        self.statusButton.bt_charger = 0
        self.bt_charger_off.setStyleSheet("background-color: blue;")
        self.bt_charger_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_brakeOn(self):
        self.statusButton.bt_disableBrake = 1
        self.bt_disableBrake_on.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_brakeOff(self):
        self.statusButton.bt_disableBrake = 0
        self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    # -
    def pressed_setpose(self):
        self.statusButton.bt_setpose = 1
        # self.statusColor.lbc_setpose = 2
        self.clicked_stop()
        
    def released_setpose(self):
        self.statusButton.bt_setpose = 0
        # self.statusColor.lbc_setpose = 0
        self.clicked_stop()


    def clicked_forwards(self):
        # self.statusButton.bt_forwards = 1
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 1
        self.bt_forwards.setStyleSheet("background-color: blue;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_backwards(self):
        # self.statusButton.bt_backwards = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 2
        self.bt_backwards.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_rotation_left(self):
        # self.statusButton.bt_rotation_left = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 3
        self.bt_rotation_left.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_rotation_right(self):
        # self.statusButton.bt_rotation_right = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 4
        self.bt_rotation_right.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_stop(self):
        # self.statusButton.bt_stop = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_backwards = 0

        self.statusButton.bt_moveHand = 0

        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: blue;")

    def pressed_changePage2(self):
        self.statusButton.bt_changePage2 = 1
        self.bt_changePage2.setStyleSheet("background-color: blue;")

    def released_changePage2(self):
        self.statusButton.bt_changePage2 = 0
        self.bt_changePage2.setStyleSheet("background-color: white;")
        self.handMode = self.handMode2

    def pressed_changePage1(self):
        self.statusButton.bt_changePage1 = 1
        self.bt_changePage1.setStyleSheet("background-color: blue;")

    def released_changePage1(self):
        self.statusButton.bt_changePage1 = 0
        self.bt_changePage1.setStyleSheet("background-color: white;")
        self.handMode = self.handMode1

    def pressed_doNextPlan(self):
        self.statusButton.bt_doNextPlan = 1
        self.bt_doNextPlan.setStyleSheet("background-color: blue;")

    def released_doNextPlan(self):
        self.statusButton.bt_doNextPlan = 0
        self.bt_doNextPlan.setStyleSheet("background-color: rgb(80,249,255);")

    def clicked_exit_fr_info(self):
        self.status_getInfo = 0

    def pressed_upSpeed(self):
        self.statusButton.vs_speed += 10
        if self.statusButton.vs_speed >= 100:
            self.statusButton.vs_speed = 100

    def pressed_reduceSpeed(self):
        self.statusButton.vs_speed -= 10
        if self.statusButton.vs_speed < 5:
            self.statusButton.vs_speed = 5

    # --  --
    def clicked_password_agree(self):
        self.password_data = ""
        self.status_cancelMission = 0

    def clicked_password_cancel(self):
        self.password_data = ""
        self.status_cancelMission = 0

    def clicked_hideSetting(self):
        self.status_setting = 0
        self.password_data = ""
        self.enable_showToyoWrite = 0

    def pressed_setting(self):
        self.bt_setting.setStyleSheet("background-color: blue;")	
        self.setting_status = 1

    def released_setting(self):
        self.bt_setting.setStyleSheet("background-color: white;")	
        self.setting_status = 0

    # - 
    def clicked_password_n0(self):
        if (len(self.password_data) < 4):
            self.password_data += "0"

    def clicked_password_n1(self):
        if (len(self.password_data) < 4):
            self.password_data += "1"

    def clicked_password_n2(self):
        if (len(self.password_data) < 4):
            self.password_data += "2"

    def clicked_password_n3(self):
        if (len(self.password_data) < 4):
            self.password_data += "3"

    def clicked_password_n4(self):
        if (len(self.password_data) < 4):
            self.password_data += "4"

    def clicked_password_n5(self):
        if (len(self.password_data) < 4):
            self.password_data += "5"

    def clicked_password_n6(self):
        if (len(self.password_data) < 4):
            self.password_data += "6"

    def clicked_password_n7(self):
        if (len(self.password_data) < 4):
            self.password_data += "7"

    def clicked_password_n8(self):
        if (len(self.password_data) < 4):
            self.password_data += "8"

    def clicked_password_n9(self):
        if (len(self.password_data) < 4):
            self.password_data += "9"

    def clicked_password_clear(self):
        self.password_data = ""

    def clicked_password_delete(self):
        lenght = len(self.password_data)
        if (lenght > 0):
            self.password_data = self.password_data[:-1]

    # Show label
    def set_dateTime(self):
        time_now = datetime.now()
        # dd/mm/YY H:M:S
        dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
        self.lbv_date.setText(dt_string)

    def set_labelValue(self): # App_lbv()
        self.pb_qualityWifi.setValue(self.valueLable.lbv_qualityWifi)
        self.pb_speed.setValue(self.statusButton.vs_speed)
        # --

        self.lbv_coordinates_x.setText(str(self.valueLable.lbv_coordinates_x))
        self.lbv_coordinates_y.setText(str(self.valueLable.lbv_coordinates_y))
        self.lbv_coordinates_r.setText(str(self.valueLable.lbv_coordinates_r))

        self.lbv_pingServer.setText(self.valueLable.lbv_pingServer)
        self.lbv_jobRuning.setText(self.valueLable.lbv_jobRuning)
        # self.lbv_goalFollow_id.setText(self.valueLable.lbv_goalFollow_id)
        self.lbv_route_message.setText(self.valueLable.lbv_route_message)

        self.lbv_status_lidarLoc.setText(self.valueLable.lbv_loc_status)
        self.lbv_map_match.setText(self.valueLable.lbv_map_match_status)

        self.lbv_errorPath.setText(self.valueLable.lbv_errorPath)

    def set_labelColor(self):
        # ---- Safety
        # -
        if (self.statusColor.lbc_safety_ahead == 0):
            self.lbc_safety1.setStyleSheet("background-color: green; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: green; color: white")

        elif (self.statusColor.lbc_safety_ahead == 1):
            self.lbc_safety1.setStyleSheet("background-color: red; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
        elif (self.statusColor.lbc_safety_ahead == 2):
            self.lbc_safety1.setStyleSheet("background-color: orange; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
        else:
            self.lbc_safety1.setStyleSheet("background-color: yellow;")
            # self.lbc_safety_ahead.setStyleSheet("background-color: yellow;")
        # -
        if (self.statusColor.lbc_safety_behind == 0):
            self.lbc_safety2.setStyleSheet("background-color: green; color: white")
            # self.lbc_safety_behind.setStyleSheet("background-color: green; color: white")

        elif (self.statusColor.lbc_safety_behind == 1):
            self.lbc_safety2.setStyleSheet("background-color: red; color: white")
            # self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
        elif (self.statusColor.lbc_safety_behind == 2):
            self.lbc_safety2.setStyleSheet("background-color: orange; color: white")
            # self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
        else:
            self.lbc_safety2.setStyleSheet("background-color: yellow;")
            # self.lbc_safety_behind.setStyleSheet("background-color: yellow;")

        # ---- Battery
        if (self.statusColor.lbc_battery == 0):
            self.lbv_battery.setStyleSheet("background-color: white; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        elif (self.statusColor.lbc_battery == 1):
            self.lbv_battery.setStyleSheet("background-color: green; color: white;")
            self.lb_v.setStyleSheet("color: white;")
        elif (self.statusColor.lbc_battery == 2):
            self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        elif (self.statusColor.lbc_battery == 3):
            self.lbv_battery.setStyleSheet("background-color: red; color: white;")
            self.lb_v.setStyleSheet("color: white;")
        elif (self.statusColor.lbc_battery == 4):
            self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        else: # -- charging
            self.lbv_battery.setStyleSheet("background-color: white; color: black;")
            self.lb_v.setStyleSheet("color: black;")

        # ---- Thanh trang thai AGV
        if (self.statusColor.cb_status == 0):
            self.cb_status.setStyleSheet("background-color: green; color: white;")
        elif (self.statusColor.cb_status == 1):
            self.cb_status.setStyleSheet("background-color: orange; color: black;")	
        elif (self.statusColor.cb_status == 2):
            self.cb_status.setStyleSheet("background-color: red; color: white;")
        else:
            self.cb_status.setStyleSheet("background-color: white;")

        # ---- Trang thai che do dang hoat dong
        if (self.valueLable.modeRuning == self.modeRun_byhand):
            self.bt_passHand.setStyleSheet("background-color: blue;")	
            self.bt_passAuto.setStyleSheet("background-color: white;")	

        elif (self.valueLable.modeRuning == self.modeRun_auto):
            self.bt_passHand.setStyleSheet("background-color: white;")	
            self.bt_passAuto.setStyleSheet("background-color: blue;")
        else:	
            self.bt_passHand.setStyleSheet("background-color: white;")	
            self.bt_passAuto.setStyleSheet("background-color: white;")

        # -- Button Clear error
        # if (self.statusColor.lbc_button_clearError == 1):
        #     self.lbc_button_clearError.setStyleSheet("background-color: blue;")
        # elif (self.statusColor.lbc_button_clearError == 0):
        #     self.lbc_button_clearError.setStyleSheet("background-color: white;")

        # -- Button Power
        # if (self.statusColor.lbc_button_power == 1):
        #     self.lbc_button_power.setStyleSheet("background-color: blue;")
        # elif (self.statusColor.lbc_button_power == 0):
        #     self.lbc_button_power.setStyleSheet("background-color: white;")

        # -- Blsock
        # if (self.statusColor.lbc_blsock == 1):
        #     self.lbc_blsock.setStyleSheet("background-color: blue;")
        # elif (self.statusColor.lbc_blsock == 0):
        #     self.lbc_blsock.setStyleSheet("background-color: white;")

        # -- EMG
        # if (self.statusColor.lbc_emg == 1):
        #     self.lbc_emg.setStyleSheet("background-color: blue;")
        # elif (self.statusColor.lbc_emg == 0):
        #     self.lbc_emg.setStyleSheet("background-color: white;")	

        # -- Port: RTC Board
        # if (self.statusColor.lbc_port_rtc == 1):
        #     self.lbc_port_rtc.setStyleSheet("background-color: blue; color: white")
        # elif (self.statusColor.lbc_port_rtc == 0):
        #     self.lbc_port_rtc.setStyleSheet("background-color: red; color: white;")

        # -- Port: RS485
        # if (self.statusColor.lbc_port_rs485 == 1):
        #     self.lbc_port_rs485.setStyleSheet("background-color: blue; color: white")
        # elif (self.statusColor.lbc_port_rs485 == 0):
        #     self.lbc_port_rs485.setStyleSheet("background-color: red; color: white;")

        # -- Port: NAV350
        # if (self.statusColor.lbc_port_nav350 == 1):
        #     self.lbc_port_nav350.setStyleSheet("background-color: blue; color: white")
        # elif (self.statusColor.lbc_port_nav350 == 0):
        #     self.lbc_port_nav350.setStyleSheet("background-color: red; color: white;")

        # -- Sensor Up
        # if (self.statusColor.lbc_limit_up == 1):
        #     self.lbc_limit_up.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_limit_up == 0):
        #     self.lbc_limit_up.setStyleSheet("background-color: white; color: black;")

        # -- Sensor Down
        # if (self.statusColor.lbc_limit_down == 1):
        #     self.lbc_limit_down.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_limit_down == 0):
        #     self.lbc_limit_down.setStyleSheet("background-color: white; color: black;")

        # -- Sensor detect lift
        # if (self.statusColor.lbc_detect_lifter == 1):
        #     self.lbc_detect_lifter.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_detect_lifter == 0):
        #     self.lbc_detect_lifter.setStyleSheet("background-color: white; color: black;")

        # --
        self.statusButton.ck_remote = self.ck_remote.isChecked()
        # --
        if self.statusColor.lbc_setpose == 0:
            self.bt_setpose.setStyleSheet("background-color: white; color: black")
        elif self.statusColor.lbc_setpose == 1:
            self.bt_setpose.setStyleSheet("background-color: green; color: black")
        elif self.statusColor.lbc_setpose == 2:
            self.bt_setpose.setStyleSheet("background-color: blue; color: black")
        else:
            self.bt_setpose.setStyleSheet("background-color: red; color: white")
            
    def controlShow_followMode(self):
        if (self.valueLable.modeRuning == self.modeRun_launch):
            self.modeRuning = self.modeRun_launch

        elif (self.valueLable.modeRuning == self.modeRun_byhand):
            self.modeRuning = self.modeRun_byhand

        elif (self.valueLable.modeRuning == self.modeRun_auto):
            self.modeRuning = self.modeRun_auto
        else:
            self.modeRuning = self.modeRun_auto

        if self.saveModeRuning != self.modeRuning:
            self.saveModeRuning = self.modeRuning
            self.status_cancelMission = 0
            self.status_setting = 0
            self.status_getInfo = 0

        if (self.modeRuning == self.modeRun_launch):
            self.fr_launch.show()
            self.fr_run.hide()
            self.show_launch()

        else:
            self.fr_run.show()
            self.fr_launch.hide()

            if (self.modeRuning == self.modeRun_auto):
                if self.status_getInfo == 1:
                    self.fr_infoModeAuto.show()
                    self.fr_auto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_handMode_onof.hide()
                    self.fr_infoDevice.hide()
                    self.fr_password.hide()

                else:
                    self.fr_auto.show()
                    self.fr_infoModeAuto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_handMode_onof.hide()
                    self.fr_infoDevice.hide()
                    self.fr_password.hide()

            elif (self.modeRuning == self.modeRun_byhand):
                if self.status_cancelMission == 1:
                    self.fr_password.show()
                    self.fr_handMode_onof.hide()
                    self.fr_infoModeAuto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_auto.hide()
                    self.fr_infoDevice.hide()

                elif self.status_setting == 1:
                    pass

                elif self.status_getInfo == 1:
                    self.fr_infoDevice.show()
                    self.fr_handMode_onof.hide()
                    self.fr_infoModeAuto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_auto.hide()
                    self.fr_password.hide()

                else:
                    if self.handMode == self.handMode1:
                        self.fr_handMode_move.show()
                        self.fr_handMode_onof.hide()
                        self.fr_infoModeAuto.hide()
                        self.fr_infoDevice.hide()
                        self.fr_auto.hide()
                        self.fr_password.hide()

                    else:
                        self.fr_handMode_onof.show()
                        self.fr_handMode_move.hide()
                        self.fr_infoModeAuto.hide()
                        self.fr_infoDevice.hide()
                        self.fr_auto.hide()
                        self.fr_password.hide()

    def process_normal(self):
        self.set_dateTime()
        self.lbv_ip.setText(self.valueLable.lbv_ip)
        self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
        # self.lbv_mac.setText(self.valueLable.lbv_mac)
        # self.lbv_namePc.setText(self.valueLable.lbv_namePc)

        # --
        # if self.valueLable.listPath != self.valueLable.listPath_pre:
        self.TheCanvas.isUpdatePath = True
            # self.valueLable.listPath_pre = self.valueLable.listPath
        
        self.TheCanvas.makePath(self.valueLable.listPath)
        self.TheCanvas.isUpdateAGV = True
        self.TheCanvas.makeAGV(self.valueLable.lbv_coordinates_x, self.valueLable.lbv_coordinates_y, self.valueLable.lbv_coordinates_r)
        self.fr_pathPlan.update()

    def process_slow(self):
        self.lbv_battery.setText(self.valueLable.lbv_battery)

    def process_fast(self):
        # - combo box
        if (self.valueLable.listError != self.valueLable.listError_pre):
            self.valueLable.listError_pre = self.valueLable.listError
            self.cb_status.clear()
            self.cb_status.addItems(self.valueLable.listError)

        # - address set pose
        if self.valueLable.isUpdateListIdSetpose:
            self.valueLable.isUpdateListIdSetpose = False
            self.cb_listAddressSetPose.clear()
            self.cb_listAddressSetPose.setEditable(True)
            self.cb_listAddressSetPose.addItems(self.valueLable.listIDSetpose)
            line_edit = self.cb_listAddressSetPose.lineEdit()
            line_edit.setAlignment(Qt.AlignCenter)
            line_edit.setReadOnly(True)
        # -
        # if (self.valueLable.list_logError != self.valueLable.list_logError_pre):
        #     self.valueLable.list_logError_pre = self.valueLable.list_logError
        #     self.cb_logError.clear()
        #     lg = len(self.valueLable.list_logError) 
        #     for i in range(lg):
        #         self.cb_logError.addItem(self.valueLable.list_logError[i])

        # - 
        self.statusButton.bt_setting = self.status_getInfo

        # # --
        self.set_labelValue()
        # # --
        self.set_labelColor()
        # # --
        self.controlShow_followMode()

        if self.statusButton.bt_getInfo == 1:
            delta_t = rospy.Time.now() - self.timeSave_getInfo
            if delta_t.to_sec() > 0.5:
                self.status_getInfo = 1

        else:
            self.timeSave_getInfo = rospy.Time.now()

        if self.statusButton.bt_cancelMission == 1:
            delta_t = rospy.Time.now() - self.timeSave_cancelMisson
            if delta_t.to_sec() > 0.5 and self.valueLable.modeRuning == self.modeRun_byhand:
                self.status_cancelMission = 1
        else:
            self.timeSave_cancelMisson = rospy.Time.now()


    def show_launch(self):
        self.lbv_launhing.setText(self.valueLable.lbv_launhing)
        self.lbv_numberLaunch.setText(str(self.valueLable.lbv_numberLaunch))
        # --
        value = self.valueLable.percentLaunch
        if (value < 0):
            value = 0

        if (value > 100):
            value = 100

        self.pb_launch.setValue(value)
        # --
        # -- Port: RTC Board
        if (self.statusColor.lbc_port_rtc == 1):
            self.lbc_lh_rtc.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_rtc == 0):
            self.lbc_lh_rtc.setStyleSheet("background-color: red; color: white;")

        # -- Port: Driver Left
        if (self.statusColor.lbc_port_driverLeft == 1):
            self.lbc_lh_driver_left.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_driverLeft == 0):
            self.lbc_lh_driver_left.setStyleSheet("background-color: red; color: white;")

        # -- Port: Driver Right
        if (self.statusColor.lbc_port_driverRight == 1):
            self.lbc_lh_driver_right.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_driverRight == 0):
            self.lbc_lh_driver_right.setStyleSheet("background-color: red; color: white;")

        # -- Port: Lidar
        if (self.statusColor.lbc_port_lidar == 1):
            self.lbc_lh_lidar.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_lidar == 0):
            self.lbc_lh_lidar.setStyleSheet("background-color: red; color: white;")

    def show_password(self):
        data = ""
        lenght = len(self.password_data)

        for i in range(lenght):
            data += "*"
        
        self.lbv_pw_data.setText(data)

        if (self.password_data == self.password_right):
            self.bt_pw_agree.setEnabled(True)
        else:
            self.bt_pw_agree.setEnabled(False)

    def showPathPlan(self):
        pass





        




