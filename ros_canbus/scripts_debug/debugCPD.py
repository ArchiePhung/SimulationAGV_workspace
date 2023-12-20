#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from message_pkg.msg import CAN_send, CAN_status, CAN_received, CPD_read, CPD_write
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canBus.msg import *

from datetime import datetime


class DEDUG_MAIN():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('DEDUG_CPD', anonymous=False)
        self.rate = rospy.Rate(30)
        
        # -- Board CPD | Toyo
        rospy.Subscriber("/CPD_read", CPD_read, self.callback_CPD, queue_size = 10) # 
        self.status_CPD = CPD_read()
        self.timeStampe_CPD = rospy.Time.now()

        self.path_log = "/home/stivietnam/catkin_ws/src/ros_canBus/scripts_debug/debug/CPD_debug.txt"

        now = datetime.now()
        current_time = now.strftime("%B/%d|%H:%M:%S")

        self.file_log = open(self.path_log, "a+")
        self.file_log.write("\nInit Node at: | " + str(current_time) + "+++++++++++++++++++++++")
        self.file_log.write("\n----------------------------------------")
        self.file_log.close()

    def writeDataOC(self, data):
        now = datetime.now()
        current_time = now.strftime("%B/%d|%H:%M:%S")
        self.file_log = open(self.path_log, "a+")
        self.file_log.write("\nData at time: " + str(current_time) + "| status | i1 | i2 | i3 | i4 | i5 | i6 | i7 | i8 | i9 | i10 | i11 | i12 | " + str(data.status) + " " + str(data.input1) + " " + str(data.input2) + " " + str(data.input3) + " " + str(data.input4) + " " + str(data.input5) + " " + str(data.input6) + " " + str(data.input7) + " " + str(data.input8) + " " + str(data.input9) + " " + str(data.input10) + " " + str(data.input11) + " " + str(data.input12))
        # self.file_log.write("\nsensor_limitAhead: " + str(data.sensor_limitAhead) + )
        # self.file_log.write("\nsensor_limitBehind: " + str(data.sensor_limitBehind))
        # self.file_log.write("\nsensor_checkRack: " + str(data.sensor_checkRack))
        self.file_log.close()

    def callback_CPD(self, data):
        self.status_CPD = data
        self.timeStampe_CPD = rospy.Time.now()
        self.writeDataOC(self.status_CPD)

    def detectLost_CPD(self):
        delta_t = rospy.Time.now() - self.timeStampe_CPD
        if delta_t.to_sec() > 4.0:
            return 1
        return 0
    
    def run(self):
        while not rospy.is_shutdown():
            if self.detectLost_CPD():
                now = datetime.now()
                current_time = now.strftime("%B/%d|%H:%M:%S")

                self.file_log = open(self.path_log, "a+")
                self.file_log.write("\nLost data CPD at time: | " + str(current_time))
                self.file_log.close()

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = DEDUG_MAIN()
    # Keep the main thread running, otherwise signals are ignored.
    class_1.run()

if __name__ == '__main__':
    main()




    