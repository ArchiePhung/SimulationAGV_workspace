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
        rospy.init_node('DEDUG_MAIN', anonymous=False)
        self.rate = rospy.Rate(30)
        
        # -- Board MAIN - POWER
        rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
        self.main_info = POWER_info()
        self.timeStampe_main = rospy.Time.now()
        self.voltage = 24.5

        self.path_log = "/home/stivietnam/catkin_ws/src/ros_canBus/scripts_debug/debug/Main_debug.txt"

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
        self.file_log.write("\nData at time: " + str(current_time) + " | vol | volA | char | charA | Breset | Bpower | EMC | CANstt | " + str(round(data.voltages, 2)) + " " + str(data.voltages_analog) + " " + str(data.charge_current) + " " + str(data.charge_analog) + " " + str(data.stsButton_reset) + " " + str(data.stsButton_power) + " " + str(data.EMC_status) + " " + str(data.CAN_status))
        # self.file_log.write("\nsensor_limitAhead: " + str(data.sensor_limitAhead) + )
        # self.file_log.write("\nsensor_limitBehind: " + str(data.sensor_limitBehind))
        # self.file_log.write("\nsensor_checkRack: " + str(data.sensor_checkRack))
        self.file_log.close()

    def callback_Main(self, data):
        self.main_info = data
        self.timeStampe_main = rospy.Time.now()
        self.writeDataOC(self.main_info)

    def detectLost_Main(self):
        delta_t = rospy.Time.now() - self.timeStampe_main
        if delta_t.to_sec() > 4.0:
            return 1
        return 0
    
    def run(self):
        while not rospy.is_shutdown():
            if self.detectLost_Main():
                now = datetime.now()
                current_time = now.strftime("%B/%d|%H:%M:%S")

                self.file_log = open(self.path_log, "a+")
                self.file_log.write("\nLost data MAIN at time: | " + str(current_time))
                self.file_log.close()

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = DEDUG_MAIN()
    # Keep the main thread running, otherwise signals are ignored.
    class_1.run()

if __name__ == '__main__':
    main()




    