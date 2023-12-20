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


class DEDUG_HC():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('DEDUG_HC', anonymous=False)
        self.rate = rospy.Rate(30)
        
        # -- Board HC 82
        rospy.Subscriber("/HC_info", HC_info, self.callback_HC, queue_size= 1) # lay thong tin trang thai cua node va cam bien sick an toan.
        self.HC_info = HC_info()
        self.timeStampe_HC = rospy.Time.now()

        self.path_log = "/home/stivietnam/catkin_ws/src/ros_canBus/scripts_debug/debug/HC_debug.txt"

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
        self.file_log.write("\nData at time: " + str(current_time) + " | stt | zAhead | zBehind | vacham | " + str(data.status) + " " + str(data.zone_sick_ahead) + " " + str(data.zone_sick_behind) + " " + str(data.vacham))
        # self.file_log.write("\nsensor_limitAhead: " + str(data.sensor_limitAhead) + )
        # self.file_log.write("\nsensor_limitBehind: " + str(data.sensor_limitBehind))
        # self.file_log.write("\nsensor_checkRack: " + str(data.sensor_checkRack))
        self.file_log.close()

    def callback_HC(self, data):
        self.HC_info = data
        self.timeStampe_HC = rospy.Time.now()
        self.writeDataOC(self.HC_info)

    def detectLost_Main(self):
        delta_t = rospy.Time.now() - self.timeStampe_HC
        if delta_t.to_sec() > 0.6:
            return 1
        return 0
    
    def run(self):
        while not rospy.is_shutdown():
            if self.detectLost_Main():
                now = datetime.now()
                current_time = now.strftime("%B/%d|%H:%M:%S")

                self.file_log = open(self.path_log, "a+")
                self.file_log.write("\nLost data HC at time: | " + str(current_time))
                self.file_log.close()

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = DEDUG_HC()
    # Keep the main thread running, otherwise signals are ignored.
    class_1.run()

if __name__ == '__main__':
    main()




    