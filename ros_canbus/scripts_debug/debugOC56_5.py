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


class DEDUG_OC():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('DEDUG_OC565', anonymous=False)
        self.rate = rospy.Rate(30)

        # -- Board OC 56 | conveyor5
        rospy.Subscriber("/status_conveyor11", Status_conveyor, self.callback_conveyor11) # 
        self.status_conveyor11 = Status_conveyor()
        self.timeStampe_conveyor1 = rospy.Time.now()

        self.path_log = "/home/stivietnam/catkin_ws/src/ros_canBus/scripts_debug/debug/OC11_debug.txt"

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
        self.file_log.write("\nData at time: " + str(current_time) + " | status | limitAhead | limitBehind | checkRack |" + str(data.status) + " " + str(data.sensor_limitAhead) + " " + str(data.sensor_limitBehind) + " " + str(data.sensor_checkRack))
        # self.file_log.write("\nsensor_limitAhead: " + str(data.sensor_limitAhead) + )
        # self.file_log.write("\nsensor_limitBehind: " + str(data.sensor_limitBehind))
        # self.file_log.write("\nsensor_checkRack: " + str(data.sensor_checkRack))
        self.file_log.close()

    def callback_conveyor11(self, data):
        self.status_conveyor11 = data
        self.timeStampe_conveyor1 = rospy.Time.now()
        self.writeDataOC(self.status_conveyor11)

    def detectLost_OC12(self):
        delta_t = rospy.Time.now() - self.timeStampe_conveyor1
        if delta_t.to_sec() > 4.0:
            return 1
        return 0
    
    def run(self):
        while not rospy.is_shutdown():
            if self.detectLost_OC12():
                now = datetime.now()
                current_time = now.strftime("%B/%d|%H:%M:%S")

                self.file_log = open(self.path_log, "a+")
                self.file_log.write("\nLost data OC12 at time: | " + str(current_time))
                self.file_log.close()

            self.rate.sleep()

def main():
    # Start the job threads
    class_1 = DEDUG_OC()
    # Keep the main thread running, otherwise signals are ignored.
    class_1.run()

if __name__ == '__main__':
    main()




    