#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path


class SUB():
    def __init__(self):
        rospy.init_node('sub', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(30)

        # -- Board HC 82
        self.sub_ = rospy.Subscriber("/HC_info", HC_info, self.callback_) # lay thong tin trang thai cua node va cam bien sick an toan.
        # self.sub_backup = rospy.Subscriber("/chatter_backup", String, self.callback_, queue_size= 10) # lay thong tin trang thai cua node va cam bien sick an toan.
        # self.sub_HC.unregister()
        # self.sub_.get_num_connections()
        self.HC_info = HC_info()
        self.timeStampe_HC = rospy.Time.now()
        self.pub_subHC = rospy.Publisher("/statusHC",String, queue_size= 20)

        self.save_numConnect = 0
        self.count = 0
        self.countReSub = 0
        self.sttResub = 0

    def findSpline(self):
         
        
    def callback_(self, msg):
        pass

    def run(self):
        while not rospy.is_shutdown():     

            self.rate.sleep()
 
def main():
    # Start the job threads
    class_1 = SUB()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()
    