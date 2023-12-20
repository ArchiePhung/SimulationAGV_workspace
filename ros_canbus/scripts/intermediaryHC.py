#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import time
from decimal import *
import math
import rospy

from message_pkg.msg import *
from sti_msgs.msg import *
from std_msgs.msg import Int16, String
#--------------------------------------------------------------------------------- ROS
class InterHC():
    def __init__(self):
        rospy.init_node('intermediaryHC', anonymous=False)
        self.rate = rospy.Rate(20)
        # SUB - PUB
        rospy.Subscriber("/HC_infoRaw", HC_info, self.callback_hc, queue_size=1) # lay thong tin trang thai cua node va cam bien sick an toan.
        self.timeStampe_HC = rospy.Time.now()
        self.is_HCInfoRAW = False

        self.pub_HC = rospy.Publisher("/HC_info", HC_info, queue_size=100)
        self.msgHCInfo = HC_info()

    def callback_hc(self, data):
        self.msgHCInfo = data
        self.pub_HC.publish(self.msgHCInfo)
        # self.timeStampe_HC = rospy.Time.now()
        self.is_HCInfoRAW = True

    # def detectLost_hc(self):
    #     delta_t = rospy.Time.now() - self.timeStampe_HC
    #     if (delta_t.to_sec() > 0.6):
    #         return 1
    #     return 0

    # def run(self):
    #     if self.process == 0:
    #         print("DEBUG ADDITION ALL RIGHT!")
    #         self.process = 1
    #     else:		
    #         self.log_status()
    #         self.rate.sleep()

def main():
    # Start the job threads
    class_1 = InterHC()
    rospy.spin()

if __name__ == '__main__':
    main()