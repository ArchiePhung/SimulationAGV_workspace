#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove, StopPoint
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, acos, modf, atan2, fabs, tan, degrees, radians
from math import pi as PI
# import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cmath

class TTT():
    def __init__(self):
        rospy.init_node('ttttt', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(10)

        # param
        self.angleStop = rospy.get_param('~angleStop',165.) 

        rospy.Subscriber('/list_pointRequestMove', ListPointRequestMove, self.callback_poseRobot, queue_size = 20)
        rospy.spin()

    def callback_poseRobot(self, data):
        print(data.enable)


def main():
    # Start the job threads
    class_1 = TTT()
    # class_1.run()
    # class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()