#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sti_msgs.msg import APathParking, PointOfPath, HC_info, MotorDrive_respond, Velocities
from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin, degrees, radians, acos
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker



class ParkingAGV():
  def __init__(self):
    rospy.init_node('parking_agv', anonymous=False)
    print("initial node!")
    self.rate = rospy.Rate(30)

    # Pub topic
    self.pub_cmd_vel = rospy.Publisher('/cmd_fvel', Velocities, queue_size=20)
    self.time_tr = rospy.get_time()
    self.saveTime = rospy.get_time()
    self.rate_pubVel = 30

  def stop(self):
    self.curr_velocity = 0.
    for i in range(2):
      self.pub_cmd_vel.publish(Velocities())
        
  def pub_cmdVelIndividual(self, _mode, _angleRequest, _velRequest, rate): # 1 cho tung kenh, 2 cho song kenh
    vel = Velocities()
    vel.selectMode = _mode # 0: dk dong co chinh | 1: dk dong co lai 
    vel.angleRequest = int(_angleRequest)
    vel.velRequest = int(_velRequest)
    
    if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
      self.time_tr = rospy.get_time()
      self.pub_cmd_vel.publish(vel)
    else :
      pass
  
  def pub_cmdVelNavigation(self, twist, rate):
    vel = Velocities()
    vel.selectMode = 2 # 2: dieu khien song song
    vel.twist = twist
    
    if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
      self.time_tr = rospy.get_time()
      self.pub_cmd_vel.publish(vel)
    else :
      pass

  def run(self):
    twist = Twist()
    while not rospy.is_shutdown():   
      if rospy.get_time() - self.saveTime > 2.
      twist.linear.x = -0.1 
      self.pub_cmdVelNavigation(twist, self.rate_pubVel)
      rospy.sleep(3)
      self.stop()
      twist.linear.x = 0.1 
      self.pub_cmdVelNavigation(twist, self.rate_pubVel)
      rospy.sleep(3)
      self.stop()
      self.rate.sleep()

def main():
    # Start the job threads
    class_1 = ParkingAGV()
    # class_1.run()
    class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()