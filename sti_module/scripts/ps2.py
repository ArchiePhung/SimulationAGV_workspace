#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 05/06/2020t

Funtion:
    1, PS2 + Application.
    2, Nhan va thuc hien nhiem vu tu sti_control.
    3, Doc tay ps2 va gui che do cho Sti control.

in case:
    AGV:
        1, Conveyer 
        2, Lift and lower
    
LEFT 
DOWN 

GREEN
RED 
BlUE 
PINK 

Regulations:
    A, Funtion Of PS2:
        1, Navigation by 6 buttons (two speed levels):
            1, Forward                                - TRIANGLE
            2, Backward                               - CROSS 
            3, Turn left                              - SQUARE 
            4, Turn right                             - CIRCLE 
            5, Slow Combined button( to safe)         - UP
			6, Fast Combined button( to safe)         - DOWN
        2, Conveyor:
            1, Lift                                   - R1
            2, Lower                                  - R2
            3, Turn left to right                     - L1
            4, Turn right to left (stand behind AGV)  - L2
        3, Select Mode.
            1, Run by hand simple                     - L3
            2, Run auto                               - R3

        4, Setpose                   				  - LEFT
        5, Reset all                				  - RIGHT			

---------------------------------------------------------------
Mission =
    1    Receive above
    2    Receive below
    3    Returns above
    4    Returns below 
Status:
    0    Not connect
    1    Connected
....................... 

"""
import roslib
import sys
import time
from decimal import *
import math
import rospy
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
# from std_msgs import Bool
class read_peripheral():
	def __init__(self):
		rospy.init_node('sti_PS2', anonymous=False)
		self.rate = rospy.Rate(20)

		rospy.Subscriber("/ps2_status", Ps2_msgs, self.ps2_callback)
		self.ps2Button = Ps2_msgs()
		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)		
		self.velPs2 = Twist()

		self.vel = Twist()       # gui toc do

		self.mode = 1  # 1: hand  2: auto

		self.by_hand = 1
		self.auto = 23
		self.status_move = 0  # 0 - stop | 1: running
		self.mode_operate = self.auto
		self.linear_x = 0
		self.angular_z = 0
		self.pre_time_setpose = 0
		self.pre_time_reset = 0
		self.pre_mess = ""               # lưu tin nhắn hiện tại.
	      # -- ps2 
		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 15. 	
  #-- Read
	def ps2_callback(self, dat):
		self.ps2Button = dat


	def navigation_by_hand(self, ps2_button):
		vel_Ps2 = Velocities()
		# print "up:", self.ps2Button.up.data
		# print "down:", self.ps2Button.down.data
		if ps2_button.up.data == 1 and ps2_button.down.data == 0:   # Fast 
			if (ps2_button.triangle.data == 1) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 0): # forward
				vel_Ps2.linear_x = 0.55
				vel_Ps2.angular_z = 0.0

			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 1) and (ps2_button.square.data == 0): # backward
				vel_Ps2.linear_x = -0.55
				vel_Ps2.angular_z = 0.0

			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 1): # turn left
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = 0.4
			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 1) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 0): # turn right
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = -0.4
			else:
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = 0.0
		elif ps2_button.up.data == 0 and ps2_button.down.data == 1:  # Slow
			if (ps2_button.triangle.data == 1) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 0): # forward
				vel_Ps2.linear_x = 0.35
				vel_Ps2.angular_z = 0.0

			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 1) and (ps2_button.square.data == 0): # backward
				vel_Ps2.linear_x = -0.35
				vel_Ps2.angular_z = 0.0

			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 0) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 1): # turn left
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = 0.3
			elif (ps2_button.triangle.data == 0) and (ps2_button.circle.data == 1) and (ps2_button.cross.data == 0) and (ps2_button.square.data == 0): # turn right
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = -0.3
			else:
				vel_Ps2.linear_x = 0.0
				vel_Ps2.angular_z = 0.0
		else:
			vel_Ps2.linear_x = 0.0
			vel_Ps2.angular_z = 0.0

		return vel_Ps2

			
	def log_mess(self, typ, mess, val):
		if self.pre_mess != mess:
			if typ == "info":
				rospy.loginfo (mess + ": %s", val)
			elif typ == "warn":
				rospy.logwarn (mess + ": %s", val)
			else:
				rospy.logerr (mess + ": %s", val)
		self.pre_mess = mess


	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
			self.pub_vel.publish(twist)
		# else :
			# rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

	def run(self):
		PS2_velRead = Velocities()
		PS2_velRead = self.navigation_by_hand(self.ps2Button)
		
		self.velPs2.linear.x  = PS2_velRead.linear_x
		self.velPs2.angular.z = PS2_velRead.angular_z

		# self.pub_cmdVel(self.velPs2, self.rate_cmdvel, rospy.get_time())
		self.pub_vel.publish(self.velPs2)
		# self.rate.sleep()
		time.sleep(0.1)



def main():
    # Start the job threads
	class_1 = read_peripheral()		
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
    main()