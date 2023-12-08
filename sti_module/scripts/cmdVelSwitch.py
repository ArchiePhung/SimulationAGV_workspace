#!/usr/bin/env python
# author : Anh Tuan - 18-9-2020
# update : Anh Tuan - 28-9-2020

import os
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class cmdVelSwitch:
    def __init__(self):
        rospy.init_node('cmdVelSwitch', anonymous=True)
        self.rate = rospy.Rate(50)

        # self.lms100_port = rospy.get_param('~lms100_port','')  

        rospy.Subscriber('/cmd_vel_moveBase', Twist, self.cb1, queue_size = 100)
        rospy.Subscriber('/cmd_vel', Twist, self.cb2, queue_size = 100)
        rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.cb3, queue_size = 100)
        rospy.Subscriber('/cmd_vel_EMC', Bool, self.cb4, queue_size = 100)

        self.cmdVel_mc    = rospy.Publisher('/cmd_vel_mc', Twist, queue_size=100)
        self.cmdVel_EMC    = rospy.Publisher('/cmd_vel_mc', Twist, queue_size=100)

        self.vel_moveBase = Twist()
        self.vel_conlai = Twist()
        self.cmd_vel_mc = Twist()
        

        self.is_vel_moveBase = False
        self.is_vel_conlai = False
        self.is_localPlan = False
        self.is_cmdVel_EMC = False

    def cb1(self,vel): 
        if self.is_vel_moveBase == False :
            self.is_vel_moveBase = True
        self.vel_moveBase = vel

    def cb2(self,vel): 
        if self.is_vel_conlai == False :
            self.is_vel_conlai = True
        self.vel_conlai = vel

    def cb3(self,plan):
        if self.is_localPlan == False :
            self.is_localPlan = True
    
    def cb4(self,emc):
        self.is_cmdVel_EMC = emc.data

    def raa(self): 
        while not rospy.is_shutdown():
            
            # if self.is_cmdVel_EMC == False :
            # rospy.logwarn("EMC = False")
            if self.is_vel_moveBase == True :
                self.is_vel_moveBase = False
                # self.is_localPlan = False
                self.cmd_vel_mc = self.vel_moveBase
                rospy.logwarn("cmdVel - MoveBase")
                self.cmdVel_mc.publish(self.cmd_vel_mc)
                

            elif self.is_vel_conlai == True : 
                self.is_vel_conlai = False
                self.cmd_vel_mc = self.vel_conlai
                rospy.logwarn("cmdVel - conlai")
                self.cmdVel_mc.publish(self.cmd_vel_mc)

            # if self.is_cmdVel_EMC == True :  
            #     self.cmdVel_mc.publish(Twist())
            #     rospy.logwarn("EMC = True")
                

            self.rate.sleep()

def main():
    
    try:
        m = cmdVelSwitch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()