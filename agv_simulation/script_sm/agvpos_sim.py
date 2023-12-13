#!/usr/bin/env python3  
# -*- coding: utf-8 -*-

"""
    Author: ArchieP 
    Beginning date: 20/5/2023
    Lastest Modify date: 20/5/2023
"""

import roslib
import rospy

import time
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from simulate_agvs.msg import *

from sti_msgs.msg import NN_cmdRequest, Move_request, Velocities
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Pose

class AGV_SIM():
    def __init__(self):

        # -- ros init --
        rospy.init_node('agv_sim')
        self.rate_hz = 60
        self.rate = rospy.Rate(self.rate_hz)
        
        # -- ros subcriber
        rospy.Subscriber('/cmd_fvel', Velocities, self.handle_cmdVel, queue_size = 20)
        self.msg_cmdVel = Velocities()
        # self.is_receive_cmdVel = 0

        # rospy.Subscriber('/request_move', Move_request, self.handle_requestMove, queue_size = 20)
        # self.msg_requestMove = Move_request()
        # self.is_receive_requestMove = 0

        # -- ros publish 
        self.pub_agv_pose = rospy.Publisher('/agv_pose', Pose, queue_size = 20)
        self.data_agv_pose = Pose()

        # -- Mô tả vị trí ban đầu của AGV hiển thị trên tf rviz   -- 
        self.x_start = rospy.get_param('x_start')
        self.y_start = rospy.get_param('y_start')
        self.theta_start = rospy.get_param('theta_start')
        
        # -- Mô tả vị trí trước đó của AGV --
        self.agv_pre_pose_x = self.x_start
        self.agv_pre_pose_y = self.y_start
        self.agv_pre_angle = self.theta_start

        # -- Thông số khác của AGV -- 
        self.translation = (self.x_start, self.y_start, 0.0)
        self.quanternion = quaternion_from_euler(0, 0, self.theta_start)
        
        self.agv_angle = self.agv_pre_angle
        
        # -- Khai báo tf -- 
        self.br = tf.TransformBroadcaster()
        self.agv_frame = "agv1"
        self.origin_frame = "world"
        
        # -- quy trình của AGV -- 
        self.pre_t = time.time()
        self.time_now = time.time()
        self.pre_mess = ""
        self.is_Vel = 0
        self.is_allowRun = 1  
        self.is_receive_vel = 0   

    # def handle_requestMove(self, data):
    #     self.msg_requestMove = data 
        # self.is_receive_rqMove = 1

    def handle_cmdVel(self, data):
        self.msg_cmdVel = data
        self.is_receive_vel = 1
    
    def run(self):
        while not rospy.is_shutdown():
            self.br.sendTransform(self.translation, self.quanternion, rospy.Time.now(), self.agv_frame, self.origin_frame)
            
            if self.is_receive_vel == 1:                         
                # print("Tao vẫn đang chạy nha", time.strftime("%H:%M:%S", time.localtime()))
                self.time_now = time.time() - self.pre_t

                self.agv_angle = self.agv_pre_angle + self.msg_cmdVel.twist.angular.z * self.time_now
                self.data_agv_pose.position.x = self.agv_pre_pose_x + self.msg_cmdVel.twist.linear.x * math.cos(self.agv_angle ) * self.time_now 
                self.data_agv_pose.position.y = self.agv_pre_pose_y + self.msg_cmdVel.twist.linear.x * math.sin(self.agv_angle ) * self.time_now 

                self.translation = (self.data_agv_pose.position.x, self.data_agv_pose.position.y, 0)
                self.quanternion = quaternion_from_euler(0, 0, self.agv_angle)
                
                self.data_agv_pose.orientation.x = self.quanternion[0]
                self.data_agv_pose.orientation.y = self.quanternion[1]
                self.data_agv_pose.orientation.z = self.quanternion[2]
                self.data_agv_pose.orientation.w = self.quanternion[3]

                self.agv_pre_angle = self.agv_angle
                self.agv_pre_pose_x = self.data_agv_pose.position.x
                self.agv_pre_pose_y = self.data_agv_pose.position.y
                self.pre_t = time.time()

            else:
                self.pre_t = time.time()
                
            self.pub_agv_pose.publish(self.data_agv_pose)
            self.rate.sleep()

def main():
    print('Programmer started!')
    agv1 = AGV_SIM()
    agv1.run()
    print('Programmer stopped!!')

if __name__ == '__main__':
    main()
