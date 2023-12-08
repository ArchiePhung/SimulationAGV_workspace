#!/usr/bin/python

import os 
import time
import rospy
import tf
from std_msgs.msg import String, Bool

import sys
import struct
import string
import roslib
import serial
import signal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, atan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import *
from message_pkg.msg import Parking_request, Parking_respond

class ParkingNAV():
    def __init__(self):

        rospy.init_node('Parking_NAV', anonymous=True)
        # get param
        self.rate = rospy.get_param('~rate',50) 
        self.rate = rospy.Rate(200)

        # Sub topic
        rospy.Subscriber("Request_parking", Parking_request, self.request_callback)
        self.req_parking = Parking_request() 
        self.is_request_parking = False

        rospy.Subscriber('/robot_pose', Pose, self.getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.is_odom_rb = False
        self.odom_rb = Odometry()

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.rate_pubVel = 30

        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)
        self.auto_reset = 0

        # Tf
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        #avera
        self.odom_x_ht = 0.
        self.odom_y_ht = 0.
        self.odom_g = 0.

        self.step_moveForward = 0
        self.x_odom_start = 0.0
        self.y_odom_start = 0.0

        self.step_Rotary = 0
        self.angle_odom_start = 0.0

        self.min_vel = 0.03
        self.min_rol = 0.1

        self.max_vel = 0.1
        self.max_rol = 0.25


        # self.poseThenTransform = Pose()

        self.time_tr = rospy.get_time()
        self.time_waitTransfrom = rospy.get_time()

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def request_callback(self, data):
        self.req_parking = data
        self.is_request_parking = True


    def getPose(self, data):
        self.poseRbMa = data
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb_ht = euler[2]

        self.is_pose_robot = True

    def cbGetRobotOdom(self, robot_odom_msg):
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,\
                        robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.odom_x_ht = robot_odom_msg.pose.pose.position.x
        self.odom_y_ht = robot_odom_msg.pose.pose.position.y

        self.odom_g = theta
        self.is_odom_rb = True

    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def pub_Stop(self):
        for i in range(0,2,1):
            self.pub_cmd_vel.publish(Twist())
    
    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def transformPoseNAV(self, frame_world, frame_id_pointTarget, frame_agvNAV, point_target):
        poseThenTransform = Pose()
        self.tf_broadcaster.sendTransform((point_target.position.x, point_target.position.y, -1.0),
                                        (0.0, 0.0, point_target.orientation.z, point_target.orientation.w),
                                        rospy.Time.now(),
                                        frame_id_pointTarget,
                                        frame_world)

        self.tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, rospy.Time(), rospy.Duration(10))
        pos, orien = self.tf_listener.lookupTransform(frame_id_pointTarget, frame_agvNAV, rospy.Time())
        poseThenTransform.position.x = pos[0]
        poseThenTransform.position.y = pos[1]
        poseThenTransform.position.z = pos[2]
        poseThenTransform.orientation.x = orien[0]
        poseThenTransform.orientation.y = orien[1]
        poseThenTransform.orientation.z = orien[2]
        poseThenTransform.orientation.w = orien[3]

        return poseThenTransform

    # def transformPoseNAV(self, frame_world, frame_id_pointTarget, frame_agvNAV, point_target):
    #     poseStampedThenTransform = PoseStamped()
    #     self.tf_broadcaster.sendTransform((point_target.position.x, point_target.position.y, -1.0),
    #                                     (0.0, 0.0, point_target.orientation.z, point_target.orientation.w),
    #                                     rospy.Time.now(),
    #                                     frame_id_pointTarget,
    #                                     frame_world)

    #     self.tf_listener.waitForTransform(frame_id_pointTarget, frame_agvNAV, rospy.Time(), rospy.Duration(10))
    #     poseStampedThenTransform = self.tf_listener.transformPoint(frame_id_pointTarget, self.poseStampedAGV)

    #     return poseStampedThenTransform.pose

    def move_forward(self, S, direct, velocity_min, velocity_max, permission_tolerance):
        twist = Twist()
        if self.step_moveForward == 0:
            self.x_odom_start = self.odom_x_ht
            self.y_odom_start = self.odom_y_ht
            self.step_moveForward = 1

        elif self.step_moveForward == 1:
            S_moved = self.calculate_distance(self.odom_x_ht, self.odom_y_ht, self.x_odom_start, self.y_odom_start)
            if fabs(S_moved - S) >= permission_tolerance and S_moved - S <= permission_tolerance:
                twist.angular.z = 0
                vel = velocity_max*(fabs(S_moved - S)/S)
                if vel <= velocity_min:
                    vel = velocity_min
                if direct == 1: # tien
                    twist.linear.x = vel
                elif direct == 2: # lui
                    twist.linear.x = vel*(-1)
                print("S= %s , S_moved= %s, direct= %s" %(S, S_moved, direct))
                self.pub_cmdVel(twist, self.rate_pubVel)
                return 0

            else:
                self.pub_Stop()
                self.step_moveForward = 0
                return 1
        else:
            return 0

    def rotary_around(self, Angle, direct, velocity_min, velocity_max, Angle_StartDecel, permission_tolerance):
        twist = Twist()
        if self.step_Rotary == 0:
            self.angle_odom_start = self.odom_g
            self.step_Rotary = 1
        elif self.step_Rotary == 1:
            Angle_turned = self.odom_g - self.angle_odom_start
            if fabs(Angle_turned) > PI:
                Angle_turned = 2*PI - fabs(Angle_turned)
            else:
                Angle_turned = fabs(Angle_turned)
            if fabs(Angle_turned - Angle) >= permission_tolerance and Angle_turned - Angle <= permission_tolerance:
                twist.linear.x = 0
                vel = 0.0
                if fabs(Angle_turned - Angle) <= Angle_StartDecel:
                    vel = velocity_max*(fabs(Angle_turned - Angle)/Angle)
                    if vel <= velocity_min:
                        vel = velocity_min
                else:
                    vel = velocity_max
                if direct == 1: # quay trai
                    twist.angular.z = vel
                elif direct == 2: # quay phai
                    twist.angular.z = vel*(-1)

                print("Angle= %s , Angle_turned= %s, direct= %s" %(Angle, Angle_turned, direct))
                self.pub_cmdVel(twist, self.rate_pubVel)
                return 0

            else:
                self.pub_Stop()
                self.step_Rotary = 0
                return 1

    # def follow_target(self, velocity_min, velocity_max, vel_rotMax, distance_decel, permission_tolerance):
    #     vel_x = 0.0
    #     vel_rot = 0.0
    #     twist = Twist()
    #     poseThenTransform = Pose()
    #     poseThenTransform = self.transformPoseNAV('frame_map_nav350', 'frame_target', 'frame_robot', self.req_parking.pose)
    #     euler = euler_from_quaternion((poseThenTransform.orientation.x,\
    #                                     poseThenTransform.orientation.y,\
    #                                     poseThenTransform.orientation.z,\
    #                                     poseThenTransform.orientation.w))
    #     angleAGV = euler[2]
    #     distance = sqrt(poseThenTransform.position.x*poseThenTransform.position.x + poseThenTransform.position.y*poseThenTransform.position.y)
    #     angle = PI - fabs(angleAGV)

    #     if distance >= permission_tolerance or poseThenTransform.position.x < permission_tolerance:
    #         if distance <= distance_decel:
    #             vel_x = 0.5*distance
    #             if vel_x >= velocity_max:
    #                 vel_x = velocity_max
    #             if vel_x <= velocity_min:
    #                 vel_x = velocity_min

    #         else:
    #             vel_x = velocity_max

    #         twist.linear.x = vel_x*(-1)

    #         kg = 1.4
    #         vel_rot = kg*angle
    #         if vel_rot >= vel_rotMax:
    #             vel_rot = vel_rotMax
    #         if angleAGV > 0:
    #             twist.angular.z = vel_rot
    #         else:
    #             twist.angular.z = vel_rot*(-1)

    #         self.pub_cmdVel(twist, self.rate_pubVel)
    #         return 0

    #     else:
    #         self.pub_Stop()
    #         return 1


    def follow_target(self, velocity_min, velocity_max, vel_rotMax, distance_decel, distance_ahead, permission_tolerance):
        vel_x = 0.0
        vel_rot = 0.0
        twist = Twist()
        selectAngle = 0
        directMode1 = 0 # dung voi truong hop selectAngle = 1 | 1 quay trai, 2 quay phai
        poseThenTransform = Pose()
        poseThenTransform = self.transformPoseNAV('map', 'frame_target', 'base_footprint', self.req_parking.pose)
        euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                        poseThenTransform.orientation.y,\
                                        poseThenTransform.orientation.z,\
                                        poseThenTransform.orientation.w))
        angleAGV = euler[2]
        distance = sqrt(poseThenTransform.position.x*poseThenTransform.position.x + poseThenTransform.position.y*poseThenTransform.position.y)
        angle = 0.0

        if distance >= permission_tolerance or poseThenTransform.position.x < permission_tolerance: # dieu kien den target
            if distance <= distance_decel: # tinh van toc dai theo khoang cach
                vel_x = 0.5*distance
                if vel_x >= velocity_max:
                    vel_x = velocity_max
                if vel_x <= velocity_min:
                    vel_x = velocity_min

            else:
                vel_x = velocity_max

            twist.linear.x = vel_x*(-1)

            if fabs(poseThenTransform.position.x) >= distance_ahead or fabs(poseThenTransform.position.y) >= 0.005: # tinh van toc goc
                angle_folow = atan(fabs(poseThenTransform.position.y)/distance_ahead)
                if poseThenTransform.position.y > 0 and angleAGV > 0:
                    if angleAGV > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 1
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 2
                elif poseThenTransform.position.y > 0 and angleAGV < 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 2
                elif poseThenTransform.position.y < 0 and angleAGV < 0:
                    if angleAGV > angle_folow:
                        angle = PI - angle_folow - fabs(angleAGV)
                        directMode1 = 2
                    else:
                        angle = angle_folow + fabs(angleAGV) - PI
                        directMode1 = 1
                elif poseThenTransform.position.y < 0 and angleAGV > 0:
                    angle = PI + angle_folow - fabs(angleAGV)
                    directMode1 = 1
                selectAngle = 2

            else:
                angle = PI - fabs(angleAGV)
                selectAngle = 2

            kg = 1.4
            vel_rot = kg*angle
            if vel_rot >= vel_rotMax:
                vel_rot = vel_rotMax

            if selectAngle == 2:
                if angleAGV > 0:
                    twist.angular.z = vel_rot
                else:
                    twist.angular.z = vel_rot*(-1)

            else:
                if directMode1 == 1:
                    twist.angular.z = vel_rot
                elif directMode1 == 2:
                    twist.angular.z = vel_rot*(-1)

            self.pub_cmdVel(twist, self.rate_pubVel)
            return 0

        else:
            self.pub_Stop()
            return 1

    def raa(self):
        process = 0
        SGo_forward = 0.0
        AGO_rotary = 0.0
        direct_forward = 0 # 1 tien, 2 lui
        direct_rotary = 0 # 1 quay trai, 2 quay phai
        while not rospy.is_shutdown() :
            if process == 0:
                print("wait receive all need data......")
                c_k = 0
                if self.is_pose_robot == True:
                    c_k = c_k + 1
                if self.is_odom_rb == True:
                    c_k = c_k + 1
                if c_k == 2:
                    process = 1
                    print("Done receive all need data!")

            elif process == 1: # cho tin hieu Parking
                if self.is_request_parking == True:
                    print("Received data request Parking, Start Process!")
                    self.time_waitTransfrom = rospy.get_time()
                    process = 2

            elif process == 2: # tinh quang duong di chuyen
                poseThenTransform = Pose()
                if rospy.get_time() - self.time_waitTransfrom <= 1.5:
                    poseThenTransform = self.transformPoseNAV('map', 'frame_target', 'base_footprint', self.req_parking.pose)

                else:
                    print(poseThenTransform)
                    if fabs(poseThenTransform.position.y) <= 0.02:
                        self.time_waitTransfrom = rospy.get_time()
                        process = 3 # chuyen sang buoc tinh goc quay de lui vao ke
                    else:
                        euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                                        poseThenTransform.orientation.y,\
                                                        poseThenTransform.orientation.z,\
                                                        poseThenTransform.orientation.w))
                        angle = euler[2]

                        # if fabs(angle) >  # viet them truong hop AGV lech goc lon
                        if fabs(angle) > PI/2:
                            SGo_forward = fabs(poseThenTransform.position.y)/sin(PI - fabs(angle))
                        else:
                            SGo_forward = fabs(poseThenTransform.position.y)/sin(fabs(angle))

                        if (poseThenTransform.position.y > 0 and angle > 0) or (poseThenTransform.position.y < 0 and angle < 0):
                            direct_forward = 2
                        else:
                            direct_forward = 1

                        self.step_moveForward = 0
                        process = -20

            elif process == -20: # di chuyen vao duong thang target
                if self.move_forward(SGo_forward, direct_forward, self.min_vel, self.max_vel, 0) == 1:
                    self.time_waitTransfrom = rospy.get_time()
                    process = 2 # kiem tra lai xem da chinh xac chua

            elif process == 3: # tinh goc can quay
                poseThenTransform = Pose()
                if rospy.get_time() - self.time_waitTransfrom <= 1.5:
                    poseThenTransform = self.transformPoseNAV('map', 'frame_target', 'base_footprint', self.req_parking.pose)

                else:
                    print(poseThenTransform)
                    euler = euler_from_quaternion((poseThenTransform.orientation.x,\
                                                    poseThenTransform.orientation.y,\
                                                    poseThenTransform.orientation.z,\
                                                    poseThenTransform.orientation.w))
                    angle = euler[2]
                    print(PI - fabs(angle))
                    if (PI - fabs(angle)) <= 0.04:
                        process = 4 # chuyen sang buoc parking

                    else:
                        AGO_rotary = PI - fabs(angle)
                        if angle > 0:
                            direct_rotary = 1
                        else:
                            direct_rotary = 2

                        self.step_Rotary = 0
                        process = -30

            elif process == -30: # quay vao duong thang target
                if self.rotary_around(AGO_rotary, direct_rotary, self.min_rol, self.max_rol, PI/4.0, 0.015) == 1:
                    self.time_waitTransfrom = rospy.get_time()
                    process = 3 # kiem tra lai xem da chinh xac chua

            elif process == 4: # parking vao ke
                print("Start Parking!")
                # if self.follow_target(self.min_vel, self.max_vel, self.min_vel, 0.3, 0) == 1:
                if self.follow_target(self.min_vel, self.max_vel, self.min_vel, 0.3, 0.2, 0) == 1:
                    print("Done Parking!")
                    process = 5


            # print(process)
            self.rate.sleep()

def main():
    
    try:
        m = ParkingNAV()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()












