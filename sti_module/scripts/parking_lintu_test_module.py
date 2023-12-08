#!/usr/bin/env python

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import *
from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs
import time
import threading
import signal
import os

class StiAutomaticParking():
    def __init__(self):
        rospy.init_node('parking_v10', anonymous=True)
        self.rate = rospy.Rate(100)

        #get param 
        self.dolech_x_CamVsRObot = rospy.get_param('~dolech_x_CamVsRObot',0.033)
        self.tolerance_max       = rospy.get_param('~tolerance_max',0.05)
        self.tolerance_min       = rospy.get_param('~tolerance_min',0.02)
        self.distance_max        = rospy.get_param('~distance_max',1)
        self.distance_min        = rospy.get_param('~distance_min',0.02)
        self.x_robot             = rospy.get_param('~x_robot',1.045)
        self.kc_tag_robot_setup  = rospy.get_param('~kc_tag_robot_setup',1.5)
        self.checkke             = rospy.get_param('~checkke',False)
        self.kc_checkke          = rospy.get_param('~kc_checkke',1)

        # self.rate = rospy.Rate(20)
        self.tf_listener = tf.TransformListener()
        self.tf_listen_angle = tf.TransformListener()
        self.tf_listen_angle_ql = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster_ql = tf.TransformBroadcaster()

        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 100)
        self.pub_cmd_vel    = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        # self.pub_log        = rospy.Publisher('/data_log', Pose, queue_size=100)

        self.pub_status = rospy.Publisher('/parking_status', Parking_status, queue_size=100) 
        # rospy.Subscriber("/parking_control", Parking_control, self.callSup)

        rospy.Subscriber("/magneticLine1", Magnetic_line, self.callSup1)
        # rospy.Subscriber("/check_shelves", Check_shelves, self.callSup4)

        self.d_shelves = Check_shelves()
        self.is_odom_received = False  
        rospy.on_shutdown(self.fnShutDown)


        # ---- bien global
        self.idTag_target = 14
        self.tolerance = .0
        self.distance_target = .0
        self.goc_taget = .0
        self.tt_x = .0 # kc Tag <-> tam robot
        self.tt_z = .0
        self.tt_g = .0
        self.is_marker_pose_received = False 
        self.is_marker_pose_received2 = False 
        self.tag_g = .0 # robot huong vao apriltag thi tag_g =0
        self.find_id = 0 # if robot dang nhin thay
        self.id_legal = False

        self.rb_x = .0
        self.rb_z = .0 
        self.rb_g = .0

        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0

        self.enable_parking = False
        self.solanhoatdong  = 0
        self.v_robot = False

        self.time_ht = rospy.get_time()
        self.time_tr = rospy.get_time()
        self.rate_cmdvel = 30. 

        # time
        self.t1_ht = rospy.get_time()
        self.t1_tr = rospy.get_time()
        self.t2_ht = rospy.get_time()
        self.t2_tr = rospy.get_time()
        self.t3_ht = rospy.get_time()
        self.t3_tr = rospy.get_time()
        #zone
        self.zone_robot = Zone_lidar_2head()
        self.job = 0
        #lintu 
        self.line1 = 0.
        self.line2 = 0.
        self.line1_stt = False
        self.line2_stt = False
        self.vel_line = 0.15 
        self.sl_loc = 30 # 20 sap xi 1s
        self.step_charge = 0 

        self.step_runros = 0
        self.step_runlintu = 0
        self.s_runros = 0.

        self.odom_x_ht = 0.
        self.odom_y_ht = 0.

        self.min_vel_theta = 0.15
        self.min_vel_x = 0.08

        # thread 2
        #get param 
        self.kalman_r = rospy.get_param('~kalman_r',0.1)
        self.kalman_p = rospy.get_param('~kalman_p',1.0)
        self.kalman_q = rospy.get_param('~kalman_q',0.1)

        # rospy.init_node('tag_detections_filter')
        # self.rate = rospy.Rate(100)

        self.sub_info_marker = rospy.Subscriber('/tag_detections_d435_parking', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
        # self.pub_filter = rospy.Publisher('/tag_detections_filter', Point, queue_size=1)

        # kalman
        self.k1 = self.x_ht1 = self.x_truoc1 = .0
        self.f1 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k2 = self.x_ht2 = self.x_truoc2 = .0
        self.f2 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k3 = self.x_ht3 = self.x_truoc3 = .0
        self.f3 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q

        # check ke 
        self.dem_ke = 0 
        self.dem_time_checkke = 0
        self.dem_time_checkke_yes = 0
        self.is_zone_robot = False

    def callSup1(self,line1):
        if self.line1_stt == False : self.line1_stt = True
        if line1.status == 1 : self.line1 = line1.value
        else : self.line1 = -2 # error magLine

    def pub_cmdVel(self, twist , rate):

        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist())

    def cbGetRobotOdom(self, robot_odom_msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,\
                        robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.odom_x = robot_odom_msg.pose.pose.position.x
        self.odom_y = robot_odom_msg.pose.pose.position.y
        # self.odom_g = (theta*180)/np.pi
        # print self.odom_g
        self.odom_g = theta
        
        # van toc robot 
        if((fabs(robot_odom_msg.twist.twist.linear.x ) < 0.01) and \
            (fabs(robot_odom_msg.twist.twist.angular.z ) < 0.01) ) :
            self.v_robot = True
        else :
            self.v_robot = False

    def run1_lineMag_checkke(self, vel ):
        twist = Twist()

        if self.line1_stt == True :
            self.line1_stt = False
            if self.line1 == -1 : 
                # print("ngoai line ")
                return 0
                
            if self.line1 > -1 and self.line1 < 16 : 
                
                err = self.line1 - 7.5 
                # print("err= {}".format(err))
                kp = 0.05
                twist.linear.x = vel # 0.15 
                twist.angular.z = -1 * err * kp
                self.pub_cmdVel(twist,self.rate_cmdvel)
                # print("twist= {}".format(twist))
                

            if self.line1 == 20 :
                # print("line ngang") 
                self.pub_cmd_vel.publish(Twist())
                # rospy.logwarn("ok")
                return 1
        else : return 0 


    def turn_ar(self, theta, tol_theta, vel_rot):
        if fabs(theta) > tol_theta: # +- 10 do
            if theta > 0: #quay trai
                # print "b"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < 0.15:
                    vel_th = 0.15

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            if theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -0.15:
                    vel_th = -0.15

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return -10 

    def run_ros(self, quangduong ):
		twist = Twist()
		if self.step_runros == 0 :
			self.odom_x_ht = self.odom_x
			self.odom_y_ht = self.odom_y
			self.step_runros = 1 

		if self.step_runros == 1 : 
			s = self.fnCalcDistPoints(self.odom_x,self.odom_x_ht,self.odom_y,self.odom_y_ht)
			if quangduong > 0 : # tien 
				# rospy.loginfo("tien")
				if s < quangduong : 
					twist.linear.x = 0.15 
					self.pub_cmdVel(twist,self.rate_cmdvel)
				else :
					self.step_runros = 2 
			if quangduong < 0 : # lui
				# rospy.loginfo("lui")
				if s < fabs(quangduong) : 
					twist.linear.x = -0.15 
					self.pub_cmdVel(twist,self.rate_cmdvel)
				else :
					self.step_runros = 2 

			if quangduong == 0 : # stop
				self.step_runros = 2 
		if self.step_runros == 2 :
			# rospy.loginfo("stop")
			self.pub_cmd_vel.publish(Twist())
			return 1 

    def fnGetDataTag(self,sl_loc): 
        x_ht = z_ht = g_ht = .0
        x = z = g = .0
        dem = 0
        for i in range(sl_loc):
            # print tt_x
            while x_ht == self.tt_x :
                dem = dem
            else:
                # print "dem :" ,dem
                if( sl_loc - dem <= 10):
                    x = x + self.tt_x 
                    z = z + self.tt_z 
                    g = g + self.tt_g 
                    # print "tt_x: ", tt_x
                    
                x_ht = self.tt_x
                dem = dem + 1
        # lay trung binh 5 lan cuoi
        x = x / 10.
        z = z / 10.
        g = g / 10.
        return x, z, g

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        # print "fnCalcDistPoints"
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def average_filter(self, sl_loc):
        x_ht = z_ht = g_ht = .0
        x = z = g = .0
        dem = 0
        dem_gt_dau = 0
        tol_cp = 0.01

        while dem < sl_loc:
            if x_ht != self.tt_x :
                if dem_gt_dau < 50:
                    dem_gt_dau = dem_gt_dau + 1
                elif fabs(self.tt_x) >= fabs(x_ht) - tol_cp and fabs(self.tt_x) <= fabs(x_ht) + tol_cp:
                    x = x + self.tt_x 
                    z = z + self.tt_z 
                    g = g + self.tt_g 
                    dem = dem + 1
                    print dem

                x_ht = self.tt_x

        x = x / sl_loc
        z = z / sl_loc
        g = g / sl_loc
        return x, z, g
    

        #### thread 2 ######

    #THREAD 2
    def my_kalman1(self,input):
        self.k1 = self.f1[1]/(self.f1[1] + self.f1[0])
        self.x_ht1 = self.x_truoc1 + self.k1*(input-self.x_truoc1)
        self.f1[1] = (1-self.k1)*self.f1[1]+ fabs(self.x_truoc1-self.x_ht1)*self.f1[2]
        self.x_truoc1 = self.x_ht1    
        return self.x_ht1

    def my_kalman2(self,input):
        self.k2 = self.f2[1]/(self.f2[1] + self.f2[0])
        self.x_ht2 = self.x_truoc2 + self.k2*(input-self.x_truoc2)
        self.f2[1] = (1-self.k2)*self.f2[1]+ fabs(self.x_truoc2-self.x_ht2)*self.f2[2]
        self.x_truoc2 = self.x_ht2      
        return self.x_ht2

    def my_kalman3(self,input):
        self.k3 = self.f3[1]/(self.f3[1] + self.f3[0])
        self.x_ht3 = self.x_truoc3 + self.k3*(input-self.x_truoc3)
        self.f3[1] = (1-self.k3)*self.f3[1]+ fabs(self.x_truoc3-self.x_ht3)*self.f3[2]
        self.x_truoc3 = self.x_ht3    
        return self.x_ht3


    def cbGetMarkerOdom(self, markers_odom_msg): # 5Hz

        if self.is_marker_pose_received2 == False:
            self.is_marker_pose_received2 = True

        sl_tag = len(markers_odom_msg.detections)
        if sl_tag == 0 : 
            self.find_id = 0 
            self.tag_g = .0 
        else :
            for sl_tag in range(sl_tag):            
                # check id
                id = markers_odom_msg.detections[sl_tag-1].id[0] 

                if self.is_marker_pose_received == False:
                    self.is_marker_pose_received = True
                if id == self.idTag_target:
                    self.find_id = id 
                    marker_odom = markers_odom_msg.detections[sl_tag-1]

                    self.tag_g = atan2(marker_odom.pose.pose.pose.position.x, marker_odom.pose.pose.pose.position.z)

        
        name_tag = "tag_" + str(int(self.idTag_target))
        if self.is_marker_pose_received == True :
            if self.find_id == self.idTag_target :
                self.id_legal = True
                self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(5)) 
                position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
                quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]

                self.tt_x = self.my_kalman1(position[0])
                # tt_z = position[0] # 
                self.tt_z = self.my_kalman2(position[2])
                self.tt_g = self.my_kalman3(theta) # radian
                self.is_marker_pose_received = False
                # print "tt_x= %s , tt_z= %s, tt_g= %s" %(self.tt_x, self.tt_z, self.tt_g)
                # print "tt_x=",position[0]
            else :
                self.id_legal = False

    def raa(self):
        buoc = 0
        theta_tg = 0.0
        goc_can_xoay = 0.0
        odom_x_ht = 0.0
        odom_y_ht = 0.0
        x_target = 0.0
        direc_turn_again = 0
        angle_ql = 0.0
        while not rospy.is_shutdown() :

            if buoc == 0:
                if self.id_legal == True and self.find_id != 0 : 
                    self.id_legal = False
                    buoc = 1
                else:
                    buoc = -1
                    self.t1_tr = rospy.get_time()

            elif buoc == -1:
                twist = Twist()
                if ( self.id_legal == False ) and (rospy.get_time() - self.t1_tr < 5 ):
                    twist.angular.z = 0.3
                    self.pub_cmdVel(twist,self.rate_cmdvel)

                elif ( self.id_legal == False or self.find_id == 0 ) and (rospy.get_time() - self.t1_tr > 5 ):
                    self.pub_cmd_vel.publish(Twist())
                    buoc = -2
                    self.t1_tr = rospy.get_time()
                    rospy.sleep(1.0)

                elif self.id_legal == True and self.find_id != 0 : 
                    self.id_legal = False
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 1

            elif buoc == -2 :
                rospy.loginfo("quay phai tim TAG")
                twist = Twist()
                if (rospy.get_time() - self.t1_tr < 30 ):
                    if self.id_legal == False or self.find_id == 0 :
                    
                        twist.angular.z = -0.3
                        self.pub_cmdVel(twist,self.rate_cmdvel)
                        
                    elif self.id_legal == True and self.find_id != 0 : 
                        self.id_legal = False
                        self.pub_cmd_vel.publish(Twist())
                        buoc  = 1
                else: 
                    buoc = -3

            elif buoc == -3:
                self.pub_cmd_vel.publish(Twist())
                rospy.loginfo("quay tim TAG > 30s")

            elif buoc == 1:
                if fabs(self.tag_g) > 0.1: #2do
                    twist = Twist()
                    if self.tag_g > 0 : twist.angular.z = -0.15
                    else :         twist.angular.z = 0.15

                    self.pub_cmdVel(twist, self.rate_cmdvel)
                    # self.pub_cmd_vel.publish(twist)
                else:

                    self.pub_cmd_vel.publish(Twist())
                    if self.v_robot == True: # robot xac nhan da dung
                        buoc = 2
                        # rospy.loginfo("buoc: %s",buoc) 

            elif buoc == 2:
                # self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(self.sl_loc) # rad
                self.rb_x, self.rb_z, self.rb_g = self.average_filter(self.sl_loc)
                # print "position_x = %s, position_z = %s, theta = %s, tt_x= %s , tt_z= %s, tt_g= %s" %(self.tt_x, self.tt_z, self.tt_g, self.rb_x, self.rb_z, self.rb_g)
                # rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s" ,self.rb_x, self.rb_z, self.rb_g)
                # x_target = self.rb_x - 0.15
                x_target = round(self.rb_x,3) - self.dolech_x_CamVsRObot
                print x_target
                if fabs(x_target) <= 0.035:
                    goc_can_xoay = PI/2.0
                    buoc = 3
                else:

                    if x_target > 0: # quay trai
                        direc_turn_again = 1 # lat quay phai
                        self.rb_g = fabs(self.rb_g)
                    else:
                        direc_turn_again = 2 # lat quay trai
                        self.rb_g = -fabs(self.rb_g)
                    self.tf_broadcaster.sendTransform((0, 0, 0),
                                                    tf.transformations.quaternion_from_euler(0, 0, self.rb_g),
                                                    rospy.Time.now(),
                                                    "f_angle",
                                                    "base_footprint")

                    self.tf_listen_angle.waitForTransform("odom","f_angle",rospy.Time(),rospy.Duration(5)) 
                    position, quaternion = self.tf_listen_angle.lookupTransform("odom", "f_angle", rospy.Time())
                    quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                    theta_tg = tf.transformations.euler_from_quaternion(quaternion)[2]
                    goc_can_xoay = theta_tg - self.odom_g
                    # print(angle_bt, angle_fn)
                    if fabs(goc_can_xoay) >= PI:
                            theta_t = (2*PI - fabs(goc_can_xoay))
                            if goc_can_xoay > 0:
                                goc_can_xoay = -theta_t
                            else:
                                goc_can_xoay = theta_t
                    # print theta_tg
                    buoc = 3

            elif buoc == 3:
                rb_z_temp = self.rb_z - self.x_robot/2
                print rb_z_temp
                self.distance_target = self.kc_tag_robot_setup
                if rb_z_temp > (self.distance_target + 0.05) : # robot dung xa dau line tu 
                    print 'qua xa'
                    x_target = ((self.rb_z-fabs(rb_z_temp - self.distance_target))/self.rb_z)*self.rb_x - self.dolech_x_CamVsRObot
                    self.s_runros = (rb_z_temp - self.distance_target)/sin(fabs(goc_can_xoay))
                    self.step_runros = 0
                    buoc = 4

                elif rb_z_temp < (self.distance_target - 0.05) : # robot dung qua gan
                    print 'qua gan'
                    x_target = ((self.rb_z+fabs(rb_z_temp - self.distance_target))/self.rb_z)*self.rb_x - self.dolech_x_CamVsRObot
                    self.s_runros = (rb_z_temp - self.distance_target)/sin(fabs(goc_can_xoay)) 
                    self.step_runros = 0
                    buoc =4
                else:
                    buoc = 5

            elif buoc == 4:

                if self.run_ros(self.s_runros ) == 1 :
                    buoc = 5
                    rospy.sleep(1.0)

            elif buoc == 5:

                if self.line1 > -1 and self.line1 < 16 and fabs(x_target) < 0.5:
                    buoc = 12
                    self.step_runlintu = 0
                else : 
                    buoc = 6

            elif buoc == 6:
                # theta = (theta*180)/np.pi
                # print theta
                theta = theta_tg - self.odom_g
                # print(angle_bt, angle_fn)
                if fabs(theta) >= PI:
                        theta_t = (2*PI - fabs(theta))
                        if theta > 0:
                            theta = -theta_t
                        else:
                            theta = theta_t
                # print theta
                twist = Twist()
                if fabs(theta) > 0.025:
                    if theta > 0: # quay trai
                        twist.linear.x = 0.0
                        twist.angular.z = 0.2
                    elif theta < 0: # quay phai
                        twist.linear.x = 0.0
                        twist.angular.z = -0.2

                    self.pub_cmdVel(twist, self.rate_cmdvel)
                    # self.pub_cmd_vel.publish(twist)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    rospy.sleep(1.0)
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    buoc = 7

            elif buoc == 7:
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                rospy.loginfo("x_target: %s ", x_target)
                # rospy.loginfo("s: %s", s )
                if fabs(x_target) - fabs(s) > 0.0: 
                    twist.linear.x = + ( fabs(x_target) - fabs(s) ) + 0.02
                    if twist.linear.x > 0.2 : twist.linear.x = 0.2
                    if twist.linear.x < 0.08 : twist.linear.x = 0.08
                    self.pub_cmdVel(twist, self.rate_cmdvel)
                    # self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    # odom_x_ht = self.odom_x
                    # odom_y_ht = self.odom_y
                    buoc = 8
                    # rospy.loginfo("buoc: %s",buoc)
            
            elif buoc == 8: # tinh toan goc quay lai
                goc_quay_lai = 0.0
                if direc_turn_again == 1: # quay phai
                    goc_quay_lai = -PI/2

                elif direc_turn_again == 2: # quay trai
                    goc_quay_lai = PI/2

                self.tf_broadcaster_ql.sendTransform((0, 0, 0),
                                                tf.transformations.quaternion_from_euler(0, 0, goc_quay_lai),
                                                rospy.Time.now(),
                                                "f_angle_quay_lai",
                                                "base_footprint")

                self.tf_listen_angle_ql.waitForTransform("odom","f_angle_quay_lai",rospy.Time(),rospy.Duration(5)) 
                position, quaternion = self.tf_listen_angle_ql.lookupTransform("odom", "f_angle_quay_lai", rospy.Time())
                quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                angle_ql = tf.transformations.euler_from_quaternion(quaternion)[2]
                buoc = 9
            
            elif buoc == 9:
                theta = angle_ql - self.odom_g
                # print(angle_bt, angle_fn)
                if fabs(theta) >= PI:
                        theta_t = (2*PI - fabs(theta))
                        if theta > 0:
                            theta = -theta_t
                        else:
                            theta = theta_t
                # print theta
                twist = Twist()
                if fabs(theta) > 0.025:
                    if theta > 0: # quay trai
                        twist.linear.x = 0.0
                        twist.angular.z = 0.2
                    elif theta < 0: # quay phai
                        twist.linear.x = 0.0
                        twist.angular.z = -0.2
                    self.pub_cmdVel(twist, self.rate_cmdvel)
                    # self.pub_cmd_vel.publish(twist)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    rospy.sleep(1.0)
                    buoc = 10

                # print theta
            elif buoc == 10:
                if fabs(self.tag_g) > 0.05 : #2do
                    if self.tag_g > 0 : twist.angular.z = -0.15
                    else :         twist.angular.z = 0.15
                    self.pub_cmdVel(twist,self.rate_cmdvel)

                else:
                    self.pub_cmd_vel.publish(Twist())
                    if self.v_robot == True:
                        buoc = 11
                        rospy.loginfo("buoc : %s", buoc)

            elif buoc == 11:
                if self.line1 > -1 and self.line1 < 16:
                    buoc = 12
                    self.step_runlintu = 0
                else : 
                    buoc = 2 # da thu 77 , -5 deu k duoc 

            elif buoc == 12:
                if self.run1_lineMag_checkke(self.vel_line) == 1 : buoc = 13
            print buoc
            self.rate.sleep()


def main():
    
    try:
        m = StiAutomaticParking()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
