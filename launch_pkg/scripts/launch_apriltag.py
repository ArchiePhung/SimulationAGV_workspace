#!/usr/bin/env python
# author : Anh Tuan - 4-1-2021
'''
	launch node apriltag_ros when move Parking or Setpose 
	launch node via cmd with python 
	D435.launch : 8% CPU 
	apriltag_ros : tag_threads : 8 -> chiem 17% cpu 
	apriltag_ros : tag_threads : 2 -> chiem 10% cpu 
	apriltag_ros : tag_threads : 1 -> chiem 7%  cpu 
'''

from sti_msgs.msg import *
import roslaunch
import rospy
import string

class Start_launch:
	def __init__(self):
		rospy.init_node('launch_apriltag', anonymous=True)
		self.rate = rospy.Rate(10)

		# launch
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid )
		
		#get param       
		self.apriltag_setpose  = rospy.get_param('~path_tagSetpose','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/aruco/apriltag_d435_setpose.launch')    
		self.apriltag_parking  = rospy.get_param('~path_tagParking','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/aruco/apriltag_d435_parking.launch') 
		self.check_shelves     = rospy.get_param('~path_checkShelves','/home/amr1-l300/catkin_ws/src/check_shelves/launch/check_shelves.launch')
		self.zone3d            = rospy.get_param('~path_zone3d','/home/amr1-l300/catkin_ws/src/safe_pcl/launch/zone3d.launch')

		# topic ManageLaunch
		rospy.Subscriber("/setpose_control", Setpose_control, self.call_check0)
		rospy.Subscriber("/setpose_status", Setpose_status, self.call_check1)

		rospy.Subscriber("/parking_control", Parking_control, self.call_check2)
		rospy.Subscriber("/parking_status", Parking_status, self.call_check3)

		
		# variable check 
		self.check0 = False
		self.check1 = False
		self.check2 = False
		self.check3 = False

	# check 
	def call_check0(self,data): 
		if data.setposeTag != 0 : self.check0 = True
		else : self.check0 = False

	def call_check1(self,data): 
		if data.status == 6 : self.check1 = True
		else: self.check1 = False

	def call_check2(self,data): 
		if data.idTag !=0 : self.check2 = True
		else : self.check2 = False

	def call_check3(self,data): 
		if data.status == 11 : self.check3 = True
		else: self.check3 = False 

	def start_launch1(self, file_launch):
		self.launch1 = roslaunch.parent.ROSLaunchParent(self.uuid , [file_launch])
		self.launch1.start()

	def start_launch2(self, file_launch):
		self.launch2 = roslaunch.parent.ROSLaunchParent(self.uuid , [file_launch])
		self.launch2.start()
	
	def start_launch3(self, file_launch):
		self.launch3 = roslaunch.parent.ROSLaunchParent(self.uuid , [file_launch])
		self.launch3.start()

	def start_launch4(self, file_launch):
		self.launch4 = roslaunch.parent.ROSLaunchParent(self.uuid , [file_launch])
		self.launch4.start()


	def raa(self):
		step = 1
		while not rospy.is_shutdown():
			if step == 1 : 
				print("step: {}".format(step))
				if  self.check0 == True : step = 21 
				if  self.check2 == True : step = 22 
					
			
			if step == 21 : 
				print("step: {}".format(step))
				self.start_launch1(self.apriltag_setpose)
				step = 31 

			if step == 31 : 
				print("step: {}".format(step))
				# if self.check0 == False or self.check1 == True: # bi lap lai khi chua reset
				if self.check0 == False :
					self.launch1.shutdown()
					step = 1
					print("shutdown 1")
			
			if step == 22 : 
				print("step: {}".format(step))
				self.start_launch2(self.apriltag_parking) #10% cpu
				# self.start_launch3(self.zone3d) #10% cpu
				self.start_launch4(self.check_shelves) 
				step = 32

			if step == 32 : 
				print("step: {}".format(step))
				# if self.check2 == False or self.check3 == True: # bi lap lai khi chua reset
				if self.check2 == False :
					self.launch2.shutdown() 
					# self.launch3.shutdown()
					self.launch4.shutdown()
					step = 1 
					print("shutdown 2")


			self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()