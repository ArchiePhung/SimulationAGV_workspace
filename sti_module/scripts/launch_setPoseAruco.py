#!/usr/bin/env python
# author : PhucHoang 30/08
'''
	launch node setPoseAruco when receive data webconsole 
'''

from sti_msgs.msg import *
import roslaunch
import rospy
import string

class Start_launch:
    def __init__(self):
        rospy.init_node('launch_setPoseAruco', anonymous=True)
        self.rate = rospy.Rate(10)

        # launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid )
        
        #get param  
        self.setPoseAruco  = rospy.get_param('~path_setPoseAruco','/home/amr1-l300/catkin_ws/src/sti_module/launch/getPoseTag.launch') 

        # topic ManageLaunch
        rospy.Subscriber("/setPoseAruco_control", Status_getPoseAruco, self.call_check0)
        
        # variable check 
        self.check0 = False
        self.check1 = False
        self.check2 = False
        self.check3 = False

    # check 
    def call_check0(self,data): 
        if data.status_addPose != 0 : self.check0 = True
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


    def raa(self):
        step = 1
        while not rospy.is_shutdown():
            if step == 1 : 
                print("step: {}".format(step))
                if  self.check0 == True : step = 21 
                if  self.check2 == True : step = 22 
                    
            
            if step == 21 : 
                print("step: {}".format(step))
                self.start_launch1(self.setPoseAruco)
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