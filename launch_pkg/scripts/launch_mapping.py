#!/usr/bin/env python
# author : PhucHoang - 13-9-2021
'''
	launch node mapping
'''

from sti_msgs.msg import *
from std_msgs.msg import *
import roslaunch
import rospy
import string
import time
import os
from subprocess import call

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        
    def start(self):

        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
        self.launch.start()

    def stop(self):
        self.launch.shutdown()

class Start_launch:
    def __init__(self):
        rospy.init_node('launch_mapping', anonymous=True)
        self.rate = rospy.Rate(50)

        # launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid )
        
        #get param       
        self.path_cartographer  = rospy.get_param('~path_cartographer','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/cartogepher/cartographer.launch')    
        self.path_convertMap = rospy.get_param('~path_cartographer','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/mapping/convert_map.launch')

        self.path_mapFolder = rospy.get_param('~path_mapFolder','/home/amr1-l300/catkin_ws/src/launch_pkg/map')

        self.launch_cartographer = Launch(self.path_cartographer)
        self.launch_convertMap = Launch(self.path_convertMap)
        # topic ManageLaunch
        rospy.Subscriber("/control_mapping", Int8, self.call_check0)

        self.pub_StatusMapping = rospy.Publisher('/status_mapping', Int8, queue_size= 10)
        self.pub_ListMap = rospy.Publisher('/listNameMap', List_MapName, queue_size= 10)

        self.arrMapname = List_MapName()

        # variable check 
        self.check0 = 0
        self.stt_MP = 0
        self.time_start_launch = rospy.get_time()

    def exportMapName(self):
        arr_nameFile = []
        arr_name = os.listdir(self.path_mapFolder)
        for f in arr_name:
            num = f.find('.yaml')
            if num != -1:
                arr_nameFile.append(f[:num])

        self.arrMapname.MapName = arr_nameFile
        self.pub_ListMap.publish(self.arrMapname)


    def saveMap(self):
        localtime = time.localtime(time.time())
        _tail = str(localtime[3]) + "_" + str(localtime[4]) + "_" + str(localtime[2]) + "_" + str(localtime[1])
        os.system("rosrun map_server map_saver --occ 60 --free 10 -f /home/amr1-l300/catkin_ws/src/launch_pkg/map/map2d_" + _tail + " map:=/new_map")

    def pubStatus(self, sst_mp):
        dt_mp = Int8()
        dt_mp.data = sst_mp
        self.pub_StatusMapping.publish(dt_mp)

    # check 
    def call_check0(self,data): 
        # if data.data != 0 : self.check0 = True
        # else : self.check0 = False

        self.check0 = data.data

    def raa(self):
        step = 1
        while not rospy.is_shutdown():

            self.exportMapName()
            
            if step == 1 : 
                print("step: {}".format(step))
                if  self.check0 == 1 : 
                    step = 21 
                    self.stt_MP = 1
                    
            if step == 21 : 
                print("step: {}".format(step))
                self.launch_cartographer.start()
                self.launch_convertMap.start()
                # self.time_start_launch = rospy.get_time()
                step = 31 

            if step == 31 : 
                print("step: {}".format(step))
                # if self.check0 == False or self.check1 == True: # bi lap lai khi chua reset

                if self.check0 == 2: # Save Map
                    self.check0 = 1
                    self.saveMap()
                    time.sleep(1.5)
                    self.pubStatus(2)
                    print("Save map Done!")

                if self.check0 == 0:
                    self.launch_cartographer.stop()
                    self.launch_convertMap.stop()
                    self.stt_MP = 0
                    step = 1
                    print("shutdown mapping")


            self.pubStatus(self.stt_MP)
            self.rate.sleep()

            # time.sleep(0.05)


def main():

    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()