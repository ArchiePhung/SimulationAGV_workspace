#!/usr/bin/env python
# author : PhucHoang - 20-9-2021
'''
	launch node mapinfo
'''

from sti_msgs.msg import *
from std_msgs.msg import *
import roslaunch
import rospy
import string
import time
import os
from subprocess import call
import yaml
from nav_msgs.msg import OccupancyGrid

class Launch:
    def __init__(self, file_launch, arr = ['']):
        # -- parameter
        self.fileLaunch = file_launch
        self.cli_args = arr
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        
    def start(self):
        self.launch_files = [(self.fileLaunch, self.cli_args)]
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, self.launch_files)
        self.launch.start()

    def stop(self):
        self.launch.shutdown()

class Start_launch(Launch):
    def __init__(self):
        rospy.init_node('launch_mapinfo', anonymous=True)
        self.rate = rospy.Rate(50)


        self.path_mapInfo = rospy.get_param('~path_mapInfo','/home/amr1-l300/catkin_ws/src/sti_module/launch/mapinfo.launch')
        self.path_mapFolder = rospy.get_param('~path_mapFolder','/home/amr1-l300/catkin_ws/src/launch_pkg/map')
        self.path_fileYaml = '/home/amr1-l300/catkin_ws/src/launch_pkg/map/map2d.yaml'

        self.path_fileunknow = '/home/amr1-l300/catkin_ws/src/launch_pkg/map/'

        self.arrMapname = List_MapName()
        self.nameMapSelect = ''
        self.pathMapSelect = ''

        # variable check 
        self.check0 = 0
        self.stt_MI = 0
        self.data_recieve = 0
        self.is_changed = False
        self.is_recieve_param = False
        self.is_recieve_mapInfo = False
        self.time_start_pub = time.time()
        self.time_receive = time.time()
        self.time_wait_recieve_map = time.time()

        # topic ManageLaunch
        rospy.Subscriber("/mapInfoRequest", Request_mapInfo, self.call_check0)
        rospy.Subscriber("/map_info", OccupancyGrid, self.call_check1)
        rospy.Subscriber("/conditionMapinfo", Bool, self.call_check2)

        self.launch_mapInfo = Launch(self.path_mapInfo)
        
        self.pub_StatusMapInfo = rospy.Publisher('/status_mapInfo', Int8, queue_size= 10)
        self.pub_ListMap = rospy.Publisher('/listNameMap', List_MapName, queue_size= 10)


    def exportMapName(self):
        arr_nameFile = []
        arr_name = os.listdir(self.path_mapFolder)
        for f in arr_name:
            num = f.find('.yaml')
            if num != -1:
                arr_nameFile.append(f[:num])

        self.arrMapname = arr_nameFile
        self.pub_ListMap.publish(self.arrMapname)

    
    def change_value(self, name_file_read, name_file_write):
        with open(name_file_read, 'rU') as f:
            doc = yaml.safe_load(f)
            # print(doc)
        # doc[key] = value

        with open(name_file_write, 'w') as f:
            yaml.dump(doc, f, default_flow_style=False)

        return 10


    def pubStatus(self, sst_mi):
        dt_mi = Int8()
        dt_mi.data = sst_mi
        self.pub_StatusMapInfo.publish(dt_mi)

    # check 
    def call_check0(self,data): 
        self.check0 = data.requestStatus

        if self.check0 == 1 and self.nameMapSelect != data.mapName:

            self.nameMapSelect = data.mapName
            self.pathMapSelect = self.path_fileunknow + self.nameMapSelect+'.yaml'
            setattr(self.launch_mapInfo, 'cli_args', ['path_map:=' + self.pathMapSelect])
            self.is_recieve_param = True
        
        if self.check0 == 2:
            self.exportMapName()

    def call_check1(self, data):
        self.is_recieve_mapInfo = True

    def call_check2(self, data):
        self.time_receive = time.time()


    def raa(self):
        step = 1
        while not rospy.is_shutdown():
            # print step
            if step == 1:
                if self.check0 == 1 and self.is_recieve_param == True:
                    self.launch_mapInfo.start()
                    self.is_recieve_param = False
                    self.time_wait_recieve_map = time.time()
                    step = 2

            if step == 2:
                if self.is_recieve_mapInfo == True:
                    self.is_recieve_mapInfo = False
                    self.stt_MI = 1
                    step = 3
                    self.time_receive = time.time()
                    print("run mapInfo")

                elif time.time() - self.time_wait_recieve_map > 10:
                    self.launch_mapInfo.stop()
                    self.is_changed = False
                    self.nameMapSelect = ''
                    self.stt_MI = 0
                    step = 1
                    print("shutdown mapInfo because dont recieve data of map")
                
            if step == 3:
                if self.check0 == 0 or (time.time() - self.time_receive > 3600):
                    self.launch_mapInfo.stop()
                    self.is_changed = False
                    self.nameMapSelect = ''
                    self.stt_MI = 0
                    step = 1
                    print("shutdown mapInfo")

                if self.check0 == 3 and self.is_changed == False:
                    if self.change_value(self.pathMapSelect, self.path_fileYaml) == 10:
                        self.pubStatus(3)
                        self.is_changed = True
                        print("Done change Map!")

            if time.time() - self.time_start_pub > 0.1:
                self.pubStatus(self.stt_MI)
                self.time_start_pub = time.time()

            self.rate.sleep()

def main():
    print("start main")

    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
