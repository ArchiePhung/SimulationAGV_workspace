#!/usr/bin/env python
# author : PhucHoang - 20-9-2021
'''
	launch node MergerMap
'''

from sti_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Point
import roslaunch
import rospy
import string
import time
import os
from subprocess import call
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData

class Launch:
    def __init__(self, file_launch, arr = []):
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


class Start_launch():
    def __init__(self):
        rospy.init_node('launch_mergerMap', anonymous=True)
        self.rate = rospy.Rate(50)
        
        #get param       
        # self.path_mergerMap  = rospy.get_param('~path_mergerMap','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/cartogepher/cartographer.launch')    
        self.path_mergerMap  = rospy.get_param('~path_mergerMap','/home/hoang/Documents/shiv/src/sti_module/launch/merger_map.launch') 

        # self.path_mapFolder = rospy.get_param('~path_mapFolder','/home/amr1-l300/catkin_ws/src/launch_pkg/launch/map')
        self.path_mapFolder = rospy.get_param('~path_mapFolder','/home/hoang/map')

        self.launch_mergerMap = Launch(self.path_mergerMap)

        # topic ManageLaunch
        rospy.Subscriber("/request_mapmerger", Int8, self.call_check0)
        rospy.Subscriber("/param_mergerMap", Parameter_mergerMap, self.call_check1)
        rospy.Subscriber("/map", OccupancyGrid, self.call_check2)

        self.pub_StatusMergerMap = rospy.Publisher('/status_mergerMap', Int8, queue_size= 10)
        self.pub_ListMap = rospy.Publisher('/listNameMap', List_MapName, queue_size= 10)

        self.arrMapname = List_MapName()

        # variable check 
        self.check0 = 0
        self.stt_MM = 0
        self.is_receive_mapName = 0
        self.is_receive_param = False
        self.is_receive_MapMerger = False
        self.time_start_launch = rospy.get_time()

    def exportMapName(self):
        arr_nameFile = []
        arr_name = os.listdir(self.path_mapFolder)
        for f in arr_name:
            num = f.find('.yaml')
            if num != -1:
                arr_nameFile.append(f[:num])

        self.arrMapname = arr_nameFile
        self.pub_ListMap.publish(self.arrMapname)


    def saveMap(self):
        localtime = time.localtime(time.time())
        _tail = str(localtime[3]) + "_" + str(localtime[4]) + "_" + str(localtime[2]) + "_" + str(localtime[1])
        os.system("rosrun map_server map_saver --occ 60 --free 10 -f /home/hoang/map/mapMerger" + _tail + " map:=/map")

    # def saveMap(self):
    #     localtime = time.localtime(time.time())
    #     _tail = str(localtime[3]) + "_" + str(localtime[4]) + "_" + str(localtime[2]) + "_" + str(localtime[1])
    #     os.system("rosrun map_server map_saver --occ 60 --free 10 -f /home/amr1-l300/catkin_ws/src/launch_pkg/map/map2d_" + _tail + " map:=/new_map")

    def pubStatus(self, sst_mm):
        dt_mm = Int8()
        dt_mm.data = sst_mm
        self.pub_StatusMergerMap.publish(dt_mm)

    # check 
    def call_check0(self,data): 
        self.check0 = data.data

    def call_check1(self, data):

        arg = ['map_global:=/home/hoang/map/'+data.pathMapGlobal+'.yaml',
                'map_local:=/home/hoang/map/'+data.pathMapLocal+'.yaml',
                'x_origin:='+str(data.X_origin),
                'y_origin:='+str(data.Y_origin),
                'az_origin:='+str(data.OZ_origin),
                'aw_origin:='+str(data.OW_origin),
                'x_point1:='+str(data.X_point1),
                'y_point1:='+str(data.Y_point1),
                'x_point2:='+str(data.X_point2),
                'y_point2:='+str(data.Y_point2)]
        setattr(self.launch_mergerMap, 'cli_args', arg)
        self.is_receive_param = True

    def call_check2(self, data):
        if self.check0 == 2:
            self.is_receive_MapMerger = True


    def raa(self):
        step = 1
        while not rospy.is_shutdown():

            # print step
            if step == 1:
                if self.check0 == 1 and self.is_receive_param == True:
                    self.is_receive_param = False
                    self.launch_mergerMap.start()
                    self.stt_MM = 1
                    print("run mapMerger")
                    step = 2

            if step == 2:
                if self.check0 == 2 and self.is_receive_MapMerger == True:
                    self.check0 = 1
                    self.is_receive_MapMerger = False
                    self.saveMap()
                    time.sleep(1.5)
                    self.pubStatus(2)
                    print("Save map Done!")

                if self.check0 == 0:
                    self.launch_mergerMap.stop()
                    self.stt_MM = 0
                    step = 1
                    print("shutdown mapmerger")

            self.pubStatus(self.stt_MM)
            self.rate.sleep()

def main():

    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()