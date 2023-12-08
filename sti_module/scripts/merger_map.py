#!/usr/bin/env python
# author : PhucHoang - 13-9-2021
'''
	launch node mapping
'''


import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist ,Pose ,Point, PointStamped, TransformStamped, Point32
from sensor_msgs.msg import PointCloud
from sti_msgs.msg import *
from std_msgs.msg import *
from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, radians, degrees
import time
import signal
import os
import threading
import signal

class MergerMap(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        rospy.init_node('MergerMap', anonymous=False)
        self.rate = rospy.Rate(50)

        self.is_killed = 0
        self.X1_local = rospy.get_param('~X1_local',-7.18489074707) #-7.18489074707
        self.Y1_local = rospy.get_param('~Y1_local',-3.17659091949) #-3.17659091949
        self.X2_local = rospy.get_param('~X2_local',-2.63227009773) #-2.63227009773
        self.Y2_local = rospy.get_param('~Y2_local',4.88385868073) #4.88385868073

        self.x_tf = rospy.get_param('~x_tf',2.574746) #2.574746
        self.y_tf = rospy.get_param('~y_tf',0.79094) #0.79094
        self.az_tf = rospy.get_param('~az_tf',-0.015402) #-0.015402
        self.aw_tf = rospy.get_param('~aw_tf',0.999881) #0.999881

        self.quata = ( 0,0,self.az_tf,self.aw_tf)
        self.euler = euler_from_quaternion(self.quata)
        self.euler_tf = self.euler[2]

        self.uints_XY = 0.05
        self.uints_Angle = radians(0.04)

        rospy.Subscriber('/global_map', OccupancyGrid, self.subGlobalMap)
        rospy.Subscriber('/local_map', OccupancyGrid, self.subLocalMap)
        rospy.Subscriber('/config_map', Map_config, self.subConfigMap)

        rospy.Subscriber("/request_mapmerger", Int8, self.call_check0)

        self.pub_saveMap = rospy.Publisher('/confirm_publishMap', Int8, queue_size=10)
        self.pubMap    = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.data_mapGlobal = OccupancyGrid()
        self.data_mapLocal = OccupancyGrid()
        self.data_mapPub = OccupancyGrid()

        self.data_mapPub.header.frame_id = 'map'
        self.data_mapPub.header.stamp = rospy.Time()
        self.data_mapPub.info.map_load_time = rospy.Time()

        self.data_cutMap = []
        self.dataPub = []

        self.dataPoint_cutMap = PointCloud()
        self.dataPoint_cutMap.header.frame_id = "local_map"
        self.dataPoint_cutMap.header.stamp = rospy.Time()

        self.data_pointTF = PointCloud()
        self.t = (1,0)

        self.widthMapMerger = 0
        self.heightMapMerger = 0
        self.valdis = 0.3
        self.numberEmptyBox = 0 

        self.is_subGlobal = False
        self.is_subLocal = False  
        self.is_subConfigMap = False
        self.is_subParamMergerMap = False
        self.check0 = 0

    # def subParamMerMap(self, data):
    #     self.X1_local = data.pointCutStart.x
    #     self.Y1_local = data.pointCutStart.y
    #     self.X2_local = data.pointCutStop.x
    #     self.Y2_local = data.pointCutStop.y

    #     self.x_tf = data.localMapCoordinates.position.x
    #     self.y_tf = data.localMapCoordinates.position.y
    #     self.az_tf = data.localMapCoordinates.orientation.z
    #     self.aw_tf = data.localMapCoordinates.orientation.w

    #     self.quata = ( 0,0,self.az_tf,self.aw_tf)
    #     self.euler = euler_from_quaternion(self.quata)
    #     self.euler_tf = self.euler[2]

    #     self.is_subParamMergerMap = True

    def call_check0(self,data): 
        self.check0 = data.data


    def subConfigMap(self, data):
        uints_XY = data.type_units_XY if data.type_units_XY != 0.0 else self.uints_XY
        uints_Angle = data.type_units_Angle if data.type_units_Angle != 0.0 else self.uints_Angle
        if data.status == 1: self.x_tf = self.x_tf + uints_XY
        if data.status == 2: self.x_tf = self.x_tf - uints_XY
        if data.status == 3: self.y_tf = self.y_tf + uints_XY
        if data.status == 4: self.y_tf = self.y_tf - uints_XY
        if data.status != 5: 
            self.euler_tf = self.euler_tf + uints_Angle
            q = quaternion_from_euler(0.0, 0.0, self.euler_tf)
            self.az_tf = q[2]
            self.aw_tf = q[3]
        if data.status != 6: 
            self.euler_tf = self.euler_tf - uints_Angle
            q = quaternion_from_euler(0.0, 0.0, self.euler_tf)
            self.az_tf = q[2]
            self.aw_tf = q[3]

        self.is_subConfigMap = True

        rospy.loginfo('x = %lf, y = %lf, az = %lf, aw = %lf',self.x_tf, self.y_tf, self.az_tf, self.aw_tf)

    def subGlobalMap(self, data):
        self.data_mapGlobal = data
        self.data_mapPub.info.resolution = data.info.resolution
        self.numberEmptyBox = int(round(self.valdis/data.info.resolution))
        self.is_subGlobal = True
        # print hex(id(self.arrMapGlobal))

        # print type(self.data_mapGlobal.data)
        # print self.arrayDataMap

    def subLocalMap(self, data):
        self.data_mapLocal = data
        self.is_subLocal = True
        print "done recieve local map"


    def exportDataLocal(self):

        # if self.is_subParamMergerMap == True:
        del self.data_cutMap[:]
        del self.dataPoint_cutMap.points[:]

        row_start = int(round((self.Y1_local - self.data_mapLocal.info.origin.position.y)/self.data_mapLocal.info.resolution))
        col_start = int(round((self.X1_local - self.data_mapLocal.info.origin.position.x)/self.data_mapLocal.info.resolution))
        row_stop = int(round((self.Y2_local - self.data_mapLocal.info.origin.position.y)/self.data_mapLocal.info.resolution))
        col_stop = int(round((self.X2_local - self.data_mapLocal.info.origin.position.x)/self.data_mapLocal.info.resolution))

        if row_start > row_stop:
            row_start, row_stop = row_stop, row_start
        if col_start > col_stop:
            col_start, col_stop = col_stop, col_start

        if row_stop >= self.data_mapLocal.info.height : row_stop = self.data_mapLocal.info.height
        if col_stop >= self.data_mapLocal.info.width : col_stop = self.data_mapLocal.info.width


        for row in range(row_start, row_stop, 1):
            for col in range(col_start, col_stop, 1):

                pointLocalMap = Point32()

                d_map = self.data_mapLocal.data[row*self.data_mapLocal.info.width + col]
                self.data_cutMap.append(d_map)

                pointLocalMap.x = col*self.data_mapLocal.info.resolution + self.data_mapLocal.info.origin.position.x
                pointLocalMap.y = row*self.data_mapLocal.info.resolution + self.data_mapLocal.info.origin.position.y
                # rospy.loginfo('x = %lf, y = %lf "',pointLocalMap.point.x, pointLocalMap.point.y)

                self.dataPoint_cutMap.points.append(pointLocalMap)
                # print self.dataPoint_cutMap

        print "done export data local map"

    def TransForm(self):

        self.tf_broadcaster.sendTransform((self.x_tf, self.y_tf, 0.0),
                                        (0.0, 0.0, self.az_tf, self.aw_tf), 
                                        rospy.Time.now(),
                                        "local_map",
                                        "map")
        
        # tf.header.stamp = rospy.Time.now()
        # tf.header.frame_id = "map"
        # tf.child_frame_id = "local_map"
        # tf.transform.translation.x = self.x_tf
        # tf.transform.translation.y = self.y_tf
        # tf.transform.translation.z = 0.0
        # tf.transform.rotation.x = 0
        # tf.transform.rotation.y = 0
        # tf.transform.rotation.z = self.az_tf
        # tf.transform.rotation.w = self.aw_tf
        # self.tf_broadcaster.sendTransform(tf)

    def TransformPoint(self):
        del self.data_pointTF.points[:]
        point_tf = PointStamped()
        self.tf_listener.waitForTransform("map","local_map",rospy.Time(),rospy.Duration(10)) 
        self.data_pointTF = self.tf_listener.transformPointCloud("map", self.dataPoint_cutMap)
        print('Done TransForm!')
        # for i in range(0, len(self.dataPoint_cutMap), 1):
        #     self.dataPoint_transform.append(self.tf_listener.transformPoint("map", self.dataPoint_cutMap[i]))
        # point_tf = self.tf_listener.transformPoint("map", pointTransform)

    def funPubMap(self):
        
        dataP = list(self.dataPub)
        # print id(dataP)
        self.TransformPoint()

        for i in range(0, len(self.data_pointTF.points), 1):
            point_dt = self.data_pointTF.points[i]  
            # print self.dataPoint_transform[i]
            row = int(round((point_dt.y - self.data_mapPub.info.origin.position.y)/self.data_mapPub.info.resolution))
            col = int(round((point_dt.x - self.data_mapPub.info.origin.position.x)/self.data_mapPub.info.resolution))
            # print i
            # print row, col
            ss_row = self.data_mapPub.info.height - 1 - self.numberEmptyBox
            ss_col = self.data_mapPub.info.width - 1 - self.numberEmptyBox
            if row > ss_row:
                print('qua tren')
                self.dataPub = list(self.dataPub)
                height_new = row + 1 + self.numberEmptyBox
                numberHeight_add = (height_new - self.data_mapPub.info.height)*self.data_mapPub.info.width
                for j in range(0, numberHeight_add, 1):
                    dataP.append(-1)
                    self.dataPub.append(-1)

                # self.dataPub = list(self.dataPub)
                # self.dataPub = dataP
                self.dataPub = tuple(self.dataPub)
                self.data_mapPub.info.height = height_new

            if col > ss_col:
                print('qua phai')
                self.dataPub = list(self.dataPub)
                width_new = col + 1 + self.numberEmptyBox
                col_add = width_new - self.data_mapPub.info.width
                for j in range(0, self.data_mapPub.info.height, 1):
                    for k in range (0, col_add, 1):
                        dataP.insert(self.data_mapPub.info.width*(j+1) + j*col_add, -1) 
                        self.dataPub.insert(self.data_mapPub.info.width*(j+1) + j*col_add, -1) 

                # self.dataPub = list(self.dataPub)
                # self.dataPub = dataP
                self.dataPub = tuple(self.dataPub)
                self.data_mapPub.info.width = width_new

            if row < 0:
                print('qua duoi')
                self.dataPub = list(self.dataPub)
                height_new = self.data_mapPub.info.height + abs(row) + self.numberEmptyBox
                numberHeight_add = (height_new - self.data_mapPub.info.height)*self.data_mapPub.info.width
                for j in range(0, numberHeight_add, 1):
                    dataP.insert(0, -1)
                    self.dataPub.insert(0, -1)

                self.data_mapPub.info.origin.position.y = self.data_mapPub.info.origin.position.y - (height_new - self.data_mapPub.info.height)*self.data_mapPub.info.resolution
                row = int(round((point_dt.y - self.data_mapPub.info.origin.position.y)/self.data_mapPub.info.resolution))
                # print(row)
                # self.dataPub =  dataP
                self.dataPub = tuple(self.dataPub)
                self.data_mapPub.info.height = height_new

            if col < 0:
                print('qua trai')
                self.dataPub = list(self.dataPub)
                width_new = self.data_mapPub.info.width + abs(col) + self.numberEmptyBox
                col_add = width_new - self.data_mapPub.info.width
                for j in range(0, self.data_mapPub.info.height, 1):
                    for k in range (0, col_add, 1):
                        dataP.insert(self.data_mapPub.info.width*j + j*col_add, -1) 
                        self.dataPub.insert(self.data_mapPub.info.width*j + j*col_add, -1)

                self.data_mapPub.info.origin.position.x = self.data_mapPub.info.origin.position.x - (width_new - self.data_mapPub.info.width)*self.data_mapPub.info.resolution
                col = int(round((point_dt.x - self.data_mapPub.info.origin.position.x)/self.data_mapPub.info.resolution))
                self.dataPub = tuple(self.dataPub)
                self.data_mapPub.info.width = width_new

            dataP[row*self.data_mapPub.info.width + col] = self.data_cutMap[i]

        print "done publish new map"
        self.data_mapPub.data = tuple(dataP)


    def run(self): 
        buoc = 0
        is_pub = 0
        while not self.shutdown_flag.is_set() or (self.is_killed == 1):

            # print(buoc)
            if buoc == 0:
                if self.is_subLocal == True:
                    self.exportDataLocal()
                    buoc = 1

            elif buoc == 1:

                if self.is_subGlobal == True:
                    # self.is_subGlobal = False
                    self.dataPub = self.data_mapGlobal.data
                    self.data_mapPub.info.height = self.data_mapGlobal.info.height
                    self.data_mapPub.info.width = self.data_mapGlobal.info.width
                    self.data_mapPub.info.origin = self.data_mapGlobal.info.origin
                    buoc = 2

            elif buoc == 2:

                if self.check0 == 2 :
                    print("saving map.....")
                    self.check0 = 0
                    self.funPubMap()
                    self.pubMap.publish(self.data_mapPub)
                    is_pub = 1

                elif is_pub == 1 and self.check0 != 2:
                    self.pubMap.publish(self.data_mapPub)

            # time.sleep(0.01)
            self.rate.sleep()
        self.is_killed = 1
        print('Thread #%s stopped' % self.threadID)
        

class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass
 
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit


def main():
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = MergerMap(1)
        thread1.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown() or (thread1.is_killed == 1):
            # if thread1.is_subParamMergerMap == True:
            thread1.TransForm()
            # time.sleep(0.001)
        thread1.is_killed = 1
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
    print('Exiting main program')

if __name__ == '__main__':
    main()