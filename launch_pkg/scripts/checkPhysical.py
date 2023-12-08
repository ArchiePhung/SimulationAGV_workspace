#!/usr/bin/env python3
# author : Anh Tuan - 18-09-2020
# update : Anh Tuan - 29-09-2020
# update : Anh Tuan - 20-01-2020 : them xoa log ROS
# update : BEE 		- 10-06-2021

# code get IP's PC 
# import os                                                                                                                                                           
# import re                                                                                                                                                           
# ipv4 = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen('ip addr show wlp2s0').read()).groups()[0]                                                     
# print(ipv4)

import serial
import os
from message_pkg.msg import Status_port
import roslaunch
import rospy
import string
import subprocess, platform
import time

class CheckPhysical:
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('check_physical', anonymous=True)
        self.rate = rospy.Rate(1)

        self.port_nav350 = rospy.get_param('port_nav350', '')   
        self.port_main  = rospy.get_param('port_main', '')    
        self.port_oc     = rospy.get_param('port_oc', '')
        self.port_hc     = rospy.get_param('port_hc', '')
        self.port_imu     = rospy.get_param('port_imu', '')
        self.port_loadcell = rospy.get_param('port_loadcell', '')
        self.port_driverAll = rospy.get_param('port_driverAll', '')

        self.pub_statusPort = rospy.Publisher('/status_port', Status_port, queue_size= 50)
        self.statusPort = Status_port()

        self.pre_time = time.time()

    def ethernet_check(self, address):
        try:
            # print address
            output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(output)
            result = str(output).find('time=') 
            # print(result) # yes : >0 
            if result != -1:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbSerial_check_c3(self, nameport):
        try:
            output = subprocess.check_output("ls -{} {} {} {} {}".format('l','/dev/','|','grep', nameport ), shell=True)
            vitri = output.find("stibase")

            name = output[(vitri):(vitri+len(nameport))]
            # print(name)
            if name == nameport:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbSerial_check(self, nameport):
        try:
            output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=True)
            locate = str(output).find(nameport)
            if locate != -1:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbCamera_check(self,nameport):
        try:
            output = subprocess.check_output("rs-enumerate-devices -{}".format('s'), shell=True)
            # print(output)
            vitri = str(output).find(nameport[1:])
            # name = output[(vitri):(vitri+len(nameport))]
            # print(result)
            # print(nameport[1:])
            if vitri > 1 : return 1
        except Exception as e:
            # print("no port")
            return 0

    def run(self):

        while not rospy.is_shutdown():
          # -- LMS100
            self.pre_time = time.time()
            self.statusPort.lms100 = True
            t = time.time() - self.pre_time
            # print ("t: ", t)
          # -- TiM551
            self.statusPort.tim551 = True
          # -- NAV350
            if self.ethernet_check(self.port_nav350) == 1:
                self.statusPort.nav350 = True
            else:
                self.statusPort.nav350 = False

          # -- D435
            self.statusPort.camera = True
            
          # -- POWER ( main 82)        
            if self.usbSerial_check(self.port_main) == 1:
                self.statusPort.main = True
            else:
                self.statusPort.main = False

          # -- MC 
            self.statusPort.mc = True         

          # -- SC          
            self.statusPort.sc = True

          # -- OC          
            if self.usbSerial_check(self.port_oc) == 1:
                self.statusPort.oc = True
            else:
                self.statusPort.oc = False

          # -- HC          
            if self.usbSerial_check(self.port_hc) == 1:
                self.statusPort.hc = True
            else:
                self.statusPort.hc = False

            # -- HMI          
            self.statusPort.hmi = True

            # -- magLine          
            self.statusPort.magLine = True

            # -- IMU
            if self.usbSerial_check(self.port_imu) == 1:
                self.statusPort.imu = True
            else:
                self.statusPort.imu = False
            
            # -- loadcell
            if self.usbSerial_check(self.port_loadcell) == 1:
                self.statusPort.loadcell = True
            else:
                self.statusPort.loadcell = False

            # -- driverAll
            if self.usbSerial_check(self.port_driverAll) == 1:
                self.statusPort.driverall = True
            else:
                self.statusPort.driverall = False

            self.statusPort.new1 = True
            self.statusPort.new2 = True
            
            self.pub_statusPort.publish(self.statusPort)

            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = CheckPhysical()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()