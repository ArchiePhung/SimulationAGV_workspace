#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import signal
import rospy
import threading

class GoalControl(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.saveTimeMainTheard = rospy.get_time()
        self.startLoop = False
        self.dem = 100

        self.rate = rospy.Rate(30)

    def checkLostMainTheard(self):
        if self.startLoop:
            dentalTime = rospy.get_time() - self.saveTimeMainTheard
            if dentalTime > 3.:
                return 1
        return 0

    def run(self):
        while not self.shutdown_flag.is_set(): 
            print(100/self.dem)
            self.dem = self.dem -1 
            self.startLoop = True
            self.saveTimeMainTheard = rospy.get_time()
            self.rate.sleep()


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

    rospy.init_node('goal_control', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        br = GoalControl(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            # ck.timeMainTheard = br.saveTimeMainTheard
            # ck.startCheck = br.startLoop
            # print(ck.startCheck)
            if br.checkLostMainTheard():
                print("lost theard 1")
            time.sleep(0.05)
 
    except ServiceExit:
        br.shutdown_flag.set()
        br.join()

    print('Exiting main program')

if __name__ == '__main__':
    main()