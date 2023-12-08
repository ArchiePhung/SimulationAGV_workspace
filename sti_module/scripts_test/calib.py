# import numpy as np
# import matplotlib.pyplot as plt
# from scipy import interpolate
# x = np.array([0.008, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8])
# y = np.array([0.11, 0.22, 0.3, 0.4, 0.55, 0.65, 0.7, 0.75, 0.8, 0.9, 1., 1.2, 1.25, 1.3, 1.35, 1.4])
# f = interpolate.interp1d(x, y)

# xnew = 
# ynew = f(xnew)   # use interpolation function returned by `interp1d`
# print(ynew)
# # plt.plot(x, y, 'o', xnew, ynew, '-')
# # plt.show()


#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def talker():
    pubMarker = rospy.Publisher('/visualization_markerPFL', Marker, queue_size=20)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        # marker.scale.z = 10.

        point = Point()
        point.x = 1.
        point.y = 1.
        marker.points.append(point)

        pubMarker.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass