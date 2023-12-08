#!/usr/bin/env python

# AUTHOR: HOANG VAN QUAGN - BEE
# DATE: 13/07/2021
# https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
# https://docs.opencv.org/4.5.2/d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c

from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import  Pose , Point
from sensor_msgs.msg import PointCloud2, LaserScan
from decimal import Decimal

from math import atan2, sin, cos, sqrt, fabs
from math import pi as PI
import rospy
import time
import threading
import signal
import math
import numpy as np

class Point_2d:
	x = 0.0
	y = 0.0

class landmark_detection():
	def __init__(self):
    
		rospy.init_node('landmark_detection', anonymous = True)
		self.rate = rospy.Rate(200)

        # topic pub-sub
		rospy.Subscriber("/scan_lms100", LaserScan, self.call_sub)
		self.pub_scan = rospy.Publisher('/lidar_filter', LaserScan, queue_size= 10) 
        
		self.data_lidar = LaserScan()
		self.is_readLidar = False

		self.angle_start = -PI/2.0
		self.angle_end = PI/2.0

		# -- average_filter
		self.count_data = 0
		self.out_data = LaserScan()
		self.first_data = 0
		self.data_filter1 = LaserScan()
		self.data_filter2 = LaserScan()
		self.data_filter3 = LaserScan()
		# -- 
		self.step_filter = 0
		self.list_ranges = []
		self.list_intensities = []
		self.list_ranges_total = []
		self.list_intensities_total = []
		# -- 
		self.list_coordinate = []

	def call_sub(self, data):
		self.data_lidar = data
		self.is_readLidar = 1
		# print ("--------------")
		# print ("number_point: ", len(data.ranges))
		# # num = data.
		# print ("number_point2: ", len(data.intensities))
		# self.find_maxIntensities(data)

	def distance_cal(self, p1, p2): # p1, p2 - Point_2d()
		distance = sqrt(pow((p1.x - p2.x), 2) + pow((p2.y - p2.y), 2))
		return distance

	def filter_average(self, data_lidar, number_count):
		if (self.is_readLidar):
			self.is_readLidar = 0
			if (self.first_data == 0):
				self.list_ranges_total = list(data_lidar.ranges)
				self.list_intensities_total = list(data_lidar.intensities)
				self.first_data = 1
				self.count_data += 1
			else:
				length = len(data_lidar.ranges)
				for nb in range(length):
					# self.list_ranges[nb] = (data_lidar.ranges[nb] + self.list_ranges[nb][nb])/2.0
					self.list_ranges_total[nb] += data_lidar.ranges[nb]
					self.list_intensities_total[nb] += data_lidar.intensities[nb]
				self.count_data += 1

		if (self.count_data >= number_count):
			out_data = data_lidar
			self.first_data = 0
			length = len(data_lidar.ranges)
			for nb in range(length):
				self.list_ranges_total[nb] = self.list_ranges_total[nb]/self.count_data
				self.list_intensities_total[nb] = self.list_intensities_total[nb]/self.count_data				
			out_data.ranges = self.list_ranges_total
			out_data.intensities = self.list_intensities_total
			self.count_data = 0
			return 1, out_data
		else:
			return 0, LaserScan()

	def filter_threshold(self, data_lidar, ranges_min, ranges_max, intensities):
		out_data = data_lidar
		length = len(out_data.ranges)
		for nb in range(length):
			if (data_lidar.ranges[nb] < ranges_min or data_lidar.ranges[nb] > ranges_max or data_lidar.intensities[nb] < intensities):
				out_data.ranges[nb] = 0.0
				out_data.intensities[nb] = 0.0
		return out_data

	def convert_to_coordinate(self, data_lidar):
		list_point = []
		number_point = len(data_lidar.ranges)
		# print ("number_point: ", number_point)
		for ag in range(number_point):
			point_now = Point_2d()
			angle_now = data_lidar.angle_min + ag*data_lidar.angle_increment
			point_now.x = round(cos(angle_now)*data_lidar.ranges[ag], 3)
			point_now.y = round(sin(angle_now)*data_lidar.ranges[ag], 3)
			list_point.append(point_now)
		return list_point

	def find_maxIntensities(self, data_lidar):
		max_intensities = 0.0
		pos_intensities = 0
		length = len(data_lidar.intensities)
		for i in range(length):
			if (max_intensities < data_lidar.intensities[i]):
				max_intensities = data_lidar.intensities[i]
				pos_intensities = i
		print ("------------------")
		print ("pos_intensities: ", pos_intensities)
		print ("pos_range: ", data_lidar.ranges[pos_intensities])
		print ("max_intensities: ", max_intensities)

	def find_clusterPoints(self, list_point, length_cluster, min_distance): # list_point - Point_2d() | length_cluster, min_distance - meter.
		length = len(list_point)
		cluster_now = []
		cluster_all = []
		total_length = 0.0
		point_old = list_point[0]
		point_now = list_point[0]
		point_origin = Point_2d()

		# -- find by number_point
		for nb in range(length):
			point_now = list_point[nb]
			# -- khoang canh u diem goc den diem hien tai.
			dis = self.distance_cal(point_origin, point_now)
			# -- khoang cach giua 2 diem lien tiep.
			dis_betweent = self.distance_cal(point_old, point_now)
			# --
			if (dis >= min_distance and dis_betweent < 0.04):
				cluster_now.append(point_now)
				total_length += dis
			else:
				# -- filter by length_cluster
				if (abs(total_length - length_cluster) < 0.04):
					cluster_all.append(cluster_now)
					cluster_now = []
					total_length = 0.0
				else:
					cluster_now = []
					total_length = 0.0

			point_old = point_now

		return cluster_all

	def show_dataFilter(self):
		if (self.step_filter == 0): # -- loc trung binh.
			sts, self.data_filter1 = self.filter_average_vs1(self.data_lidar, 40)
			if (sts == 1):
				self.step_filter = 1
				print ("step 0")

		elif (self.step_filter == 1): # -- loc nguong.
			self.data_filter2 = self.filter_threshold(self.data_filter1, 0.5, 5.0, 1200)
			self.step_filter = 2
			print ("step 1")

		elif (self.step_filter == 2):
			self.data_filter2.header.stamp = rospy.Time.now()
			self.pub_scan.publish(self.data_filter2)
			self.step_filter = 0
			print ("step 2")

	def show_dataFilter_vs1(self):
		if (self.step_filter == 0):
			sts, self.data_filter1 = self.filter_average(self.data_lidar, 40)
			if (sts == 1):
				self.data_filter2 = self.filter_threshold(self.data_filter1, 0.5, 5.0, 1200)
				# -- pub
				self.data_filter2.header.stamp = rospy.Time.now()
				self.pub_scan.publish(self.data_filter2)
				# -- convert_to_coordinate
				self.list_coordinate = self.convert_to_coordinate(self.data_filter2)
				ll = len(self.list_coordinate)
				# print ("ll: ", ll)
				for nb in range(ll):
					if (self.list_coordinate[nb].x != 0):
						print "P" + str(nb) + "[" + str(self.list_coordinate[nb].x) + ";" + str(self.list_coordinate[nb].y)  + "]"
				self.step_filter = 1

	def approxPolyDP(delf, list_point, accuracy):


	def run(self):
		while not rospy.is_shutdown():
			self.show_dataFilter_vs1()
			self.rate.sleep()


	x = (x ,y)
	y = (x, y)
	coef = np.polyfit(x, y, 1)
	A = coef[0]
	B = coef[1]
	C = A*x[0] + B*x[1]
	shortest_dis = shortest_distance(x, y, A, B, C)

	def shortest_distance(x1, y1, a, b, c):    
	    d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b)) 
	    print("Perpendicular distance is", d)



	def douglasPeucker (pointList, epsilon)
	    # -- Find the point with the maximum distance
	    dmax = 0
	    index = 0
	    end = len(pointList)
	    for nb in range (2, (end - 1)):
	        dis = perpendicularDistance(pointList[i], Line(pointList[1], pointList[end])) 
	        if (dis > dmax) {
	            index = nb
	            dmax = dis
	        }
	    }
    
	    ResultList = []
	    
	    # If max distance is greater than epsilon, recursively simplify
	    if (dmax > epsilon) {
	        # Recursive call
	        recResults1[] = DouglasPeucker(pointList[1...index], epsilon)
	        recResults2[] = DouglasPeucker(pointList[index...end], epsilon)

	        # Build the result list
	        ResultList[] = {recResults1[1...length(recResults1) - 1], recResults2[1...length(recResults2)]}
	    } else {
	        ResultList[] = {pointList[1], pointList[end]}
	    }
	    # Return the result
	    return ResultList[]

def main():
	program = landmark_detection()
	program.run()
 
if __name__ == '__main__':
	main()
