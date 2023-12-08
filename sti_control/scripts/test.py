#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import roslib

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

def int_to_bytes( val, n): # int to n bytes
	ss = b''
	x = 0
	t = 0
	for i in range(0, n):
		t += pow(256, n - i)*x
		print ("-----------i: ", i)
		print ("tt: ", t)
		x = int((val - t)/pow(256, n - i - 1) )
		print ("xx: ", x)
		# ss += self.int_to_byte(x)
	return ss

int_to_bytes(15000, 4)
# t = 0
# x = (150000 - t)/pow(256, 4 - 1 - 1) 

# print ("x: ", x)
