#!/usr/bin/env python

import sys
import time
import struct

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from decimal import *
import math

def int_to_byte(val): # int to a bytes
    if val > 255:
        rospy.logerr("int_to_byte: Val error: %s", val)
        val = 255
    elif val < 0:
        rospy.logerr("int_to_byte: Val error: %s", val)
        val = 0
    return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

def int_to_bytes(val, n):
    ss = b''
    x = 0
    t = 0
    for i in range(0, n):
        t += pow(256, n - i)*x
        x = int((val - t)/pow(256, n - i - 1) )
        ss += self.int_to_byte(x)
    return ss

def byte_to_int(byt): # byte to int (standard)
    x = 0
    x = int(byt)
    return x

def bytes_list_to_coor_vs1(list_byte, pos_byte): # 4 byte
    n = 0
    x = 0
    v = pos_byte + 1

    for t in range(4):
        x = int(list_byte[pos_byte + t])
        n += x*pow(256, 3 - t)

    val = 0
    if (n > pow(256, 3)):
        val = (n - pow(256, 4))
    else:
        val  = n
    return val

def float_to_bytes(value_in):
    out = b''
    out = bytearray(struct.pack("f", value_in))
    return out

def convert_():
    byte_ = b'\xE7\x00\x05\x01' # 93001D01
    # bytes_as_bits = ''.join(format(byte, '08b')[::-1] for byte in byte_) # dao bit 0 <-> 1
    bytes_as_bits = ''.join(format(byte, '08b') for byte in byte_) # 
    print ("bit: ", bytes_as_bits)

def check_bit():
    # byte_ = b''
    # hex_string = "E7000501"
    # byte_ = bytes.fromhex(hex_string)
    # # bytes_as_bits = ''.join(format(byte, '08b')[::-1] for byte in byte_) # dao bit 0 <-> 1
    # bytes_as_bits = ''.join(format(byte, '08b') for byte in byte_) # 
    # print ("bit: ", bytes_as_bits)
    val = self.arrBin_to_dec("1110", 0, 4)
    print("val: ", val)


def main():
    byte_ = b'\xE7\x00\x05\x01' # 93001D01
    print(type(float_to_bytes(3.14)))
    print("bit: ", byte_)

if __name__ == '__main__':
    main()