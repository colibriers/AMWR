#!/usr/bin/env python

import rospy
from colibri_ultra.msg import Ultrasonic
import serial
import time
import string
import binascii

# import numpy as np

ultrasonic_1 = []
ultrasonic_2 = []
ultrasonic_3 = []
ultrasonic_4 = []
ultrasonic_send_str = []
t = serial.Serial('/dev/ttyS4', 9600)
t.timeout = 1
t.close()
t.open()


def hexShow(argv):
    result = ''
    hlen = len(argv)
    for i in xrange(hlen):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex + ' '
    return result


# print 'hexShow:', result


def ultrasonic_info():
    global ultrasonic_1, ultrasonic_2, ultrasonic_3, ultrasonic_4
    rospy.init_node('ultra_front_pub')
    pub = rospy.Publisher('/ultra_front', Ultrasonic, queue_size=4)
    rate = rospy.Rate(10)  # 10hz
    ultradata = Ultrasonic()

    while not rospy.is_shutdown():

        ultradata.header.frame_id = "ultrasonic"
        ultradata.header.stamp = rospy.Time.now()
        t.flushInput()

        t.write('\xe8')
        t.write('\x02')
        t.write('\x34')
        datastr = t.read(2)
        ultrasonic_send_str = list(datastr)
        if len(datastr) == 2:
            ultradata.ultra_1 = (ord(ultrasonic_send_str[0]) * 256 + ord(ultrasonic_send_str[1])) / 1000.0

        t.write('\xe8')
        t.write('\x02')
        t.write('\x3c')
        datastr = t.read(2)
        ultrasonic_send_str = list(datastr)
        if len(ultrasonic_send_str) == 2:
            ultradata.ultra_2 = (ord(ultrasonic_send_str[0]) * 256 + ord(ultrasonic_send_str[1])) / 1000.0

        t.write('\xe8')
        t.write('\x02')
        t.write('\x44')
        datastr = t.read(2)
        ultrasonic_send_str = list(datastr)
        if len(ultrasonic_send_str) == 2:
            ultradata.ultra_3 = (ord(ultrasonic_send_str[0]) * 256 + ord(ultrasonic_send_str[1])) / 1000.0

        t.write('\xe8')
        t.write('\x02')
        t.write('\x4c')
        datastr = t.read(2)
        ultrasonic_send_str = list(datastr)
        if len(ultrasonic_send_str) == 2:
            ultradata.ultra_4 = (ord(ultrasonic_send_str[0]) * 256 + ord(ultrasonic_send_str[1])) / 1000.0

        # rospy.loginfo(ultradata)
        pub.publish(ultradata)

        rate.sleep()


if __name__ == '__main__':
    try:
        ultrasonic_info()
    except rospy.ROSInterruptException:
        t.close()
        pass
