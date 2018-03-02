#!/usr/bin/env python

import rospy
from colibri_battery.msg import Battery
import serial
import time
import string
import logging.handlers

VT = []
ASC2_num = 30
B_v = ()
Battery_send_num = []
B_V_bat_cell = []
B_Temp = []
t = serial.Serial('/dev/ttyS5', 9600)
t.close()


#LOG_FILE = '/home/colibri/colibri_ws/src/colibri_battery/log/bat_state.log'
#handler = logging.handlers.RotatingFileHandler(LOG_FILE, maxBytes=1024 * 1024, backupCount=5)
#fmt = '%(asctime)s - %(filename)s:%(lineno)s - %(name)s - %(message)s'
#formatter = logging.Formatter(fmt)
#handler.setFormatter(formatter)
#logger = logging.getLogger('bat_state')
#logger.addHandler(handler)
#logger.setLevel(logging.DEBUG)


def battery_info():
	global Battery_send_num, B_C_stt, B_SOC, B_Alarm, B_Curr_Discharge, B_Total_Volt
	pub = rospy.Publisher('battery_info', Battery, queue_size=10)
	rospy.init_node('battery_pub')
	rate = rospy.Rate(1)  # 1hz
	batten = Battery()
	while not rospy.is_shutdown():
		# hello_str = "hello world %s" % rospy.get_time()

		t.timeout = 3
		t.open()
		# t.reset_input_buffer()
		t.write(':000200000ee8~')
		str_1 = t.read(174)
		Battery_send_str = list(str_1)
		del Battery_send_num[:]
		del B_V_bat_cell[:]
		if len(Battery_send_str) == 174:
			for i in range(1, 173):
				Battery_send_num = Battery_send_num + [string.atoi(str_1[i], 16)]
			Battery_send_num = [':'] + Battery_send_num + ['~']
			i = 25
			B_Total_Volt = Battery_send_num[i] * 256 * 16 + Battery_send_num[i + 1] * 256 + \
				Battery_send_num[i + 2] * 16 + Battery_send_num[i + 3]	
			i = 99
			B_Curr_Discharge = Battery_send_num[i] * 256 * 16 + Battery_send_num[i + 1] * 256 + \
				Battery_send_num[i + 2] * 16 + Battery_send_num[i + 3]
			i = 119
			B_C_stt = Battery_send_num[i] * 256 * 16 + Battery_send_num[i + 1] * 256 + \
				Battery_send_num[i + 2] * 16 + Battery_send_num[i + 3]
			i = 127
			B_Alarm = Battery_send_num[i] * 256 * 16 + Battery_send_num[i + 1] * 256 + \
				Battery_send_num[i + 2] * 16 + Battery_send_num[i + 3]

			i = 161
			B_SOC = Battery_send_num[i] * 16 + Battery_send_num[i + 1]
		t.close()
		time.sleep(0.5 - 0.22)
		batten.Chg_state = B_C_stt
		batten.SOC = B_SOC
		batten.Voltage = B_Total_Volt * 2 / 1000.0
		batten.Bat_alarm = B_Alarm
#		rospy.loginfo(batten)
		pub.publish(batten)

#		logger.info("%s %s %2.2s %s", B_Curr_Discharge, B_SOC, B_Total_Volt * 2, B_C_stt)

if __name__ == '__main__':
	try:
		battery_info()
	except rospy.ROSInterruptException:
		pass
