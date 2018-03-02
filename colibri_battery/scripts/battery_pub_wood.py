#!/usr/bin/env python

import serial
import time
import rospy
from colibri_battery.msg import Battery
import string
import logging.handlers


def hexShow(argv):
	result = ''
	h_len = len(argv)
	for j in xrange(h_len):
		if isinstance(argv[j], str):
			halo = ord(argv[j])
		else:
			halo = argv[j]
		haxe = '%02x' % halo
		result += haxe + ' '
	print 'hexShow:', result


def current(data):
	int_num = int(data, 16)
	if int_num >= 32768:
		return int_num - 65536
	else:
		return int_num


def chksum_calculate(reserved_list):
	chksum_value = 0
	for listatem in reserved_list:
		chksum_value = chksum_value + listatem
	chksum_value = 65536 - chksum_value % 65536
	chksum_str = '%04x' % chksum_value
	chksum_list = list(chksum_str)
	list_chksum = []
	del list_chksum[:]
	for list_in in chksum_list:
		chksum_str_in = ord(list_in)
		list_chksum.append(chksum_str_in)
	return list_chksum


def battery_info():
	global B_C_stt, B_SOC, B_Alarm, B_cycle
	B_C_stt = -1;
	pub = rospy.Publisher('battery_info', Battery, queue_size=10)
	rospy.init_node('battery_pub')
	rate = rospy.Rate(1)  # 1hz
	batten = Battery()

	send_data_42 = [0x7E, 0x32, 0x35, 0x30, 0x30, 0x34, 0x36, 0x39, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x46,	0x44, 0x41, 0x36, 0x0D]
	send_data_42[7:9] = [0x34, 0x32]
	send_data_42[9:13] = [0x45, 0x30, 0x30, 0x32]
	send_data_42[13:15] = [0x30, 0x32]
	# send_data_42[15:19] = [0x46, 0x44, 0x33, 0x30]
	send_data_42[15:19] = chksum_calculate(send_data_42[1:15])

	send_data_44 = [0x7E, 0x32, 0x35, 0x30, 0x30, 0x34, 0x36, 0x39, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x46, 0x44, 0x41, 0x36, 0x0D]
	send_data_44[7:9] = [0x34, 0x34]
	send_data_44[9:13] = [0x45, 0x30, 0x30, 0x32]
	send_data_44[13:15] = [0x30, 0x32]
	send_data_44[15:19] = chksum_calculate(send_data_44[1:15])

	bat_serial = serial.Serial('/dev/ttyS5', 9600, parity='N', stopbits=1)
	# bat_serial.close()
	#print bat_serial.portstr
	received_date = []

	while not rospy.is_shutdown():
		bat_serial.timeout = 0.1
		# bat_serial.open()
		# bat_serial.reset_input_buffer()
		del received_date[:]
		bat_serial.write(send_data_42)
		while True:
			try:
				read = bat_serial.read()
				if len(read) == 0:
					break
				else:
					received_date.append(read)
			except KeyboardInterrupt:
				break
		#print len(received_date)
		
		if len(received_date) == 128:
			B_C_stt = current(''.join(received_date[97:101]))
			# bat_volt = int(''.join(received_date[101:105]), 16)
			power_rest = int(''.join(received_date[105:109]), 16)
			power_total = int(''.join(received_date[111:115]), 16)
			B_SOC = power_rest*100/power_total
			B_cycle = int(''.join(received_date[115:119]), 16)

		bat_serial.reset_input_buffer()
		del received_date[:]
		bat_serial.write(send_data_44)
		while True:
			read = bat_serial.read()
			if len(read) == 0:
				break
			else:
				received_date.append(read)

		if len(received_date) == 88:
			B_Alarm = int(''.join(received_date[73:75]), 16)

		try:					
			batten.Chg_state = B_C_stt
			batten.SOC = B_SOC
			batten.Bat_alarm = B_Alarm
			batten.Bat_cycle = B_cycle
			#rospy.loginfo(batten)
			pub.publish(batten)
			rate.sleep()
		except NameError:
			print('NameError: ')
			pass
			

if __name__ == '__main__':
	try:
		battery_info()
	except rospy.ROSInterruptException:
		pass

