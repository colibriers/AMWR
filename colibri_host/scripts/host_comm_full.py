#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python
 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr
 This is distributed under GNU LGPL license, see license.txt
"""
import rospy
from colibri_msgs.msg import Coordinator, NavState
import modbus_tk
import modbus_tk.defines as mdef
from modbus_tk import modbus_tcp
from modbus_tk.utils import threadsafe_function, flush_socket, to_data
import datetime
from datetime import time as fc_time
import numpy as np

robot_id = 11

cur_time = datetime.time()

cur_x_sign = 1 
cur_y_sign = 1 
cur_yaw_sign = 1 
robot_cur_x = 0
robot_cur_y = 0
robot_cur_yaw = 0

target_x_sign = 1 
target_y_sign = 1 
target_yaw_sign = 1 
target_x = 0
target_y = 0
target_yaw = 0

mode_info = 2
IO_status = 8
alarm_info = 8

task_finish_flag = 0
cur_task_id = 0
pre_task_id = 0
next_task_id = 0
executed_cnt = 0

target_node = 0
target_heading = 0
at_target_flag = 0

cur_seg_id = 1
followup_seg_num = 10
followup_seg_vec = [1, 2, 3, 4, 5, 6, 7, 8]


robot2host_info = [robot_id, cur_time.second, cur_time.minute, cur_time.hour, \
				 cur_x_sign, robot_cur_x, cur_y_sign, robot_cur_y, cur_yaw_sign, robot_cur_yaw, \
				 target_x_sign, target_x, target_y_sign, target_y, target_yaw_sign, target_yaw, \
				 mode_info, IO_status, alarm_info, \
				 task_finish_flag, cur_task_id, pre_task_id, next_task_id, executed_cnt, \
				 target_node, target_heading, at_target_flag, \
				 cur_seg_id, followup_seg_num]
robot2host_info[29:89] = followup_seg_vec

server = modbus_tcp.TcpServer(1502, "192.168.11.11")
slaver = server.add_slave(1)
logger = modbus_tk.utils.create_logger("console", record_format="%(message)s")

host2robot_info = Coordinator()

def getsign(data):
	if data < 0:
		return 0
	else:
		return 1


def callback(data):
	global robot2host_info
	cur_x_sign = getsign(data.cur_x) 
	cur_y_sign =  getsign(data.cur_y) 
	cur_yaw_sign = getsign(data.cur_yaw) 
	robot_cur_x = np.int16(abs(data.cur_x) * 100)
	robot_cur_y = np.int16(abs(data.cur_y) * 100)
	robot_cur_yaw = np.int16(abs(data.cur_yaw) * 100)

	target_x_sign = getsign(data.target_x) 
	target_y_sign =  getsign(data.target_y) 
	target_yaw_sign = getsign(data.target_yaw) 
	target_x = np.int16(abs(data.target_x) * 100)
	target_y = np.int16(abs(data.target_y) * 100)
	target_yaw = np.int16(abs(data.target_yaw) * 100)
	mode_info = np.int16(data.err_code)
	IO_status = 10
	task_finish_flag = np.int16(data.task_succ_flag)
	target_node = data.target_node
	target_heading = np.int16(data.target_heading)
	at_target_flag = np.int16(data.at_target_flag)
	cur_seg_id = data.cur_seg
	followup_seg_num = 60
	cur_time = datetime.time()

	robot2host_info = [robot_id, cur_time.second, cur_time.minute, cur_time.hour, \
					 cur_x_sign, robot_cur_x, cur_y_sign, robot_cur_y, cur_yaw_sign, robot_cur_yaw, \
					 target_x_sign, target_x, target_y_sign, target_y, target_yaw_sign, target_yaw, \
					 mode_info, IO_status, alarm_info, \
					 task_finish_flag, cur_task_id, pre_task_id, next_task_id, executed_cnt, \
					 target_node, target_heading, at_target_flag, \
					 cur_seg_id, followup_seg_num]

	robot2host_info[29:89] = followup_seg_vec
	
	slaver.set_values("slaver_data", 30000, robot2host_info)

rospy.init_node('host_comm_node')
logger.info("running...")
rospy.Subscriber('/nav_state', NavState, callback)


def setup():
	server.stop()
	server.start()
	slaver.add_block("master_data", mdef.HOLDING_REGISTERS, 40000, 100)  # address 0, length 100
	slaver.add_block("slaver_data", mdef.ANALOG_INPUTS, 30000, 100)
	slaver.set_values("slaver_data", 30000, robot2host_info)


def loop():
	while not rospy.is_shutdown():
		# slaver.set_values("a", 40000, [1, 2, 3, 4, 5, 6, 7, 8])
		# slaver.set_values("c", 0, [1, 0, 1, 0, 1, 1])
		client_data = slaver.get_values("master_data", 40000, 60)
		time_client = fc_time(client_data[3], client_data[2], client_data[1])
		
		host2robot_info.basic_ctrl = np.int8(client_data[4])
		# node = oct(client_data[5])
		host2robot_info.target_node = np.int8(client_data[5])
		host2robot_info.target_heading = client_data[6] / 100
		# segs_num = oct(client_data[7])
		host2robot_info.route_segs_num = np.int8(client_data[7])
		host2robot_info.segs_vector = client_data[8:58]
		# get local time
		#rospy.loginfo(' segs_vector %s', host2robot_info.segs_vector)
		host2robot_info.header.stamp = rospy.Time.now()
		host2robot_info.header.frame_id = 'hostpc'
		# rospy.loginfo(host2robot_info)
		pub = rospy.Publisher('/coordinator', Coordinator, queue_size=2)
		pub.publish(host2robot_info)

		rate = rospy.Rate(5)  
		rate.sleep()

	rospy.spin()


def destory():
	logger.info("destory")
	server.stop()


if __name__ == "__main__":
	setup()
	try:
		loop()
	except rospy.ROSInterruptException:
		destory()
	finally:
		destory()
