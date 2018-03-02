#!/usr/bin/python

import rospy
from colibri_msgs.msg import NavState


def modbus_data():
	pub = rospy.Publisher('/nav_state', NavState, queue_size=10)
	rospy.init_node('modbus_data_pub')
	rate = rospy.Rate(5)  # 1hz
	navstate = NavState()
	data = 5
	while not rospy.is_shutdown():
		if data < 40:
			data += 2
		else:
			data = 5

		navstate.achieve_flag = data
		navstate.at_target_flag = data
		navstate.cur_x = data
		navstate.cur_y = data
		navstate.cur_yaw = data
		navstate.err_code = data
		navstate.target_heading = data
		navstate.target_node =data
		navstate.target_x = data
		navstate.target_y = data
		navstate.target_yaw = data

		rospy.loginfo(navstate)
		pub.publish(navstate)
		rate.sleep()

if __name__ == '__main__':
	try:
		modbus_data()
	except rospy.ROSInterruptException:
		pass
