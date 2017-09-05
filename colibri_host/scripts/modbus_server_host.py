#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python
 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr
 This is distributed under GNU LGPL license, see license.txt
"""
import sys
import serial
import threading
import random
import roslib
import rospy
from colibri_host.msg import Coordinator, NavState
import modbus_tk
import modbus_tk.defines as mdef
from modbus_tk import modbus_tcp
from modbus_tk.utils import threadsafe_function, flush_socket, to_data
import datetime
import time as sys_time
from datetime import time as fc_time
import numpy as np

# PORT = 1
# PORT = "/dev/ttyUSB0"
AIV_ID = 14
Axis_X = 30
Axis_Y = 30
course_theta = 20
Modul_info = 8
status_IO = 8
alarm_info = 8
Task_complt = 1
Current_Task = 6
Previous_Task = 5
Next_Task = 7
Num_finished = 70
Current_rail = 2
follow_up_Num = 10
Target_node = 1
Target_heading = 1
At_target_flag = 1
Target_x = 1
Target_y = 1
Target_yaw = 1
ID_followup = [2, 3, 4, 5, 6, 7, 8]
locals_time = datetime.time()

slaver_updata = [AIV_ID, locals_time.second, locals_time.minute, locals_time.hour, \
				 Axis_X, Axis_Y, course_theta, \
				 Target_x, Target_y, Target_yaw, \
				 Modul_info, \
				 status_IO, \
				 alarm_info, \
				 Task_complt, Current_Task, Previous_Task, Next_Task, Num_finished, \
				 Target_node, Target_heading, At_target_flag, \
				 Current_rail, follow_up_Num]
slaver_updata[23:65] = ID_followup

server = modbus_tcp.TcpServer(1502, "192.168.11.14")
slaver = server.add_slave(1)
logger = modbus_tk.utils.create_logger("console", record_format="%(message)s")
client_data = [57]

NavState1 = NavState()

'''
def ROS():
    pub = rospy.Publisher('Coordinator_info', Coordinator, queue_size=10)
    rospy.init_node('Coordinator_pub')
    rate = rospy.Rate(1)  # 1hz
    Centralserver = Coordinator()
    rospy.loginfo(Centralserver)
    pub.publish(Centralserver)
    rate.sleep()
'''
Centralserver = Coordinator()


def callback(data):
	# rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
	Axis_X = int(data.cur_x * 100)
	Axis_Y = int(data.cur_y * 100)
	course_theta = int(data.cur_yaw * 100)
	Target_x = int(data.target_x * 100)
	Target_y = int(data.target_y * 100)
	Target_yaw = int(data.target_yaw * 100)
	Modul_info = data.err_code
	status_IO = 255
	Task_complt = data.achieve_flag
	Target_node = data.target_node
	Target_heading = data.target_heading
	At_target_flag = data.at_target_flag
	Current_rail = data.cur_seg
	locals_time = datetime.time()
	slaver_updata = [30, locals_time.second, locals_time.minute, locals_time.hour, \
					 Axis_X, Axis_Y, course_theta, Target_x, Target_y, Target_yaw, \
					 Modul_info, \
					 status_IO, \
					 alarm_info, \
					 Task_complt, Current_Task, Previous_Task, Next_Task, Num_finished, \
					 Target_node, Target_heading, At_target_flag, \
					 Current_rail, follow_up_Num]
	slaver_updata[23:65] = ID_followup


rospy.init_node('Coordinator_pub')
logger.info("running...")
rospy.Subscriber('/nav_state', NavState, callback)


def setup():
	server.stop()
	server.start()
	slaver.add_block("master_data", mdef.HOLDING_REGISTERS, 40000, 100)  # address 0, length 100
	slaver.add_block("slaver_data", mdef.ANALOG_INPUTS, 30000, 100)


# set the values of registers at address 0
# slaver.set_values("a", 0, range(100))
# slaver.set_values("a", 0, [14, 24, 34])
# slave_list.append(slave1)


def loop():
	while not rospy.is_shutdown():
		# slaver.set_values("a", 40000, [1, 2, 3, 4, 5, 6, 7, 8])
		# slaver.set_values("c", 0, [1, 0, 1, 0, 1, 1])
		client_data = slaver.get_values("master_data", 40000, 60)
		time_client = fc_time(client_data[1], client_data[2], client_data[3])
		Centralserver.basic_ctrl = np.int8(client_data[4])
		# node = oct(client_data[5])
		Centralserver.target_node = np.int8(client_data[5])
		Centralserver.target_heading = client_data[6] / 100
		# segs_num = oct(client_data[7])
		Centralserver.route_segs_num = np.int8(client_data[7])
		Centralserver.segs_vector = client_data[8:58]
		# get local time
		rospy.loginfo(' segs_vector %s', Centralserver.segs_vector)
		Centralserver.header.stamp = rospy.Time.now()
		Centralserver.header.frame_id = 'hostpc'
		# rospy.loginfo(Centralserver)
		pub = rospy.Publisher('/coordinator', Coordinator, queue_size=10)
		pub.publish(Centralserver)

		rate = rospy.Rate(2)  # 1hz
		slaver.set_values("slaver_data", 30000, slaver_updata)
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
