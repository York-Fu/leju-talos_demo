#!/usr/bin/env python
# coding=utf-8

import rospy
import rosservice
from ros_msg_node.srv import *

def service_alive(process_service):
	service_list = rosservice.get_service_list()
	if process_service in service_list:
		return True
	else:
		return False

def wait_for_processMsgservice():
	if service_alive("/ros_msg_node/process_msg_process"):
		rospy.wait_for_service("/ros_msg_node/process_msg_process")
		client = rospy.ServiceProxy("/ros_msg_node/process_msg_process", ProcessMsg)
		return client
	else:
		rospy.logerr("serevice not alive")
		return False


def sprint(*msg):
	print_msg = ""
	client = wait_for_processMsgservice()
	if client:
		for i in msg:
			if not isinstance(i,str):
				i = str(i)
			print_msg = print_msg + i + " "
		client("print",print_msg)

def sdebug(debug_msg):
	client = wait_for_processMsgservice()
	if client:
		client("debug",debug_msg)

def sinfo(info_msg):
	client = wait_for_processMsgservice()
	if client:
		client("info",info_msg)

def swarn(warn_msg):
	client = wait_for_processMsgservice()
	if client:
		client("warn",warn_msg)

def serror(error_msg):
	rospy.logerr(error_msg)
	client = wait_for_processMsgservice()
	if client:
		client("error",str(error_msg))

def finishsend():
	if service_alive("/ros_msg_node/process_msg_process"):
		finish_pub.publish("finish")
		rospy.wait_for_service("/ros_msg_node/finish_msg_process")
		client = rospy.ServiceProxy("/ros_msg_node/finish_msg_process", RunningFinish)
		client()