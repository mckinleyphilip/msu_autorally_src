#!/usr/bin/env python
#
# base_controlly.py
#
#	Base controller node for the autorally platform. Accepts geometry_msgs/Twist from the cmd_vel topic (published by move_base) and translates them into autorally compliant chassisCommand msgs.
#
# GAS 2018-05-30

import rospy
import argparse
import numpy as np
import cv2
import math
import time
import rospy
import collections

from geometry_msgs.msg import Twist
from autorally_msgs.msg import chassisCommand



class BaseController():
	def __init__(self, debug, node_name):
		
		self.debugging = args.debug
		self.name = node_name
		self.throttle_mappings = {}
		 
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		 
		 # Set up subscriber to twist_cmds coming from move_base package
		self.twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
		self.twist_cmd_sub =  rospy.Subscriber(self.twist_cmd_topic, Twist, self.cmd_callback, queue_size=10)
		
		
		# Set up publisher for sending chassisCommands to the autorally core controller 
		self.chassisCommand_topic = '/{}/chassisCommand'.format(self.name)
		self.chassisCommand_pub = rospy.Publisher(self.chassisCommand_topic, chassisCommand, queue_size=10)
		
		
		# Grab parameters from the ros param server
		self.wheelbase = rospy.get_param('/vehicle_wheelbase ', 1.0)
		self.frame_id = rospy.get_param('~frame_id', 'odom')
		self.max_vel_x = rospy.get_param('/autorally_move_base/TebLocalPlannerROS/max_vel_x', 26.82 )
		
		
		# Shared command values (set by twist_cmd callback, used for sending chassisCommands)
		self.steering = -5
		self.throttle = 0
		self.frontBrake = -5
		
		self.load_throttle_calibration()
		
		if self.debugging:
			rospy.loginfo('{} initialized!'.format(self.name))
			
		
		self.run()
		
		
	def load_throttle_calibration(self):
		""" Loads the throttle positioning to output speed mappings from the throttle calibration file
				and stores the mapping into a sorted dictionary, self.throttle_mappings """
		if self.debugging:
			rospy.loginfo('Loading calibration')
		
		# Check for existance of the throttle calibration file on the ros param server
		if not rospy.has_param('throttle_calib_file'):
			rospy.signal_shutdown('Throttle calibration file has not been loaded into the parameter server')
		else:
			throttle_calib_file = rospy.get_param('throttle_calib_file')
		
		# Load the throttle calibration file into a dict
		with open(throttle_calib_file) as f:
			for line in f:
				line = line.replace('\'',"")
				(key, val) = line.split(':')
				self.throttle_mappings[key] = float(val)
		
		# Sort the dict
		self.throttle_mappings = collections.OrderedDict(sorted(self.throttle_mappings.items()))
		
		
	def run(self):
		while not rospy.is_shutdown():
			r = rospy.Rate(10) # 10hz
			self.send_chassis_command_msg()
			if self.debugging:
				rospy.loginfo('sent!')
			r.sleep()	
	
	
	def send_chassis_command_msg(self):
		""" Formats the current desired control commands into a chassisCommand msg format and publishes it """
		# Might need a lock here before accessing the shared command values?
		cmd = chassisCommand()
		cmd.header.stamp = rospy.get_rostime()
		cmd.sender = self.name
		cmd.steering = self.steering 
		cmd.throttle = self.throttle
		cmd.frontBrake = self.frontBrake
		
		self.chassisCommand_pub.publish(cmd)
	
	
	
	def cmd_callback(self, data):
		""" Callback for the cmd_vel twist message topic. """
		# Might need to implement locks on the command values since the publishing and subscribing threads could touch them at the same time?
		if self.debugging:
			rospy.loginfo(data)

		v = data.linear.x
		steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z)
		
		
		# Convert the msg velocity (m/s) to a normalized chassisCommand value
#		#normalized_v = v / self.max_vel_x
		normalized_v = v
		
		# Set the command values
		self.throttle = normalized_v
		self.steering = steering
			
		
	def convert_trans_rot_vel_to_steering_angle(self, v, omega):
		""" Translates the angular z direction component from the twist message to a steering angle 
			Code from http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
			"""
		if omega == 0 or v == 0:
			return 0
		
		radius = v / omega
		return math.atan(self.wheelbase / radius)
	
	def on_shutdown(self):
		""" Define any last tasks for this node to perform here """
		rospy.loginfo('{} is shutting down!'.format(self.name))
		
	
if __name__ == '__main__':
	
	node_name = 'BaseController'
	
	# Parse arguments
	parser = argparse.ArgumentParser(description='{} node. Accepts geometry_msgs/Twist msgs from "cmd_vel" and translates them to autorally specific ChassisCommand msgs.'.format(node_name))
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node(node_name,anonymous=False)
	
	try:
		node = BaseController(debug=args.debug, node_name=node_name)
		#rospy.spin()
	except rospy.ROSInterruptException: pass
	
	