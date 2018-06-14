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
from autorally_msgs.msg import wheelSpeeds



class BaseController():
	def __init__(self, debug, node_name):
		
		self.debugging = args.debug
		self.name = node_name
		self.throttle_mappings = {}
		
		self.most_recent_twist_cmd = 0
		self.goal_speed = 0
		self.goal_steering = 0
		self.front_wheel_speed = 0
		
		self.steering = -5
		self.throttle = -5
		self.frontBrake = -5
		
		
		# PID Controller variables
		self.integral_error = 0
		self.throttle_KP = 0.15
		self.throttle_KD = 0.0
		self.throttle_KI = 0.001
		self.throttle_IMax = 0.2
		
		
		self.load_throttle_calibration()
		 
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		 
		 # Set up subscriber to twist_cmds coming from move_base package
		self.twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
		self.twist_cmd_sub =  rospy.Subscriber(self.twist_cmd_topic, Twist, self.twist_cmd_callback, queue_size=10)
		
		
		# Set up subscriber to the wheel speeds topic for feedback to the PID throttle controller
		self.wheelspeed_sub = rospy.Subscriber('/wheelSpeeds', wheelSpeeds, self.wheelSpeeds_callback, queue_size = 10)
		
		# Set up publisher for sending chassisCommands to the autorally core controller 
		self.chassisCommand_topic = '/{}/chassisCommand'.format(self.name)
		self.chassisCommand_pub = rospy.Publisher(self.chassisCommand_topic, chassisCommand, queue_size=10)
		
		
		# Grab parameters from the ros param server
		self.wheelbase = rospy.get_param('/vehicle_wheelbase ', 0.57)
		self.frame_id = rospy.get_param('~frame_id', 'odom')
		self.max_vel_x = rospy.get_param('/autorally_move_base/TebLocalPlannerROS/max_vel_x', 26.82 )
		
		
		
		
		if self.debugging:
			rospy.loginfo('{} initialized!'.format(self.name))
			
		
		rospy.spin()
		
	
	
	def interpolatekey(self, goal_speed):
		""" Accepts a goal speed (m/s) and returns throttle positioning required to achieve that speed.
			Value is found by interpolating the throttle calibration file values """
		top_mapped_speed = self.throttle_mappings.values()[-1]
		
		if goal_speed > top_mapped_speed:
			rospy.logerr('Goal speed is higher than any defined in throttle calibration file! '\
						 'Setting current speed to highest defined throttle: {} -> {} m/s'.format(self.throttle_mappings.keys()[-1], top_mapped_speed))
			goal_speed = top_mapped_speed
			
		return np.interp(goal_speed, self.throttle_mappings.values(), self.throttle_mappings.keys())
			
			
			
	def wheelSpeeds_callback(self, data):
		""" Listens to the autorally wheelSpeeds topic and issues new throttle commands based on the feed back from a PID controller """
		
		# Consider the average speed of the front two wheels 
		self.front_wheel_speed = 0.5 * (data.lfSpeed + data.rfSpeed)
		
		
		# This section of the code is adapted from ConstantSpeedController.cpp in the autorally_control package
		if self.goal_speed > 0.01: # m/s
			self.integral_error += self.goal_speed - self.front_wheel_speed
			
			if ( self.integral_error > (self.throttle_IMax / self.throttle_KI)):
				self.integral_error = (self.throttle_IMax / self.throttle_KI)
			
			if ( self.integral_error < -(self.throttle_IMax / self.throttle_KI)):
				self.integral_error = -(self.throttle_IMax / self.throttle_KI)
			
			throttle = self.interpolatekey(self.goal_speed) + self.throttle_KP * (self.goal_speed - self.front_wheel_speed)
			throttle += self.throttle_KI + self.integral_error
			throttle = max( 0.0 , min( 1.0 , throttle))
			self.throttle = throttle
			
			
		else:
			self.throttle = 0.0
			
		self.steering = self.goal_steering
		self.send_chassis_command_msg()
		
		
		
		
		
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
				self.throttle_mappings[float(key)] = float(val)
		
		# Sort the dict
		self.throttle_mappings = collections.OrderedDict(sorted(self.throttle_mappings.items()))
		
		
#		print(self.throttle_mappings)
#		print('\n\n')
#		print(self.throttle_mappings[0.11])
#		print('\n\n')
#		print(self.throttle_mappings.keys())
#		print('\n\n')
#		print(self.throttle_mappings.values())
#		print('\n\n')
#		print(self.interpolatekey(25.1))
		
		
	
	
	# Can delete
	# This is for publishing messages at a different frequency than the subscribers messages come in at
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
		
		print(cmd)
		self.chassisCommand_pub.publish(cmd)
	
	
	
	def twist_cmd_callback(self, data):
		""" Callback for the cmd_vel twist message topic. """
		
		if self.most_recent_twist_cmd != data:
			if self.debugging:
				rospy.loginfo('New twist command')
				rospy.loginfo(data)
			
			self.most_recent_twist_cmd = data

			self.goal_speed = data.linear.x
			self.goal_steering = self.convert_trans_rot_vel_to_steering_angle(v = data.linear.x, omega = data.angular.z)
			self.goal_steering = ( self.goal_steering  / ( math.pi / 2))
			
			
			

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
	
	
