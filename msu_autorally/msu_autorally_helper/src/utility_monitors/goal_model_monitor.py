#!/usr/bin/env python
#
# Goal Model Monitor
#
#	This is a ROS node that communicates with the utility monitor nodes and creates a fitness score based off of the goal model
#
# GAS 2018-03-19

import rospy
import argparse
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import String
from copy import deepcopy
import json




class GoalModelNode():
	def __init__(self):
		# Define Goal Model
		
		# Monitored Utils
		self.collision_events = []
		
		# Configure Publishers
		self.collision_events_request_pub = rospy.Publisher('/util_comms/collision_events_request', Bool, queue_size=10)


		# Configure Subscribers
		self.collision_events_sub = rospy.Subscriber('/util_comms/collision_events', String,self.on_collision_events_update)
		
		rospy.on_shutdown(self.on_shutdown)
		
		
		#r = rospy.Rate(0.2) # rate in Hz
		#while not rospy.is_shutdown():
		#	self.request_updates()
		#	r.sleep()

		rospy.spin()
		
	def on_shutdown(self):
		pass
		
	
	def on_collision_events_update(self, update_msg):
		self.collision_events = json.loads(update_msg.data)
		print('Collision Events: {}'.format(self.collision_events))
		
		
		
	def request_updates(self):
		print('Sending Requests')
		msg = Bool()
		msg.data = True
		self.collision_events_request_pub.publish(msg)
		
		
		

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='This node is responsible communicates with the utility monitor nodes and creates a fitness score based off of the goal model')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node('Goal_Model_Node',anonymous=False)
	
	try:
		monitor = GoalModelNode()
	except rospy.ROSInterruptException: pass
	
	
