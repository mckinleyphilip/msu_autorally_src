#!/usr/bin/env python
#
# Empty Node
#
#	Base ROS node
#
# GAS 2018-03-21

import rospy
import argparse
import numpy as np





class EmptyNode():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('Empty_node',anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = cmd_args.debug
		
		
		
		
		"""
		CODE HERE...
		
		# Configure Publishers
		#self.collision_events_request_pub = rospy.Publisher('/util_comms/collision_events_request', Bool, queue_size=10)


		# Configure Subscribers
		#self.collision_events_sub = rospy.Subscriber('/util_comms/collision_events', String,self.on_collision_events_update)
		
		"""
		
		
		

		rospy.spin()
		
	def on_shutdown(self):
		pass
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	
	
	try:
		node = EmptyNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
