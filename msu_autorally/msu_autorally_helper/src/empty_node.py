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
	def __init__(self):
		
		
		# Configure Publishers
		#self.collision_events_request_pub = rospy.Publisher('/util_comms/collision_events_request', Bool, queue_size=10)


		# Configure Subscribers
		#self.collision_events_sub = rospy.Subscriber('/util_comms/collision_events', String,self.on_collision_events_update)
		
		rospy.on_shutdown(self.on_shutdown)
		

		rospy.spin()
		
	def on_shutdown(self):
		pass
		
	
		
		

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node('Empty_node',anonymous=False)
	
	try:
		node = EmptyNode()
	except rospy.ROSInterruptException: pass
	
	
