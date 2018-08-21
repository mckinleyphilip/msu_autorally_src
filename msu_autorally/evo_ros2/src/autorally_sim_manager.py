#!/usr/bin/env python
#
# Autorally Sim Manager Node
#
#	
#
# GAS 2018-08-21

import rospy
import argparse
import numpy as np

from evo_ros2.msg import EvoROS2State



class AutorallySimManagerNode():
	def __init__(self, cmd_args):
		
		# Init Node
		self.node_name = 'autorally_sim_manager'
		rospy.init_node(self.node_name, anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		
		self.set_up_evo_ros2_communications()
		
		rospy.spin()
	

		
	def set_up_evo_ros2_communications(self):
		self.evo_ros2_comm_topic = rospy.get_param('EVO_ROS_COMM_TOPIC')
		self.evo_ros2_comm_pub = rospy.Publisher(self.evo_ros2_comm_topic, EvoROS2State, queue_size=10, latch = False)
		self.evo_ros2_comm_sub = rospy.Subscriber(self.evo_ros2_comm_topic, EvoROS2State, self.on_evo_ros2_state_change)
	
	
	def set_evo_ros2_state(self, new_state_value):
		rospy.set_param('evo_ros2_state', new_state_value)
		msg = EvoROS2State()
		msg.sender = self.node_name
		msg.state = new_state_value
		self.evo_ros2_comm_pub.publish(msg)
		
			
	def on_evo_ros2_state_change(self, msg):
		if self.debug:
			rospy.logwarn('{} - In state: {}'.format(self.node_name, msg.state))
			
		if msg.state == 2:
			self.set_evo_ros2_state(3)
			
		if msg.state == 4:
			# start sim
			raw_input('In state {} - press enter to advance'.format(msg.state))
			self.set_evo_ros2_state(5)

		if msg.state == 5:
			# sim running
			raw_input('In state {} - press enter to advance'.format(msg.state))
			self.set_evo_ros2_state(6)
			
	
	def on_shutdown(self):
		pass
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = AutorallySimManagerNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
