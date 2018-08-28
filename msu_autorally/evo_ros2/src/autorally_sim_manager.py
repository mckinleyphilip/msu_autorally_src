#!/usr/bin/env python
#
# Autorally Sim Manager Node
#
#	
#
# GAS 2018-08-21

import rospy
import roslaunch
import argparse
import numpy as np
import threading

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
		
		# Get ros params
		self.mission_launch_info = rospy.get_param('sim_manager/MISSION_LAUNCH_FILE')
		
		
		# Set up threading event - Used to sync main thread and the evo-ros2 communication threads since the ros launch api has to be called from the main thread
		self.event = threading.Event()
		self.event.clear()
		
		self.set_up_evo_ros2_communications()
		
		while not rospy.is_shutdown():
			
			# This will block until the event flag is set to true or the timeout value (in seconds) is reached
			#	Returns true if the flag has been set and false if the timeout value is hit
			if not self.event.wait(timeout = 2.0):
				continue
			
			state = rospy.get_param('evo_ros2_state')
			self.event.clear()
			
			if self.debug:
				rospy.logwarn('{} - in main thread and state {}'.format(self.node_name, state))
			
			if state == 4:
				# start sim
				raw_input('In state {} - press enter to advance'.format(state))
				self.start_sim()
				self.set_evo_ros2_state(5)
				continue
	
			if state == 5:
				# sim running
				raw_input('In state {} - press enter to advance'.format(state))
				self.end_sim()
				self.set_evo_ros2_state(6)
				continue
		

	
	def start_sim(self):
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		self.mission_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.mission_launch_info ))
		self.mission_launch.start()
	
	def end_sim(self):
		self.mission_launch.shutdown()
	
	def set_up_evo_ros2_communications(self):
		self.evo_ros2_comm_topic = rospy.get_param('EVO_ROS_COMM_TOPIC')
		self.evo_ros2_comm_pub = rospy.Publisher(self.evo_ros2_comm_topic, EvoROS2State, queue_size=10, latch = True)
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
			self.event.set()
			# start sim

		if msg.state == 5:
			self.event.set()
			# sim running

			
	
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
	
	
