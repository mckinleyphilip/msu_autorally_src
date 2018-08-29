#!/usr/bin/env python
#
# Software Manager Node
#
#	Base ROS node
#
# GAS 2018-08-20

import rospy
import roslaunch
import argparse
import numpy as np
import time

import threading

from evo_ros2.srv import SoftReset
from evo_ros2.msg import EvoROS2State

class SoftwareManagerNode():
	def __init__(self, cmd_args):
		
		# Init Node
		self.node_name = 'software_managaer_node'
		rospy.init_node(self.node_name, anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		# Get ros params
		self.world_launch_info = rospy.get_param('software_manager/WORLD_LAUNCH_FILE')
		self.platform_launch_info = rospy.get_param('software_manager/PLATFORM_LAUNCH_FILE')
		self.sim_manager_launch_info = rospy.get_param('software_manager/SIM_MANAGER_LAUNCH_FILE')
		
		""" # Soft reset capabilities still being developed
		# Set up connection for Evo-ROS2 soft reset
		rospy.wait_for_service('/SoftReset')
		self.soft_rest_service = rospy.ServiceProxy('/SoftReset', SoftReset, persistent=False)
		"""
		
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
			
			if state == 1:
				self.start_sim_env()
				
				self.set_evo_ros2_state(2)
				continue
			
			if state == 2:
				self.start_sim_manager()
				# Software manager will set new state when ready
				continue
				
			if state == 6:
				self.hard_reset()
				self.set_evo_ros2_state(7)
				continue
				
			

		
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
		
		if msg.state == 1:
			self.event.set()
			#self.start_sim_env()
			#self.set_evo_ros2_state(2)
		
		if msg.state == 2:
			self.event.set()
			#self.start_sim_manager()
			# Software manager will set new state when ready
			
		if msg.state == 6:
			self.event.set()
			#self.hard_reset()
			#self.set_evo_ros2_state(7)
			
		
	
	
	def start_sim_manager(self):
		self.sim_manager_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.sim_manager_launch_info))
		self.sim_manager_launch.start()
		
	def start_sim_env(self):
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		
		self.world_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.world_launch_info ))
		self.platform_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.platform_launch_info))
		
		self.world_launch.start()
		self.platform_launch.start()
		rospy.sleep(2)


	def hard_reset(self):
		self.world_launch.shutdown()
		self.platform_launch.shutdown()
		self.sim_manager_launch.shutdown()
		
		
	def soft_reset(self):
		self.platform_launch.shutdown()
		self.soft_rest_service()
		
		
		
		
	def on_shutdown(self):
		self.hard_reset()
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	try:
		node = SoftwareManagerNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
