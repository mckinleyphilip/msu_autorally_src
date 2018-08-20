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

from evo_ros2.msg import ResetStatus

class SoftwareManagerNode():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('software_managaer_node',anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		# Get ros params
		self.world_launch_info = rospy.get_param('software_manager/WORLD_LAUNCH_FILE', ['msu_autorally_helper', 'spawn_gazebo_world.launch'])
		self.platform_launch_info = rospy.get_param('software_manager/PLATFORM_LAUNCH_FILE', ['msu_autorally_helper','spawn_autorally_platform.launch'])
		
		
		"""
		CODE HERE...
		
		# Configure Publishers
		#self.collision_events_request_pub = rospy.Publisher('/util_comms/collision_events_request', Bool, queue_size=10)


		# Configure Subscribers
		#self.collision_events_sub = rospy.Subscriber('/util_comms/collision_events', String,self.on_collision_events_update)
		
		"""
		
		
		
		

		self.start_sim_env()
		
		if self.debug:
			rospy.logwarn('Started env')
		
		raw_input('soft reset...')
		self.soft_reset()
		
		if self.debug:
			rospy.logwarn('finished soft reset spawn')
		
		raw_input('hard reset...')
		self.hard_reset()
		
		if self.debug:
			rospy.logwarn('finished hard reset')
		
		raw_input('start_sim_env')
		self.start_sim_env()
		
		if self.debug:
			rospy.logwarn('restarted env')
			
		rospy.spin()

		
	
	def start_sim_env(self):
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		
		self.world_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.world_launch_info ))
		self.platform_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.platform_launch_info))
		
		self.world_launch.start()
		self.platform_launch.start()
	
	
	def hard_reset(self):
		self.reset_node_process.stop()
		self.world_launch.shutdown()
		self.platform_launch.shutdown()
		time.sleep(7)
		
		
	def soft_reset(self):
		reset_node = roslaunch.core.Node('evo_ros2', 'reset_sim.py', name='reset_sim')
		reset_node_launch = roslaunch.scriptapi.ROSLaunch()
		reset_node_launch.start()
		self.reset_node_process = reset_node_launch.launch(reset_node)
		self.soft_reset_status_sub = rospy.Subscriber('/evo_ros2/reset_status', ResetStatus, self.soft_reset_status_callback)
		
		
	def soft_reset_status_callback(self, msg):
		if self.debug:
			rospy.logwarn('Soft reset status : {}--{}'.format(msg.status, msg.text))
		
		
		
	def on_shutdown(self):
		pass
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	try:
		node = SoftwareManagerNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
