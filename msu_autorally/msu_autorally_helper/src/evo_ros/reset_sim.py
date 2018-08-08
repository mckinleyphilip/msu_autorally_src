#!/usr/bin/env python
#
# Reset Sim
#
#	Node for resetting the Gazebo Autorally simulation
#	
#	Resetting takes 4 steps
#		1. Kill a required node in the vehicle launch file to trigger a ROS shutdown of all nodes launched from that file
#			  Note: If the Gazebo world is spawned from a seperate launch file this will be left running
#		2. Delete the vehile model using the Gazebo delelete_model service
#		3. Use the Gazebo reset_world service
#		4. Use the Gazebo reset_simulation service
#
#	At this point the vehicle launch file can be relaunched in a clean simulation environment
#
# GAS 2018-08-07

import rospy
import argparse
import numpy as np
import os
import time

from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel


class ResetSimNode():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('reset_sim_node',anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = cmd_args.debug
		
				
		# Get model_name, namespace, and required_node from ROS params
		# TO-DO: get these from the ros param server
		self.model_name = 'autoRallyPlatform'
		self.namespace = 'autorally_platform'
		self.required_node = 'empty_required_node'
		
		
		# Set up Gazebo services
		reset_world_service = '/{}/gazebo/reset_world'.format(self.namespace)
		reset_simulation_service = '/{}/gazebo/reset_simulation'.format(self.namespace)
		delete_model_service = '/{}/gazebo/delete_model'.format(self.namespace)
		pause_physics = '/{}/gazebo/pause_physics'.format(self.namespace)
		unpause_physics = '/{}/gazebo/unpause_physics'.format(self.namespace)
		
		
		if self.debug:
			rospy.loginfo('Waiting or reset_world and reset_simulation Gazebo services...')
		rospy.wait_for_service(reset_world_service)
		rospy.wait_for_service(reset_simulation_service)
		rospy.wait_for_service(delete_model_service)
		rospy.wait_for_service(pause_physics)
		rospy.wait_for_service(unpause_physics)
		
		self.reset_world = rospy.ServiceProxy(reset_world_service, Empty, persistent=False)
		self.reset_simulation = rospy.ServiceProxy(reset_simulation_service, Empty, persistent=False)
		self.delete_model = rospy.ServiceProxy(delete_model_service, DeleteModel, persistent=False)
		self.pause_physics = rospy.ServiceProxy(pause_physics, Empty, persistent=False)
		self.unpause_physics = rospy.ServiceProxy(unpause_physics, Empty, persistent=False)
		
		
		# Start the reset
		self.run_reset()
		
		
		
	def run_reset(self):
		
		#self.pause_physics()
		
		
		
			
		# 1 - Kill required node
		if self.debug:
			rospy.loginfo('Killing required node...')
		
		return_value = os.system('rosnode kill {}'.format(self.required_node))
		if return_value is not 0: # Unix returns a 0 for successful execution of tasks
			rospy.logerr('Reset Sim Node: Failed to kill \'{}\' ROS node!'.format(self.required_node))
			rospy.signal_shutdown('Reset Sim Node: Failed to kill \'{}\' ROS node!'.format(self.required_node))
			return
		
		if self.debug:
			rospy.loginfo('Reuired node killed!')


		# 2 - Delete model 
		if self.debug:
			rospy.loginfo('Attempting to delete model...')
		
		try:
			resp = self.delete_model(str('{}'.format(self.model_name)))
			#resp = self.delete_model('{model_name: ' + self.model_name + '}')
			if self.debug:
				rospy.loginfo('Model delete response: {}'.format(resp))
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to delete model: \'{}\'!'.format(self.model_name))
			rospy.logerr('Exception: {}'.format(exc))
			rospy.signal_shutdown('Reset Sim Node: Failed to delete model: \'{}\'!'.format(self.model_name))
			return
			
		if self.debug:
			rospy.loginfo('Model Deleted!')
		

		# 3 - Reset World
		try:
			resp = self.reset_world()
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to reset world!')
			rospy.logerr('Exception: {}'.format(exc))
			rospy.signal_shutdown('Reset Sim Node: Failed to reset world!')
			return
		
		if self.debug:
			rospy.loginfo('World reset!')
			
		# 4 - Reset Simulation
		try:
			resp = self.reset_simulation()
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to reset simulation!')
			rospy.logerr('Exception: {}'.format(exc))
			rospy.signal_shutdown('Reset Sim Node: Failed to reset simulation!')
			return
		
		if self.debug:
			rospy.loginfo('Simulation reset!')
			
		#self.unpause_physics()		
		
	def on_shutdown(self):
		pass
		"""
		self.reset_world.close()
		self.reset_simulation.close()
		self.delete_model.close()
		self.pause_physics.close()
		self.unpause_physics.close()
		"""
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Test node for resetting the Gazebo simulation')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above
	
	try:
		node = ResetSimNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
