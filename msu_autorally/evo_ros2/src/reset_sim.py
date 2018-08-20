#!/usr/bin/env python
#
# Reset Sim
#
#	Node for resetting the ROS-Gazebo simulation
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
from gazebo_msgs.srv import GetWorldProperties
from evo_ros2.msg import ResetStatus
from evo_ros2.srv import *


class ResetSimNode():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('reset_sim_node',anonymous=False)
		self.param_namespace = '/evo_ros2'
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		
		# Configure Publishers
		self.reset_status_pub = rospy.Publisher('evo_ros2/reset_status', ResetStatus, queue_size=10)
			
		# Get model_name, namespace, and required_node from ROS params
		self.model_name = rospy.get_param('{}/model_name'.format(self.param_namespace), 'autoRallyPlatform')
		self.namespace = rospy.get_param('{}/namespace'.format(self.param_namespace), 'autorally_platform')
		self.required_node = rospy.get_param('{}/required_node'.format(self.param_namespace), 'platform_empty_required_node')
		
		if self.debug:
			rospy.loginfo('\n\t Model Name: {} \n\t Namespace: {} \n\t Required Node: {}'.format(self.model_name, self.namespace, self.required_node))
		
		
		
		# Set up Gazebo services
		reset_world_service = '/{}/gazebo/reset_world'.format(self.namespace)
		reset_simulation_service = '/{}/gazebo/reset_simulation'.format(self.namespace)
		delete_model_service = '/{}/gazebo/delete_model'.format(self.namespace)
		pause_physics = '/{}/gazebo/pause_physics'.format(self.namespace)
		unpause_physics = '/{}/gazebo/unpause_physics'.format(self.namespace)
		get_world_properties = '/{}/gazebo/get_world_properties'.format(self.namespace)
		
		
		if self.debug:
			rospy.loginfo('Waiting or reset_world and reset_simulation Gazebo services...')
		rospy.wait_for_service(reset_world_service)
		rospy.wait_for_service(reset_simulation_service)
		#rospy.wait_for_service(delete_model_service)
		rospy.wait_for_service(pause_physics)
		rospy.wait_for_service(unpause_physics)
		rospy.wait_for_service(get_world_properties)
		
		self.reset_world = rospy.ServiceProxy(reset_world_service, Empty, persistent=False)
		self.reset_simulation = rospy.ServiceProxy(reset_simulation_service, Empty, persistent=False)
		#self.delete_model = rospy.ServiceProxy(delete_model_service, DeleteModel, persistent=False)
		self.pause_physics = rospy.ServiceProxy(pause_physics, Empty, persistent=False)
		self.unpause_physics = rospy.ServiceProxy(unpause_physics, Empty, persistent=False)
		self.get_world_properties = rospy.ServiceProxy(get_world_properties, GetWorldProperties, persistent=False)
		
		
		self.service_handle = rospy.Service('SoftReset', SoftReset, self.run_reset)
		rospy.spin()
		
			
	def run_reset(self, empty):
		
		#self.pause_physics()
		
		msg = ResetStatus()
		msg.status = 0
		msg.text = 'Beginning reset simulation process'
		self.reset_status_pub.publish(msg)
		
		return_response = SoftResetResponse()
		return_response.status = 1
		return_response.text = ''
			
		# 1 - Kill required node
		if self.debug:
			rospy.loginfo('Killing required node...')
		
		return_value = os.system('rosnode kill {}'.format(self.required_node))
		if return_value is not 0: # Unix returns a 0 for successful execution of tasks
			rospy.logerr('Reset Sim Node: Failed to kill \'{}\' ROS node!'.format(self.required_node))
			
			msg = ResetStatus()
			msg.status = 2
			msg.text = 'An error occured when attempting to kill the required node'
			self.reset_status_pub.publish(msg)
			
			#rospy.signal_shutdown('Reset Sim Node: Failed to kill \'{}\' ROS node!'.format(self.required_node))
			
			return_response.text = 'An error occured when attempting to kill the required node'
			return return_response
		
		if self.debug:
			rospy.loginfo('Reuired node killed!')




		# 2 - Delete model 
		if self.debug:
			rospy.loginfo('Attempting to delete model...')
		
		delete_model_service = '/{}/gazebo/delete_model'.format(self.namespace)
		try:
			rospy.wait_for_service(delete_model_service, timeout=10)
			self.delete_model = rospy.ServiceProxy(delete_model_service, DeleteModel, persistent=False)
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to connect to delete model service')
			rospy.logerr('Exception: {}'.format(exc))
			
			msg = ResetStatus()
			msg.status = 2
			msg.text = 'An error occured when attempting to delete the model'
			self.reset_status_pub.publish(msg)
			
			#rospy.signal_shutdown('Reset Sim Node: Failed to connect to delete model service')
			
			
			return_response.text  = 'An error occured when attempting to delete the model'
			return return_response
		
		try:
			resp = self.delete_model(self.model_name)
			#resp = self.delete_model('{model_name: ' + self.model_name + '}')
			if self.debug:
				rospy.loginfo('Model delete response: {}'.format(resp))
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to delete model: \'{}\'!'.format(self.model_name))
			rospy.logerr('Exception: {}'.format(exc))
			
			msg = ResetStatus()
			msg.status = 2
			msg.text = 'An error occured when attempting to delete the model'
			self.reset_status_pub.publish(msg)
			
			#rospy.signal_shutdown('Reset Sim Node: Failed to delete model: \'{}\'!'.format(self.model_name))
			
			return_response.text = 'An error occured when attempting to delete the model'
			return return_response
		
		# Test if the model is actually deleted from the simulation
		try:
			resp = self.get_world_properties()
			
			if self.model_name in resp.model_names:
				rospy.logerr('Reset Sim Node: model: {} still found after call to deletion service!'.format(self.model_name))
				
				msg = ResetStatus()
				msg.status = 2
				msg.text = 'An error occured when attempting to delete the model'
				self.reset_status_pub.publish(msg)
				
				#rospy.signal_shutdown('Reset Sim Node: model: {} still found after call to deletion service!'.format(self.model_name))
				
				return_response.text = 'An error occured when attempting to delete the model'
				return return_response
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to get world properties!')
			rospy.logerr('Exception: {}'.format(exc))
			#rospy.signal_shutdown('Reset Sim Node: Failed to get world properties!')
			
			return_response.text = 'An error occured when attempting to delete the model'
			return return_response
		
		if self.debug:
			rospy.loginfo('Model Deleted!')
		
		
		
		
		# 3 - Reset World
		try:
			resp = self.reset_world()
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to reset world!')
			rospy.logerr('Exception: {}'.format(exc))
			
			msg = ResetStatus()
			msg.status = 2
			msg.text = 'An error occured when attempting to reset the sim world'
			self.reset_status_pub.publish(msg)
			
			#rospy.signal_shutdown('Reset Sim Node: Failed to reset world!')
			
			
			return_response.text = 'An error occured when attempting to reset the sim world'
			return return_response
		
		if self.debug:
			rospy.loginfo('World reset!')
			
			
			
			
		# 4 - Reset Simulation
		try:
			resp = self.reset_simulation()
		except rospy.ServiceException as exc:
			rospy.logerr('Reset Sim Node: Failed to reset simulation!')
			rospy.logerr('Exception: {}'.format(exc))
			
			msg = ResetStatus()
			msg.status = 2
			msg.text = 'An error occured when attempting to reset the simulation'
			self.reset_status_pub.publish(msg)
			
			#rospy.signal_shutdown('Reset Sim Node: Failed to reset simulation!')
			
			return_response.text = 'An error occured when attempting to reset the simulation'
			return return_response
		
	
		
		
		msg = ResetStatus()
		msg.status = 1
		msg.text = 'The simulation was reset successfully'
		self.reset_status_pub.publish(msg)
		
		if self.debug:
			rospy.loginfo('Simulation reset!')
		
		
		return_response.status = 0
		return_response.text = 'The simulation was reset successfully'
		return return_response	
		
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
	
	
