#!/usr/bin/env python  
import roslib
import rospy
import rostopic
import numpy as np
import argparse
import os
import json

from copy import deepcopy
from datetime import datetime
from tf.transformations import quaternion_from_euler

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates

#  /autorally_platform/gazebo/link_states
# holds more info on parts of the autorally
#topic_name = "/autorally_platform/gazebo/model_states"
topic_name = "/autorally_platform/gazebo/link_states"





class review_visualizer_logger_node():
	
	# Init node
	def __init__(self, cmd_args):
		
		rospy.init_node('review_visualizer_logger',anonymous=False)

		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		
		# Write command line arguments into class variables
		self.debug = cmd_args.debug
		self.logging_step_size = cmd_args.speed
		self.animation_name = cmd_args.name
		self.log_dir = cmd_args.log_dir
		self.log_name = cmd_args.log_name
		
		
		self.log = {}
		self.objects = []
		self.models = []
		self.model_poses = []
		
		# Determine Physics Step Size
		# Need to determine the physics step size of the simulator so we can log the time delta between position entries in the log file
		#	To do this we sample the topic and measure the average change of the simulation time between messages
		#	Note we wouldn't have to do this if the model_states or link_states topics were time stamped!
		# {
		self.physics_counter = 200
		self.physics_step_size = -1
		self.sample_times = []
		
		self.determine_physics_step_size_sub = rospy.Subscriber(topic_name, LinkStates, self.determine_physics_step_size)
		
		while self.physics_step_size == -1:
			pass
		
		# }
		
		
		if self.debug:
			self.print_debug_header()
		
		# Init log
		self.init_log()
		
		
		self.callback_counter = 0 # Used to implement a slower logging frequency than what model / link states are published at
		self.state_sub = rospy.Subscriber(topic_name, ModelStates, self.read_states)
		
		
		rospy.spin()

	# Fills out everything in the log besides the frames
	def init_log(self):
		self.find_objects()
		self.log['name'] = self.animation_name
		self.log['timeStep'] = self.logging_step_size
		self.log['objects'] = self.objects
		self.log['frames'] = []
		
		
	# Finds all of the objects in the simulation
	def find_objects(self):
		for index, model in enumerate(self.models):
			new_obj = {}
			new_obj['name'] = model
			
			if "box" in model:
				new_obj['mesh'] = 'cube'
			elif "autoRally" in model:
				if "wheel" in model:
					new_obj['mesh'] = 'sphere'
					new_obj['scale'] = [0.19,0.19,0.19]
				elif "base_link" in model:
					new_obj['mesh'] = 'cube'
					new_obj['scale'] = [0.9,0.25,0.25]
				else:
					new_obj['mesh'] = 'sphere'
					new_obj['scale'] = [0.0,0.0,0.0]
			else:
				new_obj['mesh'] = 'sphere'
				new_obj['scale'] = [0.0,0.0,0.0]
				
			
			#new_obj['translation'] = [self.model_poses[index].position.x, self.model_poses[index].position.y, self.model_poses[index].position.z]
			
			
			self.objects.append(deepcopy(new_obj))
			
			
	def read_states(self, msg):
		self.callback_counter += 1
		
		
		
		if self.callback_counter % float(self.logging_step_size / self.physics_step_size) < 1.0:
			
			self.callback_counter = 0
			 
			print('New frame at: {}'.format(rospy.get_time()))
	
			new_frame = {}
			for index, model in enumerate(msg.name):
				new_translation = {}
				
				new_translation["t"] = [msg.pose[index].position.x, msg.pose[index].position.z, msg.pose[index].position.y]
				
				
				# Transform euler representation of rotation to a quaternion
				# = quaternion_from_euler(msg.pose[index].orientation.x, msg.pose[index].orientation.z, msg.pose[index].orientation.y)
				#new_translation["r"] = [q[0], q[1], q[2], q[3]]
#				
#
#				
				# This is not working right yet
				new_translation["r"] = [msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z, msg.pose[index].orientation.w]
				
				new_frame[model] = deepcopy(new_translation)
				
			self.log['frames'].append(deepcopy(new_frame))

		
		

		
	# Last activities to perform before the node shuts down
	def on_shutdown(self):
		self.write_log()
		if self.debug:
			print('Log file saved!')
		
		
		
	# Writes the log to file
	def write_log(self):
		if not os.path.isdir(self.log_dir):
			os.makedirs(self.log_dir)
		
		with open(self.log_dir + self.log_name + '.json', 'w+') as outfile:
			json.dump(self.log, outfile, indent=2, sort_keys=True)
		
		
		
		
	# Prints an information header for debugging purposes
	def print_debug_header(self):
		print("""\n 
review_visualizer_logger_node
	Debugging on!
	Listening for states on topic: {}
	Topic frequency: {}
	Calculated physics step size: {} 
	Logging step size: {} 
	Log Directory: {} 
	Log Name: {} """\
			.format(topic_name, 1.0 / self.physics_step_size, self.physics_step_size, self.logging_step_size, self.log_dir, self.log_name))
			
			
			
	# Each time a msg is published, a sample of the current simulation time is taken.
	#	When the counter expires, the average physics step size is calculated
	def determine_physics_step_size(self, msg):
		self.sample_times.append(rospy.get_time())
		if self.physics_counter > 0:
			self.physics_counter -= 1
		else:
			self.determine_physics_step_size_sub.unregister()
			self.physics_step_size = np.mean(np.diff(self.sample_times))
			self.models = msg.name
			self.model_poses = msg.pose
		
		
	
		

			



if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='review_visualizer_logger. Records the physics state of simulated objects and outputs a JSON file formatted to work with review (https://review.github.io/) for playback of the simulation')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	parser.add_argument('-s', '--speed', type=float, default=0.2, help='The time sep (in seconds) betweeen each logging of object positions, default 0.2')
	parser.add_argument('-n', '--name', type=str, default=str(datetime.now()), help='The name of logged animation / experiment')
	parser.add_argument('-ld', '--log_dir', type=str, default=str(os.path.dirname(os.path.realpath(__file__)) + '/logs/'), help='Save directory for log file')
	parser.add_argument('-ln', '--log_name', type=str, default='test', help='Log file name')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above
	
	
	
	try:
		node = review_visualizer_logger_node(cmd_args = args)
	except rospy.ROSInterruptException: pass
	
	
