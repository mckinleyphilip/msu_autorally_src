#!/usr/bin/env python
#
# Autorally Sim Manager Node
#
#	
#
# GAS 2018-08-21

import rospy
import roslaunch
import rosnode
import argparse
import numpy as np
import threading
import copy

from evo_ros2.msg import EvoROS2State
from evo_ros2.msg import LogEvent
from evo_ros2.msg import Float64Array



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
		self.utility_monitors_launch_info = rospy.get_param('sim_manager/UTILITY_MONITORS_LAUNCH_FILE')
		self.logging_rate = rospy.get_param('LOGGING_RATE', 10)
		
		# Set up threading event - Used to sync main thread and the evo-ros2 communication threads since the ros launch api has to be called from the main thread
		self.event = threading.Event()
		self.event.clear()
		self.lock = threading.Lock()
		
		# Set up communication topics
		self.set_up_evo_ros2_communications()
		
		# Set up other member variables
		self.log_event_times = []
		self.sim_start_time = 0
		self.result_headers = []
		
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
				self.start_sim()
				self.set_evo_ros2_state(5)
				continue
	
			if state == 5:
				# sim running
				self.sleep_rate = rospy.Rate(self.logging_rate) # Hz
				
				# Perform logging while the mission nodes are still a subset of all ros nodes (they exist)
				while set(self.mission_nodes) < set(rosnode.get_node_names()):
					self.log_event_trigger()
					self.sleep_rate.sleep()
				
				result_msg = self.end_sim()
				self.set_evo_ros2_state(6, msg = result_msg)
				continue
		

	def log_collection_cb(self, msg):
		if self.debug:
			rospy.logwarn('Received log data from: {}'.format(msg.sender))
		
		self.lock.acquire()
		
		try:
			if self.debug:
				rospy.logwarn('Processing log data from: {}'.format(msg.sender))
		
			self.collected_logs.append(msg.log_data)
			self.result_headers.append(msg.data_header)
		finally:
			self.lock.release()
		
		
	def compile_results(self):
		self.results = []
		for index, header in enumerate(self.result_headers):
			self.results.append((header, self.collected_logs[index]))
		rospy.loginfo('\n\n{}: Results'.format(self.node_name))
		rospy.loginfo(self.results)
		
		
	def log_event_trigger(self):
		current_time = 0
		while current_time == 0:
			current_time = rospy.get_rostime()
		
		sim_time = current_time - self.sim_start_time
		
		msg = LogEvent()
		msg.sender = self.node_name
		msg.time = sim_time
		msg.event = 0 # TRIGGER
		self.log_event_topic.publish(msg)
		
		self.log_event_times.append(sim_time.to_sec())


	def log_event_request(self):
		self.collected_logs = []
		self.result_headers.append('time')
		self.collected_logs.append(self.log_event_times)
		
		current_time = 0
		while current_time == 0:
			current_time = rospy.get_rostime()
			
		msg = LogEvent()
		msg.time = current_time
		msg.event = 1 # REQUEST
		self.log_event_topic.publish(msg)
		
	
	def start_sim(self):
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		sleep_rate = rospy.Rate(10) # Hz
		
		
		# Get list of ros nodes that are present before the utility monitors launch file is started
		pre_utility_monitors_nodes = rosnode.get_node_names()
		
		# Start mission launch file which is responsible for controlling the platform through the simulation
		self.utility_monitors_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.utility_monitors_launch_info))
		self.utility_monitors_launch.start()
		
		rospy.sleep(2)
		# Wait for new nodes to show up
		while pre_utility_monitors_nodes == rosnode.get_node_names():
			sleep_rate.sleep()
		
		
		# Get list of ros nodes that are present before the mission launch file is started
		pre_mission_nodes = rosnode.get_node_names()
		self.utility_monitors_nodes = list(set(pre_mission_nodes) - set(pre_utility_monitors_nodes))
		
		
		# Start mission launch file which is responsible for controlling the platform through the simulation
		self.mission_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch.rlutil.resolve_launch_arguments(self.mission_launch_info ))
		self.mission_launch.start()
		
		rospy.sleep(2)
		# Wait for new nodes to show up
		while pre_mission_nodes == rosnode.get_node_names():
			sleep_rate.sleep()
			
		# Get list of ros nodes that are present after the mission launch file has been started and compare it to the ones 
		# 	alive prior to determine the nodes that were spawned as a result of the mission launch file
		post_mission_nodes = rosnode.get_node_names()
		self.mission_nodes = list(set(post_mission_nodes) - set(pre_mission_nodes))
		rospy.logwarn('Mission nodes: {}'.format(self.mission_nodes))
		
		
		# Get the sim start time
		while self.sim_start_time == 0:
			self.sim_start_time = rospy.get_rostime()
	
	def end_sim(self):
		self.log_event_request()
		
		if self.debug:
			rospy.logwarn('Utility Monitor Nodes: {}'.format(self.utility_monitors_nodes))
		
		
		while len(self.collected_logs) <= len(self.utility_monitors_nodes):
			if self.debug:
				rospy.logwarn('Waiting for results...')
			self.sleep_rate.sleep()
					
		
		if self.debug:	
			self.compile_results()		
		
		self.mission_launch.shutdown()
		self.utility_monitors_launch.shutdown()
		
		# Load results into the next state change msg
		msg = EvoROS2State()
		msg.result = []
		for x in range(len(self.collected_logs)):
			msg.result.append(Float64Array())
		
		for index, log_array in enumerate(self.collected_logs):
			msg.result[index].header = self.result_headers[index]
			msg.result[index].data = copy.deepcopy(log_array)
	
		return msg
		
	
	def set_up_evo_ros2_communications(self):
		# Evo ROS state event topic
		self.evo_ros2_comm_topic = rospy.get_param('EVO_ROS_COMM_TOPIC')
		self.evo_ros2_comm_pub = rospy.Publisher(self.evo_ros2_comm_topic, EvoROS2State, queue_size=10, latch = True)
		self.evo_ros2_comm_sub = rospy.Subscriber(self.evo_ros2_comm_topic, EvoROS2State, self.on_evo_ros2_state_change)
		
		# Evo ROS logging topics
		self.log_event_topic = rospy.Publisher(rospy.get_param('EVO_ROS_LOG_EVENT_TOPIC'), LogEvent, queue_size=10)
		self.log_collection_topic = rospy.Subscriber(rospy.get_param('EVO_ROS_LOG_COLLECTION_TOPIC'), LogEvent, self.log_collection_cb)
	
	
	def set_evo_ros2_state(self, new_state_value, msg = EvoROS2State()):
		rospy.set_param('evo_ros2_state', new_state_value)
		msg.sender = self.node_name
		msg.state = new_state_value
		#print(msg)
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
		self.mission_launch.shutdown()
		self.utility_monitors_launch.shutdown()
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = AutorallySimManagerNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
