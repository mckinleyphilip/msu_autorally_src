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

from gazebo_msgs.srv import GetWorldProperties

from evo_ros2.msg import EvoROS2State
from evo_ros2.msg import LogEvent
from evo_ros2.msg import Float64Array

from autorally_msgs.msg import wheelSpeeds
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from gazebo_msgs.msg import ModelStates



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
		self.max_sim_time = rospy.get_param('sim_manager/MAX_SIM_TIME', 300)
		self.world_properties_service = rospy.get_param('ROS_GAZEBO_WORLD_PROPERTIES_SERVICE')
		
		# Get Access to Gazebo World properties
		rospy.logwarn('{} - Waiting for Gazebo Properties'.format(self.node_name))
		rospy.wait_for_service(self.world_properties_service)
		self.getWorldProp = rospy.ServiceProxy(self.world_properties_service, GetWorldProperties)
		
		# Prepare log
		self.prepare_log()
		
		# Sent up monitored topics
		self.wheel_speed_topic = rospy.Subscriber('/wheelSpeeds', wheelSpeeds, self.wheel_speed_topic_cb)
		self.goal_speed_topic = rospy.Subscriber('/cmd_vel', Twist, self.goal_speed_topic_cb)
		self.goal_status_topic = rospy.Subscriber('/goal_status', Float64, self.goal_status_cb)
		self.model_states = rospy.Subscriber("/autorally_platform/gazebo/model_states", ModelStates, self.model_states_cb)
		
		
		
		# Set up threading event - Used to sync main thread and the evo-ros2 communication threads since the ros launch api has to be called from the main thread
		self.event = threading.Event()
		self.event.clear()
		self.lock = threading.Lock()
		
		# Set up communication topics
		self.set_up_evo_ros2_communications()
		
		# Set up other member variables
		self.sim_start_time = 0

		
		while not rospy.is_shutdown():
			
			# This will block until the event flag is set to true or the timeout value (in seconds) is reached
			#	Returns true if the flag has been set and false if the timeout value is hit
			if not self.event.wait(timeout = 1.0):
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
				self.current_time = 0
				
				# Perform logging while the mission nodes are still a subset of all ros nodes (they exist) and less than max time has occured
				while (set(self.mission_nodes) < set(rosnode.get_node_names()) and self.current_time < self.max_sim_time):
					#print('{}/{}'.format(self.current_time, self.max_sim_time))
					self.log_event() # Log events update current time
					self.sleep_rate.sleep()
				
				result_msg = self.end_sim()
				self.set_evo_ros2_state(6, msg = result_msg)
				continue
		

	def prepare_log(self):
		try:
			current_time = self.getWorldProp().sim_time 
		except:
			rospy.logerr('{} - Failed to get sim time!'.format(self.node_name))
			
		self.result_headers = ['Time', 'Goal Speed', 'Actual Speed', 'Goal Status', 'Pos X', 'Pos Y', 'Pos Z', 'Ori X', 'Ori Y', 'Ori Z', 'Ori W']
		self.last_goal_speed = 0
		self.last_actual_speed = 0
		self.last_goal = 0
		self.pos = [0,0,0]
		self.ori = [0,0,0,0]
		self.log = [[current_time],[self.last_goal_speed],[self.last_actual_speed], [self.last_goal], [self.pos[0]], [self.pos[1]], [self.pos[2]], [self.ori[0]], [self.ori[1]], [self.ori[2]], [self.ori[3]],]
		
		
	def log_event(self):
		try:
			current_time = self.getWorldProp().sim_time 
		except:
			rospy.logerr('{} - Failed to get sim time!'.format(self.node_name))
		self.current_time = current_time	
		self.log[0].append(current_time)
		self.log[1].append(self.last_goal_speed)
		self.log[2].append(self.last_actual_speed)
		self.log[3].append(self.last_goal)
		self.log[4].append(self.pos[0])
		self.log[5].append(self.pos[1])
		self.log[6].append(self.pos[2])
		self.log[7].append(self.ori[0])
		self.log[8].append(self.ori[1])
		self.log[9].append(self.ori[2])
		self.log[10].append(self.ori[3])
		
	def model_states_cb(self, msg):
		platform_name = "autoRallyPlatform"
		for index, model in enumerate(msg.name):
			if model == platform_name:
				pose = msg.pose[index]
				self.pos = [pose.position.x, pose.position.y, pose.position.z]
				self.ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
			
		
	def goal_status_cb(self, msg):
		if msg.data % 2 == 0:
			self.last_goal = float(msg.data/2)
		
	def wheel_speed_topic_cb(self, msg):
		self.last_actual_speed = (msg.lfSpeed + msg.rfSpeed) / 2.0
	
	def goal_speed_topic_cb(self, msg):
		self.last_goal_speed = msg.linear.x * 1.5
		

	def start_sim(self):
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		sleep_rate = rospy.Rate(10) # Hz
		
		# Get list of ros nodes that are present before the mission launch file is started
		pre_mission_nodes = rosnode.get_node_names()

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
		self.mission_launch.shutdown()

		
		# Load results into the next state change msg
		msg = EvoROS2State()
		msg.result = []
		for x in range(len(self.log)):
			msg.result.append(Float64Array())
		
		#print(self.log)
		#print(self.result_headers)
		
		for index, log_array in enumerate(self.log):
			msg.result[index].header = self.result_headers[index]
			msg.result[index].data = copy.deepcopy(log_array)
		return msg
		
	
	def set_up_evo_ros2_communications(self):
		# Evo ROS state event topic
		self.evo_ros2_comm_topic = rospy.get_param('EVO_ROS_COMM_TOPIC')
		self.evo_ros2_comm_pub = rospy.Publisher(self.evo_ros2_comm_topic, EvoROS2State, queue_size=10, latch = True)
		self.evo_ros2_comm_sub = rospy.Subscriber(self.evo_ros2_comm_topic, EvoROS2State, self.on_evo_ros2_state_change)
		
	
	
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
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = AutorallySimManagerNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
