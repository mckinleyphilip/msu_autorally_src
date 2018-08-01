#!/usr/bin/env python
#
# Move Base Goal Logger
#
#	Records 2D navigation goals published on the "/move_base/goal" topic (commonly used for publishing goals when using
#		the RVIZ GUI) and logs the positions and quaternions into a yaml file
#
# GAS 2018-08-01

import rospy
import argparse
import numpy as np
import os
import rospkg

import time
import datetime
from move_base_msgs.msg import MoveBaseActionGoal

class MoveBaseGoalLogger():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('move_base_goal_logger_node',anonymous=False)
		rospy.loginfo('Node initiated!')
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = cmd_args.debug
		self.log_dir = cmd_args.log_dir
		self.log_name = cmd_args.log_name
		
		# Set up member variables
		self.p_seq = list()
		self.quat_seq = list()

		# Configure Subscribers
		goal_topic = '/move_base/goal'
		self.move_base_goals_sub = rospy.Subscriber(goal_topic, MoveBaseActionGoal, self.log_goal)
		if self.debug:
			rospy.loginfo('{} topic subscribed to!'.format(goal_topic))
		
		

		rospy.spin()
	
	
	def log_goal(self, goal):
		goal_points = goal.goal.target_pose.pose.position
		goal_orient = goal.goal.target_pose.pose.orientation
		
		self.p_seq.append([goal_points.x,goal_points.y,goal_points.z])
		self.quat_seq.append([goal_orient.x,goal_orient.y,goal_orient.z,goal_orient.w])
		
		if self.debug:
			rospy.loginfo('Goal Recv\'d!')
			rospy.loginfo(goal_points)
			rospy.loginfo(goal_orient)
	
		
	# Writes the log to file
	def write_log(self):
		if not os.path.isdir(self.log_dir):
			os.makedirs(self.log_dir)
		
		with open(self.log_dir + self.log_name + '.yaml', 'w+') as outfile:
			outfile.write('# This file was created using the move_base_goal_logger\n')
			outfile.write('# Manually insert description here...\n')
			outfile.write('move_base_seq:\n')
			outfile.write('  p_seq: {}\n'.format(self.p_seq))
			outfile.write('  quat_seq: {}\n'.format(self.quat_seq))
			
			
	def on_shutdown(self):
		if self.debug:
			rospy.loginfo('{} total goals recorded!'.format(len(self.p_seq)))
		self.write_log()
		if self.debug:
			rospy.loginfo('Log file written!')
		
	
if __name__ == '__main__':
	
	# get an instance of RosPack with the default search paths
	rospack = rospkg.RosPack()
	
	# get the file path for rospy_tutorials
	package_path = rospack.get_path('msu_autorally_helper')
	
	# Parse arguments
	parser = argparse.ArgumentParser(description='Move Base Goal Logger. Records 2D navigation goals published on the "/move_base/goal" topic (commonly used for publishing goals when using the RVIZ GUI) and logs the positions and quaternions into a yaml file')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	parser.add_argument('-ld', '--log_dir', type=str, default=str(package_path + '/waypoints/'), help='Save directory for log file')
	parser.add_argument('-ln', '--log_name', type=str, default=str('waypoints_'+str(datetime.datetime.now().strftime("%m-%d-%y_%H:%M"))), help='Log file name')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = MoveBaseGoalLogger(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
