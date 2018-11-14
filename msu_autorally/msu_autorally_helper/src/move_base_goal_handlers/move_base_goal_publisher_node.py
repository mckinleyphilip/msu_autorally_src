#!/usr/bin/env python
#
# Move Base Goal Publisher Node
#
#	Takes goal positions and quaternions stored in ROS params and constructs a sequence of pose goals that are published to the ROS action server
#
# GAS 2018-08-01
import rospy
import math
import argparse

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64


class MoveBaseSeq():

	def __init__(self, cmd_args):

		rospy.init_node('move_base_goal_publisher_node',anonymous=False)
		
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = cmd_args.debug
		
		
		# List of goal positions
		points = rospy.get_param('move_base_seq/p_seq')

		#List of goal quaternions:
		quat_seq = rospy.get_param('move_base_seq/quat_seq')
		
		#List of goal poses:
		self.pose_seq = list()
		self.goal_cnt = 0
		
		# Create a sequence of goal poses by
		#	 Unpacking(*) the sequence of point coordinates into a Point constructor
		#	 Unpacking(*) the sequence of quaternions into a Quaternion constructor
		#	 And feeding both into a Pose constructor
		for index, point in enumerate(points):
			print("\tGoal: {}\nPos:\n{}\nQuat:\n{}\n".format(index, Point(*point),Quaternion(*quat_seq[index]) ) )
			self.pose_seq.append(Pose(Point(*point),Quaternion(*quat_seq[index])))
			
		# Publish goals topic
		self.goal_status_pub = rospy.Publisher('goal_status', Float64, queue_size=10)
			
			
		#Create action client
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
		#wait = self.client.wait_for_server(rospy.Duration(5.0))
		self.client.wait_for_server()
		#if not wait:
		#	rospy.logerr("Action server not available!")
		#	rospy.signal_shutdown("Action server not available!")
		#	return
		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting goals achievements ...")
		self.movebase_client()

	# Last activities to perform before the node shuts down
	def on_shutdown(self):
		pass
		
	def active_cb(self):
		rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

	def feedback_cb(self, feedback):
		#To print current pose at each feedback:
		#rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
		#rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
		pass

	def done_cb(self, status, result):
		self.goal_cnt += 1
		
		# Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
		
		# 2 - PREEMPTED - The goal received a cancel request after it started executing
		#		and has since completed its execution (Terminal State)
		if status == 2:
			rospy.loginfo("PREEMPTED: Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")


		# 3 - SUCCEEDED - The goal was achieved successfully by the action server (Terminal State)
		if status == 3:
			
			rospy.loginfo("SUCCEEDED: Goal pose "+str(self.goal_cnt)+" reached") 
			self.goal_status_pub.publish(self.goal_cnt)
			if self.goal_cnt < len(self.pose_seq):
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.header.stamp = rospy.Time.now()
				next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
				rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
				#rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
			else:
				rospy.loginfo("Final goal pose reached!")
				rospy.signal_shutdown("Final goal pose reached!")
				self.goal_status_pub.publish(self.goal_cnt)
				rospy.sleep(0.25)
				return

		# 4 - ABORTED - The goal was aborted during execution by the action server due
		#    	to some failure (Terminal State)
		if status == 4:
			rospy.logerr("ABORTED: Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
			return


		# 5 - REJECTED - The goal was rejected by the action server without being processed,
		#    	because the goal was unattainable or invalid (Terminal State)
		if status == 5:
			rospy.logerr("REJECTED: Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
			return


		# 8 - RECALLED -The goal received a cancel request before it started executing
		#    		and was successfully cancelled (Terminal State)
		if status == 8:
			rospy.loginfo("RECALLED: Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

	def movebase_client(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = self.pose_seq[self.goal_cnt]
		rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
		rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		rospy.spin()

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Move base goal publisher node. Takes rosparams move_base/p_seq and move_base/quat_seq and loads them into a sequence of move_base goals that are published to the autorally')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above
	
	
	try:
		node = MoveBaseSeq(cmd_args = args)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation finished.")
