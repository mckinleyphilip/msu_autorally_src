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
import random

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
                self.direction = cmd_args.direction
                
                # Let rosparams override command line arguments
                if rospy.has_param('force_direction'):
                    self.direction = rospy.get_param('force_direction')
		
		# List of goal positions
		points = rospy.get_param('move_base_seq/p_seq')

		#List of goal quaternions:
		quat_seq = rospy.get_param('move_base_seq/quat_seq')
		
		#List of goal poses:
		self.pose_seq = list()
		self.goal_cnt = 0
		
                # RANDOMIZE DIRECTION
                if self.direction not in ('clockwise', 'counterclockwise', 'counter-clockwise'):
                    print('Using random direction...')
                    if random.random() < 0.5:
                        self.direction = 'clockwise'
                    else:
                        self.direction = 'counterclockwise'

                if self.direction == 'clockwise':
                    print('moving clockwise')
                    rospy.set_param('direction', -1)
                    
                    # reverse all points except last -- "complicated" because of duplicates

                    # GENERALIZED ROUTINE for points
                    #upper_bnd = int((len(points)-2)/4)*2
                    #for i in range(0, upper_bnd, 2):
                    #    tmp = points[i]
                    #    j = len(points)-2 - (i+2)
                    #    points[i] = points[i+1] = points[j]
                    #    points[j] = points[j+1] = tmp

                    # hard coded version
                    points[0] = points[5]
                    points[4] = points[1]
                    
                    points[1] = points[0]
                    points[5] = points[4]

                    quat_seq[2] = quat_seq[7]
                    quat_seq[6] = quat_seq[3]

                    quat_seq[3] = quat_seq[2]
                    quat_seq[7] = quat_seq[6]
                else:
                    # move counter-clockwise -- default order
                    print('moving counter-clockwise')
                    rospy.set_param('direction', 1)

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
		rospy.loginfo("Goal pose %s is now being processed by the Action Server..."%self.goal_cnt)

	def feedback_cb(self, feedback):
		#To print current pose at each feedback:
		#rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
		#rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt)+" received")
		pass

	def done_cb(self, status, result):
		self.goal_cnt += 1
		
		# Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
		
		# 2 - PREEMPTED - The goal received a cancel request after it started executing
		#		and has since completed its execution (Terminal State)
		if status == 2:
			rospy.loginfo("PREEMPTED: Goal pose %s received a cancel request after it started executing, completed execution!" % (self.goal_cnt-1))
		# 3 - SUCCEEDED - The goal was achieved successfully by the action server (Terminal State)
		elif status == 3:
			
			rospy.loginfo("SUCCEEDED: Goal pose %s reached" % (self.goal_cnt-1)) 
			self.goal_status_pub.publish(self.goal_cnt-1)
			if self.goal_cnt < len(self.pose_seq):
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.header.stamp = rospy.Time.now()
				next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
				rospy.loginfo("Sending goal pose %s to Action Server"%self.goal_cnt)
				rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
			else:
				rospy.loginfo("Final goal pose reached!")
				rospy.signal_shutdown("Final goal pose reached!")
				#self.goal_status_pub.publish(self.goal_cnt)
				rospy.sleep(1.0)
		# 4 - ABORTED - The goal was aborted during execution by the action server due
		#    	to some failure (Terminal State)
		elif status == 4:
			rospy.logerr("ABORTED: Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
		# 5 - REJECTED - The goal was rejected by the action server without being processed,
		#    	because the goal was unattainable or invalid (Terminal State)
		elif status == 5:
			rospy.logerr("REJECTED: Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
		# 8 - RECALLED -The goal received a cancel request before it started executing
		#    		and was successfully cancelled (Terminal State)
		elif status == 8:
			rospy.loginfo("RECALLED: Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

	def movebase_client(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = self.pose_seq[self.goal_cnt]
		rospy.loginfo("Sending goal pose %s to Action Server" % self.goal_cnt)
		rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		rospy.spin()

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Move base goal publisher node. Takes rosparams move_base/p_seq and move_base/quat_seq and loads them into a sequence of move_base goals that are published to the autorally')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
        parser.add_argument('-f', '--force_direction', type=str, dest='direction',
        default='', help='Force direction of travel about the track')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above
	
	
	try:
		node = MoveBaseSeq(cmd_args = args)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation finished.")
