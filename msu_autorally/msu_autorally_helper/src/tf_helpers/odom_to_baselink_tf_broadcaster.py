#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry



def handle_pose(msg):
	br = tf.TransformBroadcaster()
	pose = msg.pose.pose
	br.sendTransform(
		(pose.position.x, pose.position.y, pose.position.z), # Position
		(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), # Orientation
		rospy.Time.now(), # Time
		'base_link', # Child Frame
		'odom') # Parent Frame



if __name__ == '__main__':
	rospy.init_node('odom_to_baselink_tf_broadcaster')
	rospy.Subscriber('/pose_estimate', Odometry, handle_pose)
	rospy.spin()
