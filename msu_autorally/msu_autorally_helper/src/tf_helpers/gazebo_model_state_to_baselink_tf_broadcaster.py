#!/usr/bin/env python  
import roslib
import rospy

import tf
from gazebo_msgs.msg import ModelStates


topic_name = "/autorally_platform/gazebo/model_states"
platform_name = "autoRallyPlatform"

def handle_pose(msg):
	br = tf.TransformBroadcaster()
	
	for index, model in enumerate(msg.name):
		if model == platform_name:
			pose = msg.pose[index]
			#print('Model: {} \n\t Pose: {}'.format(model,pose))
			br.sendTransform(
				(pose.position.x, pose.position.y, pose.position.z), # Position
				(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), # Orientation
				rospy.Time.now(), # Time
				'base_link', # Child Frame
				'odom') # Parent Frame



if __name__ == '__main__':
	rospy.init_node('gazebo_model_states_to_baselink_tf_broadcaster')
	rospy.Subscriber(topic_name, ModelStates, handle_pose)
	rospy.spin()
