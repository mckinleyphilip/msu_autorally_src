#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('kinect_tf_broadcaster')
	b1 = tf.TransformBroadcaster()
	b2 = tf.TransformBroadcaster()
	b3 = tf.TransformBroadcaster()
	b4 = tf.TransformBroadcaster()
	b5 = tf.TransformBroadcaster()
	x = 0.5
	z = 0.13
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		b1.sendTransform((x, 0.0, z),
						(0.0, 0.0, 0.0, 1.0),
						rospy.Time.now(),
						"autorally_platform/camera_rgb_frame", # child
						"base_link") # Parent
		b2.sendTransform((x, 0.0, z),
						(0.0, 0.0, 0.0, 1.0),
						rospy.Time.now(),
						"autorally_platform/camera_depth_frame",
						"autorally_platform/camera_rgb_frame")
		b3.sendTransform((x, 0.0, z),
						(0.0, 0.0, 0.0, 1.0),
						rospy.Time.now(),
						"autorally_platform/camera_link",
						"autorally_platform/camera_rgb_frame")
		b4.sendTransform((x, 0.0, z),
						(0.0, 0.0, 0.0, 1.0),
						rospy.Time.now(),
						"autorally_platform/camera_rgb_optical_frame",
						"autorally_platform/camera_rgb_frame")
		b5.sendTransform((x, 0.0, z),
						(0.0, 0.0, 0.0, 1.0),
						rospy.Time.now(),
						"autorally_platform/camera_depth_optical_frame",
						"autorally_platform/camera_depth_frame")
		rate.sleep()
