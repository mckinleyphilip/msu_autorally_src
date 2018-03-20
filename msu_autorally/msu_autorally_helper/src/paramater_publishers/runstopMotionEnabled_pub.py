#!/usr/bin/env python
#
# Runstop Motion Enabled Pub
#
#	This creates a ROS publisher to enable the runstop command for the Autorally allowing the rover to start moving
#
# GAS 2018-02-28

import rospy
from std_msgs.msg import String
from autorally_msgs.msg import runstop

pub = rospy.Publisher('/runstop', runstop, queue_size=10)
rospy.init_node('runstopMotionEnabled_pub')
r = rospy.Rate(1) # 1hz

while not rospy.is_shutdown():
	msg = runstop()
	msg.sender = 'runstopMotionEnabled_pub'
	msg.motionEnabled = True
	pub.publish(msg)
	r.sleep()
