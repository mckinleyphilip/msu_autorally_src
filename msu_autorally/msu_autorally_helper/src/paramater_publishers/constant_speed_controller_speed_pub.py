#!/usr/bin/env python
#
# Constant speed controller speed Pub
#
#	This creates a ROS publisher to enable that will speed speed commands (as m/s) to the constant speed controller
#
# GAS 2018-02-28

import rospy
import argparse
import time

from std_msgs.msg import Float64

DEFAULT_SPEED = 3.0

# Parse arguments
parser = argparse.ArgumentParser(description='This node is responsible for publishing the speed (m/s) for the constant speed controller')
parser.add_argument('--speed', type=float, default=DEFAULT_SPEED, help='The speed for the AutoRally rover in m/s')
args, unknown = parser.parse_known_args() # Only parse arguments defined above

pub = rospy.Publisher('/constantSpeedController/speedCommand', Float64, queue_size=10)
rospy.init_node('speed_controller_speed_pub')
r = rospy.Rate(10) # 10hz


RUNNING_SPEED = args.speed * 2.5

t_end = time.time() + 14
while time.time() < t_end:
	print(args.speed)
	pub.publish(args.speed)
	time.sleep(0.1)

while not rospy.is_shutdown():
	print(RUNNING_SPEED)
	pub.publish(RUNNING_SPEED)
	r.sleep()
