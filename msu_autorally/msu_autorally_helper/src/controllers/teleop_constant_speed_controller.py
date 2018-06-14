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

STARTING_SPEED = 3.0
RUNNING_SPEED = 5.0

# Parse arguments
parser = argparse.ArgumentParser(description='This node is responsible for publishing the speed (m/s) for the constant speed controller')
parser.add_argument('--speed', type=float, default=RUNNING_SPEED, help='The speed for the AutoRally rover in m/s')
args, unknown = parser.parse_known_args() # Only parse arguments defined above

pub = rospy.Publisher('/constantSpeedController/speedCommand', Float64, queue_size=10)
rospy.init_node('speed_controller_speed_pub')
r = rospy.Rate(10) # 10hz




t_end = time.time() + 14
while time.time() < t_end:
	print(args.speed)
	pub.publish(STARTING_SPEED)
	time.sleep(0.1)

while not rospy.is_shutdown():
	print(RUNNING_SPEED)
	pub.publish(args.speed)
	r.sleep()
