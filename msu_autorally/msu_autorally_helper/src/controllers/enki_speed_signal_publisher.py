#!/usr/bin/env python
#
# Speed Signal Publisher
#
#	This creates a ROS publisher that will post speed commands (as m/s) to the constant speed controller
#
# GAS 2018-06-28

import rospy
import argparse
import time
import math

from std_msgs.msg import Float64


class SpeedSignalPublisherNode():
	def __init__(self, cmd_args):
		
		# Init Node
		self.node_name = 'speed_signal_publisher_node'
		rospy.init_node(self.node_name, anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		self.running = True
		self.speed_pub = rospy.Publisher('/constantSpeedController/speedCommand', Float64, queue_size=10)
		self.sleep_rate = rospy.Rate(10) # Hz
		
		# Get enki speed signal
		time_step = 0.1
		if rospy.has_param('ENKI_INT') and rospy.get_param('ENKI_INT') and rospy.has_param('ENKI_GENOME'):
			self.speed_signal = rospy.get_param('ENKI_GENOME')
		
		# Rospy returns 0 if the call to get_time() does not receive a value from /clock before the timeout period
		#	This happens commonly on slower than real time simulation, thus loop until non-zero value is returned
		self.start_time = 0
		while self.start_time == 0:
			self.start_time = rospy.get_time()

		
		while self.running:
			self.speed_pub.publish(self.enki_speed_signal_function(time_step))
			self.sleep_rate.sleep()
			
	# Publishes elements from the enki genome as the speed setting for every milisecond
	def enki_speed_signal_function(self, time_step):
		current_time = 0
		while current_time == 0:
			current_time = rospy.get_time()
		time = (current_time - self.start_time)
		
		rounding_pos = int(math.log10(1/time_step))
		index = int(round(time, rounding_pos) * (1/time_step))
		
		if index < len(self.speed_signal):
			return self.speed_signal[index]
		else:
			self.running = False
			return 0
		

	def on_shutdown(self):
		pass
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Empty ROS node')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = SpeedSignalPublisherNode(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
