#!/usr/bin/env python
#
# Example monitor
#
#	Exampale Monitor Node
#
# GAS 2018-08-29

import rospy
import argparse
import numpy as np

from evo_ros2.msg import LogEvent

from std_msgs.msg import Float64


class ExmapleMonitor():
	def __init__(self, cmd_args):
		
		# Init Node
		self.node_name = 'ExmapleMonitor'
		rospy.init_node(self.node_name, anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Write command line arguments into class variables
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		
		# Sent up pub/sub commmunications
		self.set_up_monitored_topic()
		self.set_up_log_event_communications()
		
		# Set up other member variables
		self.last_msg = ''
		self.data_list = []
		self.time_list = []
		
		
		rospy.spin()
	
	
	def set_up_monitored_topic(self):
		topic_name = ''
		self.monitored_topic = rospy.Subscriber(topic_name, Float64, self.monitored_topic_cb)
		
	
	def monitored_topic_cb(self, msg):
		self.last_msg = msg.data
		
		
	def set_up_log_event_communications(self):
		# Evo ROS logging topics
		self.log_event_topic = rospy.Subscriber(rospy.get_param('/EVO_ROS_LOG_EVENT_TOPIC'), LogEvent, self.log_event)
		self.log_collection_topic = rospy.Publisher(rospy.get_param('/EVO_ROS_LOG_COLLECTION_TOPIC'), LogEvent, queue_size=10)
		
	def log_event(self,msg):
		
		if msg.event == 0: # Trigger
			self.data_list.append(self.last_msg)
			self.time_list.append(msg.time)
		elif msg.event == 1: # Request
			send_logged_data()
			
	
	def send_logged_data(self):
		msg = LogEvent()
		msg.time = rospy.get_rostime()
		msg.sender = self.node_name
		msg.event = 2 # Reply
		msg.log_data = self.data_list
		self.log_collection_topic.publish(msg)
		
		
	def on_shutdown(self):
		pass
		
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description="Exampale Monitor Node")
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	
	
	try:
		node = ExmapleMonitor(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
