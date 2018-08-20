#!/usr/bin/env python
#
# Evo-ROS2 Transporter
#
#	Base ROS node
#
# GAS 2018-08-15

import rospy
import argparse
import numpy as np

import zmq

class Transporter():
	def __init__(self, cmd_args):
		
		# Init Node
		rospy.init_node('transporter_node',anonymous=False)
		
		# Register shutdown hook
		rospy.on_shutdown(self.on_shutdown)
		
		# Get relevent parameters
		self.debug = (cmd_args.debug or rospy.get_param('/DEBUG',False))
		if not rospy.has_param('GENOME_MAPPING'):
			rospy.logerr('Transporter: No genome mapping found!')
			rospy.signal_shutdown('Transporter: No genome mapping found!')
			return
		else:
			self.genome_mapping = rospy.get_param('GENOME_MAPPING')
		
		
		# Set up socket communication using ZMQ
		self.server_ip_addr = rospy.get_param('SERVER_IP_ADDR','127.0.0.1')
		self.genome_port = rospy.get_param('GENOME_PORT',5557)
		self.result_port = rospy.get_param('RESULT_PORT',5558)
		
		context = zmq.Context()
		self.genome_receiver = context.socket(zmq.PULL)
		self.genome_receiver.connect('tcp://{}:{}'.format(self.server_ip_addr, self.genome_port))
		self.result_sender = context.socket(zmq.PUSH)
		self.result_sender.connect('tcp://{}:{}'.format(self.server_ip_addr, self.result_port)) 
		
		
		if self.debug:
			rospy.loginfo('Genome Connection: {}'.format('tcp://{}:{}'.format(self.server_ip_addr, self.genome_port)))
			rospy.loginfo('Result Connection: {}'.format('tcp://{}:{}'.format(self.server_ip_addr, self.result_port)))
		
		self.current_genome = dict()
		
		self.run()
		
		
	
	def run(self):
		while not rospy.is_shutdown():
			self.recv_genome(self.genome_receiver.recv_json())
			#self.write_genome_to_ros_params()

	
	
	def recv_genome(self, raw_genome):
		if raw_genome == 'end':
			rospy.logerr('Transporter: Ending signal has been received from server')
			rospy.signal_shutdown('Transporter: Received ending signal from server')
			return
		else:
			self.parse_genome(raw_genome)
	
	
	def parse_genome(self, raw_genome):
		for index, element in enumerate(self.genome_mapping):
			self.current_genome[element] = raw_genome[index]
		
		if self.debug:	
			rospy.loginfo(self.current_genome)
			
	def write_genome_to_ros_params(self):
		for element in self.current_genome:
			#if not rospy.has_param(element):
			rospy.set_param(element, self.current_genome[element])
			
		
	def on_shutdown(self):
		pass
		
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Evo-ROS2 Transporter')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = Transporter(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
