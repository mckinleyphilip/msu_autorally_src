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
import pprint

import zmq

from evo_ros2.msg import EvoROS2State

class Transporter():
	def __init__(self, cmd_args):
		
		# Init Node
		self.node_name = 'transporter_node'
		rospy.init_node(self.node_name, anonymous=False)
		
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
		
		self.context = zmq.Context()
		self.genome_receiver = self.context.socket(zmq.PULL)
		self.genome_receiver.connect('tcp://{}:{}'.format(self.server_ip_addr, self.genome_port))
		self.result_sender = self.context.socket(zmq.PUSH)
		self.result_sender.connect('tcp://{}:{}'.format(self.server_ip_addr, self.result_port)) 
		
		
		if self.debug:
			rospy.loginfo('Genome Connection: {}'.format('tcp://{}:{}'.format(self.server_ip_addr, self.genome_port)))
			rospy.loginfo('Result Connection: {}'.format('tcp://{}:{}'.format(self.server_ip_addr, self.result_port)))
		
		self.current_genome = dict()
		
		self.set_up_evo_ros2_communications()
		
		
		self.set_evo_ros2_state(0)

		rospy.spin()
		
	
	def send_result(self, msg):
		self.results = dict()
		self.results['result'] = list()
		self.results['genome'] = self.raw_genome
		self.results['metadata'] = self.metadata
		self.results['enki_genome'] = self.enki_genome
		for index, result in enumerate(msg.result):
			self.results['result'].append((msg.result[index].header, msg.result[index].data))
		
		if self.debug:
			rospy.loginfo('\n\n Transporter Result sent')
			print('Send Msg Contents:')
			for key in self.results.keys():
				print(key)
				if isinstance(self.results[key], (list,)):
					for item in self.results[key]:
						if isinstance(item, (list, )):
							print('\t{}...'.format(item[0:2]))
						elif isinstance(item, (tuple, )):
							print('\t{}...'.format(item[0]))
						else:
							print('\t{}'.format(item))
			
			#rospy.loginfo(self.results)
		
		#pp = pprint.PrettyPrinter(indent=4)
		#pp.pprint(self.results['result'])
		
		
		self.result_sender.send_json(self.results)
			
		
		
			
	def set_up_evo_ros2_communications(self):
		self.evo_ros2_comm_topic = rospy.get_param('EVO_ROS_COMM_TOPIC')
		self.evo_ros2_comm_pub = rospy.Publisher(self.evo_ros2_comm_topic, EvoROS2State, queue_size=10, latch = True)
		self.evo_ros2_comm_sub = rospy.Subscriber(self.evo_ros2_comm_topic, EvoROS2State, self.on_evo_ros2_state_change)
	
	
	def set_evo_ros2_state(self, new_state_value):
		rospy.set_param('evo_ros2_state', new_state_value)
		msg = EvoROS2State()
		msg.sender = self.node_name
		msg.state = new_state_value
		self.evo_ros2_comm_pub.publish(msg)	
		
	
	def on_evo_ros2_state_change(self, msg):
		if self.debug:
			rospy.logwarn('{} - In state: {}'.format(self.node_name, msg.state))
			
		if msg.state == 0:
			self.recv_genome(dict(self.genome_receiver.recv_json()))
			self.set_evo_ros2_state(1)
		
		if msg.state == 3:
			self.write_genome_to_ros_params()
			self.set_evo_ros2_state(4)
			
		if msg.state == 6:
			
			self.send_result(msg)
		
		if msg.state == 7:
			# Simulation reset is complete, ready for new genome
			self.set_evo_ros2_state(0)

	
	def recv_genome(self, msg):		
		
		# Parse received message
		if "genome" in msg:
			if msg['genome'] == 'end':
				rospy.logerr('Transporter: Ending signal has been received from server')
				rospy.signal_shutdown('Transporter: Received ending signal from server')
				return
			else:
				print('Genome received: {}'.format(msg['genome']))
				self.raw_genome = msg['genome']
				self.parse_genome(self.raw_genome)
		else:
			rospy.logerr('Transporter: Received message did not contain a genome!')
			self.set_evo_ros2_state(0)
		
		
		# Check if enki is being used as a front-end to manipulate world traits
		if "enki_genome" in msg:
			print('Found enki genome!')
			rospy.set_param('ENKI_INT', True)
			self.enki_genome = msg['enki_genome']
			
			# Check for multiple enki genomes and flatten them
			if any(isinstance(i, list) for i in msg['enki_genome']):
				number_genomes = len(msg['enki_genome'])
				print('number_genomes {}'.format(number_genomes))
				flat_list = [item for sublist in msg['enki_genome'] for item in sublist]
				#print(flat_list)
				rospy.set_param('ENKI_GENOME', flat_list)
				
			else:
				rospy.set_param('ENKI_GENOME', msg['enki_genome'])
			
		else:
			self.enki_genome = ''
			
			
		if "metadata" in msg:
			print('Found metadata {}'.format(msg['metadata']))
			self.metadata = msg['metadata']
		else:
			self.metadata = ''
					
			
			
			
	
	
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
		self.result_sender.close()
		self.genome_receiver.close()
		self.context.destroy()
		
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Evo-ROS2 Transporter')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	try:
		node = Transporter(cmd_args = args)
	except rospy.ROSInterruptException:
		pass
	
	
