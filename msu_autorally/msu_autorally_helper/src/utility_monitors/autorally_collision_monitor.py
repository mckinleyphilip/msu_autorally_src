#!/usr/bin/env python
#
# Autorally Collision Monitor
#
#	This creates a ROS node which listens to the contact sensors on the autorally gazebo model and tracks collisions
#
# GAS 2018-03-14

import rospy
import argparse
import numpy as np
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool
from std_msgs.msg import String
from copy import deepcopy
import json


class CollisionEvent():
	internal_actor = ""
	external_actor = ""
	time = 0
	impact_force = [0,0,0]
	
	def display(self):
		print('Collision Event\n'
			'\t Internal Actor: {}\n'
			'\t External Actor: {}\n'
			'\t Time: {}\n'
			'\t Impact Force: {}\n'
			.format(self.internal_actor, self.external_actor, self.time, self.impact_force))
			

class ContactMonitorNode():
	def __init__(self):
		# Create a list to store collision events
		self.collision_events = []
		
		# Configure Publishers
		self.collision_events_pub = rospy.Publisher('/util_comms/collision_events', String, queue_size=10)


		# Configure Subscribers
		self.chassis_contact_sub = rospy.Subscriber("/autorally_platform/chassis_contact_sensor_state", ContactsState,self.chassis_contact_test)
		self.collision_events_request_sub = rospy.Subscriber('/util_comms/collision_events_request', Bool,self.on_collision_events_request)
		
		rospy.on_shutdown(self.on_shutdown)
		rospy.spin()
		
	def on_shutdown(self):
		print('\n\n {} Collision events recorded.'.format(len(self.collision_events)))
		#for event in self.collision_events:
		#	event.display()
		
	
	def on_collision_events_request(self, msg):
		if msg.data == True:
			out_msg = String
			#print('\n\n Collision events recorded: {}'.format(self.collision_events))
			out_msg.data = json.dumps(self.collision_events)
			out_msg.data = str(self.collision_events)
			#print('\n\n Encoded: {}'.format(out_msg.data))
			self.collision_events_pub.publish(out_msg)
			
		
		
	def chassis_contact_test(self, contacts):
		# If a contact state is detected, record it as a collision event and append it to the collision events list
		if contacts.states:
			# Each collision consists of multiple states, so we will record the forces acting on each state and average them for the total force of the collision
			x_forces = []
			y_forces = []
			z_forces = []
			for collision in contacts.states:
				x_forces.append(collision.total_wrench.force.x)
				y_forces.append(collision.total_wrench.force.y)
				z_forces.append(collision.total_wrench.force.z)
				
			
			new_collision = CollisionEvent()
			new_collision.impact_force = [np.mean(x_forces),np.mean(y_forces),np.mean(z_forces)]
			new_collision.time = contacts.header.stamp.secs
			
			# There is no garuntee on the ordering of the objects being collided, so we will search for "chassis" and assume that is hte internal actor
			new_collision.internal_actor = "Chassis"
			if "chassis" in collision.collision2_name:
				new_collision.external_actor = collision.collision1_name
			else:
				new_collision.external_actor = collision.collision2_name
			
			new_collision.display()
			self.collision_events.append(deepcopy(new_collision.__dict__))
			

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='This node is responsible for tracking collisions during simulation of the Autorally platform')
	#parser.add_argument('--speed', type=float, default=3.0, help='The speed for the AutoRally rover in m/s')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node('collision_monitor',anonymous=False)
	
	try:
		monitor = ContactMonitorNode()
	except rospy.ROSInterruptException: pass
	
	
