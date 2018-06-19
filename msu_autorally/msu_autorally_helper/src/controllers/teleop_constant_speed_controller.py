#!/usr/bin/env python
#
# Constant speed controller speed Pub
#
#	This creates a ROS publisher to enable that will speed speed commands (as m/s) to the constant speed controller based off of input from the keyboard
#
# GAS 2018-06-14


import rospy
import argparse
import numpy as np
import curses
import os

from std_msgs.msg import Float64
from autorally_msgs.msg import wheelSpeeds

class keyboard_tele_op_node():
	def __init__(self):
		
		
		self.rate = rospy.Rate(10) # Publish at 10hz
		self.current_speed_setting = 0 # Current speed setting (m/s)
		self.speed_step_size = 0.5 # How much should the speed change per key press (m/s)
		self.current_speed_actual = 0 # How fast the autorally is currently moving (m/s)
		
		self.help_msg = """ Press 'w' to increase speed by : {} \n Press 's' to decrease speed by: {} \n Press '0' to stop. \n\n""".format(self.speed_step_size, self.speed_step_size)
		self.last_key_screen = ""
		
		# Set up key capturing
		self.win = curses.initscr()
		self.win.nodelay(True)
		self.key=""
		self.reset_window()
		
		
		# Configure Publishers
		self.speed_cmd_pub = rospy.Publisher('/tele_op_PID_speed_controller/speedCommand', Float64, queue_size=10)


		# Configure Subscribers
		self.wheel_speed_sub = rospy.Subscriber('/wheelSpeeds', wheelSpeeds, self.wheel_speeds_callback)
		
		rospy.on_shutdown(self.on_shutdown)
		
		self.run()
			
	
	def wheel_speeds_callback(self, data):
		self.current_speed_actual = (data.lfSpeed + data.rfSpeed) / 2.0
		self.reset_window()
		
		
		
	def run(self):
		while not rospy.is_shutdown():
			self.check_key()
			self.speed_cmd_pub.publish(self.current_speed_setting)
			self.rate.sleep()
			
	
	def reset_window(self):
		self.win.clear()
		self.win.addstr(self.help_msg)
		self.win.addstr(""" Current Actual Speed: {} \n\n """.format(self.current_speed_actual))
		self.win.addstr(self.last_key_screen)
		
		
	def check_key(self):
		try:
			self.key = self.win.getkey()
			
			
			self.last_key_screen = str(self.key)
			self.win.addstr(self.key)
			
			if str(self.key) == 'w' or str(self.key) == 'W':
				self.current_speed_setting += self.speed_step_size
				self.last_key_screen += '\n Inreasing speed. \n Current Speed Setting = {}\n'.format(self.current_speed_setting)
				#self.win.addstr('\n Inreasing speed. \n Current Speed Setting = {}\n'.format(self.current_speed_setting))
			
			if str(self.key) == 's' or str(self.key) == 'S':
				self.current_speed_setting -= self.speed_step_size
				self.last_key_screen += '\n Decreasing speed. \n Current Speed Setting = {}\n'.format(self.current_speed_setting)
				#self.win.addstr('\n Decreasing speed. \n Current Speed Setting = {}\n'.format(self.current_speed_setting))
				
			if str(self.key) == '0':
				self.current_speed_setting = 0
				self.last_key_screen += '\n Stopping. \n Current Speed Setting = {}\n'.format(self.current_speed_setting)
				#self.win.addstr('\n Stopping. \n Current Speed Setting = {}\n'.format(self.current_speed_setting))
					
			self.reset_window()	
				
			
		except Exception as e:
			# no input
			pass
				
			
		
	def on_shutdown(self):
		
		# Send a flag to tell the nodelet that it no longer needs to send chassisCommands
		self.current_speed_setting = -99
		self.speed_cmd_pub.publish(self.current_speed_setting)
		
		
		# Curses messes with the terminal printing and text will not display properly unless we reset the terminal state when we are done
		os.system('reset')
		pass
		
	

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Keyboard tele op node.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node('keyboard_tele_op_node',anonymous=False)
	
	try:
		node = keyboard_tele_op_node()
		curses.wrapper(node)
	except rospy.ROSInterruptException: pass
	
	


