#
# GAS 11-06-18


# For command line argument parsing
import argparse

# For communication with evo_ros2
import zmq

# For data / fitness evaluation
import pandas as pd
from sklearn.metrics import mean_squared_error

# For DEAP
import random
from deap import creator, base, tools, algorithms
import numpy as np

# For email notifications
import smtplib
from email.mime.text import MIMEText

# For showing DEAP plots
import matplotlib.pyplot as plt
import networkx

# For EA running time
import time

# For creating directories and saving logs
import os
from os import listdir
from os.path import isfile, join
import json
import collections
import datetime

# For saving plots
import pickle

# For multiple evaluations at once
import threading

import pprint


class DEAP_EA():
	def __init__(self, cmd_args):
		self.debug = cmd_args.debug
		
		# EA Params
		self.experiment_name = "test"
		self.run_number = '1'
		self.ind = [0.3, 0.06, 0.1, 0.05, 0.01, 0.02, 0.1, 0.5, 0.1, 0.5, 0.1] #default
		
		
		# Socket Communication Params      
		self.ip_addr = '127.0.0.1'
		#self.ip_addr = '35.9.28.201'
		self.send_port = 5023
		self.recv_port = 5033

		# Email Notification Params
		self.email_receiver_list = ['glen.a.simon@gmail.com']
		
		# Multi-threading Params
		self.lock = threading.Lock()
		
		self.gen = 0
		
		try:
			 # Set up 
			self.detailed_log = dict()
			self.set_up_sockets()
			self.set_up_dirs()
			
			print('\nAbout to start exp: {} \n\t run: {}'.format(self.experiment_name, self.run_number))
			raw_input("Press enter to run")
		
			# Run
			self.run()
		finally:
			self.socket.close()
			self.receiver.close()
			self.context.destroy()
		
	
	
	### Run the EA ###
	def run(self):
		
		self.socket.send_json(self.ind)
		
		print('Waiting Result')

		return_data = dict(self.receiver.recv_json())
		ind = list(return_data['Genome'])
		result = dict(return_data['Result'])
		self.fitness = self.evaluate_result(ind, result)
		
		# Final logging and notifications
		self.create_run_log()
		print('Run log created.')
		self.write_run_log()
		print('Log saved!')
	
	


	
	def evaluate_result(self, ind, result):
		print('Recv\'d Result')
		df = pd.DataFrame.from_dict(dict(result))
		
		# Get rid of first all 0s entry
		df = df.truncate(before=2)
		
		self.df = df
		
		# Speeds
		avg_speed = df['Actual Speed'].mean()
		max_speed = df['Actual Speed'].max()
		norm_avg_speed = avg_speed / 15
		norm_max_speed = max_speed / 15
		
		# Waypoints
		waypoints_achieved = df['Goal Status'].max()
		norm_wp = waypoints_achieved / 4.0
		
		if norm_wp == 1.0:
		    # Time
		    time_elapsed = df['Time'].max()
		    norm_time_elapsed =  15.2 / time_elapsed
		
		    # Distance
		    dx = np.diff(df['Pos X'])
		    dy = np.diff(df['Pos Y'])
		    d = np.hypot(dx, dy)
		    d = np.insert(d,0,0)
		    total_distance = np.sum(d)
		    norm_distance = 152.67 / total_distance
		
		else:
		    # Time
		    time_elapsed = df['Time'].max()
		    norm_time_elapsed =  time_elapsed / 300.0
		
		    # Distance
		    dx = np.diff(df['Pos X'])
		    dy = np.diff(df['Pos Y'])
		    d = np.hypot(dx, dy)
		    d = np.insert(d,0,0)
		    total_distance = np.sum(d)
		    norm_distance = (total_distance / 222) / 2
		
		raw_fitness = [avg_speed, max_speed, waypoints_achieved, time_elapsed, total_distance]
		norm_fitness = [norm_avg_speed * 2, norm_max_speed * 2, norm_wp * 3, norm_time_elapsed * 1, norm_distance * 1]
		total_fitness = sum(norm_fitness)
		
		print('Raw Fitness: {}'.format(raw_fitness))
		print('Total Fitness: {}'.format(total_fitness))
		
		# add individual to detailed log
		if str(ind) not in self.detailed_log.keys():
			self.detailed_log[str(ind)] = {
				"gen": self.gen,
				"fitness": total_fitness,
				"rawFitness": raw_fitness,
				"dataFrame": df.to_json()
			}

		return (total_fitness, )
			
		

	### Set up communication sockets ###
	def set_up_sockets(self):
		#Initialize the socket for data
		
		# Setup the socket to send data out on.
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.PUSH)
		#socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
		self.socket.bind('tcp://{}:{}'.format(self.ip_addr, self.send_port))

		# Setup the socket to read the responses on.
		self.receiver = self.context.socket(zmq.PULL)
		self.receiver.bind('tcp://{}:{}'.format(self.ip_addr, self.recv_port))
		
		print('Sending Connection: {}'.format('tcp://{}:{}'.format(self.ip_addr, self.send_port)))
		print('Result Connection: {}'.format('tcp://{}:{}'.format(self.ip_addr, self.recv_port)))

	   
	
	### Create Experiment and Run directories if not already made ###
	def set_up_dirs(self):
		
		# Make sure that a directory for this experiment has been made
		self.experiment_directory = 'logs/' + self.experiment_name
		if not os.path.isdir(self.experiment_directory):
			print('No directory for experiment...Creating now.')
			os.makedirs(self.experiment_directory)
			
		# Make sure that a directory for this run has been made
		self.run_directory = self.experiment_directory + '/run' + str(self.run_number)
		if not os.path.isdir(self.run_directory):
			print('No directory for run...Creating now.')
			os.makedirs(self.run_directory)
			
	### Create the run log ###
	def create_run_log(self):
		self.run_log = collections.OrderedDict()
		
		self.run_log['experiment_name'] = self.experiment_name
		self.run_log['run_number'] = self.run_number
		self.run_log['run_date'] = str(datetime.datetime.now())
		self.run_log['best_ind'] = self.ind
		self.run_log['best_ind_fitness'] = self.fitness
		self.run_log['detailed_log'] = self.df.to_json()
		
		
	### Write the run log to disk ###
	def write_run_log(self):
		with open(self.run_directory + '/log' + '.json', 'w+') as outfile:
			json.dump(self.run_log, outfile, indent=2)
		self.df.to_csv(self.run_directory + '/best_ind_details.csv')
		
		
		
	
	

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Front end DEAP EA for PID Study')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	node = DEAP_EA(cmd_args = args)
