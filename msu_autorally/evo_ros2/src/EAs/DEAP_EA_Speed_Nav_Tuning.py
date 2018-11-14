#
# GAS 10-15-18


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

import copy


class DEAP_EA():
	def __init__(self, cmd_args):
		self.debug = cmd_args.debug
		
		# EA Params
		self.experiment_name = "nav_tuning_exp1"
		self.genome_size = 11
		self.tourn_size = 2
		self.pop_size = 50
		self.number_generations = 25
		
		
		starting_run_number = 2
		number_of_runs = 2
		
		#Running Params
		self.timeout = 500 * 1000

		# Socket Communication Params      
		self.ip_addr = '127.0.0.1'
		#self.ip_addr = '35.9.28.201'
		self.send_port = 5023
		self.recv_port = 5033

		# Email Notification Params
		self.email_receiver_list = ['glen.a.simon@gmail.com']
		
		# Multi-threading Params
		self.lock = threading.Lock()
		
		
		try:
			# Set up Socket communication
			self.set_up_sockets()
			
			print('About to start {} runs of experiment {}'.format(number_of_runs - starting_run_number+1,self.experiment_name))
			print('\tStarting at run: {}'.format(starting_run_number))
			raw_input("Press enter to run")
			
			for i in range(starting_run_number, number_of_runs+1):
				self.run_number = i
				
				# Set up for run
				self.detailed_log = dict()
				self.set_up_EA()
				self.set_up_dirs()
				
				print('\nAbout to start exp: {} \n\t run: {}'.format(self.experiment_name, self.run_number))			
				# Run
				self.start_time = time.time()
				self.run()
				
				# Final logging and notifications
				self.create_run_log()
				print('Run log created.')
				self.write_run_log()
				print('Log saved!')
				
				self.email_notification(json.dumps(self.email_log, indent=2))
				print('Email notification sent!')
		finally:

			self.socket.close()
			self.receiver.close()
			self.context.destroy()
		
	
	
	### Run the EA ###
	def run(self):
		self.population = self.toolbox.population(n=self.pop_size)
		self.history.update(self.population)

		self.ending_pop, self.summary_log = self.eaSimpleCustom(cxpb=0.5, mutpb=0.2)
		self.end_time = time.time()
		print('\n\nRun finished at: {} \n\tTaking: {} seconds'.format(datetime.datetime.now(), (self.end_time - self.start_time)))
		

	def eaSimpleCustom(self, cxpb, mutpb):
		population = self.population
		toolbox = self.toolbox
		stats=self.stats
		halloffame=self.hof
		ngen = self.number_generations
		
		self.gen = 0
		
		logbook = tools.Logbook()
		logbook.header = ['gen', 'nevals'] + (stats.fields if stats else [])

		# Evaluate the individuals with an invalid fitness
		invalid_ind = [ind for ind in population if not ind.fitness.valid]
		
		fitnesses = self.custom_eval_fit_mapping(invalid_ind)
		
		
		for ind, fit in zip(invalid_ind, fitnesses):
			ind.fitness.values = fit

		if halloffame is not None:
			halloffame.update(population)

		record = stats.compile(population) if stats else {}
		logbook.record(gen=0, nevals=len(invalid_ind), **record)

		# Begin the generational process
		for gen in range(1, ngen + 1):
			print('\n\n###### GENERATION {} ######'.format(gen))
			self.gen = gen
			
			# Select the next generation individuals
			offspring = toolbox.select(population, len(population))

			# Vary the pool of individuals
			offspring = algorithms.varAnd(offspring, toolbox, cxpb, mutpb)

			# Evaluate the individuals with an invalid fitness
			invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
			
			
			
			#fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
			fitnesses = self.custom_eval_fit_mapping(invalid_ind)
			
			
			for ind, fit in zip(invalid_ind, fitnesses):
				ind.fitness.values = fit

			# Update the hall of fame with the generated individuals
			if halloffame is not None:
				halloffame.update(offspring)

			# Replace the current population by the offspring
			population[:] = offspring

			# Append the current generation statistics to the logbook
			record = stats.compile(population) if stats else {}
			logbook.record(gen=gen, nevals=len(invalid_ind), **record)

		return population, logbook
			
	
	def evaluate_result(self, ind, result):
		print('Recv\'d Result')
		df = pd.DataFrame.from_dict(dict(result))
		
		# Get rid of first all 0s entry
		df = df.truncate(before=2)
		
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
	
	
	def custom_eval_fit_mapping(self, individuals):
		fitnesses = [float('Inf') for i in range(len(individuals))]
		print('{} individuals need to be evaluated.'.format(len(individuals)))
		# Send out individuals to be evaluated
		for ind in individuals:
			self.socket.send_json(ind)
				
		# While there are any fitnesses not yet evaluated, recv results from socket.
		num_evaluated = 0
		while any(fit == float('Inf') for fit in fitnesses):
			print('\nWaiting results')
			
			socks = dict(self.poller.poll(self.timeout))
			if socks:
				if socks.get(self.receiver) == zmq.POLLIN:
					return_data = dict(self.receiver.recv_json(zmq.NOBLOCK))
					#data = json.loads(self.receiver.recv_json(zmq.NOBLOCK))
			else:
				print('Timeout on receiver socket occured!')
				non_resolved_ind = fitnesses.index(float('Inf'))
				self.socket.send_json(individuals[non_resolved_ind])
				continue
			
			
			ind = list(return_data['Genome'])
			result = dict(return_data['Result'])
			fitness = self.evaluate_result(ind, result)
			
			# Write the fitness into the spot in fitnesses corresponding to the position of the individual
			try:
				index = individuals.index(ind)
			except:
				print('Received individual from previous generation!')
				continue
			
			# The populaton can have multiple copies of an individual and thus we need to assign each a unique fitness
			#print(ind)
			while True:
				
				if fitnesses[index] == float('Inf'):
					fitnesses[index] = fitness
					break
				else:
					if index > len(individuals):
						print('Error - index out of range!')
						exit()
					else:
						old_index = index
						index = individuals[old_index+1:].index(ind)
						index += old_index + 1
			num_evaluated += 1
			#print('fitnesses: {}'.format(fitnesses))
			print('\n\n{}/{} individuals evaluated'.format(num_evaluated, len(individuals)))
		return fitnesses
		
		
	
	
	
	### Set up individual's shape, fitness function, and EA operators ###
	def set_up_EA(self):
		creator.create("FitnessMax", base.Fitness, weights=(1.0, ))
		creator.create("Individual", list, fitness=creator.FitnessMax)

		self.toolbox = base.Toolbox()
		self.toolbox.register("attr_float", random.random)
		self.toolbox.register("individual", tools.initRepeat, creator.Individual,
				 self.toolbox.attr_float, n=self.genome_size)
		self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)
		
		# Set up Evo Algo operators
		self.toolbox.register("mate", tools.cxTwoPoint)
		self.toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
		self.toolbox.register("select", tools.selTournament, tournsize=self.tourn_size)
		
		# Set up EA history
		self.history = tools.History()
		
		# Decorate the variation operators
		self.toolbox.decorate("mate", self.history.decorator)
		self.toolbox.decorate("mutate", self.history.decorator)
		
		# Set up hall of fame and statistics
		self.hof = tools.HallOfFame(10)
		self.stats = tools.Statistics(lambda ind: ind.fitness.values)
		self.stats.register("avg", np.mean)
		self.stats.register("std", np.std)
		self.stats.register("min", np.min)
		self.stats.register("max", np.max)


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
		
		# Setup ZMQ poller
		self.poller = zmq.Poller()
		self.poller.register(self.receiver, zmq.POLLIN)
		
		print('Sending Connection: {}'.format('tcp://{}:{}'.format(self.ip_addr, self.send_port)))
		print('Result Connection: {}'.format('tcp://{}:{}'.format(self.ip_addr, self.recv_port)))

	   
	
	
	### Email notification ###
	def email_notification(self, results):
		sender = 'evo.ros2.result.sender@gmail.com'
		msg = MIMEText(str(results))
		msg['From'] = sender
		msg['To'] = str(self.email_receiver_list)
		msg['Subject'] = 'EA Results'
	
		HOST = "smtp.gmail.com"
		PORT = "587"
		SERVER = smtplib.SMTP()
		SERVER.connect(HOST, PORT)
		USER = "evo.ros2.result.sender@gmail.com"
		PASSWD = "evoRos2Rocks"
		SERVER.ehlo()
		SERVER.starttls()
		SERVER.login(USER,PASSWD)

		#SERVER.set_debuglevel(True)
		SERVER.sendmail(sender, self.email_receiver_list, msg.as_string())
		SERVER.quit()

	
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
		
		self.hof_fitnesses = list()
		for ind in self.hof:
			self.hof_fitnesses.append(ind.fitness.values)
		
		self.run_log['experiment_name'] = self.experiment_name
		self.run_log['run_number'] = self.run_number
		self.run_log['run_date'] = str(datetime.datetime.now())
		self.run_log['running_time'] = (self.end_time - self.start_time)
		self.run_log['best_ind'] = self.hof[0]
		self.run_log['best_ind_fitness'] = self.hof[0].fitness.values
		self.run_log['summary_log'] = self.summary_log
		self.email_log = copy.deepcopy(self.run_log)
		self.run_log['hall_of_fame'] = list(self.hof)
		self.run_log['hall_of_fame_fitnesses'] = self.hof_fitnesses
		self.run_log['detailed_log'] = self.detailed_log
		
		
	### Write the run log to disk ###
	def write_run_log(self):
		with open(self.run_directory + '/log' + '.json', 'w+') as outfile:
			json.dump(self.run_log, outfile, indent=2)
		details_of_best_ind = self.detailed_log[str(self.hof[0])]
		df = pd.read_json(details_of_best_ind['dataFrame'], orient='columns')
		df = df.sort_index()
		df.to_csv(self.run_directory + '/best_ind_details.csv')
		
		

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Front end DEAP EA for nav tuning Study')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

node = DEAP_EA(cmd_args = args)
