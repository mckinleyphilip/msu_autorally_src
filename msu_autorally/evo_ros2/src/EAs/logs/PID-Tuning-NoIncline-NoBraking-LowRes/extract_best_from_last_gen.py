#
# Extract the best individual from the last generation
# GAS 10-16-18

# For command line argument parsing
import argparse

# For data / fitness evaluation
import pandas as pd

# For showing DEAP plots
import matplotlib.pyplot as plt

# For creating directories and saving logs
import os
from os import listdir
from os.path import isfile, join
import json
import collections
import datetime

# For saving plots
import pickle

# for debugging import
import pprint


import re



class plot_generator():
	
	def __init__(self, cmd_args):
		self.debug = cmd_args.debug
		
		
		run_multiple = True
		self.number_runs = 40
		
		
		if run_multiple:
			for i in range(1,self.number_runs+1):
				
				print('Processesing run {}'.format(i))
				self.run_number = i
			
				self.run_directory = 'run{}/'.format(self.run_number)
				filename = '{}log.json'.format(self.run_directory)
			
				self.import_file(filename)
				self.create_csv_of_best()
		else:
			self.run_number = 1
			
			self.run_directory = 'run{}/'.format(self.run_number)
			filename = '{}log.json'.format(self.run_directory)
		
			self.import_file(filename)
			self.create_csv_of_best()
		
		
	def create_csv_of_best(self):
		
		highest_gen = 0
		for ind in self.detailed_log:
			if self.detailed_log[ind]['gen'] > highest_gen:
				highest_gen = self.detailed_log[ind]['gen']
		
		print('{} generations found'.format(highest_gen))
		
		last_gen = list()
		for ind in self.detailed_log:
			if self.detailed_log[ind]['gen'] == highest_gen:
				last_gen.append(ind)
				
		best_fitness = 999
		best_ind = list()
		for ind in last_gen:
			if self.detailed_log[ind]['fitness'] < best_fitness:
				
				best_ind = ind.split(',')
				best_fitness = self.detailed_log[ind]['fitness']
		
		for i, num in enumerate(best_ind):
			num = re.sub('[\[\]]', '', num)
			best_ind[i] = float(num)
			
		print('\tBest ind : {} \n\t Best fit: {}'.format(best_ind, best_fitness))	
		
		best = dict()
		best['hall_of_fame'] = [best_ind]
		best['hall_of_fame_fitnesses'] = [best_fitness]
		best['run_number'] = self.run_number
		with open( self.run_directory + '/best_from_last_gen' + '.json', 'w+') as outfile:
			json.dump(best, outfile, indent=2)
			
				
		
			
		
			
		
	# Import log from file and unpack it into member variables ###
	def import_file(self, filename):
		with open(filename) as f:
			self.run_log = json.load(f)
			
		self.experiment_name = self.run_log['experiment_name']
		self.run_number = self.run_log['run_number']
		self.run_date = self.run_log['run_date']
		self.running_time = self.run_log['running_time']
		self.best_ind = self.run_log['best_ind']
		self.best_fitness = self.run_log['best_ind_fitness']
		self.summary_log = self.run_log['summary_log']
		self.hof = self.run_log['hall_of_fame']
		self.hof_fitnesses= self.run_log['hall_of_fame_fitnesses']
		self.detailed_log = self.run_log['detailed_log']
		
		self.summary_log_dict = dict()
		self.summary_log_dict['gen'] = list()
		self.summary_log_dict['avg'] = list()
		self.summary_log_dict['min'] = list()
		for gen in self.summary_log:
			gen = dict(gen)
			self.summary_log_dict['gen'].append(gen['gen'])
			self.summary_log_dict['avg'].append(gen['avg'])
			self.summary_log_dict['min'].append(gen['min'])
			



	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Plot generator for PID Study')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	node = plot_generator(cmd_args = args)
