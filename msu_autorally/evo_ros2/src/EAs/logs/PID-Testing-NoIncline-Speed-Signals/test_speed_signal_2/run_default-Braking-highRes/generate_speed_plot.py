#
# Used to generate PID-tuning experient plots
# GAS 10-16-18

# For command line argument parsing
import argparse

# For data / fitness evaluation
import pandas as pd
from sklearn.metrics import mean_squared_error

# For showing DEAP plots
import matplotlib.pyplot as plt
import networkx

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
from pprint import pprint


class plot_generator():
	
	def __init__(self, cmd_args):
		self.debug = cmd_args.debug
		
		self.title = 'C0 - Braking - Test 2 '
		self.run_directory = './'
		filename = '{}log.json'.format(self.run_directory)
	
		self.import_file(filename)
		self.create_run_plots()
		
		
		
		
	# Import log from file and unpack it into member variables ###
	def import_file(self, filename):
		with open(filename) as f:
			self.run_log = json.load(f)
			
		self.experiment_name = self.run_log['experiment_name']
		self.run_number = self.run_log['run_number']
		self.run_date = self.run_log['run_date']
		self.best_ind = self.run_log['best_ind']
		self.best_fitness = self.run_log['best_ind_fitness']
		self.detailed_log = self.run_log['detailed_log']
			

	### Create run plots ###
	def create_run_plots(self):
		
		# Speed Signal of Best Ind
		print('Preparing plot...')
		plt.rcParams.update({'font.size': 16})
		self.df = pd.read_json(self.detailed_log, orient='columns')
		self.df = self.df.sort_index()
		self.df = self.df[['Actual Speed', 'Goal Speed', 'Error', 'Time']]
		#print(list(self.df.columns))
		ax2 = self.df.plot(x='Time')
		#ax2.legend(['Actual Speed', 'Error', 'Goal Speed'], loc="upper right")
		ax2.legend(['Actual Speed', 'Goal Speed', 'Error'], bbox_to_anchor=(1.0, 0.8))
		plt.ylabel('Meters / Second')
		ax2.set_title('{} Speed Signal'.format(self.title))
		plt.text(0.5, 0.03, 'Fitness: {}'.format(self.best_fitness), horizontalalignment='center', verticalalignment='center', transform=ax2.transAxes)
		fig2 = ax2.get_figure()
		plt.margins(x = 0.0, y = 0.1)
		
		print('Saving plot')
		fig2.savefig('{}/formatted_best_ind_speed_plot.png'.format(self.run_directory), bbox_inches='tight')
		plt.show()
		
	
	
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Plot generator for PID Study')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	node = plot_generator(cmd_args = args)
