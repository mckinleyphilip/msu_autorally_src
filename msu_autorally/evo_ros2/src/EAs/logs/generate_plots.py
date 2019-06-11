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
		self.base_dir = cmd_args.base_dir
                if self.base_dir and self.base_dir[-1] == '/':
                    self.base_dir = self.base_dir[:-1]

                self.run_number = 1
		self.run_directory = '{}/run{}'.format(
                    self.base_dir, self.run_number)
		filename = '{}/log.json'.format(self.run_directory)

                print('attmepting to open %s...' % filename)
                while os.path.isfile(filename):
		    print('Processesing run {}'.format(self.run_number))

                    self.import_file(filename)
                    self.create_run_plots()

                    self.run_number += 1
		    
                    self.run_directory = '{}/run{}'.format(
                        self.base_dir, self.run_number)
		    filename = '{}/log.json'.format(self.run_directory)
                    print('attmepting to open %s...' % filename)
		
		
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
			

	### Create run plots ###
	def create_run_plots(self):
		
		# Fitnesses of Population over Generations
		gen = self.summary_log_dict['gen']
		avg_fit = self.summary_log_dict['avg']
		best_fit = self.summary_log_dict['min']

		print('Preparing plot 1...')
		fig, ax1 = plt.subplots()
		line1 = ax1.plot(gen, best_fit, "b-", label="Minimum (Best) Fitness")
		ax1.set_xlabel("Generation")
		ax1.set_ylabel("Fitness")
		line2 = ax1.plot(gen, avg_fit, "r-", label="Average Fitness")
		lns = line1 + line2
		labs = [l.get_label() for l in lns]
		ax1.legend(lns, labs, loc="best")
		ax1.set_title('Run {} Fitnesses over Generations'.format(self.run_number))

		print('Saving plot 1')
		fig.savefig('{}/fitness_best_avg_graph.png'.format(self.run_directory), bbox_inches='tight')
		
		# Speed Signal of Best Ind
		print('Preparing plot 2...')
		details_of_best_ind = self.detailed_log[str(self.hof[0])]
		self.df = pd.read_json(details_of_best_ind['dataFrame'], orient='columns')
		self.df = self.df.sort_index()
		self.df = self.df[['Actual Speed', 'Goal Speed', 'Error', 'Time']]
		#print(list(self.df.columns))
		ax2 = self.df.plot(x='Time')
		#ax2.legend(['Actual Speed', 'Error', 'Goal Speed'], loc="upper right")
		ax2.legend(['Actual Speed', 'Goal Speed', 'Error'], bbox_to_anchor=(1.0, 0.8))
		plt.ylabel('meters / second')
		ax2.set_title('Run {} Best Individuals Speed Signal'.format(self.run_number))
		plt.text(0.5, 0.03, 'Fitness: {}'.format(self.best_fitness), horizontalalignment='center', verticalalignment='center', transform=ax2.transAxes)
		fig2 = ax2.get_figure()
		plt.margins(x = 0.0, y = 0.1)
		print('Saving plot 2')
		fig2.savefig('{}/best_ind_speed_plot.png'.format(self.run_directory), bbox_inches='tight')
		
		# Just best fitness over generations
		print('Preparing plot 3...')
		fig3, ax3 = plt.subplots()
		line1 = ax3.plot(gen, best_fit, "b-", label="Minimum (Best) Fitness")
		ax3.set_xlabel("Generation")
		ax3.set_ylabel("Fitness")
		lns = line1
		labs = [l.get_label() for l in lns]
		ax3.legend(lns, labs, loc="best")
		ax3.set_title('Run {} Fitnesses over Generations'.format(self.run_number))

		print('Saving plot 3')
		fig3.savefig('{}/fitness_best_graph.png'.format(self.run_directory), bbox_inches='tight')
	
	
	def print_genealogy_tree(self):
		graph = networkx.DiGraph(self.history.genealogy_tree)
		graph = graph.reverse()     # Make the grah top-down
		print(graph)
		colors = [self.toolbox.evaluate(self.history.genealogy_history[i])[0] for i in graph]
		networkx.draw(graph, node_color=colors)
		plt.show()
	
	
if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Plot generator for PID Study')
	parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal.')
        parser.add_argument('-p', '--path', dest='base_dir', type=str,
                help='A base directory to use to look for run files')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above

	node = plot_generator(cmd_args = args)
