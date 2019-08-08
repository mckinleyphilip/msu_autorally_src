#
# Used to generate PID-tuning experient plots
# GAS 10-16-18

# For command line argument parsing
import argparse

# For data / fitness evaluation
import pandas as pd

# For showing DEAP plots
import matplotlib.pyplot as plt

# For creating directories and saving logs
import os
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
        for run_folder in os.listdir('./'):
            run_number = int(run_folder[3:])
            if os.path.isfile('run%s/log.json' % run_number):
                print('Processesing run {}'.format(run_number))
                
                self.run_directory = 'run{}/'.format(run_number)
                filename = '{}log.json'.format(self.run_directory)
                
                self.import_file(filename)
                self.create_run_plots()
            else:
                print('Empty run folder found! \'%s\'' % run_folder)

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
        self.summary_log_dict['max'] = list()
        for gen in self.summary_log:
            gen = dict(gen)
            self.summary_log_dict['gen'].append(gen['gen'])
            self.summary_log_dict['avg'].append(gen['avg'])
            self.summary_log_dict['min'].append(gen['min'])
            self.summary_log_dict['max'].append(gen['max'])

    ### Create run plots ###
    def create_run_plots(self):
        # Fitnesses of Population over Generations
        gen = self.summary_log_dict['gen']
        avg_fit = self.summary_log_dict['avg']
        best_fit = self.summary_log_dict['max']

        print('Preparing plot 1...')
        fig, ax1 = plt.subplots()
        line1 = ax1.plot(gen, best_fit, "b-", label="Max Fitness")
        ax1.set_xlabel("Generation")
        ax1.set_ylabel("Fitness")
        line2 = ax1.plot(gen, avg_fit, "r-", label="Average Fitness")
        lns = line1 + line2
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, loc="best")
        ax1.set_title('Run {} Fitnesses over Generations'.format(self.run_number))

        print('Saving plot 1')
        fig.savefig('{}/fitness_best_avg_graph.png'.format(self.run_directory), bbox_inches='tight')

        # Just best fitness over generations
        print('Preparing plot 3...')
        fig3, ax3 = plt.subplots()
        line1 = ax3.plot(gen, best_fit, "b-", label="Max Fitness")
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
    args, unknown = parser.parse_known_args() # Only parse arguments defined above

    node = plot_generator(cmd_args = args)
