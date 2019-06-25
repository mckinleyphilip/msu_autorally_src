import signal

# For cmd line arg parsing
import argparse

# For communications with evo_ros2
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

# For EA running time
import time

# For creating directories and saving logs
import os, sys
from os import listdir
from os.path import isfile, join
import json
import collections
import datetime

import copy

class Nav_Tuning_DEAP_EA():
    def __init__(self, cmd_args):
        self.debug = cmd_args.debug
        
        # EA Params
        self.experiment_name = "nav_tuning_new_params_rand_dir"
        
        self.pop_size = 50
        self.num_generations = 25
        
        self.genome_size = 18 # could make this automatic...
        self.tourn_size = 2
        
        starting_run_number = 1
        num_runs = 1
        
        # Running Params
        self.timeout = 500 * 1000
        
        # Socket Communication Params
        self.ip_addr = '35.9.128.222'
        self.send_port = 5023
        self.recv_port = 5033
        
        # Email Notification Params
        self.email_reciever_list = ['fleckjo1@msu.edu']
        
        self.gen_start_time = 0
        self.gen_time_list = []

        try:
            self.setup_sockets()
            signal.signal(signal.SIGINT, self.shutdown_handler)
            
            print('About to start {} runs of experiment {}'.format(num_runs,
                self.experiment_name))
            print('Pop = {}, Generations = {}, IP addr = {}'.format(self.pop_size,
                self.num_generations, self.ip_addr))
            print('\tStarting at run #{}'.format(starting_run_number))
            raw_input('Press enter to run')
            
            for i in range(starting_run_number, starting_run_number + num_runs):
                if self.gen_time_list:
                    avg_gen_time = np.mean(self.gen_time_list)
                else:
                    # Static Estimate:
                    # observed avg: ~10 min/gen + ~20 min for first eval
                    avg_gen_time = (600*self.num_generations*25 + 1200) / 26 

                seconds_left = avg_gen_time * (self.num_generations+1) *\
                    (starting_run_number+num_runs+1 - i)
                time_left_str = seconds_to_time_str(seconds_left)

                self.run_number = i
                
                # Set up for run
                self.detailed_log = dict()
                self.setup_EA()
                self.setup_dirs()
                
                print('\nStarting run #{} of experiment {}: ETR {}'.format(self.run_number,
                    self.experiment_name, time_left_str))
                
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
        except BaseException as e:
            print(e)
        finally:
            self.sender.close()
            self.reciever.close()
            self.context.destroy()

    def shutdown_handler(self, sig, frame):
        print('\nShutdown handler called...')
        self.sender.close()
        self.reciever.close()
        self.context.destroy()
        print('Shutdown handler finished')
        sys.exit(0)
   
    def setup_sockets(self):
        self.context = zmq.Context()
        self.sender = self.context.socket(zmq.PUSH)
        self.sender.bind('tcp://{}:{}'.format(self.ip_addr, self.send_port))
        
        # setup the socket to read responses on
        self.reciever = self.context.socket(zmq.PULL)
        self.reciever.bind('tcp://{}:{}'.format(self.ip_addr, self.recv_port))
        
        # setup ZMQ Poller
        self.poller = zmq.Poller()
        self.poller.register(self.reciever, zmq.POLLIN)
        
        print('Sending Connection: tcp://{}:{}'.format(self.ip_addr, self.send_port))
        print('Reciving Connection: tcp://{}:{}'.format(self.ip_addr, self.recv_port))

    def setup_EA(self):
        creator.create('FitnessMax', base.Fitness, weights=(1.0,))
        creator.create('Individual', list, fitness=creator.FitnessMax)
        
        self.toolbox = base.Toolbox()
        self.toolbox.register('attr_float', random.random)
        self.toolbox.register('individual', tools.initRepeat, creator.Individual,
                self.toolbox.attr_float, n=self.genome_size)
        self.toolbox.register('population', tools.initRepeat, list, self.toolbox.individual)
        
        # Setup Evolution Alg operators
        self.toolbox.register('mate', tools.cxTwoPoint)
        self.toolbox.register('mutate', tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
        self.toolbox.register('select', tools.selTournament, tournsize=self.tourn_size)
        
        # Setup EA history
        self.history = tools.History()
        
        # Decorate the variation operators
        # NOTE: I could add bounds here for certain params
        self.toolbox.decorate('mate', self.history.decorator)
        self.toolbox.decorate('mutate', self.history.decorator)
        
        # Setup hall of fame and statistics
        self.hof = tools.HallOfFame(10)
        self.stats = tools.Statistics(lambda ind: ind.fitness.values)
        self.stats.register('avg', np.mean)
        self.stats.register('std', np.std)
        self.stats.register('min', np.min)
        self.stats.register('max', np.max)

    def setup_dirs(self):
        self.experiment_directory = 'logs/' + self.experiment_name
        if not os.path.isdir(self.experiment_directory):
            print('No directory for experiment... Creating now.')
            os.makedirs(self.experiment_directory)

        self.run_directory = '{}/run{}'.format(self.experiment_directory, self.run_number)
        if not os.path.isdir(self.run_directory):
            print('No directory for run... Creating now.')
            os.makedirs(self.run_directory)
        else:
            print('Directory exists, may end up writing over some files...')
    
    def run(self):
        self.population = self.toolbox.population(n=self.pop_size)
        self.history.update(self.population)
        
        self.ending_pop, self.summary_log = self.eaSimpleCustom(cxpb=0.5, mutpb=0.2)
        self.run_time = time.time() - self.start_time
        time_str = seconds_to_time_str(self.run_time)
        print('\n\nRun finished at {}\n\t Taking {}'.format(datetime.datetime.now(),
            time_str))
    
    def eaSimpleCustom(self, cxpb, mutpb):
        population = self.population
        toolbox = self.toolbox
        stats = self.stats
        hall_of_fame = self.hof
        num_gens = self.num_generations

        self.gen = 0
        self.gen_start_time = time.time()
        
        logbook = tools.Logbook()
        logbook.header = ['gen', 'nevals'] + (stats.fields if stats else [])
        
        # Evaluate individuals with an invalid fitness
        invalid_ind = [ind for ind in population if not ind.fitness.valid]
        
        fitnesses = self.custom_eval_fit_mapping(invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        
        if hall_of_fame is not None:
            hall_of_fame.update(population)
        
        record = stats.compile(population) if stats else {}
        logbook.record(gen=0, nevals=len(invalid_ind), **record)
        
        self.gen_time_list.append(time.time() - self.gen_start_time)
        
        # Begin generational process
        for gen in range(1, num_gens+1):
            seconds_left = np.mean(self.gen_time_list) * (num_gens+1-gen)
            time_left_str = seconds_to_time_str(seconds_left)

            print('\n\nGENERATION {}: ETR {}'.format(gen, time_left_str))
            self.gen = gen
            self.gen_start_time = time.time()
            
            # Select offspring
            offspring = toolbox.select(population, len(population))
            
            # Vary the pool of individuals
            offspring = algorithms.varAnd(offspring, toolbox, cxpb, mutpb)
            
            # Evalueate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            
            fitnesses = self.custom_eval_fit_mapping(invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            
            # Update hall_of_fame if it exists
            if hall_of_fame is not None:
                hall_of_fame.update(offspring)
            
            # Replace the current population
            population[:] = offspring
            
            # Append stats and logs
            record = stats.compile(population) if stats else {}
            logbook.record(gen=gen, nevals=len(invalid_ind), **record)

            self.gen_time_list.append(time.time() - self.gen_start_time)
        
        return population, logbook

    def custom_eval_fit_mapping(self, individuals):
        fitnesses = [float('Inf') for i in range(len(individuals))]
        print('{} individuals need to be evaluated.'.format(len(individuals)))
        
        # Send out individuals to be evauluated
        for ind in individuals:
            self.sender.send_json(ind)
        
        # While there are fitnesses not yet evaluated, recv results
        num_evaluated = 0
        while any(fit == float('Inf') for fit in fitnesses):
            
            socks = dict(self.poller.poll(self.timeout))
            if socks:
                if socks.get(self.reciever) == zmq.POLLIN:
                    return_data = dict(self.reciever.recv_json(zmq.NOBLOCK))
            else:
                print('Timeout on reciever socket occured!')
                non_resolved_ind = fitnesses.index(float('Inf'))
                self.sender.send_json(individuals[non_resolved_ind])
                continue
            
            ind = list(return_data['Genome'])
            result = dict(return_data['Result'])
            raw_fit, fitness = self.evaluate_result(ind, result)
            
            # Try to get the index of the individual
            try:
                index = individuals.index(ind)
            except:
                print('Recieved individual from previous generation!')
                continue
            
            while True:
                if fitnesses[index] == float('Inf'):
                    fitnesses[index] = fitness
                    break
                else:
                    if index > len(individuals):
                        print('Error -index out of range!')
                        exit()
                    else:
                        old_index = index
                        try:
                            index = individuals[old_index+1:].index(ind)
                            index += old_index+1
                        except:
                            print('Error assigning fitness')
                            print(ind)
                            print(index)
                            print(individuals)
                            print(individuals[old_index+1:])
                            num_evaluated -= 1
                            break
            num_evaluated += 1
            
            raw_fit_str = '[%5.2f, %4.2f, %4.2f, %6.2f, %6.2f]' % tuple(raw_fit)
            fit_str = '%.2f'%fitness

            time_elapsed = time.time() - self.gen_start_time
            time_elapsed_str = seconds_to_time_str(time_elapsed)

            print('{:>2}/{}: {} -- {} Time Elapsed: {}'.format(num_evaluated,
                len(individuals), fit_str, raw_fit_str, time_elapsed_str))
        
        return fitnesses

    def evaluate_result(self, ind, result):
        df = pd.DataFrame.from_dict(dict(result))
        
        # Get rid of all 0s entry ???
        df = df.truncate(before=2)
        
        # Speeds
        avg_speed = df['Actual Speed'].mean()
        max_speed = df['Actual Speed'].max()
        
        norm_avg_speed = avg_speed / 15
        norm_max_speed = max_speed / 15
        
        # Waypoints
        waypoints_achieved = df['Goal Status'].max()
        norm_wp = waypoints_achieved / 4.0
        
        # Raw Time
        time_elapsed = df['Time'].max()
        
        # Raw Dist
        dx = np.diff(df['Pos X'])
        dy = np.diff(df['Pos Y'])
        d = np.hypot(dx, dy)
        total_dist = np.sum(d)
        
        if norm_wp > 0.8:
            norm_time_elapsed = 15.2 / time_elapsed
            norm_dist = 152.67 / total_dist
        
        else:
            norm_time_elapsed = time_elapsed / 300.0
            norm_dist = total_dist / 444
        
        raw_fit = [avg_speed, max_speed, waypoints_achieved, time_elapsed, total_dist]
        norm_fit = [norm_avg_speed * 2, norm_max_speed * 2, norm_wp * 3, norm_time_elapsed,
                norm_dist]
        fit = (sum(norm_fit),)
        
        # Add individual to detailed log
        if str(ind) not in self.detailed_log.keys():
            self.detailed_log[str(ind)] = {
                    'gen': self.gen,
                    'fitness': fit,
                    'rawFitness': raw_fit,
                    'dataFrame': df.to_json()
                    }
        
        return raw_fit, fit

    def create_run_log(self):
        self.run_log = collections.OrderedDict()
        
        self.hof_fitnesses = list()
        for ind in self.hof:
            self.hof_fitnesses.append(ind.fitness.values)

        self.run_log['experiment_name'] = self.experiment_name
        self.run_log['run_number'] = self.run_number
        self.run_log['run_date'] = str(datetime.datetime.now())
        self.run_log['running_time'] = (self.run_time)
        self.run_log['best_ind'] = self.hof[0]
        self.run_log['best_ind_fitness'] = self.hof[0].fitness.values
        self.run_log['summary_log'] = self.summary_log
        self.email_log = copy.deepcopy(self.run_log)
        self.run_log['hall_of_fame'] = list(self.hof)
        self.run_log['hall_of_fame_fitnesses'] = self.hof_fitnesses
        self.run_log['detailed_log'] = self.detailed_log

    def write_run_log(self):
        with open('{}/log.json'.format(self.run_directory), 'w+') as outfile:
            json.dump(self.run_log, outfile, indent=2)
        details_of_best_ind = self.detailed_log[str(self.hof[0])]
        df = pd.read_json(details_of_best_ind['dataFrame'], orient='columns')
        df = df.sort_index()
        df.to_csv(self.run_directory + '/best_ind_details.csv')

    def email_notification(self, results):
        sender = 'evo.ros2.result.sender@gmail.com'
        msg = MIMEText(str(results))
        msg['FROM'] = sender
        msg['TO'] = str(self.email_reciever_list)
        msg['Subject'] = 'EA Results'
        
        HOST = 'smtp.gmail.com'
        PORT = '587'
        SERVER = smtplib.SMTP()
        SERVER.connect(HOST, PORT)
        USER = sender
        PASSWD = 'evoRos2Rocks'
        SERVER.ehlo()
        SERVER.starttls()
        SERVER.login(USER, PASSWD)
        
        SERVER.sendmail(sender, self.email_reciever_list, msg.as_string())
        SERVER.quit()
    
def seconds_to_time_str(seconds, dec_precision=0, no_hrs=False, no_min=False):
    int_sec = int(seconds)
    frac_sec = seconds - int_sec
    
    result_str = ''
    if int(int_sec / 3600) and not no_hrs:
        int_hrs = int(int_sec / 3600)
        int_sec %= 3600
        result_str += '%d hrs ' % int_hrs
    if int(int_sec / 60) and not no_min:
        int_min = int(int_sec / 60)
        int_sec %= 60
        result_str += '%d min ' % int_min
    
    if not dec_precision:
        result_str += '%d sec' % int_sec
    else:
        result_str += '%.{}f'.format(dec_precision) % (int_sec + float_sec)
    
    return result_str

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Front end DEAP EA for nav tuning Study')
    parser.add_argument('-d', '--debug', action='store_true',
            help='Print extra output to terminal.')
    args, unknown = parser.parse_known_args()

    node = Nav_Tuning_DEAP_EA(cmd_args = args)
