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
import json, yaml
import collections
import datetime

import copy

import math

class Nav_Tuning_DEAP_EA():
    def __init__(self, cmd_args):
        self.debug = cmd_args.debug
        
        # EA Params
        self.experiment_name = "nav-tuning_18params_empty_large-search"
        
        self.pop_size = 100
        self.num_generations = 25
        self.elitism = True

        self.genome_weights_key = 'NARROW_GENOME_WEIGHTS'
        
        path_to_genome_config = '../../config/genome_mapping.yaml'
        with open(path_to_genome_config) as ymlfile:
            self.genome_config = yaml.load(ymlfile, Loader=yaml.FullLoader)

        for k in ('GENOME_WEIGHTS', 'NARROW_GENOME_WEIGHTS'):
            for i in range(len(self.genome_config[k])):
                self.genome_config[k][i] = float(self.genome_config[k][i])

        self.genome_size = len(self.genome_config[self.genome_weights_key])
        self.tourn_size = 2
        
        self.starting_run_number = 1
        self.num_runs = 5
        
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

        self.detailed_log = dict()
        self.gen=-1

        self.full_run = self.network_wrapper(self._full_run_core)
        self.single_genome = self.network_wrapper(self._single_genome_core)

    def network_wrapper(self, method):
        """
        Call this method to wrap a simulation method with a network try catch block.
        """
        def wrapped_method(*args):
            try: 
                self.setup_sockets()
                signal.signal(signal.SIGINT, self.shutdown_handler)
            
                result = method(*args)
            
            except BaseException as e:
                print(e)
            finally:
                self.sender.close()
                self.reciever.close()
                self.context.destroy()
    
            return result
         
        return wrapped_method

    def _full_run_core(self):
        print('About to start {} runs of experiment {}'.format(self.num_runs,
            self.experiment_name))
        print('Pop = {}, Generations = {}, IP addr = {}'.format(self.pop_size,
            self.num_generations, self.ip_addr))
        print('\tStarting at run #{}'.format(self.starting_run_number))
        raw_input('Press enter to run')
        
        for i in range(self.starting_run_number, self.starting_run_number + self.num_runs):
            if self.gen_time_list:
                avg_gen_time = np.mean(self.gen_time_list)
            else:
                # Static Estimate:
                # observed avg: ~20 min / 50 evals (generations eval approx 3/5 pop_size)
                avg_gen_time = (1200*self.pop_size/50 * (1 + self.num_generations*0.6)) /\
                        (self.num_generations+1) 

            seconds_left = avg_gen_time * (self.num_generations+1) *\
                (self.starting_run_number+self.num_runs+1 - i)
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

    def _single_genome_core(self, ind, param_type='default', sim_type='eval'):
        if len(ind) != self.genome_size:
            err_str = 'Specified individual has an invalid number of genomes: %d instead of %d'
            err_str = err_str % (len(ind), self.genome_size)
            raise ValueError(err_str)

        # scale down individual
        current_weights = self.genome_config[self.genome_weights_key]

        narrow_weights = self.genome_config['NARROW_GENOME_WEIGHTS']

        #print('narrow_weights: %s' % narrow_weights)
        #print('current_weights: %s' % current_weights)

        if param_type == 'default': # default is actual values
            actual_ind = ind[:]
            for i in range(len(ind)):
                ind[i] /= current_weights[i]
        elif param_type == 'narrow': # scaled down but based on old weights
            actual_ind = []
            for i in range(len(ind)):
                actual_ind.append(ind[i] * narrow_weights[i])
                ind[i] *= narrow_weights[i] / current_weights[i]
        elif param_type == 'current':
            actual_ind = [ind[i]*current_weights[i] for i in range(len(ind))]
        else:
            raise TypeError('Unrecognized param type: %s' % param_type)

        print('About to run one evaluation of the individual:\n%s' % actual_ind)
        print('\tencoding:\n%s' % ind)
        _ = raw_input('Would you like to continue? (Press ENTER)')
        
        self.start_time = time.time()


        if sim_type == 'eval':
            num_sims = 4
        else:
            num_sims = 1

        for i in range(num_sims):
            print('sending ind...')
            self.sender.send_json(ind)

        raw_fit, fitnesses = None, [float('Inf')] * num_sims


        while any([f == float('Inf') for f in fitnesses]):
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

            fitnesses[fitnesses.index(float('Inf'))] = fitness[0]

            #raw_fit_str = '[%5.2f, %4.2f, %4.2f, %6.2f, %6.2f]' % tuple(raw_fit)
            raw_fit_str = '[%5.2f, %6.2f, %4.2f]' % tuple(raw_fit)
            fit_str = '%.2f'%fitness

            time_elapsed = time.time() - self.start_time
            time_elapsed_str = seconds_to_time_str(time_elapsed)

            out_str = '{} -- {} Time Elapsed: {}'.format(fit_str, raw_fit_str,
                    time_elapsed_str)

            print(out_str)

        print('avg: %s' % (sum(fitnesses)/len(fitnesses)))
        return actual_ind

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
        #  - added a decorator to ensure that the genes stay within the valid bounds
        self.toolbox.decorate('mate', self.history.decorator)
        self.toolbox.decorate('mutate', checkBounds(0.0, 1.0), self.history.decorator)
        
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
            
            if self.elitism:
                best_ind = hall_of_fame[0]
        
        record = stats.compile(population) if stats else {}
        logbook.record(gen=0, nevals=len(invalid_ind), **record)
        
        # Only store of this time since, on avg, in each generation only 3/5
        # the pop size is evaluated.
        self.gen_time_list.append((time.time() - self.gen_start_time)*0.6)
        
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

            if self.elitism:
                worst_ind = None
                best_exists = False
                for ind in offspring:
                    if ind.fitnesses.valid:
                        if ind is best_ind:
                            best_exists = True
                            print('Best found!')
                            break
                        elif worst_ind is None or ind.fitnesses.values[0] < worst_ind.fitnesses.values[0]:
                            worst_ind = ind

                if not best_exists:
                    print('Best not found, replacing worst...')
                    worst_ind = best_ind

            
            # Evalueate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            
            fitnesses = self.custom_eval_fit_mapping(invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            
            # Update hall_of_fame if it exists
            if hall_of_fame is not None:
                hall_of_fame.update(offspring)

                if self.elitism:
                    best_ind = hall_of_fame[0]
            
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
                # resend all invalid individuals instead of just one of them
                for i in range(len(fitnesses)):
                    if fitnesses[i] == float('Inf'):
                        self.sender.send_json(individuals[i])
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
            
            #raw_fit_str = '[%5.2f, %4.2f, %4.2f, %6.2f, %6.2f]' % tuple(raw_fit)
            #fit_str = '%.2f'%fitness
            raw_fit_str = '[%5.2f, %6.2f, %1.0f]' % tuple(raw_fit)
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
        
        # Raw Time
        time_elapsed = df['Time'].max()

        # Progress
        for d in df['Direction']:
            if abs(d) == 1:
                desired_direction = d
                break

        # Goal
        goal_status = max(df['Goal Status'])
        
        prog_list = []
        for x,y in zip(df['Pos X'], df['Pos Y']):
            prog_list.append(pos_to_progress((x, y), desired_direction))
        
        # For detecting errors in simulation...
        coarseness = 50 # get progress once every 5 seconds
        jump_bdry = 0.2
        backwards_bdry = -0.0005

        coarse_prog = 0
        coarse_diff = 0
        jump_flag = False
        backwards_flag = False
        err_detected = False

        prog_actual = [0.0]
        for i in range(len(prog_list)):
            p = prog_list[i]

            # Update progress to current iff it is achieveable going forwards, otherwise repeats
            if i > 0:
                prog_diff = p - prog_actual[i-1]
                if prog_diff < 0 or jump_bdry < prog_diff:
                    prog_actual.append(prog_actual[i-1])
                    #print('[%4d] Detected backwards or abnormal forward mvnt' % i)
                else:
                    prog_actual.append(p)

            if (i % coarseness) == 0:
                if (jump_flag or backwards_flag):
                    if p - coarse_prog < backwards_bdry*coarseness:
                        err_detected = True
                    else:
                        #print('Reseting err flags...')
                        jump_flag = False
                        backwards_flag = False

                if p - coarse_prog > jump_bdry*coarseness:
                    jump_flag = True
                    #print('Jump Detected!')
                    #print('\t%.3f to %.3f -- note bdry at %f' % (coarse_prog, p, jump_bdry))
                elif p - coarse_prog < backwards_bdry*coarseness:
                    backwards_flag = True
                    #print('Significant Backwards Movement Detected!')
                    #print('\t%.3f to %.3f -- note bdry at %f' % (coarse_prog, p, backwards_bdry))
                    
                coarse_prog = p

        # Determine max progress from the last actual progress recorded
        max_prog = prog_actual[-1]

        if err_detected:
            print('Err detected, max_prog = %.2f' % max_prog)
            #max_prog = -1

        prog_df = pd.DataFrame(data={'Progress': prog_list, 'Actual Progress': prog_actual})
        df = df.join(prog_df)

        raw_fit = [max_prog, time_elapsed, goal_status]
        if max_prog > 0.95:
            fit = 4 + (1 + 80 / time_elapsed) ** 2
        else:
            fit = (1 + max_prog) ** 2

        #raw_fit = [avg_speed, max_speed, waypoints_achieved, time_elapsed, total_dist]
        #norm_fit = [norm_avg_speed * 2, norm_max_speed * 2, norm_wp * 3, norm_time_elapsed,
        #        norm_dist]
        #fit = (sum(norm_fit),)
        
        # Add individual to detailed log
        if str(ind) not in self.detailed_log.keys():
            self.detailed_log[str(ind)] = {
                    'gen': self.gen,
                    'fitness': fit,
                    'rawFitness': raw_fit,
                    'dataFrame': df.to_json()
                    }
        
        return raw_fit, (fit,)

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

def checkBounds(min_val, max_val):
    def decorator(func):
        def wrapper(*args, **kwargs):
            
            offspring = func(*args, **kwargs)

            for ind in offspring:
                for i in range(len(ind)):
                    if ind[i] > max_val:
                        ind[i] = max_val
                    elif ind[i] < min_val:
                        ind[i] = min_val

            return offspring
        return wrapper
    return decorator

def pos_to_progress(raw_pos, direction, ell=34.08, rad_inner=13.86, rad_outer=23.48):
    """
    Returns the effective fraction of distance around the track dependent
    on the intended direction (-1 for clockwise, 1 for counter clockwise).

    Note that the track is defined by the last three parameters:
        - ell : defines the length of the straitaway
        - rad_inner : the inner radius of the annulus
        - rad_outer : the outer radius of the annulus
    and their default values are based on estimates via rviz.
    """
    rad_avg = (rad_inner + rad_outer) / 2
    L = 2 * ell + 2 * math.pi * rad_avg

    c = math.sqrt(2)/2
    s = math.sqrt(2)/2

    rot_pos = (c*raw_pos[0] + s*raw_pos[1], -s*raw_pos[0]+c*raw_pos[1])

    # First assume counter clockwise (corrected after piecewise function)
    if -ell/2 <= rot_pos[0] and rot_pos[0] <= ell/2: # in rectangle portion
        if rot_pos[1] > 0: # in top portion
            # completed bottom half and one turn
            base_progress = ell/2 + math.pi*rad_avg
            eff_progress = base_progress + (ell/2 - rot_pos[0])
        else: # in bottom portion
            if rot_pos[0] > 0:
                eff_progress = rot_pos[0]
            else:
                # completed bottom half, two turns, and top
                base_progress = 1.5*ell + 2*math.pi*rad_avg
                eff_progress = base_progress + (ell/2 + rot_pos[0])
    else: # in annulus
        if rot_pos[0] > 0: # in right portion
            # completed bottom half
            base_progress = 0.5*ell
            eff_theta = math.pi - math.atan2(rot_pos[0] + ell/2, rot_pos[1])
        else: # in left portion
            # completed bottom half, one turn, and top
            base_progress = 1.5*ell + math.pi*rad_avg
            eff_theta = -math.atan2(rot_pos[0] - ell/2, rot_pos[1])

        eff_progress = base_progress + rad_avg*eff_theta

    eff_progress /= L # normalize

    if direction < 0:
        eff_progress = 1 - eff_progress

    return eff_progress

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

def main():
    if os.path.exists('saved_ind.yml'):
        with open('saved_ind.yml') as yml_file:
            saved_ind_list = yaml.load(yml_file, Loader=yaml.FullLoader)
    else:
        saved_ind_list = {}

    parser = argparse.ArgumentParser(description='Front end DEAP EA for nav tuning Study')
    parser.add_argument('-d', '--debug', action='store_true',
            help='Print extra output to terminal.')
    parser.add_argument('-g', '--genome', dest='genome', metavar='N',
            nargs='*', help='Define parameters to do a single evaluateion.')
    parser.add_argument('--param_type', default='default', dest='param_type',
            help='Use this flag to indicate how the above parameters need to be scaled down.')
    parser.add_argument('--load', default='', dest='load',
            help='Use a saved individual.')
    parser.add_argument('--save_as', default='', dest='save_as',
            help='Use this to identify this individual with a name to load later.')
    parser.add_argument('--evaluate', action='store_true', dest='eval')
    args, unknown = parser.parse_known_args()

    node = Nav_Tuning_DEAP_EA(cmd_args = args)

    if args.genome or args.load:
        if args.load:
            individual = saved_ind_list[args.load][:]
        else:
            individual = [float(p) for p in args.genome]

        if args.eval:
            sim_type = 'eval'
        else:
            sim_type = 'sim'

        actual_ind = node.single_genome(individual, args.param_type, sim_type)

        if args.save_as:
            ind_name = args.save_as
        else:
            ind_name = '_'

        saved_ind_list[ind_name] = actual_ind

        with open('saved_ind.yml', 'w') as yml_file:
            yml_file.write(yaml.dump(saved_ind_list))
    else:
        node.full_run()

if __name__ == '__main__':
    main()
