import os
import math
import numpy as np
import yaml, json
import argparse
import matplotlib.pyplot as plt
import zmq
from sklearn.metrics import mean_squared_error
import pandas as pd

class SocketZMQDevice():
    def __init__(self, ip_addr, send_port, recv_port, timeout):
        self.ip_addr = ip_addr
        self.send_port = send_port
        self.recv_port = recv_port
        self.timeout = timeout

    def network_decorator(self, method):
        """
        Use this to decorate a method that uses self.sender, self.reciever, self.context.
        Make sure to set the folowing before calling the decorated method:
            self.ip_addr
            self.send_port
            self.recv_port
            self.timeout
        """
        self.sender = None
        self.reciever = None
        self.context = None

        def wrapper(*args, **kwargs):
            try:
                result = None

                self.setup_sockets()

                result = method(*args, **kwargs)
            
            except BaseException as e:
                print(e)
            finally:
                if self.sender is not None:
                    self.sender.close()
                if self.reciever is not None:
                    self.reciever.close()
                if self.context is not None:
                    self.context.destroy()

            # Resets these values to None
            self.sender = None
            self.reciever = None
            self.context = None

            return result
        return wrapper

    def setup_sockets(self):
        """
        Sets up sender, reciever, context, and poller (timeout) variables for socket
        communication.
        """
        self.context = zmq.Context()
        # setup output socket
        self.sender = self.context.socket(zmq.PUSH)
        self.sender.bind('tcp://{}:{}'.format(self.ip_addr, self.send_port))
        # setup input socket
        self.reciever = self.context.socket(zmq.PULL)
        self.reciever.bind('tcp://{}:{}'.format(self.ip_addr, self.recv_port))
        # setup poller incorperate timeouts
        self.poller = zmq.Poller()
        self.poller.register(self.reciever, zmq.POLLIN)
        
        print('Sending Connection: tcp://{}:{}'.format(self.ip_addr, self.send_port))
        print('Receiving Connection: tcp://{}:{}'.format(self.ip_addr, self.recv_port))

def evaluate_nav_result(ind, result):
    result_df = pd.DataFrame.from_dict(dict(result))
    
    # Get rid of all 0s entry ???
    result_df = result_df.truncate(before=2)
    
    # Raw Time
    time_elapsed = result_df['Time'].max()

    # Progress
    for d in result_df['Direction']:
        if abs(d) == 1:
            desired_direction = d
            break

    # Goal
    goal_status = max(result_df['Goal Status'])
    
    prog_list = []
    for x,y in zip(result_df['Pos X'], result_df['Pos Y']):
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
    result_df = result_df.join(prog_df)

    raw_fit = [max_prog, time_elapsed, goal_status]
    if max_prog > 0.95:
        fit = 4 + (1 + 80 / time_elapsed) ** 2
    else:
        fit = (1 + max_prog) ** 2

    #raw_fit = [avg_speed, max_speed, waypoints_achieved, time_elapsed, total_dist]
    #norm_fit = [norm_avg_speed * 2, norm_max_speed * 2, norm_wp * 3, norm_time_elapsed,
    #        norm_dist]
    #fit = (sum(norm_fit),)
    
    
    return raw_fit, (fit,), result_df

def evaluate_PID_result(ind, result):
    result_df = pd.DataFrame.from_dict(dict(result))
    
    # Get rid of all 0s entry ???
    result_df = result_df.truncate(before=2)
    result_df['Error'] = result_df['Actual Speed'] - result_df['Goal Speed']

    fitness_val = mean_squared_error(result_df['Goal Speed'], result_df['Actual Speed'])
    
    return fitness_val, (fitness_val,), result_df

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

    # Rotation is pi/4
    c = math.sqrt(2)/2
    s = math.sqrt(2)/2

    # Get the position in a rotated coordinate fram to make distinctions easier
    rot_pos = (c*raw_pos[0] + s*raw_pos[1], -s*raw_pos[0]+c*raw_pos[1])

    # First assume counter clockwise (corrected after piecewise function)
    if -ell/2 <= rot_pos[0] and rot_pos[0] <= ell/2: # in rectangle portion
        if rot_pos[1] > 0: # in top portion
            # completed bottom half and one turn
            base_progress = ell/2 + math.pi*rad_avg
            eff_dist = base_progress + (ell/2 - rot_pos[0])
        else: # in bottom portion
            if rot_pos[0] > 0:
                # no base progress
                eff_dist = rot_pos[0]
            else:
                # completed bottom half, two turns, and top
                base_progress = 1.5*ell + 2*math.pi*rad_avg
                eff_dist = base_progress + (ell/2 + rot_pos[0])
    else: # in annulus
        if rot_pos[0] > 0: # in right portion
            # completed bottom half
            base_progress = 0.5*ell
            eff_theta = math.pi - math.atan2(rot_pos[0] + ell/2, rot_pos[1])
        else: # in left portion
            # completed bottom half, one turn, and top
            base_progress = 1.5*ell + math.pi*rad_avg
            eff_theta = -math.atan2(rot_pos[0] - ell/2, rot_pos[1])

        eff_dist = base_progress + rad_avg*eff_theta

    eff_progress = eff_dist / L # normalize

    if direction < 0:
        eff_progress = 1 - eff_progress

    return eff_progress

def seconds_to_time_str(seconds, dec_precision=0, no_hrs=False, no_min=False):
    """
    Converts an amount of seconds into a time string.
    
    If either no_hrs or no_min flag is set to True, then those units will not be used and the
    time will be given only in seconds and any other units.
    
    The dec_precision argument determines how many decimals to incorperate in the seconds portion.
    """
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

def setup_parser():
    pass

def main():
    if os.path.isfile('saved_ind.yml'):
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
    parser.add_argument('--load', default='', nargs='*', dest='load',
            help='Use a saved individual using a given list of keys.')
    parser.add_argument('--save_as', default='', dest='save_as',
            help='Use this to identify this individual with a name to load later.')
    parser.add_argument('--evaluate', action='store_true', dest='eval')
    parser.add_argument('--group_list', default='', nargs='*', dest='group_list',
            help='Identify a list of groups to compare')
    args, unknown = parser.parse_known_args()

    print_key_tree(get_key_tree(saved_ind_list))

    group_list = []
    for group_name in args.group_list:
        group_dict = saved_ind_list[group_name]
        group = []

        run_num = 1
        run_name = 'run%d' % run_num
        while run_name in group_dict.keys():
            group.append(group_dict[run_name]['genome'])

            run_num += 1
            run_name = 'run%d' % run_num

        group_list.append(group)

    compare_groups(group_list)

if __name__ == '__main__':
    log_dir = 'desktop-logs/nav-tuning_18params_uneven-track_narrow-search_elitism_avg2'
    dest = 'saved_ind.yml'
    exp_ref_name = 'narrow-search_uneven-track_avg2'
    weight_version = 'NARROW_GENOME_WEIGHTS'

    args = [log_dir, dest, exp_ref_name, weight_version]
    print(args)
    _ = raw_input('Continue? (Press ENTER)')
    add_new_exp(*args)
