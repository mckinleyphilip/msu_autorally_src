import os
import math
import numpy as np
import yaml, json
import argparse
import matplotlib.pyplot as plt

def modify_saved_data(exp_name='', ref_name='', num_ind=1, mode='best', 
        weight_version='GENOME_WEIGHTS', filename='', extend=False):
    """
    Adds data from a new experiment to a yaml file to be used in evaolutations elsewhere.
    
    TODO:
        * add functionality so that data can be loaded and stored in different locations,
          this inclides a side effect of simply loading data, changing it, and saving it
          (without adding new data)
    
    """
    if not os.path.isfile(filename):
        print('Note the filename specified \'%s\' does not exist.' % filename)
        _ = raw_input('Continue? (Press ENTER)')
        saved_data = {}
    else:
        with open(filename) as f:
            saved_data = yaml.load(f, Loader=yaml.FullLoader)
    

    # Load genome conifg file to get the weights
    # NOTE: this is done via relative path ... SENSATIVE
    genome_config = '../../config/genome_mapping.yaml'
    with open(genome_config) as genome_file:
        genome_weights_dict = yaml.load(genome_file, Loader=yaml.FullLoader)

    if weight_version not in genome_weights_dict.keys():
        raise ValueError('Invalid weight key \'%s\'' % weight_version)

    genome_weights = genome_weights_dict[weight_version]

    # Create dictionary for the new experiment
    if exp_name:
        exp_path = 'desktop-logs/%s' % exp_name
        if not os.path.isdir(exp_path):
            raise ValueError('exp_path \'%s\' does not exist' % exp_path)

        if not ref_name:
            ref_name = exp_name
        if ref_name in saved_data.keys():
            print('Ref name \'%s\' already exists in dict.' % exp_ref_name)
            _ = raw_input('Continue? (Press ENTER)')
    
        exp_dict = {'exp_name': exp_name, 'date': ''}
        for run_dir in os.listdir(exp_path):
            print(run_dir)
            with open('%s/%s/log.json' % (exp_path, run_dir)) as log_file:
                log_dict = json.load(log_file)

                exp_dict['date'] = str(log_dict['run_date']).split()[0]

                run_ind_list = []
                for i in range(num_ind):
                    new_ind = {
                        'genome': log_dict['hall_of_fame'][i],
                        'fitness': log_dict['hall_of_fame_fitnesses'][i][0]
                        }
                    for j in range(len(genome_weights)):
                        new_ind['genome'][j] *= genome_weights[j]
                    run_ind_list.append(new_ind)

                exp_dict[run_dir] = run_ind_list
                
        saved_data[ref_name] = exp_dict

    if extend:
        for ref in saved_data.keys():
            d = saved_data[ref]
            if type(d) is dict:
                if 'exp_name' in d.keys():
                    exp_path = 'desktop-logs/%s' % (d['exp_name'])
                    for run_dir in os.listdir(exp_path):
                        if type(d[run_dir]) is dict:
                            d[run_dir] = [d[run_dir]]

                        if len(d[run_dir]) < num_ind:
                            print('extending %s (%s)...' % (d['exp_name'], run_dir))
                            with open('%s/%s/log.json' % (exp_path, run_dir)) as log_file:
                                log_dict = json.load(log_file)

                                for j in range(len(d[run_dir]), num_ind):
                                    new_ind = {
                                        'genome': log_dict['hall_of_fame'][j],
                                        'fitness': log_dict['hall_of_fame_fitnesses'][j][0]
                                        }
                                    d[run_dir].append(new_ind)


    with open(filename, 'w') as dest_file:
        yaml.dump(saved_data, dest_file, indent=4)

def setup_parser():
    parser = argparse.ArgumentParser(description='Tool to help save new data from simulation ' +\
            'runs to be compared later.')
    parser.add_argument('--exp_name', metavar='name', default='', dest='exp_name',
            help='Identify the name of the experiment to load new data from.')
    parser.add_argument('--ref_name', default='', dest='ref_name',
            help='Reference name to give to the experiment to use later.')
    parser.add_argument('--weight_version', default='GENOME_WEIGHTS', dest='weight_version',
            help='Version of weights to use to scale up to actual values.')
    parser.add_argument('--num_ind', type=int, default=1, dest='num_ind',
            help='Identify how many individuals to load from each run [0, 10].')
    parser.add_argument('--mode', default='best', dest='mode',
            help='Identify how individuals are to be selected (rand or best).')
    parser.add_argument('--filename', default='saved_ind.yml', dest='filename',
            help='Identify file to load and save data to.')
    parser.add_argument('--extend', action='store_true', dest='extend',
            help='Extend the other experiments by adding specified number of individuals.')

    return parser 

def fix_fitness(obj):
    if type(obj) is dict:
        for k in obj.keys():
            v = obj[k]
            if k == 'fitness' and type(v) is list:
                obj[k] = v[0]
            else:
                obj[k] = fix_fitness(v)
    elif type(obj) is list:
        for i in range(len(obj)):
            obj[i] = fix_fitness(obj[i])
            
    return obj

def get_key_tree(dict_obj):
    key_tree = {}
    for k in dict_obj.keys():
        if type(dict_obj[k]) is dict:
            key_tree[k] = get_key_tree(dict_obj[k])
        elif type(dict_obj[k]) is list:
            key_tree[k] = type(dict_obj[k])
        else:
            key_tree[k] = dict_obj[k]

    return key_tree

def print_key_tree(key_tree, indent=''):
    for k in key_tree: 
        if type(key_tree[k]) is dict:
            print('%s%s:' % (indent, k))
            print_key_tree(key_tree[k], indent + '\t')
        else:
            print('%s%s: %s' % (indent, k, key_tree[k]))
        
def main():
    parser = setup_parser()
    args = parser.parse_args()

    print('---------------------------------------')
    print('Parameters:\n')
    for k in args.__dict__.keys():
        print('  %-20s%s' % (k, args.__dict__[k]))
    print('---------------------------------------')
    _ = raw_input('Continue? (Press ENTER)')

    modify_saved_data(**args.__dict__)


if __name__ == '__main__':
    main()

