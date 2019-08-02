import os
import math
import numpy as np
import yaml, json
import argparse
import matplotlib.pyplot as plt

def compare_ind(ind_list, normalize=False):
    """
    Submit multiple individuals to be compared, returns a dict of statistics for each gene.
    """
    stats = []
    for i in range(len(ind_list[0])):
        stats_i = {}

        genes_i = [ind[i] for ind in ind_list]
        if normalize:
            normalizer = max(1, max(genes_i))
            for j in range(len(ind_list)):
                ind_list[j][i] /= normalizer
            
            genes_i = [ind[i] for ind in ind_list]

        #print('genes_i: %s' % genes_i)
        
        stats_i['avg'] = np.mean(genes_i)
        stats_i['max'] = max(genes_i)
        stats_i['min'] = min(genes_i)
        stats_i['std'] = np.std(genes_i)

        stats.append(stats_i)

    return stats

def compare_groups(group_list, normalize=False, plot=True, screen=True):
    """
    Submit multiple groups of individuals to be compared.  Supports one or more groups.

    kwargs:
        screen: (bool) -- True prints text to the screen
        plot: (bool) -- True makes plots of the comparisons
            (only shows plots -- plan on adding more functionality later)
    """
    # Loop through groups to calculate and print statistics and create the graph
    color_list = ['red', 'orange', 'green', 'blue', 'purple']
    group_stats_list = []
    fig, ax = plt.subplots()

    overall_max = 0
    overall_min = 1

    avg_stats = [{'max': 0.0, 'avg': 0.0, 'min': 0.0, 'std': 0.0} for n in range(len(group_list[0][0]))]

    for i in range(len(group_list)):
        group_stats_list.append(compare_ind(group_list[i], normalize))

        if screen:
            print('Group #%d:' % i)
            for j in range(len(group_list[i][0])):
                stats_fmt_str = 'max: %6.2f, avg: %5.2f, min: %5.2f, std: %5.2f'
                stats_dict = group_stats_list[i][j]
                for key in stats_dict.keys():
                    avg_stats[j][key] += stats_dict[key]
                stats_str = stats_fmt_str % (stats_dict['max'], stats_dict['avg'],
                        stats_dict['min'], stats_dict['std'])
                print('gene %2d: %s' % (j, stats_str))

        if plot:
            color = color_list[i % len(color_list)]
            group_label = 'Group #%d' % i

            # Get gene data (gene values) and gene location (gene indices) from this group
            gene_data = []
            gene_location = []
            for j in range(len(group_list[i])): # loop through inds
                M = max(group_list[i][j])
                m = min(group_list[i][j])
                if overall_max < M:
                    overall_max = M
                if overall_min > m:
                    overall_min = m
                for k in range(len(group_list[i][j])): # loop through genes
                    gene_data.append(group_list[i][j][k])
                    gene_location.append(k)

            ax.scatter(gene_data, gene_location, s=50, c=color, label=group_label,
                    alpha=0.1, edgecolors='none')
    
    if screen:
        print('Avged Stats:')
        for i in range(len(avg_stats)):
            for key in avg_stats[i].keys():
                avg_stats[i][key] /= len(group_list)

            stats_fmt_str = 'max: %6.2f, avg: %5.2f, min: %5.2f, std: %5.2f'
            stats_str = stats_fmt_str % (avg_stats[i]['max'], avg_stats[i]['avg'],
                    avg_stats[i]['min'], avg_stats[i]['std'])
            print('gene %2d: %s' % (i, stats_str))


    if plot:
        # Show plot
        buff = (overall_max - overall_min) * 0.005
        plt.axis([overall_min-buff, overall_max+buff, -0.5, 17.5])
        #plt.xticks(range(0, 101, 5))
        plt.xlabel('Values')
        plt.yticks(range(18))
        plt.ylabel('Genes')

        plt.legend()
        plt.grid(True)

        plt.show()

def get_genomes(exp_dict, group_by_runs=False, limit=-1):
    clean_exp_dict = {}
    for key in exp_dict.keys():
        if 'run' not in key:
            continue
        
        ind_list = []
        for ind in exp_dict[key]:
            ind_list.append(ind['genome'])
    
        clean_exp_dict[key] = ind_list

    result = []
    run_list = clean_exp_dict.keys()
    if limit > 0:
        for i in range(len(limit)):
            run = run_list[i % len(run_list)]
            result.append(clean_exp_dict[run][int(i/len(run_list))])
    elif group_by_runs:
        for run in run_list:
            result.append(clean_exp_dict[run])
    else:
        for run in run_list:
            for ind in clean_exp_dict[run]:
                result.append(ind)
        
    return result

def parse_args():
    """
    Creates parser, reads in known args, and returns result.

    Currently only looks for --group_list inputs to use to identify groups of individuals to
    compare.
    """
    parser = argparse.ArgumentParser(description='Tool to compare genomes.')
    parser.add_argument('--group_list', default='', nargs='*', dest='group_list', metavar='ref',
            help='Identify a list of groups to compare')
    parser.add_argument('--normalize', action='store_true', dest='normalize',
            help='Normalize each gene to a value between 0 and 1.')
    args, unknown = parser.parse_known_args()
    
    return args

def main():
    if os.path.isfile('saved_ind.yml'):
        with open('saved_ind.yml') as yml_file:
            saved_ind_list = yaml.load(yml_file, Loader=yaml.FullLoader)
    else:
        saved_ind_list = {}

    args = parse_args()

    # Given ref name, get groups of genomes to compare
    # TODO: split this into another method and take care of subcases based on existing data
    if len(args.group_list) == 1:
        group_list = get_genomes(saved_ind_list[args.group_list[0]], group_by_runs=True)
    else:
        group_list = [get_genomes(saved_ind_list[group_name]) for group_name in args.group_list]
    compare_groups(group_list, args.normalize)

if __name__ == '__main__':
    main()

