import argparse
import subprocess
import os
import yaml
import re
import sys

parser = argparse.ArgumentParser(description='Script used for updating the ros_catkin_ws code on all of the robo VMs from the github repo')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
parser.add_argument('-f', '--file', dest='file', default='robo8-10_nodes.yml',
        help='Node VMs file path')
parser.add_argument('-u', '--user', dest='user', default='jared',
        help='User account on VMs')
args= parser.parse_args()

print('Starting cleaning scripts on robo nodes...')


with open(args.file, 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

for worker in cfg['worker_list']:
	#print(str(worker))
	ip = cfg['worker_list'][str(worker)]['ip']
	#print(str(ip))
	cmds = """echo 'Cleaning gazebo logs from vms';
		rm -rf ~/.gazebo/log/*;
		rm -rf ~/.ros/log/*;
		df -h;
		exec bash
		"""
	cmd_str = 'xterm -title "Connection to {}" -hold -e ssh {}@{} "{}"&'.format(worker,args.user,ip,cmds)
	os.system(cmd_str)

print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
raw_input("Press enter to run")
cmd_str = "pkill xterm"
os.system(cmd_str)

