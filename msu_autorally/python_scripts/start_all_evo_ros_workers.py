import argparse
import subprocess
import os
import yaml
import re

parser = argparse.ArgumentParser(description='Script used for starting transport_controller.py on all of the robo VMs')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting software_manager.py scripts on robo nodes...')

name_of_script = "evo_ros2.launch"
GA_IP_ADDR = '35.9.28.201'
script_arguments = "-ip {}".format(GA_IP_ADDR)
ga_hostname = 'autorally-robo1vm1'

work_nodes_file_name = 'active_nodes.yml'
#work_nodes_file_name = 'test_nodes.yml'

with open(os.path.dirname(os.path.abspath(__file__)) + '/{}'.format(work_nodes_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

for worker in cfg['worker_list']:
	print(str(worker))
	ip = cfg['worker_list'][str(worker)]['ip']
	print(str(ip))
	
	if str(worker) is ga_hostname:
		continue

	#Start Software Manager
	cmds = """echo {}...;
		cd;
		source /opt/ros/indigo/setup.bash;
		source ~/autorally_catkin_ws/devel/setup.sh;
		source ~/autorally_catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh;
		roslaunch evo_ros2 {};
		exec bash
		""".format(name_of_script, name_of_script)
	cmd_str = """xterm -hold -title "Connection to {}" -e 'ssh -t -X {} "{}"'&""".format(worker,ip,cmds)
	os.system(cmd_str)


print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)
