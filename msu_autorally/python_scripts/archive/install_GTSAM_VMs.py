import argparse
import subprocess
import os
import yaml
import re

parser = argparse.ArgumentParser(description='Script used for updating the ros_catkin_ws code on all of the robo VMs from the github repo')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting update scripts on robo nodes...')


work_nodes_file_name = 'update_nodes.yml'
#work_nodes_file_name = 'test_nodes.yml'
git_branch = "PID_evol_experiment"

with open(os.path.dirname(os.path.abspath(__file__)) + '/{}'.format(work_nodes_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

for worker in cfg['worker_list']:
	print(str(worker))
	ip = cfg['worker_list'][str(worker)]['ip']
	print(str(ip))
	
	cmds = """echo 'Forcing all ros_catkin_ws/src code to match Github';
		source /opt/ros/indigo/setup.bash;
		source ~/autorally_catkin_ws/devel/setup.sh;
		source ~/autorally_catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh;
		cd gtborg-gtsam-f538d1dc7bdd/build/;
		cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF ..;
		make install;
		exec bash
		""".format(git_branch)
	cmd_str = 'xterm -title "Connection to {}" -hold -e ssh -t -X {} "{}"&'.format(worker,ip,cmds)
	os.system(cmd_str)

print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)

