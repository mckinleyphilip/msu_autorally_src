#!/usr/bin/env python
#
# Start Basic Sim
#
#	This is a helper script for easily launching all required launch files for the basic Autorally operation in their own xterminals.
#
# GAS 2018-02-28

import argparse
import subprocess
import os
import re
import time

#parser = argparse.ArgumentParser(description='Script used for starting all of the launch files for the basic autorally operation')
#args= parser.parse_args()


# Start Gazebo launch file
print('Starting the AutoRally Gazebo Simulation launch file \n')
term_title = 'Gazebo Launch File'
cmd = 'roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch'
cmd_str = """xterm -hold -title '{}' -e '{}'&""".format(term_title,cmd)
os.system(cmd_str)
time.sleep(2)

# Start the runstop motion enable publisher
print('Starting runstop motion enable publiser \n')
cmd = 'rosrun msu_autorally_helper runstopMotionEnabled_pub.py&'
os.system(cmd)


