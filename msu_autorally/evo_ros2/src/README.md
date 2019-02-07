# evo_ros2/src #

## Wiki ##
Please see the msu_autorally_src wiki, link below, for more detailed installation and running instructions.
Wiki: https://github.com/gsimon2/msu_autorally_src/wiki

## Directory Discriptions ##

### EAs directory ###
This directory contains everything for the DEAP implemented EAs and the generated logs from experiments. It also contains the Enki speed signal importer class along with any files descriping the Enki speed signals that should be evolved against.

### enki_helper_functions directory ###
This contains the work in progress for setting up the ramp characteristic evolution experiment with Enki.

### review_visualizer directory ###
This is a work in progress for utilzing the Review Visualizer (created by Tony Clark and Jared Moore) to replay simulations.

### utility_monitors directory ###
This is a work in progress to allow Evo-ROS to accept goal-models. Very bare bones and can most likely be scraped.


## File Discriptions ##

### autorally_nav_tuning_sim_manager.py ###
This was a working file for using Evo-ROS for tuning the ROS Navigation Stack. The nav_tuning branch of the repo has expanded on this.

### autorally_sim_manager_2.py ###
This is the simulation manager script for the throttle PID tuning experiments.

### empty_node.py ###
This is a template used for creating new ROS nodes.

### reset_sim.py ###
This is a debugging script that functions the same as the SoftReset service define in the srv/ directory for this package.

### software_manager_node.py ###
This is the source code for the Evo-ROS Software Manager.

### test_result_sender.py ###
This is a debugging script for quickly sending back spoofed simulation results. Can be used to test various front-ends.

### transporter_node.py ###
This is the source code for the Evo-ROS Transporter.
