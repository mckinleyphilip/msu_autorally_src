# msu_autorally #


## Wiki ##
Please see the msu_autorally_src wiki, link below, for more detailed installation and running instructions.
Wiki: https://github.com/gsimon2/msu_autorally_src/wiki


## Directory Descriptions ##
### evo_ros2 directory ###
Source code for Evo-ROS2. Includes various EA source codes and experiment logs/results.

### msu_autorally_helper directory ###
Source code for enhanced MSU AutoRally functionality, including: support the ROS Navigation Stack, basic keyboard navigation control, move_base goal logging and replaying, and the speed signal publisher used in the throttle PID evolution experiments.
Also included are pre-configured waypoint files and a few launch files to ease the spawning of the AutoRally simulaiton and Evo-ROS.

### python_scripts directory ###
A set of python scripts for managing the robonodes.

### tmux directory ###
Contains the start_evo_ros.yaml script for starting all of the Evo-ROS instances on the robonodes when using tmuxp.


## File Descriptions ##
### EA_environment_details.yml ###
Detailed description of the required packages for running the DEAP_EA script. These are installed on the MSU desktop machines.
To activate use:

```source activate front-end```

### enki_environment_details.txt ###
Detailed description of the required packages for running Enki. These are installed on the MSU desktop machine.
To activate use:

```source activate enki```

### gazebo7_install.sh ###
Script for installing Gazebo 7. Usaged details are included in the installation steps found here: https://github.com/gsimon2/msu_autorally_src/wiki/Software-Installation-Instructions


### ros_environment_details.txt ###
Detailed description of the required packages for running the AutoRally ROS/Gazebo simulations.






