# msu_autorally_src #

This repo contains the extension work performed by Michigan State University on AutoRally platform, which was originally developed by Georgia Tech. The main focus of this repo is the improvement of control software on the AutoRally platform by integrating the ROS Navigation Stack (contained in msu_autorally/msu_autorally_helper) and providing Evo-ROS a framework developed to allow evolutionary algorithms to be executed utilizing ROS and Gazebo as an evaluation environment (contained in msu_autorally/evo_ros2).

This repo is configured to use Git LFS (large file storage) for the following file types:
- .json
- .tar.gz
- .pickle

To install and configure Git LFS please see the following link:
https://git-lfs.github.com/

## Wiki ##
Please see the msu_autorally_src wiki, link below, for more detailed installation and running instructions.
Wiki: https://github.com/gsimon2/msu_autorally_src/wiki


## Directory Descriptions ##
### autorally directory ###
The autorally directory was cloned from:https://github.com/AutoRally/autorally and modified slightly to suit the needs of the MSU research efforts. An exact list of changed files can be found https://github.com/gsimon2/msu_autorally_src/blob/master/autorally/msu_change_list.txt
For more details about the AutoRally platform please read the Readme contained in that directory.

### imu_3dm_gx4 directory ###
The imu_3dm_gx4 directory was cloned from:https://github.com/AutoRally/imu_3dm_gx4 and is needed for the autorally platform. No edits have been made inside of this directory. 

### diagrams directory ###
A few helpful diagrams can be found in the diagrams directory. These are generally used by various wiki pages.

### hector_gazebo directory ###
This is the source for the hector_gazebo sensors package. Installing via pip or apt-get often causes errors, so the source is included in this repo so that these packages can be easily installed using catkin_make.

### msu_autorally directory ###
The msu_autorally directory contains the source code for Evo-ROS2 and much of the work to integrate the ROS Navigation Stack with the AutoRally platform.

## File Discriptions ##
### gtborg-gtsam-f538d1dc7bdd.zip ###
This is required for the AutoRally Platform and is used in the installation steps.





