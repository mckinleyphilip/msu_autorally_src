string sender

uint8 state
uint8 WAITING_GENOME			= 0 # Evo-ROS2 components started and are ready to receive a genome to evaluate
uint8 GENOME_RECEIVED			= 1 # A Genome has been received and is being processed. The simulation environment will now start.
uint8 SIM_ENV_STARTED			= 2 # The simulation environment has been started.The simulation manager and utility monitors will now be started.
uint8 SIM_MANAGER_STARTED		= 3 # Simulation manager ready. Now can overwrite the evolvable ROS params.
uint8 EVOL_PARAMS_WRITTEN		= 4 # The evolvable ROS params have been written. The platform motion controller will be launched and the simulation will start
uint8 SIM_RUNNING				= 5 # The simulation is running.
uint8 SIM_COMPLETE				= 6 # The simulation has completed running. The result is available and the simulation environment can now be reset
uint8 SIM_RESET					= 7 # The simulation environment has been reset.

string text				# Extra information partaining to the status
