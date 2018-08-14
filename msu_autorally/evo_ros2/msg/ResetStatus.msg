#This message structure is based off of the GoalStatus.msg structure found at:
#	http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
uint8 reset_id
uint8 status
uint8 ACTIVE		= 0 # The reset node has started the reset process
uint8 SUCCEEDED		= 1	# The reset node has successfully reset the simulation enviroment
uint8 FAILED		= 2 # The reset node has failed to reset the simulation enviroment
string text				# Extra information partaining to the status
