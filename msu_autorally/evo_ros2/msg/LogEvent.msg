time time
string sender
uint8 event
uint8 TRIGGER			= 0 # If event = 0 (TRIGGER), all monitoring nodes will write a line to their logs
uint8 REQUEST			= 1	# If event = 1 (REQUEST), the sim manager is requesting the logs from each monitoring node
uint8 REPLY				= 2 # If event = 2 (REPLY), the monitor node is replying to a request with the monitored data stored in the log_data field
float64[] log_data
