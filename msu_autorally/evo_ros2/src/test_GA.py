#!/usr/bin/env python

import zmq

IP_ADDR = '127.0.0.1'
SEND_PORT = 5000
RECV_PORT = 5010


#Initialize the socket for data
# Setup the socket to send data out on.
context = zmq.Context()
socket = context.socket(zmq.PUSH)
#socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
socket.bind('tcp://{}:{}'.format(IP_ADDR, SEND_PORT))

# Setup the socket to read the responses on.
receiver = context.socket(zmq.PULL)
receiver.bind('tcp://{}:{}'.format(IP_ADDR, RECV_PORT))

# Setup ZMQ poller
poller = zmq.Poller()
poller.register(receiver, zmq.POLLIN)

genome = list()
genome = [1,2,3,4]
#genome = 'end'


print('Sending addr: {}'.format('tcp://{}:{}'.format(IP_ADDR, SEND_PORT)))

for i in range(2):
	#genome.append(i)
	socket.send_json(genome)
	
