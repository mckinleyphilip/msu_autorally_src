#!/usr/bin/env python
#
# lidar_obstacle_avoidance_controller
#
#	lidar_obstacle_avoidance_controller ROS node responsible for detecting obstacles on the lidar and preventing collisions
#
# GAS 2018-03-21

import rospy
import argparse
import numpy as np
import cv2
import math
import time

from sensor_msgs.msg import LaserScan
from autorally_msgs.msg import chassisCommand
from std_msgs.msg import Float64




class EmptyNode():
	def __init__(self):
		
		
		# Configure Publishers
		self.speed_pub = rospy.Publisher('/lidar_obstacle_avoidance_controller/chassisCommand', chassisCommand, queue_size=10)

		#time.sleep(20)

		# Configure Subscribers
		self.lidar_scan_sub = rospy.Subscriber('/lidar_front', LaserScan,self.lidar_scan_cb)
		
		rospy.on_shutdown(self.on_shutdown)
		

		rospy.spin()
		
	def on_shutdown(self):
		pass
		
	def filter_scan_method(self, data):
		#print(data.ranges)
		
		start_ind = 0
		end_ind = 0
		
		# Find borders 
		for ind, r in enumerate(data.ranges):
			if r == float ('Inf'):
				start_ind = ind
				break
		for ind, r in reversed(list(enumerate(data.ranges))):
			if r == float ('Inf'):
				end_ind = ind
				break
		
		frame = np.zeros((500, 500,3), np.uint8)
		angle = data.angle_max
		Vx = 250
		Vy = 250
		for ind, r in enumerate(data.ranges):
			color = (255,0,0)
			if r == float ('Inf'):
				r = data.range_max
				
			if ind == start_ind or ind == end_ind:
				pass
				#color = (0,255,0)
			
			
			x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
			y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
			angle= angle - data.angle_increment
				
			if ind >= start_ind and ind <= end_ind:
				Vx+=x
				Vy+=y
				
			
			cv2.line(frame,(250, 250),(x+250,y+250),color,1)

		cv2.line(frame,(250, 250),(250+Vx,250+Vy),(0,0,255),3)
		cv2.circle(frame, (250, 250), 2, (255, 255, 0))
		ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
		if ang > 180:
			ang -= 360
		if ang > 90:
			ang = 90
		if ang < -90:
			ang = -90
			
		cv2.putText(frame,str(ang)[:10], (50,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
		
		mid_point = int(float((ang+90)/180) * len(data.ranges))
		cv2.putText(frame,str(mid_point)[:5], (200,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
		start_ind = len(data.ranges) - mid_point - 25
		end_ind = len(data.ranges) - mid_point + 25
		cv2.putText(frame,str(start_ind)[:5], (100,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
		cv2.putText(frame,str(end_ind)[:5], (300,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
		
		angle = data.angle_max
		Vx = 250
		Vy = 250
		
		for ind, r in enumerate(data.ranges):
			color = (255,0,0)
			if r == float ('Inf'):
				r = data.range_max
			if ind >= start_ind and ind <= end_ind:
				x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
				y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
				Vx+=x
				Vy+=y
			if ind == start_ind or ind == end_ind:
				color = (0,255,0)
				cv2.line(frame,(250, 250),(x+250,y+250),color,1)
			angle= angle - data.angle_increment
		ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
		if ang > 180:
			ang -= 360
		if ang > 90:
			ang = 90
		if ang < -90:
			ang = -90
		
		
		normalized_ang = ang / 90
		
		cv2.putText(frame,str(ang)[:10], (50,450), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
		
		cv2.imshow('frame',frame)
		cv2.waitKey(1)
		
		
		cmd = chassisCommand()
		cmd.header.stamp = rospy.get_rostime()
		cmd.sender = "lidar_obstacle_avoidance_controller"
		cmd.steering = normalized_ang
		cmd.throttle = -5
		cmd.frontBrake = -5
		
		self.speed_pub.publish(cmd)

		
	
	def test_scan_method(self, data):
		#print(data.ranges)
		
		start_ind = 0
		end_ind = 0
		
		# Find borders 
		for ind, r in enumerate(data.ranges):
			if r == float ('Inf'):
				start_ind = ind
				break
		
		for ind, r in reversed(list(enumerate(data.ranges))):
			if r == float ('Inf'):
				end_ind = ind
				break
		
		frame = np.zeros((500, 500,3), np.uint8)
		angle = data.angle_max
		Vx = 250
		Vy = 250
		
		#filtered_data = data.ranges[start_ind:end_ind]
		
		for ind, r in enumerate(data.ranges):
			color = (255,0,0)
			if r == float ('Inf'):
				r = data.range_max
				
			if ind == start_ind or ind == end_ind:
				color = (0,255,0)
			
			
			x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
			y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
			angle= angle - data.angle_increment
				
			if ind >= start_ind and ind <= end_ind:
				Vx+=x
				Vy+=y
				
			
			cv2.line(frame,(250, 250),(x+250,y+250),color,1)

		
		cv2.line(frame,(250, 250),(250+Vx,250+Vy),(0,0,255),3)
		cv2.circle(frame, (250, 250), 2, (255, 255, 0))
		ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
		if ang > 180:
			ang -= 360
		if ang > 90:
			ang = 90
		if ang < -90:
			ang = -90
			
		normalized_ang = ang / 90
		cv2.putText(frame,str(ang)[:10], (50,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
		
		cv2.imshow('frame',frame)
		cv2.waitKey(1)
		
		
		cmd = chassisCommand()
		cmd.header.stamp = rospy.get_rostime()
		cmd.sender = "lidar_obstacle_avoidance_controller"
		cmd.steering = normalized_ang
		cmd.throttle = -5
		cmd.frontBrake = -5
		
		#self.speed_pub.publish(cmd)
		
		
	def lidar_scan_cb(self, data):
		self.filter_scan_method(data)
		pass

if __name__ == '__main__':
	# Parse arguments
	parser = argparse.ArgumentParser(description='Lidar based obstacle avoidance controller')
	args, unknown = parser.parse_known_args() # Only parse arguments defined above


	# Init Node
	rospy.init_node('lidar_obstacle_avoidance_controller',anonymous=False)
	
	try:
		node = EmptyNode()
	except rospy.ROSInterruptException: pass
	
	
