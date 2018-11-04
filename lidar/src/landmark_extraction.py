#!/usr/bin/env python

'''
This module extracts landmark positions from
the laser scan topic and publishes landmark
positions in global X-Y coordinates
'''

import rospy
from sensor_msgs.msg import LaserScan

def laserScanCb(msg):
	landmark = []

	left_edge = []
	right_edge = []

	print "start-------------------------------\n"
	
	for i in range(1, len(msg.ranges) - 1):
		derivative = (msg.ranges[i] - msg.ranges[i-1]) / 2

		if (derivative > 0.1):
			value = i * 0.25 - 135 
			left_edge.append(value)
			print "left_edge found"

		if (derivative < -0.1):
			value = i * 0.25 - 135
			right_edge.append(value)
			print "right_edge found"

	print "\nstop-------------------------------- \n"

def listener():
	rospy.init_node('scan_listener', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, laserScanCb)
	rospy.spin()

if __name__ == '__main__':
	try:
		print "Landmark Extraction is Alive!"
		# Cylinder Extraction and matching constants
		minimum_valid_distance = rospy.get_param("/landmark_extraction/minimum_valid_distance")
		depth_jump = rospy.get_param("/landmark_extraction/depth_jump")
		cylinder_offset = rospy.get_param("/landmark_extraction/cylinder_offset")
		max_cylinder_distance = rospy.get_param("/landmark_extraction/max_cylinder_distance")

		listener()

	except rospy.ROSInterruptException:
		pass
