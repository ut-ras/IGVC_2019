#!/usr/bin/env python

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
		listener()
	except rospy.ROSInterruptException:
		pass
