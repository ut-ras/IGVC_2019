#!/usr/bin/env python

'''
This module extracts landmark positions from
the laser scan topic and publishes landmark
positions in global X-Y coordinates
'''

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import itertools

class LandmarkExtraction:
	def __init__(self, minimum_valid_distance, depth_jump, 
				 cylinder_offset, max_cylinder_distance):
		self.depth_jump = depth_jump
		self.cylinder_offset = cylinder_offset
		self.max_cylinder_distance = max_cylinder_distance
		self.minimum_valid_distance = minimum_valid_distance

		self.landmark_msg = Float64MultiArray()

	def flatten(self, list_of_lists):
		new_list = []

		for list in list_of_lists:
			for index in list:
				new_list.append(index)

		return new_list

	def laserScanCb(self, msg):
		landmark_bearing, current_landmark, output_landmark = [], [], []
		landmark_index = -1

		left_edge_found = False

		#print "start-------------------------------\n"
		
		for angle in range(1, len(msg.ranges) - 1):
			derivative = (msg.ranges[angle] - msg.ranges[angle-1]) / 2

			# Check for infinity measurements
			if (msg.ranges[angle] > 999999999):
				continue

			if (derivative < -self.depth_jump):
				bearing = angle * 0.25 - 135 
				left_edge = bearing

				#print "left_edge found at {}".format(left_edge)

				left_edge_found = True

			if (derivative > self.depth_jump):
				bearing = angle * 0.25 - 135
				right_edge = bearing
				
				#print "right_edge found at {}".format(right_edge)
				
				if left_edge_found:
					landmark_bearing.append((right_edge + left_edge) / 2)
					landmark_index += 1

					#self.landmark_msg.data = landmark_bearing

					current_range = msg.ranges[int((landmark_bearing[landmark_index] + 135) * 4)]
					current_bearing = landmark_bearing[landmark_index]

					output_landmark.append([current_range, current_bearing]) # 2D Array for Readability here
					current_landmark = self.flatten(output_landmark) # Flatten to 1D for ROS

					self.landmark_msg.data = current_landmark

					#print "landmark: {} found between bearing ({}) and ({})".format(current_landmark, left_edge, right_edge)

				left_edge_found = False

		#print "\nstop-------------------------------- \n"

	def main(self):
		rospy.init_node('scan_listener', anonymous=True)
		pub = rospy.Publisher("/landmark", Float64MultiArray, queue_size=10)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			rospy.Subscriber("/scan", LaserScan, self.laserScanCb)
			pub.publish(self.landmark_msg)
			rate.sleep()


if __name__ == '__main__':
	try:
		print "Landmark Extraction is Alive!"
		# Cylinder Extraction and matching constants
		minimum_valid_distance = rospy.get_param("/landmark_extraction/minimum_valid_distance")
		depth_jump = rospy.get_param("/landmark_extraction/depth_jump")
		cylinder_offset = rospy.get_param("/landmark_extraction/cylinder_offset")
		max_cylinder_distance = rospy.get_param("/landmark_extraction/max_cylinder_distance")

		extractor = LandmarkExtraction(minimum_valid_distance, depth_jump,
									   cylinder_offset, max_cylinder_distance)
		extractor.main()

	except rospy.ROSInterruptException:
		pass
