#!/usr/bin/env python

'''
This module extracts landmark positions from
the laser scan topic and publishes landmark
positions in global X-Y coordinates
'''

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from math import sin, cos, radians, degrees, sqrt

class LandmarkExtraction:
	def __init__(self, minimum_valid_distance, depth_jump, 
				 cylinder_offset, max_landmark_radius):
		self.depth_jump = depth_jump
		self.cylinder_offset = cylinder_offset
		self.max_landmark_radius = max_landmark_radius
		self.minimum_valid_distance = minimum_valid_distance

		self.landmark_msg = Float64MultiArray()
		self.print_x_y = True

		self.current_landmark = []
		self.output_landmark = []
		self.landmark_index = -1

	def flatten(self, list_of_lists):
		new_list = []

		for list in list_of_lists:
			for index in list:
				new_list.append(index)

		return new_list

	def laserScanCb(self, msg):
		new_landmark_found = False
		left_edge_found = False

		#print self.output_landmark

		#print "start-------------------------------\n"

		self.output_landmark = []
		self.landmark_index = 0
		
		for angle in range(1, len(msg.ranges) - 1):
			derivative = (msg.ranges[angle] - msg.ranges[angle-1]) / 2

			# Check for infinity measurements
			if (msg.ranges[angle] > 999999999) or abs(derivative) > 999999999:
				continue

			if (derivative < -self.depth_jump):
				bearing = angle * 0.25 - 135.0 
				left_edge = bearing

				#print "left_edge found at {}".format(left_edge)

				left_edge_found = True

			if (derivative > self.depth_jump):
				bearing = angle * 0.25 - 135.0
				right_edge = bearing
				
				#print "right_edge found at {}".format(right_edge)
				
				if left_edge_found:
					landmark_bearing = (right_edge + left_edge) / 2 # <-- This right here is causing small errors by divid by 2. Will fix later
					landmark_range = msg.ranges[int((landmark_bearing + 135.0) * 4.0)]

					landmark_x = landmark_range * cos(radians(landmark_bearing))
					landmark_y = landmark_range * sin(radians(landmark_bearing))

					if self.output_landmark == [] and landmark_range < 999999999:
						new_landmark_found = True
						self.landmark_index += 1
					else:
						# Loop through all existing landmarks
						# If no correspondence is found, set new_landmark_found flag to true
						for index in range(self.landmark_index):
							distance_x = landmark_x - self.output_landmark[index][0]
							distance_y = landmark_y - self.output_landmark[index][1]

							distance_to_landmark = sqrt(distance_x**2 + distance_y**2)

							if (distance_to_landmark < max_landmark_radius):
								# Assign Correspondence here
								new_landmark_found = False
								break

							if (distance_to_landmark > max_landmark_radius) and landmark_range < 999999999:
								new_landmark_found = True

						if new_landmark_found:
							self.landmark_index += 1

					# Print landmark (X,Y)
					if self.print_x_y:
						if new_landmark_found:
							self.output_landmark.append([landmark_x, landmark_y]) # 2D Array for readability here
					# Print landmark (Range, Bearing) instead
					else:
						if new_landmark_found:
							self.output_landmark.append([landmark_range, landmark_bearing]) # 2D Array for readability here

					self.current_landmark = self.flatten(self.output_landmark) # Flatten to 1D for ROS

					self.landmark_msg.data = self.current_landmark

					#print "landmark: {} found between bearing ({}) and ({})".format(current_landmark, left_edge, right_edge)

				left_edge_found = False
				new_landmark_found = False

		#print "\nstop-------------------------------- \n"

	def main(self):
		rospy.init_node('scan_listener', anonymous=True)
		pub = rospy.Publisher("/landmark", Float64MultiArray, queue_size=10)
		rospy.Subscriber("/hokuyo/laserscan", LaserScan, self.laserScanCb)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			pub.publish(self.landmark_msg)
			rate.sleep()


if __name__ == '__main__':
	try:
		print "Landmark Extraction is Alive!"
		# Cylinder Extraction and matching constants
		minimum_valid_distance = rospy.get_param("/landmark_extraction/minimum_valid_distance")
		depth_jump = rospy.get_param("/landmark_extraction/depth_jump")
		cylinder_offset = rospy.get_param("/landmark_extraction/cylinder_offset")
		max_landmark_radius = rospy.get_param("/landmark_extraction/max_landmark_radius")

		extractor = LandmarkExtraction(minimum_valid_distance, depth_jump,
									   cylinder_offset, max_landmark_radius)
		extractor.main()

	except rospy.ROSInterruptException:
		pass
