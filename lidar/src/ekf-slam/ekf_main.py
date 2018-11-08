#!/usr/bin/env python

import rospy
from math import sin, cos, pi, atan2, sqrt, radians, degrees
from numpy import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan

import ekf_prediction, ekf_correction

# This class needs to import functions from:
# 	- Landmark extraction
# 	- EKF Prediction
# 	- EKF Correction

class ExtendedKalmanFilter:
	def __init__(self, state, covariance, robot_width, scanner_displacement,
				 control_motion_factor, control_turn_factor,
				 measurement_distance_stddev, measurement_angle_stddev):
		self.state = state
		self.covariance = covariance

		self.robot_width = robot_width
		self.scanner_displacement = scanner_displacement
		self.control_motion_factor = control_motion_factor
		self.control_turn_factor = control_turn_factor
		self.measurement_distance_stddev = measurement_distance_stddev
		self.measurement_angle_stddev = measurement_angle_stddev

		self.observations = []

		self.number_of_landmarks = 0
		self.landmark_index = -1
		self.loop_once = False

		self.DEBUG = False

	def visualize_landmarks(self, landmarks):
		num_readings = 1080
		laser_frequency = 40
		current_time = rospy.Time.now()

		scan = LaserScan()
		
		scan.header.stamp = current_time
		scan.header.frame_id = 'landmark_frame'
		scan.angle_min = -2.35619449019
		scan.angle_max = 2.35619449019
		scan.angle_increment =  0.0043633231
		scan.time_increment = 0.000061722
		scan.scan_time = 1.0
		scan.range_min = 0.2
		scan.range_max = 64.0

		scan.ranges = []
		scan.intensities = []

		bearing = []
		current_range = []

		bearing_found = False

		for i in range(len(landmarks)):
			current_range.append(sqrt(landmarks[i][0]**2 + landmarks[i][1]**2))
			bearing.append(degrees(atan2(landmarks[i][1], landmarks[i][0])))
			bearing[i] = round((bearing[i] + 135.0) * 4.0)

		for i in range(0, 1080):
			for j in range(len(bearing)):
				if i == (bearing[j]):
					scan.ranges.append(current_range[j])
					bearing_found = True
					break
			if bearing_found:
				bearing_found = False
				continue	
			else:
				scan.ranges.append(0)

		print len(scan.ranges)

		landmark_pub.publish(scan)
		#print bearing

	def retrieve_landmarks(self, msg):
		i = 0
		self.observations = []

		while i < len(msg.data):
			self.observations.append(msg.data[i:i+2])
			i += 2

		#print self.observations[-1]

	def predict(self, control):
		# Testing
		# Returns Predicted State and Predicted Covariance

		G3 = ekf_prediction.dg_dstate(self.state, control, self.robot_width)
		control_variance = ekf_prediction.sigma_control(control, self.control_motion_factor, self.control_turn_factor)
		V = ekf_prediction.dg_dcontrol(self.state, control, self.robot_width)
		R3 = dot(V, dot(control_variance, V.T))

		G_landmarks = eye(3+2*self.number_of_landmarks)
		G_landmarks[0:3,0:3] = G3

		R_landmarks = zeros([3+2*self.number_of_landmarks, 3+2*self.number_of_landmarks])
		R_landmarks[0:3,0:3] = R3

		g3 = ekf_prediction.g(self.state, control, self.robot_width)
		
		g_landmarks = zeros(3+2*self.number_of_landmarks)
		g_landmarks[0:3] = g3

		self.state = g_landmarks
		self.covariance = dot(G_landmarks, dot(self.covariance, G_landmarks.T)) + R_landmarks

		if self.DEBUG: 
			print ""
			print "Predicted State: ", self.state 
			print "State Dimensions: ", self.state.shape
			print "Predicted Covariance: ", self.covariance
			print "Covariance Dimensions: ", self.covariance.shape
			print ""

	def add_landmark(self, landmark_coords):
		# Testing
		# Updates State and Covariances for each new landmark

		self.number_of_landmarks += 1
		self.landmark_index += 1

		i = self.landmark_index

		updated_state = zeros([3+2*self.number_of_landmarks])
		updated_state[0:3] = self.state[0:3]
		updated_state[2*i+3:3+2*i+5] = landmark_coords

		updated_covariance = eye(3+2*self.number_of_landmarks)
		updated_covariance[0:3, 0:3] = self.covariance[0:3, 0:3]
		updated_covariance[3+(2*i):5+(2*i),3+(2*i):5+(2*i)] = diag([10**10, 10**10])

		self.state = updated_state
		self.covariance = updated_covariance

		if self.DEBUG:
			print ""
			print "Updated State with Landmark: ", self.state
			print ""
			print "Updated Covariance with landmark: ", self.covariance
			print ""

		return self.landmark_index


	def correct(self, measurement, landmark_index):
		# Testing
		# Returns Corrected State and Corrected Covariance

		if (self.landmark_index == -1):
			if (self.DEBUG):
				print "No landmarks detected!"
		else:
			landmark = self.state[3+2*landmark_index:3+2*landmark_index + 2]
			
			H3 = ekf_correction.dh_dstate(self.state, landmark, self.scanner_displacement)

			H_landmarks = zeros([2, 3+2*self.number_of_landmarks])
			H_landmarks[0:2, 0:3] = H3
			H_landmarks[0:2, 3+2*landmark_index:5+2*landmark_index] = -1 * H3[0:2, 0:2]

			H = H_landmarks

			Q = diag([self.measurement_distance_stddev**2, self.measurement_angle_stddev**2]) 
			K = dot(dot(self.covariance, H.T), linalg.inv(dot(H, dot(self.covariance, H.T)) + Q))

			innovation = array(measurement) - ekf_correction.h(self.state, landmark, self.scanner_displacement)
			
			self.state = self.state + dot(K, innovation)
			self.covariance = dot(eye(size(self.state)) - dot(K, H), self.covariance)

	def visualize_landmark_error_ellipse(self):
		# Helper function to visualize covariance in landmark position
		# Returns landmark position uncertainty

		return None # Replace later

	def visualize_pose_error_ellipse(self, covariance):
		# Helper function to visualize covariance in pose
		# Returns robot bearing uncertainty and position uncertainty

		eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
		angle = atan2(eigenvects[1,0], eigenvects[0,0])

		return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))

if __name__ == '__main__':
	try:
		print("EKF Main is Alive!")

		# Robot Contraints
		ticks_to_mm = rospy.get_param("/ekf/ticks_to_mm")
		robot_width = rospy.get_param("/ekf/robot_width")
		scanner_displacement = rospy.get_param("/ekf/scanner_displacement")

		# EKF Filter Constants
		control_motion_factor = rospy.get_param("/ekf/control_motion_factor")
		control_turn_factor = rospy.get_param("/ekf/control_turn_factor")
		measurement_distance_stddev = rospy.get_param("/ekf/measurement_distance_stddev")
		measurement_angle_stddev = rospy.get_param("/ekf/measurement_angle_stddev")

		# Initial Conditions
		x_0 = rospy.get_param("/ekf/x0")
		y_0 = rospy.get_param("/ekf/y0")
		theta_0 = rospy.get_param("/ekf/theta0")

		# Initial State
		initial_state = array([x_0, y_0, theta_0])

		# Initial Covariance
		initial_covariance = zeros([3,3])

		ekf = ExtendedKalmanFilter(initial_state, initial_covariance, robot_width,
								   scanner_displacement, control_motion_factor,
								   control_turn_factor, measurement_distance_stddev,
								   measurement_angle_stddev)

		rospy.init_node('EKF_main', anonymous=True)
		landmark_pub = rospy.Publisher('scan_landmarks', LaserScan, queue_size=10)
		scan_pub = rospy.Publisher('test_scan', LaserScan, queue_size=10)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			# Subscribe to encoder ticks here and initialize control array
			#ekf.predict(control)
			ekf.predict([10.0,12.0])

			# Subscribe to observations here
			rospy.Subscriber("/landmark", Float64MultiArray, ekf.retrieve_landmarks, queue_size = 10)
			#observations = [[1,2,3,2],[2,4,5,1]]

			#observations = [[2,3,4,1],[2,3,4,1]]
			#observations = [[1,2,3,-1]]

			#for obs in observations:
			#	measurement, cylinder_world, cylinder_scanner, cylinder_index = obs
			#	if cylinder_index == -1:
			#		cylinder_index = ekf.add_landmark(cylinder_world)
			#	ekf.correct(measurement, cylinder_index)

			ekf.loop_once = True

			# Publish map and pose to RVIZ here
			ekf.visualize_landmarks(ekf.observations)

			rate.sleep()

	except rospy.ROSInterruptException:
		pass
