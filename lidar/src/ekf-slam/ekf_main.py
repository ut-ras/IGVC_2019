#!/usr/bin/env python

import rospy
from math import sin, cos, pi, atan2, sqrt
from numpy import *

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

		self.number_of_landmarks = 0
		self.landmark_index = 0

	def predict(self, control):
		# Testing
		# Returns Predicted State and Predicted Covariance

		G3 = ekf_prediction.dg_dstate(self.state, control, self.robot_width)
		control_variance = ekf_prediction.sigma_control(control)
		V = ekf_prediction.dg_dcontrol(self.state, control, self.robot_width)
		R3 = dot(V, dot(control_variance, V.T))

		G_landmarks = eye(3+2*self.number_of_landmarks)
		G_landmarks[0:3,0:3] = G3

		R_landmarks = zeros([3+2*self.number_of_landmarks, 3+2*self.number_of_landmarks])
		R_landmarks[0:3,0:3] = R3

		self.state = ekf_prediction.g(self.state, control, self.robot_width) # Replace later
		self.covariance = dot(G_landmarks, dot(self.covariance, G_landmarks.T)) + R_landmarks

	def add_landmark(self, landmark_coords):
		# Testing
		# Updates State and Covariances for each new landmark

		self.number_of_landmarks += 1
		self.landmark_index += 1

		i = self.landmark_index

		updated_state = zeros(3+2*self.number_of_landmarks)
		updated_state[0:3] = self.state
		updated_state[2*i+3:2*i+5] = landmark_coords

		updated_covariance = eye(3+2*self.number_of_landmarks)
		updated_covariance[0:3] = self.covariance
		updated_covariance[2*i+3:2*i+5, 2*i+3:2*i+5] = diag([10**10, 10**10])

		return updated_state, updated_covariance


	def correct(self, measurement, landmark_index):
		# Testing
		# Returns Corrected State and Corrected Covariance

		landmark = self.state[3+landmark_index:3+landmark_index + 2]
		H3 = ekf_correction.dh_dstate(self.state, landmark, self.scanner_displacement)

		H = H3 # Change later to include new terms

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

		while not rospy.is_shutdown():
			# Subscribe to encoder ticks here and initialize control array
			ekf.predict(control)

			# Subscribe to observations here
			for obs in range(len(observations)):
				measurement, cylinder_world, cylinder_scanner, cylinder_index = obs
				if cylinder_index == -1:
					cylinder_index = ekf.add_landmark(cylinder_world)
				ekf.correct(measurement, cylinder_index)

			# Publish map and pose to RVIZ here

	except rospy.ROSInterruptException:
		pass
