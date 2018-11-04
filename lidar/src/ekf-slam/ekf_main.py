#!/usr/bin/env python

import rospy
from math import sin, cos, pi, atan2, sqrt
from numpy import *

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

	def visualize_landmark_error_ellipse(self):
		# Helper function to visualize covariance in landmark position
		# Returns landmark position uncertainty

		return None # Replace later

	def visualize_pose_error_ellipse(covariance):
		# Helper function to visualize covariance in pose
		# Returns robot bearing uncertainty and position uncertainty

		return None # Replace later

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

	except rospy.ROSInterruptException:
		pass
