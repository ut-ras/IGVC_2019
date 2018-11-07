#!/usr/bin/env python

PKG = 'lidar'

import rospy
import unittest
import ekf_main
from numpy import *

class EKF_MainTest(unittest.TestCase):
	@classmethod
	def setUpClass(self):
		self.state = [100.0, 100.0, 0.0]
		self.covariance = array([[1,2,3],
								 [0,1,0],
								 [0,0,1]])
		self.width = 150.0
		self.scannerDisplacement = 10.0
		self.control_motion_factor = 0.35
		self.control_turn_factor = 0.6
		self.measurement_distance_stddev = 600.0
		self.measurement_angle_stddev = 0.7854

		self.equalControl = [10.0,10.0]
		self.notEqualControl = [10.0, 15.0]

		self.measurement = [200.0, 250.0]
		self.no_landmarks = -1
		self.first_landmark = 0

		self.landmark_coords = self.measurement

	@staticmethod
	def reinitialize_object(self):
		ekf = ekf_main.ExtendedKalmanFilter(self.state, self.covariance, self.width, self.scannerDisplacement,
								   self.control_motion_factor, self.control_turn_factor,
								   self.measurement_distance_stddev, self.measurement_angle_stddev)

		return ekf

	def test_predict(self):
		ekf = EKF_MainTest.reinitialize_object(self)

		test_state = array([110,100,0.0])
		test_covariance = array([[7.125, 32, 3],
								 [0, 101.027222, 10.0054444],
								 [0, 10.0054444, 1.00108888]])

		ekf.predict(self.equalControl)

		self.assertTrue(allclose(ekf.state, test_state),
						'Predicted State (l=r) did not equal test state:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.state, test_state))

		self.assertTrue(allclose(ekf.covariance, test_covariance),
						'Predicted Covariance (l=r) did not equal test covariance:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.covariance, test_covariance))

		ekf = EKF_MainTest.reinitialize_object(self)

		test_state = array([112.49768531,100.2083140439,0.033333333])
		test_covariance = array([[14.85210103, 37.4469463, 2.842361335],
								 [-2.04610963, 157.3070951, 12.5145905],
								 [-0.157638665, 12.5145905, 1.00256944]]) # <-- Replace with test case

		ekf.predict(self.notEqualControl)

		self.assertTrue(allclose(ekf.state, test_state),
						'Predicted State (l!=r) did not equal test state:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.state, test_state))

		self.assertTrue(allclose(ekf.covariance, test_covariance),
						'Predicted Covariance (l!=r) did not equal test covariance:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.covariance, test_covariance))

	def test_add_landmark(self):
		ekf = EKF_MainTest.reinitialize_object(self)

		test_state = array([100,100,0.0,200.0,250.0])
		test_covariance = array([[1,2,3,0,0],
								 [0,1,0,0,0],
								 [0,0,1,0,0],
								 [0,0,0,10**10,0],
								 [0,0,0,0,10**10]])
		test_landmark_index = 0

		ekf.add_landmark(self.landmark_coords)

		self.assertTrue(allclose(ekf.state, test_state),
						'Updated State with Landmark did not equal test state:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.state, test_state))

		self.assertTrue(allclose(ekf.covariance, test_covariance),
						'Updated Covariance with Landmark did not equal test covariance:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.covariance, test_covariance))

		self.assertTrue(allclose(ekf.landmark_index, test_landmark_index),
						'Updated landmark index did not equal test landmark index:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.landmark_index, test_landmark_index))

	def test_correct(self):
		ekf = EKF_MainTest.reinitialize_object(self)

		test_state = array([-362.8160586, 99.56289878, -154.22621, 200, 250]) # <-- Replace with test case
		test_covariance = array([[1,2,3,0,0],
								 [0,1,0,0,0],
								 [0,0,1,0,0],
								 [0,0,0,10**10,0],
								 [0,0,0,0,10**10]]) # <-- Replace with test case

		ekf.add_landmark(self.landmark_coords)
		ekf.correct(self.measurement, self.first_landmark)

		self.assertTrue(allclose(ekf.state, test_state),
						'Corrected State did not equal test state:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.state, test_state))

		self.assertTrue(allclose(ekf.covariance, test_covariance),
						'Corrected Covariance did not equal test covariance:\n\nOutput:\n{}\n\nTest:\n{}\n'\
						.format(ekf.covariance, test_covariance))

	def test_visualize_landmark_error_ellipse(self):
		pass # Replace later

	def test_visualize_pose_error_ellipse(self):
		ekf = EKF_MainTest.reinitialize_object(self)

		test_eigenvals = [1,2,3] # <-- Replace with test case
		test_eigenvects = [1,2,3] # <-- Replace with test case
		test_angle = 20 # <-- Replace with test case

if __name__ == '__main__':
	unittest.main()
	#import rostest
	#rostest.rosrun(PKG, 'EKF_CorrectionTest', EKF_CorrectionTest)

