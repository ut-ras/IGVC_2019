#!/usr/bin/env python

PKG = 'lidar'

import rospy
import unittest
import ekf_prediction
from numpy import *

class EKF_PredictionTest(unittest.TestCase):
	@classmethod
	def setUpClass(self):
		self.state = [100.0, 100.0, 0.0]
		self.width = 150.0
		self.equalControl = [10.0, 10.0]
		self.notEqualControl = [10.0, 15.0]
		self.control_motion_factor = 0.35
		self.control_turn_factor = 0.6

	def test_g(self):
		test_array = array([110.0,100.0,0.0])
		output = ekf_prediction.g(self.state, self.equalControl, self.width)
		self.assertTrue(allclose(output, test_array),
					    'g (l=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_array))

		test_array = array([112.4976853,100.208314,0.0333333])
		output = ekf_prediction.g(self.state, self.notEqualControl, self.width)
		self.assertTrue(allclose(output, test_array),
					    'g (l!=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_array))

	def test_dg_dstate(self):
		test_matrix = array([[1.0,0.0,0.0],
							 [0.0,1.0,10.0],
							 [0.0,0.0,1.0]])
		output = ekf_prediction.dg_dstate(self.state, self.equalControl, self.width)
		self.assertTrue(allclose(output, test_matrix),
					    'dg_dstate (l=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

		test_matrix = array([[1.0,0.0,-0.208314],
							 [0.0,1.0,12.497685],
							 [0.0,0.0,1.0]])
		output = ekf_prediction.dg_dstate(self.state, self.notEqualControl, self.width)
		self.assertTrue(allclose(output, test_matrix),
					    'dg_dstate (l!=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

	def test_dg_dcontrol(self):
		test_matrix = array([[0.5,0.5],
							 [-0.0333333,0.0333333],
							 [-0.0066666,0.0066666]])
		output = ekf_prediction.dg_dcontrol(self.state, self.equalControl, self.width)
		self.assertTrue(allclose(output, test_matrix),
					    'dg_dcontrol (l=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

		test_matrix = array([[0.5008332356, 0.4989815895],
							 [-0.0333225316,0.0499876551],
							 [-0.0066666667,0.0066666667]])
		output = ekf_prediction.dg_dcontrol(self.state, self.notEqualControl, self.width)
		self.assertTrue(allclose(output, test_matrix),
					    'dg_dcontrol (l!=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

	def test_sigma_control(self):
		test_matrix = array([[12.25,0],
							 [0,12.25]])
		output = ekf_prediction.sigma_control(self.equalControl, self.control_motion_factor, self.control_turn_factor)
		self.assertTrue(allclose(output, test_matrix),
					    'sigma_control (l!=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

		test_matrix = array([[21.25,0],
							 [0,36.5625]])
		output = ekf_prediction.sigma_control(self.notEqualControl, self.control_motion_factor, self.control_turn_factor)
		self.assertTrue(allclose(output, test_matrix),
					    'sigma_control (l!=r) did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))		

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'EKF_PredictionTest', EKF_PredictionTest)

