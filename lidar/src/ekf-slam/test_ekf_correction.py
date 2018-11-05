#!/usr/bin/env python

PKG = 'lidar'

import rospy
import unittest
import ekf_correction
from numpy import *

class EKF_CorrectionTest(unittest.TestCase):
	@classmethod
	def setUpClass(self):
		self.state = [100.0, 100.0, 0.0]
		self.width = 150.0
		self.landmark = [200.0,250.0]
		self.scannerDisplacement = 10.0

	def test_h(self):
		test_array = array([174.9285568,1.030376827])
		output = ekf_correction.h(self.state, self.landmark, self.scannerDisplacement)
		self.assertTrue(allclose(output, test_array),
					    'h did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_array))

	def test_dh_dstate(self):
		test_matrix = array([[-0.5144957554,-0.8574929257,-8.574929257],
							 [0.0049019608,-0.0029411765,-1.029411765]])
		output = ekf_correction.dh_dstate(self.state, self.landmark, self.scannerDisplacement)
		self.assertTrue(allclose(output, test_matrix),
					    'dh_dstate did not equal test:\n\nOutput:\n{}\n\nTest:\n{}\n'\
					    .format(output, test_matrix))

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'EKF_CorrectionTest', EKF_CorrectionTest)

