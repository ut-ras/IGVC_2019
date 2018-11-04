#!/usr/bin/env python

import rospy
from math import sin, cos, pi, atan2, sqrt
from numpy import *

# Measurement Function
def h(state, landmark, scanner_displacement):
	x = state[0]
	y = state[1]
	theta = state[2]

	x_l = x + scanner_displacement * cos(theta)
	y_l = y + scanner_displacement * sin(theta)

	x_m = landmark[0]
	x_y = landmark[1]

	dx = x_m - x_l
	dy = y_m - y_l
	q = (dx)^2 + (dy)^2

	r = sqrt((dx)^2 + (dy)^2)
	alpha = (atan2(dy/dx) - theta + pi) % (2 * pi) - pi

	return array([r, alpha])

# Jacobian of Measurement Function wrt State
def dh_state(state, landmark, scanner_displacement):
	x = state[0]
	y = state[1]
	theta = state[2]

	x_l = x + scanner_displacement * cos(theta)
	y_l = y + scanner_displacement * sin(theta)

	x_m = landmark[0]
	x_y = landmark[1]

	dx = x_m - x_l
	dy = y_m - y_l
	q = (dx)^2 + (dy)^2

	dr_dx = -(dx / sqrt(q))
	dr_dy = -(dy / sqrt(q))
	dr_dtheta = (d / sqrt(q)) * (dx*sin(theta) - dy*cos(theta))
	dalpha_dx = dy / q
	dalpha_dy = -dx / q
	dalpha_dtheta = -(d / q) * (dx*cos(theta) + dy*sin(theta)) - 1.0

	return array([dr_dx, dr_dy, dr_dtheta],
				 [dalpha_dx, dalpha_dy, dalpha_dtheta])

def correct(self, measurement, landmark_index):
	# Testing
	# Returns Corrected State and Corrected Covariance


if __name__ == '__main__':
	try:
		print("EKF Correction is Alive!")
	except rospy.ROSInterruptException:
		pass
