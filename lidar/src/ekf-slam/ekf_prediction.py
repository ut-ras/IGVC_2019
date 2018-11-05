#!/usr/bin/env python

'''
This module contains a list of helper prediction functions
for the Extended Kalman Filter 
in the case of a differential wheeled robot
'''

import rospy
from math import sin, cos, pi, atan2, sqrt
from numpy import *

# State function
def g(state, control, width):
	x, y, theta = state[0:3]
	l, r = control

	if r != l:
		alpha = (r - l) / width
		radius = l / alpha

		g1 = x + (radius + width / 2.0) * (sin(theta + alpha) - sin(theta))
		g2 = y + (radius + width / 2.0) * (-cos(theta + alpha) + cos(theta))
		g3 = (theta + alpha + pi) % (2.0 * pi) - pi
	else:
		g1 = x + (l * cos(theta))
		g2 = y + (l * sin(theta))
		g3 = theta

	return array([g1, g2, g3])

# Jacobian of State Function wrt State
def dg_dstate(state, control, width):
	theta = state[2]
	l, r = control

	if r != l:
		alpha = (r - l) / width
		theta_prime = theta + alpha

		m = array([[1.0, 0.0, ((l / alpha) + width / 2.0) * (cos(theta_prime) - cos(theta))],
				  [0.0, 1.0, ((l / alpha) + width / 2.0) * (sin(theta_prime) - sin(theta))],
				  [0.0, 0.0, 1.0]])

	else:
		m = array([[1.0, 0.0, -(l * sin(theta))],
				  [0.0, 1.0, (l * cos(theta))],
				  [0.0, 0.0, 1.0]])

	return m

# Jacobian of State Function wrt Control
def dg_dcontrol(state, control, width):
	theta = state[2]
	l, r = tuple(control)

	if r != l:
		rml = r - l
		rml2 = rml * rml

		alpha = rml / width
		theta_prime = theta + alpha 

		dg1_dl = (width*r / rml2) * (sin(theta_prime) - sin(theta)) \
		- (r+l) / (2*rml) * cos(theta_prime)

		dg2_dl = (width*r / rml2) * (-cos(theta_prime) + cos(theta)) \
		- (r+l) / (2*rml) * sin(theta_prime) 

		dg1_dr = (-width*l / rml2) * (sin(theta_prime) - sin(theta)) \
		+ (r+l) / (2*rml) * cos(theta_prime)

		dg2_dr = (-width*l / rml2) * (-cos(theta_prime) + cos(theta)) \
		+ (r+l) / (2*rml) * sin(theta_prime)
	
	else:
		dg1_dl = 0.5 * (cos(theta) + (l/width) * sin(theta))
		dg1_dr = 0.5 * (cos(theta) - (l/width) * sin(theta))
		dg2_dl = 0.5 * (sin(theta) - (l/width) * cos(theta))
		dg2_dr = 0.5 * (sin(theta) + (l/width) * cos(theta))

	dg3_dl = -(1.0 / width)
	dg3_dr = (1.0 / width)

	return array([[dg1_dl, dg1_dr], [dg2_dl, dg2_dr], [dg3_dl, dg3_dr]])

# Returns Control Covariance Matrix
def sigma_control(control, control_motion_factor, control_turn_factor):
	l, r = control

	left_control_variance = (control_motion_factor * l)**2 + (control_turn_factor * (l-r))**2
	right_control_variance = (control_motion_factor * r)**2 + (control_turn_factor * (l-r))**2

	return array([[left_control_variance, 0.0],
				 [0.0, right_control_variance]])

if __name__ == '__main__':
	try:
		print("EKF Prediction is Alive!")
	except rospy.ROSInterruptException:
		pass
