#!/usr/bin/env python

'''
This module contains a list of helper prediction functions
for the Extended Kalman Filter 
in the case of a differential wheeled robot
'''

import rospy
from math import sin, cos, pi, atan2, sqrt
from std_msgs.msg import Float32MultiArray
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

class EKFPredict():
	def __init__(self, initial_state, initial_covariance, robot_width, 
					control_motion_factor, control_turn_factor):
		self.state = initial_state
		self.covariance = initial_covariance
		self.robot_width = robot_width
		self.control_motion_factor = control_motion_factor
		self.control_turn_factor = control_turn_factor

		self.number_of_landmarks = 0
		self.landmark_index = 0

		self.DEBUG = True

		self.control = [0,0]
		self.distance = [0,0]
		#self.previous_time = [0,0]
		self.previous_time = 0

	def encoderCallback(self, msg):
		current = rospy.get_time()
		time_elapsed = current - self.previous_time

		for i in range(len(msg.data)):
			self.control[i] = msg.data[i] * time_elapsed
			self.distance[i] += self.control[i]

		self.predict(self.control)
		self.previous_time = current

	def publish(self, pub):
		state = Float32MultiArray()
		state.data = self.state

		pub.publish(state)

	def predict(self, control):
		# Testing
		# Returns Predicted State and Predicted Covariance
		current_time = rospy.get_time()

		G3 = dg_dstate(self.state, self.control, self.robot_width)
		control_variance = sigma_control(self.control, self.control_motion_factor, self.control_turn_factor)
		V = dg_dcontrol(self.state, self.control, self.robot_width)
		R3 = dot(V, dot(control_variance, V.T))

		G_landmarks = eye(3+2*self.number_of_landmarks)
		G_landmarks[0:3,0:3] = G3

		R_landmarks = zeros([3+2*self.number_of_landmarks, 3+2*self.number_of_landmarks])
		R_landmarks[0:3,0:3] = R3

		g3 = g(self.state, control, self.robot_width)
		
		g_landmarks = zeros(3+2*self.number_of_landmarks)
		g_landmarks[0:3] = g3

		self.state = g_landmarks
		self.covariance = dot(G_landmarks, dot(self.covariance, G_landmarks.T)) + R_landmarks

		if self.DEBUG: 
			print ""
			print "Control: ", control
			print "Testing state estimate: ", self.distance
			#print "Predict time loop: ", rospy.get_time() - current_time
			print "Predicted State: ", self.state 
			#print "State Dimensions: ", self.state.shape
			#print "Predicted Covariance: ", self.covariance
			#print "Covariance Dimensions: ", self.covariance.shape
			#print "Number of landmarks: ", self.number_of_landmarks
			print ""

if __name__ == '__main__':
	try:
		print("EKF Prediction is Alive!")

		rospy.init_node("EKF_Prediction", anonymous=True)

		# Initial Conditions
		x_0 = rospy.get_param("/ekf/x0")
		y_0 = rospy.get_param("/ekf/y0")
		theta_0 = rospy.get_param("/ekf/theta0")

		# EKF Filter Constants
		robot_width = rospy.get_param("/ekf/robot_width")
		control_motion_factor = rospy.get_param("/ekf/control_motion_factor")
		control_turn_factor = rospy.get_param("/ekf/control_turn_factor")

		# Initial State
		initial_state = array([x_0, y_0, theta_0])

		# Initial Covariance
		initial_covariance = zeros([3,3])

		ekf_predict = EKFPredict(initial_state, initial_covariance, robot_width,
						   control_motion_factor, control_turn_factor)

		# Subscribe to encoder data here
		rospy.Subscriber('wheel_velocity', Float32MultiArray, ekf_predict.encoderCallback, queue_size=10)

		# Set up predicted state estimate
		predict_state_estimate_pub = rospy.Publisher('predicted_state', Float32MultiArray, queue_size=10)

		while not rospy.is_shutdown():
			#ekf_predict.predict()
			ekf_predict.publish(predict_state_estimate_pub)
	except rospy.ROSInterruptException:
		pass
