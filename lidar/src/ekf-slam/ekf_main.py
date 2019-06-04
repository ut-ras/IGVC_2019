#!/usr/bin/env python

import rospy
from math import sin, cos, pi, atan2, sqrt, radians, degrees
from numpy import *
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import LaserScan

import ekf_prediction, ekf_correction

# This class needs to import functions from:
# 	- Landmark extraction
# 	- EKF Prediction
# 	- EKF Correction

class ExtendedKalmanFilter:
	def __init__(self, state, covariance, robot_width, scanner_displacement,
				 control_motion_factor, control_turn_factor,
				 measurement_distance_stddev, measurement_angle_stddev, angle_offset):
		self.state = state
		self.covariance = covariance

		self.robot_width = robot_width
		self.scanner_displacement = scanner_displacement
		self.control_motion_factor = control_motion_factor
		self.control_turn_factor = control_turn_factor
		self.measurement_distance_stddev = measurement_distance_stddev
		self.measurement_angle_stddev = measurement_angle_stddev
		self.angle_offset = angle_offset

		self.max_landmark_radius = 2.5

		self.observations, self.potential_landmarks = [], []
		self.control, self.previous_control = [0,0], [0,0] 
		self.previous_time = 0
		self.distance = [0,0]

		self.number_of_landmarks = 0
		self.landmark_index = -1
		self.loop_once = False

		self.DEBUG = False

	def convert_x_y_to_range_bearing(self, measurement, angle_offset):
		range = sqrt(measurement[0]**2 + measurement[1]**2)
		#bearing = (round(degrees(atan2(measurement[1], measurement[0]))) + angle_offset) * 4
		bearing = atan2(measurement[1], measurement[0])
		#bearing = round(bearing * 4.0)

		return array([range, bearing])

	def convert_range_bearing_to_x_y(self, measurement):
		pass

	def convert_x_y_to_global_x_y(self, measurement):
		x, y, theta = self.state[:3]
		d = self.scanner_displacement

		x_l = x + d*cos(theta)
		y_l = y + d*sin(theta)

		measurement += array([x_l, y_l])
		
		r, alpha = ekf_correction.h(self.state, measurement, self.scanner_displacement)
		#r = sqrt(measurement[0]**2 + measurement[1]**2)
		#alpha = atan2(measurement[1], measurement[0])
		#alpha = radians(alpha - self.angle_offset)

		x_meas = r*cos(alpha + theta) + x
		y_meas = r*sin(alpha + theta) + y

		return [x_meas, y_meas]

	def visualize_landmarks(self, landmarks):
		num_readings = 720
		laser_frequency = 40
		current_time = rospy.Time.now()

		scan = LaserScan()
		
		scan.header.stamp = current_time
		scan.header.frame_id = 'landmark_frame'
		scan.angle_min = -1.570796
		scan.angle_max = 1.570796
		scan.angle_increment = (scan.angle_max - scan.angle_min) / num_readings
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
			current_range.append(self.convert_x_y_to_range_bearing(landmarks[i], 90)[0])
			bearing.append(self.convert_x_y_to_range_bearing(landmarks[i], 90)[1])

		for i in range(0, num_readings):
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

		#print len(scan.ranges)

		landmark_pub.publish(scan)
		#print bearing

	def publish(self, pub):
		state = Float32MultiArray()
		state.data = self.state[:3]

		pub.publish(state)

	def updatePrediction(self, msg):
		current = rospy.get_time()
		time_elapsed = current - self.previous_time

		for i in range(len(msg.data)):
			self.control[i] = msg.data[i] * time_elapsed
			self.distance[i] += self.control[i]

		self.predict(self.control)
		#print "Control: ", self.control
		self.previous_time = current

	def updateCorrection(self, msg):
		i = 0
		self.observations = []

		while i < len(msg.data):
			self.observations.append(msg.data[i:i+2])
			i += 2

		#print ""

		if len(ekf.state) <= 3:
			for obs in ekf.observations:
				#obs = self.convert_x_y_to_range_bearing(obs)
				obs = self.convert_x_y_to_global_x_y(obs)
				ekf.add_landmark(obs)
		else:
			for obs in ekf.observations:
				#print obs
				#obs = self.convert_x_y_to_global_x_y(obs)
				self.assign_landmark(obs)
		#print ""
		#print self.state[3:5], self.state[5:7], self.state[7:9], self.state[9:11]
		#print ""
		print ""
		#print "Number of landmarks: ", self.number_of_landmarks
		print "State: ", self.state
		#print "Covariance: ", self.covariance
		#print self.state[3:5], self.state[5:7]
		print ""
		#print self.observations[-1]

	def assign_landmark(self, observation):
		landmarks = self.state[3:]
		#print ""
		#print len(landmarks)
		i = 0
		while i < len(landmarks):
			#print landmarks[i:i+2]
			#global_landmark = self.convert_x_y_to_global_x_y(landmarks[i:i+2])
			#global_landmark = landmarks[i:i+2]
			global_obs = self.convert_x_y_to_global_x_y(observation)
			
			x_meas, y_meas = global_obs
			#dx = (landmarks[i] - landmark_coords[0])
			#dy = (landmarks[i+1] - landmark_coords[1])

			dx = (landmarks[i] - x_meas)
			dy = (landmarks[i+1] - y_meas)

			distance = sqrt(dx**2 + dy**2)
			#print "number of landmarks: ", self.number_of_landmarks
			#print "distance: ", distance, i
			#print [landmarks[i], landmarks[i+1]], landmark_coords
			#print ""
			# To do here: Loop thru all landmarks
			# if correspondence found, then assign correspondence and correct
			# If no correspondonce found but not at end, continue
			# If at end and no correspondence found, then add to list of landmarks
			# Finished above - will run through later
			if distance <= self.max_landmark_radius and i < len(landmarks):
				#print "correspondence found!"
				#print ""
				landmark_index = i / 2
				self.correct(observation, landmark_index)
				print "Distance: ", distance
				break
			# To do here:
			# If similar observation is found in list of potential landmarks then update counter
			# If no similar observation is found, add to list of potential landmarks
			# Once counter reaches a certain threshold, say 10 then finally update state with new landmark
			# Ignoring adding landmarks for now to test rest of the kalman filter
			elif distance > self.max_landmark_radius and i == len(landmarks)-2:
				#self.add_landmark(global_obs)
				#self.add_landmark(observation)

				#landmark_index = len(landmarks) / 2
				#self.correct(observation, landmark_index)
				print "Distance: ", distance
				pass

			i += 2

	def add_landmark(self, landmark_coords):
		# Testing
		# Updates State and Covariances for each new landmark

		self.number_of_landmarks += 1
		self.landmark_index += 1

		i = self.landmark_index

		updated_state = zeros([3+2*self.number_of_landmarks])
		updated_state[0:3] = self.state[0:3]
		updated_state[3:2*i+3] = self.state[3:]
		updated_state[2*i+3:2*i+5] = landmark_coords

		self.state = updated_state

		#print "updated state: ", updated_state

		updated_covariance = eye(3+2*self.number_of_landmarks)
		updated_covariance[0:3, 0:3] = self.covariance[0:3, 0:3]
		updated_covariance[3+(2*i):5+(2*i),3+(2*i):5+(2*i)] = diag([10**5, 10**5])

		self.covariance = updated_covariance

		#print "Current landmarks found: ", self.state[3:]

		if self.DEBUG:
			print ""
			print "Current landmarks found: ", self.state[3:] 
			print ""

		return self.landmark_index

	def predict(self, control):
		# Testing
		# Returns Predicted State and Predicted Covariance
		#current_time = rospy.get_time()

		if self.number_of_landmarks == size(self.state[3:]) / 2:
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
			#print self.number_of_landmarks, size(self.state[3:])
			g_landmarks[3:] = self.state[3:]
			g_landmarks[0:3] = g3

			self.state = g_landmarks
			# To Do - This is causing problems
			if size(self.covariance) == size(G_landmarks) and size(self.covariance) == size(R_landmarks):
				self.covariance = dot(G_landmarks, dot(self.covariance, G_landmarks.T)) + R_landmarks

			if self.DEBUG: 
				print ""
				print "Control: ", self.control
				print "Predicted State: ", self.state 
				print "State Dimensions: ", self.state.shape
				#print "Predicted Covariance: ", self.covariance
				#print "Covariance Dimensions: ", self.covariance.shape
				print "Number of landmarks: ", self.number_of_landmarks
				print ""

	def correct(self, measurement, landmark_index):
		# Testing
		# Returns Corrected State and Corrected Covariance

		if (self.landmark_index == -1):
			if (self.DEBUG):
				print "No landmarks detected!"
		elif self.number_of_landmarks == size(self.state[3:]) / 2:
			landmark = self.state[3+2*landmark_index:3+2*landmark_index + 2]
			
			H3 = ekf_correction.dh_dstate(self.state, landmark, self.scanner_displacement)

			H_landmarks = zeros([2, 3+2*self.number_of_landmarks])
			H_landmarks[0:2, 0:3] = H3
			H_landmarks[0:2, 3+2*landmark_index:5+2*landmark_index] = -1 * H3[0:2, 0:2]

			H = H_landmarks

			Q = diag([self.measurement_distance_stddev**2, self.measurement_angle_stddev**2]) 
			#print "covariance, H, Q: ", self.covariance.shape[0], H.shape[1], Q.shape[0]
			# To Do - This is causing problems
			if self.covariance.shape[0] == H.shape[1] and H.shape[0] == Q.shape[0]:
				K = dot(dot(self.covariance, H.T), linalg.inv(dot(H, dot(self.covariance, H.T)) + Q))

				# Transform global reference into local coordinates before calculating innovation
				#measurement = self.convert_x_y_to_range_bearing(array(measurement))
				#print measurement
				#measurement[1] = radians(measurement[1]) - pi/2
				#measurement[1] = radians(measurement[1])

				print "measurement: ", measurement
				print "landmark: ", landmark

				#measurement = ekf_correction.h(self.state, measurement, self.scanner_displacement)
				#innovation = measurement - ekf_correction.h(self.state, landmark, self.scanner_displacement)
				
				measurement = self.convert_x_y_to_range_bearing(measurement, 0)
				#landmark = self.convert_x_y_to_range_bearing(landmark, 0)

				#measurement = ekf_correction.h(self.state, measurement, self.scanner_displacement)
				landmark = ekf_correction.h(self.state, landmark, self.scanner_displacement)
				innovation = measurement - landmark

				#print measurement, landmark, innovation 

				#innovation = array(measurement) - array(landmark)
				innovation[1] = ((innovation[1] + pi) % (2*pi)) - pi
				print "Innovation: ", innovation

				#print ""
				#print "Kalman Gain: ", K
				#print ""

				#print H
				#print ""

				#print self.state
				#print dot(K, innovation)
				#print ekf_correction.h(self.state, landmark, self.scanner_displacement)
				self.state = self.state + dot(K, innovation)
				#print self.state
				self.covariance = dot(eye(size(self.state)) - dot(K, H), self.covariance)
				#print self.covariance
			else:
				print "Cannot multiply covarance and H or H and Q!"
		else:
			print "Number of Landmarks not equal to size of landmark state!", self.number_of_landmarks, size(self.state[3:])/2


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
		angle_offset = rospy.get_param("/ekf/angle_offset")

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
								   measurement_angle_stddev, angle_offset)

		rospy.init_node('EKF_main', anonymous=True)
		landmark_pub = rospy.Publisher('scan_landmarks', LaserScan, queue_size=10)
		scan_pub = rospy.Publisher('test_scan', LaserScan, queue_size=10)
		state_pub = rospy.Publisher('filtered_state', Float32MultiArray, queue_size=10)

		# Subscribe to observations here
		rospy.Subscriber("/landmark", Float64MultiArray, ekf.updateCorrection, queue_size = 10)

		# Subscribe to encoder data here
		rospy.Subscriber('wheel_velocity', Float32MultiArray, ekf.updatePrediction, queue_size=10)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			# Subscribe to encoder ticks here and initialize control array
			#ekf.predict(ekf.control)
			#ekf.predict([10.0,12.0])

			#observations = [[1,2,3,2],[2,4,5,1]]

			#observations = [[2,3,4,1],[2,3,4,1]]
			#observations = [[1,2,3,-1]]

			#for obs in observations:
			#	measurement, cylinder_world, cylinder_scanner, cylinder_index = obs
			#	if cylinder_index == -1:
			#		cylinder_index = ekf.add_landmark(cylinder_world)
			#	ekf.correct(measurement, cylinder_index)

			ekf.publish(state_pub)

			ekf.loop_once = True

			# Publish map and pose to RVIZ here
			ekf.visualize_landmarks(ekf.observations)

			rate.sleep()

	except rospy.ROSInterruptException:
		pass
