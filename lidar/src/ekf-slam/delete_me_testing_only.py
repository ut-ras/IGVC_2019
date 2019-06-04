import ekf_prediction, ekf_correction
from numpy import *

def main():
	state = [100.0, 100.0, 0.0, 200.0, 250.0]
	covariance = array([[1,2,3,0,0],
						[0,1,0,0,0],
						[0,0,1,0,0],
						[0,0,0,10**10,0],
						[0,0,0,0,10**10]])
	width = 150.0
	scanner_displacement = 10.0
	control_motion_factor = 0.35
	control_turn_factor = 0.6
	measurement_distance_stddev = 600.0
	measurement_angle_stddev = 0.7854

	equalControl = [10.0,10.0]
	notEqualControl = [10.0, 15.0]

	landmark_coords = [200.0, 250.0]
	measurement = [180.0, 250.0]
	no_landmarks = -1
	first_landmark = 0
	landmark_index = 0

	H3 = ekf_correction.dh_dstate(state, landmark_coords, scanner_displacement)
	H_landmarks = zeros([2, 3+2])
	H_landmarks[0:2, 0:3] = H3
	H_landmarks[0:2, 3+2*landmark_index:5+2*landmark_index] = -1 * H3[0:2, 0:2]

	H = H_landmarks
	Q = diag([measurement_distance_stddev**2, measurement_angle_stddev**2]) 
	K = dot(dot(covariance, H.T), linalg.inv(dot(H, dot(covariance, H.T)) + Q))

	print H
	print H.T
	print Q


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		pass