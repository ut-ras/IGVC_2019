#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from math import sin, cos, radians, exp, isnan
from numpy import *

class OccupancyGridMap:
	def __init__(self, resolution, width, height, origin, p_occ):
		self.resolution = resolution
		self.width = width
		self.height = height
		self.origin = origin

		self.map = OccupancyGrid()
		self.map.header.stamp = rospy.Time.now()
		self.map.header.frame_id = "map"

		self.map.info.resolution = resolution
		self.map.info.width = width
		self.map.info.height = height
		self.map.info.origin = Pose(Point(-width/20,-height/20,0), Quaternion(0,0,0,1))

		self.state = [0,0,0]
		self.log_occupied = log(p_occ / (1-p_occ))
		self.log_free = log((1-p_occ) / p_occ)

		self.unexplored_prob = -1.0
		self.min_prob = 0.0
		self.max_prob = 100.0

	def inverse_sensor_model(self, cell, measurement, state):
		pass

	def update_map(self, cell, cell_log_odds):
		if self.map.data == []:
			for x in range(self.width):
				for y in range(self.height):
					self.map.data.append(self.unexplored_prob)

		index = cell[1] * self.width
		index += cell[0]

		if not isnan(index):
			index = int(index)

		#print self.map.data[index]
		print len(self.map.data)

		if not isnan(self.map.data[index]):
			#current_log_odds = log(self.map.data[index] / (1 - self.map.data[index]))
			#current_log_odds += cell_log_odds

			#probability = 1 - (1 / (1 + exp(current_log_odds)))
			self.map.data[index] += int(cell_log_odds * self.max_prob)

	def laser_scan_cb(self, msg):
		measurement = msg.ranges

		if self.map.data == []:
			for x in range(int(self.width)):
				for y in range(int(self.height)):
					self.map.data.append(int(self.unexplored_prob))

		cell = [[-1.0]*int(self.width)]*int(self.height)

		for ray in range(len(measurement)):
			bearing = ray * 0.25 - 135.0
			current_range = measurement[ray]

			ray_x = cos(radians(bearing)) * current_range
			ray_y = sin(radians(bearing)) * current_range

			rounded_ray_x = (round(ray_x * (1/self.resolution)) / (1/self.resolution))
			rounded_ray_y = (round(ray_y * (1/self.resolution)) / (1/self.resolution))

			rounded_ray_x += self.width/2
			rounded_ray_y += self.height/2

			rounded_ray_x /= self.resolution
			rounded_ray_y /= self.resolution

			if rounded_ray_x < 9999999999 and rounded_ray_y < 9999999999:
				cell[int(ray_x)][int(ray_y)] = 1.0

			cell_log_odds = self.log_occupied

		for x in range(int(self.width)):
			for y in range(int(self.height)):
				if cell[x][y] == 0.0:
					current_cell = [x,y]
					cell_log_odds = self.log_free

					self.update_map(current_cell, cell_log_odds)
				elif cell[x][y] == 1.0:
					current_cell = [x,y]
					cell_log_odds = self.log_occupied

					self.update_map(current_cell, cell_log_odds)

	def pose_cb(self, msg):
		self.state = msg.pose

	def main(self):
		map_pub.publish(self.map)

if __name__ == '__main__':
	try:
		print('Mapper is Alive!')
		rospy.init_node('mapper', anonymous=True)

		resolution = rospy.get_param("/mapper/resolution")
		width = rospy.get_param("/mapper/width")
		height = rospy.get_param("/mapper/height")
		origin = rospy.get_param("/mapper/origin")
		p_occ = rospy.get_param("/mapper/probability_occupied")

		OCM = OccupancyGridMap(resolution, width, height, origin, p_occ)
		rospy.Subscriber("/scan", LaserScan, OCM.laser_scan_cb)

		# Replace fake_pose later once EKF_SLAM is done
		# EKF_SLAM -> Publish Pose with covariance
		rospy.Subscriber("/fake_pose", PoseWithCovariance, OCM.pose_cb)
		map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

		rate = rospy.Rate(10) # 10 Hz

		while not rospy.is_shutdown():
			OCM.main()
			rate.sleep()

	except rospy.ROSInterruptException:
		pass

