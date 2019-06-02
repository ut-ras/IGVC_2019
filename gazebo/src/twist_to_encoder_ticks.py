#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
	Copyright (C) 2012 Jon Stephan. 
	Edited: Ricky Chen.
	 
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist 

class TwistToMotors():
	def __init__(self):
		rospy.init_node("twist_to_motors")
		nodename = rospy.get_name()
		rospy.loginfo("%s started" % nodename)
	
		self.w = rospy.get_param("~base_width", 0.2)
	
		self.pub_wheel_velocity = rospy.Publisher('wheel_velocity', Float32MultiArray, queue_size=10)
		self.pub_wheel_distance = rospy.Publisher('wheel_distance', Float32MultiArray, queue_size=10)

		rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
	
		self.rate = rospy.get_param("~rate", 50)
		self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
		self.left = 0
		self.right = 0

		self.wheel_velocity, self.wheel_distance = Float32MultiArray(), Float32MultiArray()
		self.distance, self.previous = [0,0], [0,0]

	def spin(self):
		r = rospy.Rate(self.rate)
		idle = rospy.Rate(10)
		then = rospy.Time.now()
		self.ticks_since_target = self.timeout_ticks
	
		while not rospy.is_shutdown():
			while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
				self.spinOnce()
				r.sleep()
			idle.sleep()
				
	def spinOnce(self):
		# dx = (l + r) / 2
		# dr = (r - l) / w
			
		self.right = 1.0 * self.dx + self.dr * self.w / 2 
		self.left = 1.0 * self.dx - self.dr * self.w / 2
		# rospy.loginfo("publishing: (%d, %d)", left, right) 

		self.wheel_velocity.data = [self.left, self.right]
		self.pub_wheel_velocity.publish(self.wheel_velocity)

		#print type(rospy.get_rostime())

		current = rospy.get_time()
		time_elapsed = current - self.previous[0]
		self.distance[0] += self.left * time_elapsed
		self.previous[0] = current

		current = rospy.get_time()
		time_elapsed = current - self.previous[1]
		self.distance[1] += self.right * time_elapsed
		self.previous[1] = current
			
		self.wheel_distance.data = self.distance
		self.pub_wheel_distance.publish(self.wheel_distance)

		self.ticks_since_target += 1

	def twistCallback(self,msg):
		# rospy.loginfo("-D- twistCallback: %s" % str(msg))
		self.ticks_since_target = 0
		self.dx = msg.linear.x
		self.dr = msg.angular.z
		self.dy = msg.linear.y

if __name__ == '__main__':
	""" main """
	try:
		twistToMotors = TwistToMotors()
		twistToMotors.spin()
	except rospy.ROSInterruptException:
		pass