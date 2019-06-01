#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, sqrt, pi
from numpy import *

linear_vel = turn_vel = 0

def mapf(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joy_cb(msg):
	global linear_vel, turn_vel
	steer_angle = atan2(msg.axes[1], -msg.axes[0])

	x = cos(steer_angle) * -msg.axes[0]
	y = sin(steer_angle) * msg.axes[1]

	steer_mag = sqrt(x**2 + y**2)

	linear_vel = msg.axes[1]

	left_trig, right_trig = msg.buttons[6], msg.buttons[7]
	left_trig_axis, right_trig_axis = msg.axes[2], msg.axes[5]

	if left_trig and not right_trig:
		turn_vel = mapf(left_trig_axis, -1.0, 1.0, 1.0, 0.0)
	elif right_trig and not left_trig:
		turn_vel = mapf(right_trig_axis, -1.0, 1.0, -1.0, 0.0)
	else:
		turn_vel = 0

	#print linear_vel, turn_vel

def main():
	global linear_vel, turn_vel
	motor_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('joy', Joy, joy_cb)

	twist = Twist()
	rate = rospy.Rate(30)

	while not rospy.is_shutdown():
		twist.linear.x = linear_vel; twist.linear.y = 0; twist.linear.z = 0;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn_vel

		motor_pub.publish(twist)
		rate.sleep()

if __name__ == '__main__':
	try:
		print "Joy Callback is Alive!"
		rospy.init_node('joy_callback_node', anonymous=True)

		main()
	except rospy.ROSInterruptException:
		pass
