#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray

class Plotter():
	def __init__(self):
		self.counter = 0
		plt.ion()

	def plot_callback(self, msg):
		if self.counter % 10 == 0:
			#time = stamp.secs + stamp.nsecs * 1e-9
			plt.plot(msg.data[0])
			plt.axis("equal")
			plt.draw()
			plt.pause(0.00000000001)

		self.counter += 1

	def main(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			plt.show(block=True)

			print self.counter

			rate.sleep()

if __name__ == '__main__':
	try:
		print "Plotter Node is Alive!"
		rospy.init_node("Plotter_Node", anonymous=True)

		plotter = Plotter()
		rospy.Subscriber('/filtered_state', Float32MultiArray, plotter.plot_callback, queue_size=10)
		plotter.main()
	except rospy.ROSInterruptException:
		pass
