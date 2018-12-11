#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys

class send_cmd:

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.subscriber = rospy.Subscriber("/centroids",Float32, self.callback)

	def callback(self,ros_data):
		print str(ros_data.data)

	def listener(self):
		rospy.Subscriber("/centroids", Float32, callback)
		rospy.spin()



"""

def sendCmdVel(diff):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.init_node('cmd_vel_node', anonymous=True)


def listener():
	rospy.init_node('read_centroids', anonymous=True)
	rospy.Subscriber("/centroids", Float32, callback)
	rospy.spin()

def callback(ros_data):
	print str(ros_data.data)

if __name__ == '__main__':
	listener()
"""

def main(args):
	sc = send_cmd()
	rospy.init_node('send command', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"
		
if __name__ == '__main__':
	main(sys.argv)
