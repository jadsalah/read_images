#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys

class send_cmd:

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.subscriber = rospy.Subscriber("/centroids",Float32, self.callback)

	def mycommand(self,dat):
		self.pub.publish(dat)

	def callback(self,ros_data):
		vel_msg = Twist()
		r_cx = ros_data.data
		Kp = 0.0005
		err = 320 - r_cx
		vel_msg.linear.x = 2
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = Kp*err
		self.mycommand(vel_msg)
		print str(ros_data.data)


def main(args):
	sc = send_cmd()
	rospy.init_node('send_command', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"
		
if __name__ == '__main__':
	main(sys.argv)

