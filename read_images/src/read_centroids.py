#!/usr/bin/env python
"""
This node will read values from the centroids topic and send them to cmd_vel topic as commands
The Control is just A P control, but i think we will need to add the saturated I and The D controls to the real Drone
"""
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys
from std_msgs.msg import String
import numpy as np
from matplotlib import pyplot as plt
import select as slt

class send_cmd:

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.subscriber = rospy.Subscriber("/centroids",String, self.callback)
		self.centroids = np.empty((0,1), int)

	def mycommand(self,dat):
		self.pub.publish(dat)

	def saveData(self):
		np.savetxt("/home/jad/catkin_ws/src/read_images/src/dat.csv", self.centroids, delimiter=",")

	def callback(self,ros_data):
		rec_msg = ros_data.data
		r_cx = float(rec_msg.split("_")[0])
		r_sD = float(rec_msg.split("_")[1])
		self.centroids = np.append(self.centroids,r_cx)
		vel_msg = Twist()
		Kp = 0.0005
		Kp2 = 0.2
		err = 320 - r_cx
		vel_msg.linear.x = 1
		vel_msg.linear.y = Kp2*r_sD
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = Kp*err
		self.mycommand(vel_msg)
		print str(ros_data.data)


def main(args):
	sc = send_cmd()
	rospy.init_node('send_command', anonymous=True)
	rospy.spin()
	sc.saveData()
if __name__ == '__main__':
	main(sys.argv)

