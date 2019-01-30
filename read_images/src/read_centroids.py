#!/usr/bin/env python

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
		self.newr_cx = 0
		self.oldr_cx = 0

	def mycommand(self,dat):
		self.pub.publish(dat)

	def saveData(self):
		np.savetxt("/home/jad/catkin_ws/src/read_images/src/dat.csv", self.centroids, delimiter=",")

	def callback(self,ros_data):
		rec_msg = ros_data.data
		r_cx = float(rec_msg.split("_")[0])
		r_sD = float(rec_msg.split("_")[1])
		
		self.newr_cx = r_cx
		if self.oldr_cx != 0:
			self.newr_cx = self.oldr_cx + 0.15*(self.newr_cx-self.oldr_cx)
		self.centroids = np.append(self.centroids,self.newr_cx)
		vel_msg = Twist()
		Kp = 0.0005
		Kp2 = 0.02
		err = 320 - self.newr_cx
		vel_msg.linear.x = 0.5
		vel_msg.linear.y = Kp2*r_sD
		#vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = Kp*err
		self.mycommand(vel_msg)
		print str(ros_data.data)
		self.oldr_cx = self.newr_cx 


def main(args):
	sc = send_cmd()
	rospy.init_node('send_command', anonymous=True)
	rospy.spin()
	sc.saveData()
if __name__ == '__main__':
	main(sys.argv)

