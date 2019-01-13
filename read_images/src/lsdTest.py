#!/usr/bin/env python
"""
This node will read image from topic "/drone/front/compressed"
detect the centroid and will publish the centorid to the topic /centroids
Another Change will be done which is choosing specific lines to compute the avergae
"""
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from std_msgs.msg import String
import sys
from sklearn.cluster import DBSCAN
from sklearn import metrics
import math


class image_convert:

	def __init__(self):
		self.pub = rospy.Publisher('/centroids', String, queue_size=1)
		self.subscriber = rospy.Subscriber("/drone/front/compressed",CompressedImage, self.callback)
		self.lsd = cv2.createLineSegmentDetector(0)
		self.gcx = 0
		self.gcy = 0

	def findLength(self,x1,x2,y1,y2):
		return math.sqrt((x1-x2)**2 + (y1-y2)**2)

	def findIntersec(self,line1,line2):
		l1x1 = line1[0]
		l1y1 = line1[1]
		l1x2 = line1[2]
		l1y2 = line1[3]
		l2x1 = line2[0]
		l2y1 = line2[1]
		l2x2 = line2[2]
		l2y2 = line2[3]
		a1 = (l1y1-l1y2)/(l1x1-l1x2)
		a2 = (l2y1-l2y2)/(l2x1-l2x2)
		
		b1 = l1y1 - a1*l1x1
		b2 = l2y1 - a2*l2x1
		
		if(a1 == a2):
			return 0,0,0
		else:
			return 1,(b1-b2)/(a2-a1), (a2*b1-a1*b2)/(a2-a1)

	def getVanishingPoint(self,in_image):
		self.gray = cv2.cvtColor(in_image, cv2.COLOR_BGR2GRAY)
		lines = self.lsd.detect(self.gray)[0]
		self.mlines = lines
		long_lines = np.empty((0,1,4),dtype=np.float32)
		intersec = np.empty((0,2),dtype=np.float32)
		length = 10
		av1 = 0
		cnt1 = 0
		av2 = 0
		cnt2 = 0
		alfa = 0.15
		for line in lines:
			for x1,y1,x2,y2 in line:
				if(self.findLength(x1,x2,y1,y2) > length):
					if(abs(x1-x2) > 5):
						if(y1 > self.gcy or y2 > self.gcy):
							if(x1 < self.gcx or x2 < self.gcx):
								av1 = av1 + (y2 - y1)/(x2 - x1)
								cnt1 = cnt1 + 1
							else:
								av2 = av2 + (y2 - y1)/(x2 - x1)
								cnt2 = cnt2 + 1
						for i in range(len(long_lines)):
							res,x,y = self.findIntersec([x1,y1,x2,y2],long_lines[i][0])
							if(res == 1):
								intersec = np.append(intersec,[[x,y]],axis=0)
								try:
									cv2.circle(self.gray,(int(x),int(y)),3,(0,255,255),-1)
								except :
									pass
						long_lines = np.append(long_lines,[[[x1,y1,x2,y2]]],axis = 0)
		drawn_img = self.lsd.drawSegments(self.gray,long_lines)
		if(cnt1 > 1):
			av1 = av1/cnt1
		if(cnt2 > 1):
			av2 = av2/cnt2
		print "avLeft :" +str(av1)
		print "avRight :" +str(av2)
		sDiff = av2 + av1
		#Clustering
		db = DBSCAN(eps=10, min_samples=2).fit(intersec)
		db_labels = db.labels_
		cx = 0
		cy = 0
		labels, counts = np.unique(db_labels[db_labels>=0], return_counts=True)
		maxlabel = labels[np.argsort(-counts)[:1]]
		if(maxlabel.size > 0):
			for lab in range(len(db.labels_)):
				if db.labels_[lab] == maxlabel[0]:
					cx = cx + intersec[lab][0]
					cy = cy + intersec[lab][1]
			cx = cx/max(counts)
			cy = cy/max(counts)
			if(self.gcx != 0):
				cx = self.gcx - alfa*(self.gcx - cx)
				self.gcx = cx
				cx = self.gcx - alfa*(self.gcx - cx)
			self.gcx = cx
			self.gcy = cy
			try:
				cv2.circle(drawn_img,(int(cx),int(cy)),10,(0,255,0),-1)
			except:
				pass
		
		return drawn_img,cx,sDiff

	def sendCentroid(self,dat):
		#rate = rospy.Rate(10) # 10hz
		self.pub.publish(dat)
		#rate.sleep()

	def callback(self,ros_data):
		msg = ""
		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		image,cx,sDiff = self.getVanishingPoint(image_np)
		msg = str(cx) + "_" + str(sDiff)
		cv2.imshow('cv_img', image)
		if not rospy.is_shutdown():
			self.sendCentroid(msg)
		cv2.waitKey(10)


def main(args):
	ic = image_convert()
	rospy.init_node('image_feature', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"
	cv2.destroyAllWindows()



if __name__ == '__main__':
	main(sys.argv)
