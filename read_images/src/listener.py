#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import sys
from sklearn.cluster import DBSCAN
from sklearn import metrics
import math

class image_convert:
	def __init__(self):
		self.subscriber = rospy.Subscriber("/drone/front/compressed",CompressedImage, self.callback)

	def findIntersec(self,line1,line2):
		a1 = line1[0]
		a2 = line2[0]
		b1 = line1[1]
		b2 = line2[1]
		if(a1 == a2):
			return 0,0,0
		else:
			return 1,(b1-b2)/(a2-a1), (a2*b1-a1*b2)/(a2-a1)



	def getVanishingPoint(self,in_image):
		self.edges = cv2.Canny(in_image,100,200,apertureSize = 3)
		lines = cv2.HoughLines(self.edges,1,np.pi/180,70)
		if not(lines is None):
			if len(lines) >= 0:
				i = 0
				lines_dat = np.array([[0,0]],dtype=float) 
				a_s = np.array([0])
				b_s = np.array([0]) 
				for line in lines:
					for rho,theta in line:
						similar = False
						a = np.cos(theta)
						b = np.sin(theta)
						if b == 0:
							break
						slope = (-a/b)
						m_b = rho/b
						if (i > 0):
							for m in range(len(a_s)):
								if abs(a_s[m] - slope) < 3:
									if(abs(b_s[m] - m_b) < 20):
										similar = True
										break
									elif(abs(b_s[m]/a_s[m] - m_b/slope) < 20):
										similar = True
										break
									elif(abs((b_s[m]-480)/a_s[m] - (m_b-480)/slope) < 20):
										similar = True
										break
									elif(abs((a_s[m]*640+b_s[m]) - (slope*640+m_b)) < 20):
										similar = True
										break
						if similar == True:
							similar = False
							break
						if abs(slope) > 4:
							break
						if(i == 0):
							lines_dat[i][0] = slope
							lines_dat[i][1] = m_b
							a_s[i] = slope
							b_s[i] = m_b
						else:
							lines_dat = np.append(lines_dat,[[slope,m_b]],axis=0)
							a_s = np.append(a_s,[slope])
							b_s = np.append(b_s,[m_b])
						i = i+1
						x0 = a*rho
						y0 = b*rho
						x1 = int(x0 + 1000*(-b))
						y1 = int(y0 + 1000*(a))
						x2 = int(x0 - 1000*(-b))
						y2 = int(y0 - 1000*(a))
						try:
							cv2.line(in_image,(x1,y1),(x2,y2),(0,0,255),2)
						except:
							pass
						#cv2.circle(frame,(200,200),30,(0,255,255),-1)
				#find intersections
				intersec = np.array([[0,0]]) 
				for j in range(len(lines_dat)):
					for k in range(j+1,len(lines_dat)):
						sec,x,y = self.findIntersec(lines_dat[k],lines_dat[j])
						if(sec == 1):
							#print str(x)
							#print str(y)
							try:
								cv2.circle(in_image,(int(x),int(y)),10,(0,255,255),-1)
							except:
								pass
							intersec = np.append(intersec,[[x,y]],axis=0)
				#Clustering
				if(len(intersec) == 2):
					cx = intersec[1][0]
					cy = intersec[1][1]
					cv2.circle(in_image,(int(intersec[1][0]),int(intersec[1][1])),10,(0,255,0),-1)
				elif(len(intersec > 2)):
					db = DBSCAN(eps=30, min_samples=2).fit(intersec)
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
						cx = cx/counts
						cy = cy/counts
						try:
							cv2.circle(in_image,(int(cx),int(cy)),10,(0,255,0),-1)
						except:
							pass
		return in_image,cx,cy

	def sendCentroid(self,dat):
		rate = rospy.Rate(10) # 10hz
		pub.publish(dat)
		rate.sleep()

	def callback(self, ros_data):
		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		image,cx,cy = self.getVanishingPoint(image_np)
		
		cv2.imshow('cv_img', image)
		cv2.imshow('edges',self.edges)
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
