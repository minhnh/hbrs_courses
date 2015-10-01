#!/usr/bin/env python

import cv2
import numpy as np
import rospy

class ProcessImage():
	def __init__(self,image):
		rospy.loginfo("ProcessImage initialized")
		self.image = image #reference image initialization
		self.contour = None
		self.x = None
		self.y = None

	def showImage(self,windowName, imageToDisplay):
		cv2.imshow(windowName,imageToDisplay)

	def drawContours(self,MaskImage):
		gray = cv2.cvtColor(MaskImage,cv2.COLOR_BGR2GRAY)
		ret, thres = cv2.threshold(gray,10,255,0)
		contours,hierarchy = cv2.findContours(thres,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#find the largest contour
		areas = [cv2.contourArea(c) for c in contours]
		largestIndex = np.argmax(areas)
		cnt = contours[largestIndex]
		self.contours = cnt
		cv2.drawContours(self.image,[cnt],0,(255,0,0),1)

	def centroid(self):
		#find the centroid of ball
		M = cv2.moments(self.contours)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		self.x = cx
		self.y = cy

def detect_ball(filtered_image):
	''' Return whether a ball is captured '''
	return true

def process_image(image):
	''' Expect image of type IplImage '''

	#convert image to hsv
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# Set the bounds for Red color. Handle wrap around case for H > 350
	# Note: Ranges of HSV in OpenCV are
    #		- H: 0-180
    #		- S: 0-255
    #		- V: 0-255
	lower_bound = np.array([0.00*180, 0.23*256, 0.23*256])
	upper_bound = np.array([0.02*180, 1.00*256, 1.00*256])
	mask = cv2.inRange(hsv, lower_bound, upper_bound)
	lower_bound = np.array([0.96*180, 0.23*256, 0.23*256])
	upper_bound = np.array([1.00*180, 1.00*256, 0.99*256])
	mask_high = cv2.inRange(hsv, lower_bound, upper_bound)
	mask = cv2.bitwise_or(mask, mask_high)

	# Morphology operations
	# Opening to clear noise
	# Closing to clear holes inside shape
	set5 = np.ones((5, 5), np.uint8)
	set17 = np.ones((21,21), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, set5)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, set17)

	res = cv2.bitwise_and(image, image, mask=mask)

	return res

