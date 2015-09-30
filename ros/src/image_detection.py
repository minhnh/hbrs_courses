#!/usr/bin/env python

import cv2
import numpy as np
import rospy

class ProcessImage():
	def __init__(self,image):
		rospy.loginfo("ProcessImage initialized")
		self.image = None #reference image initialization
		self.contour = None
		self.x = None
		self.y = None
		
	
	def showImage(self,windowName, imageToDisplay):
		cv2.imshow(windowName,imageToDisplay)

	def maskImage(self):
		#convert image to hsv
		hsv = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)
		#cv2.imshow('hsv',hsv)
		#set the bounds for Red color
		lower_bound = np.array([0,100,100])
		upper_bound = np.array([10,255,255])

		mask = cv2.inRange(hsv,lower_bound,upper_bound)
		#cv2.imshow('mask',mask)
		res = cv2.bitwise_and(self.image,self.image,mask=mask)

		return res

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
	
	def position(self):
		height, width, channels = self.image.shape
		rel_orign_x = width / 2
		rel_origin_y = height / 2
		if(self.x > rel_origin_x):
			location = 'right'
		else :
			location = 'left'
		return location


