#!/usr/bin/env python

import cv2

#module to capture and save image

def capture():
	'''
	method to capture the webcam image
	'''
	port = 0
	tempFrames = 50
	
	#init camera
	camera = cv2.VideoCapture(port)
	#loop to clear camera buffer
	for i in xrange(tempFrames):
		temp = camera.read()
	
	#capture the image
	ret, frame = camera.read()

	#save the image in the file
	fileName = '/home/chaitanya/Downloads/capture.jpg'
	cv2.imwrite(fileName,frame)
	
	# destroy camera instance after use
	del(camera)

if __name__ == '__main__':
	print capture.__doc__
