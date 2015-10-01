#!/usr/bin/env python

import rospy
import sys
import cv2
import ball_detector_ros.process_image as bdr_pi

def test_process_image(filename):
    image = cv2.imread(filename, cv2.CV_LOAD_IMAGE_COLOR)
    if (image == None):
        print "Invalid input file"
        return 1

    position = bdr_pi.process_image(image)

    return 0

