#!/usr/bin/env python

import sys
import time
import rospy
import roslib
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import ball_detector_ros.process_image as bdr_pi
import ball_detector_ros.state_machine as bdr_sm

def test_webcam():

    bridge = CvBridge()
    frames_ignored = 60

    # Now we can initialize the camera capture object with the cv2.VideoCapture class.
    # All it needs is the index to a camera port.

    camera = cv2.VideoCapture(0)

    # Captures a single image from the camera and returns it in PIL format
    def get_image():
        # read is the easiest way to get a full image out of a VideoCapture object.
        try:
            retval, im = camera.read()
            return im
        except rospy.ROSInterruptException:
            return None

    # Ramp the camera - these frames will be discarded and are only used to allow v4l2
    # to adjust light levels, if necessary
    for i in range(frames_ignored):
        temp = get_image()

    while not rospy.is_shutdown():
        camera_capture = get_image()
        if camera_capture == None:
            pass
        # A nice feature of the imwrite method is that it will automatically choose the
        # correct format based on the file extension you provide. Convenient!
        try:
            imgmsg = bridge.cv2_to_imgmsg(camera_capture, "bgr8")
        except CvBridgeError, e:
            print e
        bdr_pi.process_image(imgmsg)
        time.sleep(0.1)

    # You'll want to release the camera, otherwise you won't be able to create a new
    # capture object until your script exits
    camera.release()

def test_process_image(filename):
    image = cv2.imread(filename, cv2.CV_LOAD_IMAGE_COLOR)
    if (image == None):
        print "Invalid input file"
        return 1

    position = bdr_pi.process_image(image)

    return 0

def main():
    bdr_sm.run()
