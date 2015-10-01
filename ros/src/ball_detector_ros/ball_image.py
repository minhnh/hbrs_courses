#!/usr/bin/env python


import roslib
import sys
import rospy
import cv2
##from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def take_picture():


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
            pass


    # Ramp the camera - these frames will be discarded and are only used to allow v4l2
    # to adjust light levels, if necessary

    for i in range(frames_ignored):
        temp = get_image()

    while not rospy.is_shutdown():
        camera_capture = get_image()
        # A nice feature of the imwrite method is that it will automatically choose the
        # correct format based on the file extension you provide. Convenient!
        try:
            imgmsg = bridge.cv2_to_imgmsg(camera_capture, "bgr8")
        except CvBridgeError, e:
            print e
        process_image(imgmsg)
        rate.sleep()

    # You'll want to release the camera, otherwise you won't be able to create a new
    # capture object until your script exits
    camera.release()


