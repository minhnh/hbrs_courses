#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

class ProcessImage():
    def __init__(self,image):
        self.image = image #reference image initialization
        self.contour = None
        self.x = None
        self.y = None

    def show_image(self, windowName):
        cv2.imshow(windowName, self.image)

    def perform_morphology(self, mask_image):
        '''
        Morphology operations
        - Opening to clear noise
        - Closing to clear holes inside shape
        Return morphology result
        '''
        set5 = np.ones((5, 5), np.uint8)
        set17 = np.ones((21,21), np.uint8)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_CLOSE, set5)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, set17)

        return mask_image

    def filter_red(self):
        '''
        Set the bounds for Red color. Handle wrap around case for H > 350
        Note: Ranges of HSV in OpenCV are
        - H: 0-180
        - S: 0-255
        - V: 0-255
        Return filtered image
        '''
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([0.00*180, 0.13*256, 0.17*256])
        upper_bound = np.array([0.02*180, 1.00*256, 1.00*256])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        lower_bound = np.array([0.96*180, 0.13*256, 0.17*256])
        upper_bound = np.array([1.00*180, 1.00*256, 0.99*256])
        mask_high = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.bitwise_or(mask, mask_high)

        # Run Morphology on mask
        mask = self.perform_morphology(mask)

        return cv2.bitwise_and(self.image, self.image, mask=mask)

    def draw_contours(self):
        ''' Draw contours around a red filtered image '''
        filtered_img = cv2.cvtColor(self.filter_red(),
                                    cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(filtered_img, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

        #find the largest contour, save None to self.contours if contours empty
        try:
            areas = [cv2.contourArea(c) for c in contours]
            largestIndex = np.argmax(areas)
        except ValueError, e:
            self.contours = None
            return

        self.contours = contours[largestIndex]
        cv2.drawContours(self.image, [self.contours], 0, (255,0,0), 1)

    def centroid(self):
        #find the centroid of ball
        M = cv2.moments(self.contours)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        self.x = cx
        self.y = cy

    def ball_position(self):
        '''returns the location of ball in the image'''

        self.draw_contours()

        if self.contours == None:
            return None

        self.centroid()

        height, width, channels = self.image.shape
        relative_OriginX = width / 2
        relative_OriginY = height / 2
        location = None
        if (self.x > relative_OriginX):
            location = 'right'
        elif (self.x < relative_OriginX):
            location = 'left'
        return location

def process_image(image_ros):
    ''' Expect image of type sensor_msgs/Image '''

    # Convert to Mat image
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(image_ros, "bgr8")
    except CvBridgeError, e:
        print e
        return None

    # Create a ProcessImage instance
    pi_instance = ProcessImage(image)

    # Calculate ball position
    position = pi_instance.ball_position()

    # Debugging functions
    # print position
    # pi_instance.show_image("Image with Contour")
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return position


