#!/usr/bin/env python

#-*- encoding: utf-8 -*-
import rospy
from std_msgs.msg import String
import cv2
import ball_detector_ros.ball_detector as bd
import sys

'''
Publisher class: Handle the publishing of output
'''
class publisher():
    def __init__(self):
        self.pub = rospy.Publisher('event_out', String, queue_size=100, latch=False)
    def publish_output(self,msg):
        self.pub.publish(msg)

'''
image_processor class: Image processing should be put inside this class
'''
class image_processor():
    def __init__(self):
	rospy.loginfo("Processor is instantiated.")

    #Call this method 'run' to do the processing 
    def run(self):	
	bd.test_process_image(sys.argv[1])
	output = "output here"	
	return output
	
    def process_dummy_func(self):
	img = cv2.imread('/home/bach/image.jpg',1)
        cv2.imshow('image',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

'''
State machine, publisher and subcriber
'''
class run():
    state = "INIT"
    def __init__(self):
	#Initiate node
	rospy.init_node('open_cv_node', anonymous=False)
        rospy.loginfo("Open_CV is now running")  
	
	#Create publisher, subcriber        
	self.input_msg = rospy.Subscriber("event_in", String, self.input_msg_cb)
	pub = publisher()

	img_proc = image_processor()	#instantiate image_processor	

	#Processing loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
	    if self.state == "PROC":	#If in processing state
		output = String()
		output.data = img_proc.run()	#Run the image processor
                pub.publish_output(output)	#Publish output
	    elif self.state == "INIT":	#If in initiate state
		rospy.loginfo("Waiting for e_start") 
    	    r.sleep()

    #Callback function
    def input_msg_cb(self, msg):
        if msg.data == 'e_start':
            rospy.loginfo("Starting")
	    self.state = "PROC"                       
        elif msg.data == 'e_stop':
            rospy.loginfo("Stoping")
	    self.state = "INIT"
        else:
	    rospy.loginfo("Waiting")

	
if __name__ == '__main__':    
    run()


    

