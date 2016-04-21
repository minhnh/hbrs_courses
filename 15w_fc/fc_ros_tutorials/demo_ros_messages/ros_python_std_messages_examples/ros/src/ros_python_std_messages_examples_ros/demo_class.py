#!/usr/bin/env python
'''
This script demostrates the way of filling
ros std_msgs by giving few examples.
'''
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray


class DemoClass():
    def __init__(self):
        self.prepare_std_msg()

    def prepare_std_msg(self):
        rospy.loginfo("Preparing std_msgs......")
        '''
        Fill Bool message
        '''
        bool_msg = Bool()

        bool_msg.data = False

        '''
        Fill Float32 message
        '''
        float32_msg = Float32()

        float32_msg.data = 1.0

        '''
        Fill Int32 message
        Please, do it your self for practice
        '''
        int16_msg = Int16()
        int16_msg.data = 1000400

        '''
        Fill String message
        Please, do it your self for practice
        '''
        string_msg = String()
        string_msg.data = "String message"

        int16array_msg = Int16MultiArray()
        int16array_msg.data = [[1, 2, 3], [70001, -14, 0 ]]
        int16array_msg.data.append([100000, 32000, 1400000])

        rospy.loginfo("std_msgs preparation done......")
        rospy.loginfo("value should overflow: %d" % \
                int16array_msg.data[2][2])
        rospy.loginfo("value 2 should overflow: %d" % \
                int16_msg.data)

