/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#ifndef VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H
#define VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H

/* ROS */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* Package */
#include <verilook_ros.h>

namespace verilook_ros
{

class FaceDetectionVerilookNode
{
public:
    FaceDetectionVerilookNode(ros::NodeHandle nh);
    ~FaceDetectionVerilookNode();

private:
    void eventInCallback(const std_msgs::String::Ptr &msg);

    ros::Publisher pub_event_out_;
    ros::Subscriber sub_event_in_;
};

}   // namespace verilook_ros

#endif  // VERILOOK_ROS_FACE_DETECTION_VERILOOK_NODE_H
