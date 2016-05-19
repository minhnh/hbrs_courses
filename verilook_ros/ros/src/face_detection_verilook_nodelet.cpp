/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

namespace verilook_ros
{

class FaceDetectionVerilookNodelet : public nodelet::Nodelet
{
public:
    FaceDetectionVerilookNodelet()
    {}

private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
        pub_event_out_ = private_nh.advertise<std_msgs::String>("event_out", 1);
        sub_event_in_ = private_nh.subscribe("event_in", 1, &FaceDetectionVerilookNodelet::eventInCallback, this);
    }

    void eventInCallback(const std_msgs::String::Ptr &msg)
    {
        if (msg->data == "e_trigger")
        {}
    }

    ros::Publisher pub_event_out_;
    ros::Subscriber sub_event_in_;
};

PLUGINLIB_EXPORT_CLASS(verilook_ros::FaceDetectionVerilookNodelet, nodelet::Nodelet);

}   // namespace verilook_ros
