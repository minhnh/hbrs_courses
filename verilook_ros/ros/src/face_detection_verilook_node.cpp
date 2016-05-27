/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */

/* Neurotec */
#include <NMedia.hpp>
#include <NBiometrics.hpp>
#include <NBiometricClient.hpp>

/* Package */
#include <verilook_wrapper.h>
#include <face_detection_verilook_node.h>

namespace verilook_ros
{

using Neurotec::NCore;

FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh)
{
    using Neurotec::Biometrics::Client::HNBiometricClient;

    HNBiometricClient hBiometricClient = NULL;

    pub_event_out_ = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_in_ = nh.subscribe("event_in", 1, &FaceDetectionVerilookNode::eventInCallback, this);

    // Obtain VeriLook license
    obtainVerilookLicenses();
}

FaceDetectionVerilookNode::~FaceDetectionVerilookNode()
{
	releaseVerilookLicenses();
    NCore::OnExit(false);
}

void FaceDetectionVerilookNode::eventInCallback(const std_msgs::String::Ptr &msg)
{
    ROS_INFO("verilook: in event_in callback...");
    if (msg->data == "e_trigger")
    {}
}

}   // namespace verilook_ros

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "face_detection_verilook");

    ros::NodeHandle nh("~");

    verilook_ros::FaceDetectionVerilookNode face_detection_verilook_node(nh);

    // Start ROS node. We need at least two threads so that VeriLook can be
    // supplied with images in the middle of a service call.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
