/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#include <string>

/* Neurotec */
#include <NCore.hpp>
#include <Core/NTypes.hpp>
#include <NMedia.hpp>
#include <NLicensing.hpp>
#include <NBiometrics.hpp>
#include <NBiometricClient.hpp>

/* Local */
#include <face_detection_verilook_node.h>

/* Definitions */
#define LICENSE_COMPONENTS \
{\
    "Biometrics.FaceDetection", \
    "Devices.Cameras", \
    "Biometrics.FaceExtraction", \
    "Biometrics.FaceMatching" \
}
#define LICENSE_SERVER  "/local"
#define LICENSE_PORT    "5000"

namespace verilook_ros
{

using Neurotec::NCore;
using Neurotec::Licensing::NLicense;

FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh)
{
    using Neurotec::Biometrics::Client::HNBiometricClient;

    const std::string Components[] = LICENSE_COMPONENTS;
    Neurotec::NResult result = Neurotec::N_OK;
    HNBiometricClient hBiometricClient = NULL;

    pub_event_out_ = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_in_ = nh.subscribe("event_in", 1, &FaceDetectionVerilookNode::eventInCallback, this);

    // Obtain VeriLook license
    ROS_INFO_STREAM(PACKAGE_NAME << ": obtaining licenses...");
    try
    {
        NCore::OnStart();
        for (unsigned int i = 0; i < sizeof(Components)/sizeof(*Components); i++)
        {
            result = NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
            if (!result)
            {
                ROS_ERROR_STREAM(PACKAGE_NAME << ": License for " << Components[i] << " is not available");
            }
            else
            {
                ROS_DEBUG_STREAM(PACKAGE_NAME << ": License for " << Components[i] << " obtained successfully");
            }
        }
    }
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(PACKAGE_NAME << ": " << std::string(e.ToString()));
        NLicense::ReleaseComponents(Components[0]);
    }
}

FaceDetectionVerilookNode::~FaceDetectionVerilookNode()
{
    const std::string Components[] = LICENSE_COMPONENTS;
    try
    {
        for (unsigned int i = 0; i < sizeof(Components); i++)
        {
            NLicense::ReleaseComponents(Components[i]);
        }
    }
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(std::string(e.ToString()));
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
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
