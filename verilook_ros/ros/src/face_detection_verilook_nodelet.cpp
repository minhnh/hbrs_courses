/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#include <string>

/* ROS */
#include <ros/ros.h>
#include <std_msgs/String.h>
// Nodelet
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

/* Local */
#include <face_detection_verilook_node.h>

namespace verilook_ros
{

class FaceDetectionVerilookNodelet : public nodelet::Nodelet
{
public:
    FaceDetectionVerilookNodelet()
    {
        ROS_INFO("verilook: face detection nodelet starting...");
        face_detection_node_ = 0;
    }
    virtual ~FaceDetectionVerilookNodelet()
    {
        if (face_detection_node_ != 0)
            delete face_detection_node_;
    }

private:
    virtual void onInit()
    {
        node_handle_ = getMTNodeHandle();
        face_detection_node_ = new verilook_ros::FaceDetectionVerilookNode(node_handle_);
//        try
//        {
//            NCore::OnStart();
//            ROS_INFO("verilook: in try...");
//            for (unsigned int i = 0; i < sizeof(Components); i++)
//            {
//                successful = NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
//                if (!successful)
//                {
//                    ROS_ERROR_STREAM("verilook: License for " << Components[i] << " is not available");
//                }
//            }
//
//            // create biometric client
//            result = NBiometricClientCreate(&hBiometricClient);
//            if (Neurotec::NFailed(result))
//            {
//                ROS_ERROR_STREAM("verilook: NBiometricClientCreate() failed (result = " << result << ")!");
//            }
//        }
//        catch (Neurotec::NError& e)
//        {
//            ROS_INFO("verilook: in catch...");
//            for (unsigned int i = 0; i < sizeof(Components); i++)
//            {
//                ROS_ERROR_STREAM(std::string(e.ToString()));
//                NLicense::ReleaseComponents(Components[i]);
//            }
//        }
//
//        if (!successful) NCore::OnExit(false);
    }

protected:
    ros::NodeHandle node_handle_;
    verilook_ros::FaceDetectionVerilookNode * face_detection_node_;

};

PLUGINLIB_EXPORT_CLASS(verilook_ros::FaceDetectionVerilookNodelet, nodelet::Nodelet);

}   // namespace verilook_ros
