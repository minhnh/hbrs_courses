#ifndef FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED
#define FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED

#include <ros/ros.h>

#include <NCore.hpp>
#include <NLicensing.hpp>
#include <NBiometrics.hpp>
#include <NBiometricClient.hpp>

namespace verilook_ros
{
    class FaceDetectionVerilookNode
    {
    public:
        FaceDetectionVerilookNode(ros::NodeHandle nh);
        ~FaceDetectionVerilookNode(void);
    protected:
        ros::NodeHandle node_handle_;
    private:
        Neurotec::Biometrics::Client::NBiometricClient m_biometricClient;

        void createTemplateFromCamera();
    };
}

#endif // FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED
