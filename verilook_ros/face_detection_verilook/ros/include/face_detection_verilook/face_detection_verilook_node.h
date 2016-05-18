/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#ifndef FACE_DETECTION_VERILOOK_FACE_DETECTION_VERILOOK_NODE_H
#define FACE_DETECTION_VERILOOK_FACE_DETECTION_VERILOOK_NODE_H

#include <ros/ros.h>

#include <NCore.hpp>
#include <Core/NTypes.hpp>
#include <NMedia.hpp>
#include <NLicensing.hpp>
#include <NBiometrics.hpp>
#include <NBiometricClient.hpp>

namespace verilook_ros
{
class FaceDetectionVerilookNode
{
public:
    explicit FaceDetectionVerilookNode(ros::NodeHandle nh);
    ~FaceDetectionVerilookNode(void);
protected:
    ros::NodeHandle node_handle_;
private:
    void createTemplateFromCamera();
};
}   // namespace verilook_ros

#endif  // FACE_DETECTION_VERILOOK_FACE_DETECTION_VERILOOK_NODE_H
