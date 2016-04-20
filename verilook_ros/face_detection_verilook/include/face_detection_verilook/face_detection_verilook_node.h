#ifndef FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED
#define FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED

#include <ros/ros.h>

namespace face_detection_verilook
{
    class FaceDetectionVerilookNode
    {
    public:
        FaceDetectionVerilookNode(ros::NodeHandle nh);
        ~FaceDetectionVerilookNode(void);
    protected:
        ros::NodeHandle node_handle_;
    };
}
#endif // FACE_DETECTION_VERILOOK_NODE_HPP_INCLUDED
