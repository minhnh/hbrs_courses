#include <face_detection_verilook/face_detection_verilook_node.h>

namespace face_detection_verilook
{
    FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh) :
        node_handle_(nh)
    {
    }

    FaceDetectionVerilookNode::~FaceDetectionVerilookNode(void)
    {
    }
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "face_detection_verilook");

    ros::NodeHandle nh("~");

    face_detection_verilook::FaceDetectionVerilookNode face_detection_verilook_node(nh);

    ROS_INFO("face_detection_verilook: beginning main");

    ros::spin();
    return 0;
}
