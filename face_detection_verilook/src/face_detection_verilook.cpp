#include <ros/ros.h>

using namespace std;

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "face_detection_verilook");
    ros::NodeHandle nh;
    ROS_INFO("face_detection_verilook: beginning main");
    ros::spin();
    return 0;
}
