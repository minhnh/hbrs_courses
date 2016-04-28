#include <face_detection_verilook/face_detection_verilook_node.h>

#include <string>

#include <NCore.hpp>
#include <NLicensing.hpp>

#define LICENSE_COMPONENTS {\
	"Biometrics.FaceDetection", \
	"Biometrics.FaceSegmentsDetection", \
	"Devices.Cameras", \
	"Biometrics.FaceExtraction", \
	"Biometrics.FaceSegmentation", \
	"Biometrics.FaceMatching", \
	"Biometrics.FaceQualityAssessment" \
}
#define LICENSE_SERVER	"/local"
#define LICENSE_PORT	"5000"

FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh) :
    node_handle_(nh)
{
    const std::string Components[] = LICENSE_COMPONENTS;
    bool successful = false;
    try
    {
        Neurotec::NCore::OnStart();
        for (unsigned int i = 0; i < sizeof(Components); i++)
        {
            Neurotec::Licensing::NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
        }
        successful = true;
    }
    catch (Neurotec::NError& e)
    {
        for (unsigned int i = 0; i < sizeof(Components); i++)
        {
            //ROS_ERROR_STREAM(e);
            Neurotec::Licensing::NLicense::ReleaseComponents(Components[i]);
        }
    }

    if (!successful) Neurotec::NCore::OnExit(false);
}

FaceDetectionVerilookNode::~FaceDetectionVerilookNode(void)
{
    const std::string Components[] = LICENSE_COMPONENTS;
    try
    {
        for (unsigned int i = 0; i < sizeof(Components); i++)
        {
            Neurotec::Licensing::NLicense::ReleaseComponents(Components[i]);
        }
    }
    catch (Neurotec::NError& ex)
    {
        //ROS_ERROR_STREAM(ex);
    }
    catch (std::exception& ex)
    {
        //ROS_ERROR_STREAM(ex.what());
    }
    Neurotec::NCore::OnExit(false);
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "face_detection_verilook");

    ros::NodeHandle nh("~");

    FaceDetectionVerilookNode face_detection_verilook_node(nh);

    ROS_INFO("face_detection_verilook: beginning main");

    ros::spin();
    return 0;
}
