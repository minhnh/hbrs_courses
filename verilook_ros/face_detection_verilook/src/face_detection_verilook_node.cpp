#include <face_detection_verilook/face_detection_verilook_node.h>

#include <string>

#include <NCore.hpp>
#include <NLicensing.hpp>
#include <NBiometrics.hpp>

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
            successful = Neurotec::Licensing::NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
            if (!successful)
            {
                ROS_ERROR_STREAM("verilook: License for " << Components[i] << " is not available");
            }
        }
    }
    catch (Neurotec::NError& e)
    {
        for (unsigned int i = 0; i < sizeof(Components); i++)
        {
            ROS_ERROR_STREAM(std::string(e.ToString()));
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
    catch (Neurotec::NError& e)
    {
        ROS_ERROR_STREAM(std::string(e.ToString()));
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    Neurotec::NCore::OnExit(false);
}


void FaceDetectionVerilookNode::createTemplateFromCamera() {
    using namespace Neurotec::Biometrics;

    NSubject subject;
    NFace face;
    face.SetCaptureOptions((NBiometricCaptureOptions)(nbcoManual | nbcoStream));
    subject.GetFaces().Add(face);
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "face_detection_verilook");

    ros::NodeHandle nh("~");

    FaceDetectionVerilookNode face_detection_verilook_node(nh);

    ROS_INFO("face_detection_verilook: beginning main");

    ros::spin();
    return 0;
}
