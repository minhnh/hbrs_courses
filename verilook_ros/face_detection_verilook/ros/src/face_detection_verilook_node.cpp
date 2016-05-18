/* Copyright 2016 Bonn-Rhein-Sieg University
 * Author: Minh Nguyen
 */
#include <string>

#include <face_detection_verilook/face_detection_verilook_node.h>

#define LICENSE_COMPONENTS \
{\
    "Biometrics.FaceDetection", \
    "Biometrics.FaceSegmentsDetection", \
    "Devices.Cameras", \
    "Biometrics.FaceExtraction", \
    "Biometrics.FaceSegmentation", \
    "Biometrics.FaceMatching", \
    "Biometrics.FaceQualityAssessment" \
}
#define LICENSE_SERVER  "/local"
#define LICENSE_PORT    "5000"

namespace verilook_ros
{
    using Neurotec::NCore;
    using Neurotec::Licensing::NLicense;

    FaceDetectionVerilookNode::FaceDetectionVerilookNode(ros::NodeHandle nh) :
        node_handle_(nh)
    {
        using Neurotec::Biometrics::Client::HNBiometricClient;

        const std::string Components[] = LICENSE_COMPONENTS;
        bool successful = false;
        Neurotec::NResult result = Neurotec::N_OK;
        HNBiometricClient hBiometricClient = NULL;

        try
        {
            NCore::OnStart();
            for (unsigned int i = 0; i < sizeof(Components); i++)
            {
                successful = NLicense::ObtainComponents(LICENSE_SERVER, LICENSE_PORT, Components[i]);
                if (!successful)
                {
                    ROS_ERROR_STREAM("verilook: License for " << Components[i] << " is not available");
                }
            }

            // create biometric client
            result = NBiometricClientCreate(&hBiometricClient);
            if (Neurotec::NFailed(result))
            {
                ROS_ERROR_STREAM("verilook: NBiometricClientCreate() failed (result = " << result << ")!");
            }
        }
        catch (Neurotec::NError& e)
        {
            for (unsigned int i = 0; i < sizeof(Components); i++)
            {
                ROS_ERROR_STREAM(std::string(e.ToString()));
                NLicense::ReleaseComponents(Components[i]);
            }
        }

        if (!successful) NCore::OnExit(false);
    }

    FaceDetectionVerilookNode::~FaceDetectionVerilookNode(void)
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


    void FaceDetectionVerilookNode::createTemplateFromCamera()
    {
        using Neurotec::Biometrics::NSubject;
        using Neurotec::Biometrics::NFace;
        using Neurotec::Biometrics::NBiometricCaptureOptions;
        using Neurotec::Biometrics::nbcoManual;
        using Neurotec::Biometrics::nbcoStream;

        NSubject subject;
        NFace face;
        face.SetCaptureOptions((NBiometricCaptureOptions)(nbcoManual | nbcoStream));
        subject.GetFaces().Add(face);
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
    ROS_INFO("verilook: face detection node starting...");
    ros::waitForShutdown();
    return 0;
}
