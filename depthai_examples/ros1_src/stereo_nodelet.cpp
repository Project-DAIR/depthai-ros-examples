#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <tuple>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace depthai_examples{


 class StereoNodelet : public nodelet::Nodelet
{

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter;
    std::unique_ptr<dai::Device> _dev;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string tfPrefix, mode;
            std::string cameraParamUri;
            std::string monoResolution = "720p";
            int badParams = 0;
            bool lrcheck, extended, subpixel, enableDepth;
            int confidence = 200;
            int LRchecktresh = 5;
            float fps = 60.0;
            int32_t exposureTimeUs = 1500;
            int32_t sensitivityIso = 100;

            badParams += !pnh.getParam("tf_prefix", tfPrefix);
            badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck",  lrcheck);
            badParams += !pnh.getParam("extended",  extended);
            badParams += !pnh.getParam("subpixel",  subpixel);
            badParams += !pnh.getParam("confidence",  confidence);
            badParams += !pnh.getParam("LRchecktresh",  LRchecktresh);
            badParams += !pnh.getParam("monoResolution",  monoResolution);
            badParams += !pnh.getParam("fps",              fps);
            badParams += !pnh.getParam("exposureTimeUs",   exposureTimeUs);
            badParams += !pnh.getParam("sensitivityIso",   sensitivityIso);
            
            if (badParams > 0)
            {   
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find %d of the parameters");
            }

            enableDepth = false;

            dai::Pipeline pipeline;
            int monoWidth, monoHeight;
            std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, fps, exposureTimeUs, sensitivityIso);
            _dev = std::make_unique<dai::Device>(pipeline);

            auto leftQueue = _dev->getOutputQueue("left", 30, false);

            auto calibrationHandler = _dev->readCalibration();

            // this part would be removed once we have calibration-api
            /*             
            std::string left_uri = camera_param_uri +"/" + "left.yaml";

            std::string right_uri = camera_param_uri + "/" + "right.yaml";

            std::string stereo_uri = camera_param_uri + "/" + "right.yaml"; 
            */

            auto boardName = calibrationHandler.getEepromData().boardName;

            leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_left_camera_optical_frame", true);
            auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 

            leftPublish  = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                            (leftQueue,
                                                                                             pnh, 
                                                                                             std::string("left/image"),
                                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                             leftConverter.get(),
                                                                                             std::placeholders::_1, 
                                                                                             std::placeholders::_2) , 
                                                                                             30,
                                                                                             leftCameraInfo,
                                                                                             "left");

            // bridgePublish.startPublisherThread();
            leftPublish->addPublisherCallback();
        }


    std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution, float fps, uint32_t exposureTimeUs, uint32_t sensitivityIso){
        dai::Pipeline pipeline;

        auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();

        // XLinkOut
        xoutLeft->setStreamName("left");

        int width, height;
        dai::node::MonoCamera::Properties::SensorResolution monoResolution;
        if(resolution == "720p"){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
            width  = 1280;
            height = 720;
        }else if(resolution == "400p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
            width  = 640;
            height = 400;
        }else if(resolution == "800p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
            width  = 1280;
            height = 800;
        }else if(resolution == "480p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
            width  = 640;
            height = 480;
        }else{
            ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
            throw std::runtime_error("Invalid mono camera resolution.");
        }

        // MonoCamera
        monoLeft->setResolution(monoResolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoLeft->setFps(fps);

        monoLeft->initialControl.setManualExposure(exposureTimeUs, sensitivityIso);

        monoLeft->out.link(xoutLeft->input);

        return std::make_tuple(pipeline, width, height);
    }
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)
}   // namespace depthai_examples
