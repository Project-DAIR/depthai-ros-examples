
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


std::tuple<dai::Pipeline, int, int> createPipeline(std::string resolution, float fps, uint32_t exposureTimeUs, uint32_t sensitivityIso){
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution; 
    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");

    int width, height;
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
    monoLeft->setFps(fps);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    
    // monoLeft->initialControl.setManualExposure(1500, 100);
    monoLeft->initialControl.setManualExposure(exposureTimeUs, sensitivityIso);

    monoLeft->out.link(xoutLeft->input);

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "left_mono_node");
    ros::NodeHandle pnh("~");
    
    std::string tfPrefix, mode;
    std::string cameraParamUri;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence = 200;
    int monoWidth, monoHeight;
    int LRchecktresh = 5;
    std::string monoResolution = "720p";
    float fps = 60.0;
    int32_t exposureTimeUs = 1500;
    int32_t sensitivityIso = 100;

    dai::Pipeline pipeline;

    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("tf_prefix",        tfPrefix);
    badParams += !pnh.getParam("monoResolution",   monoResolution);
    badParams += !pnh.getParam("fps",              fps);
    badParams += !pnh.getParam("exposureTimeUs",   exposureTimeUs);
    badParams += !pnh.getParam("sensitivityIso",   sensitivityIso);

    if (badParams > 0)
    {   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(monoResolution, fps, exposureTimeUs, sensitivityIso);

    dai::Device device(pipeline);
    auto leftQueue = device.getOutputQueue("left", 30, false);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }
   
    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    pnh, 
                                                                                    std::string("left/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    leftCameraInfo,
                                                                                    "left");

    leftPublish.addPublisherCallback();
    ros::spin();
    return 0;
}
