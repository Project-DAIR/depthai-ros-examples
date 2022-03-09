
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

dai::Pipeline createPipeline(int &lensPosition, std::string &rgb_resolution){
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    if (rgb_resolution == "1080p") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    } else if (rgb_resolution == "12mp") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
    } else if (rgb_resolution == "13mp") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_13_MP);
    } else if (rgb_resolution == "4k") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    } 
    colorCam->setInterleaved(false);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);

    // TURN OFF AUTOFOCUS
    colorCam->initialControl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::OFF);
    colorCam->initialControl.setManualFocus(lensPosition);

    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_node_fixed_focus");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int badParams = 0;
    int lensPosition = 120;
    std::string rgb_resolution = "1080p";

    badParams += !pnh.getParam("camera_name", deviceName);
    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);
    badParams += !pnh.getParam("lens_position", lensPosition);
    badParams += !pnh.getParam("rgb_resolution", rgb_resolution);

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    
    dai::Pipeline pipeline = createPipeline(lensPosition, rgb_resolution);
    dai::Device device(pipeline);
    std::shared_ptr<dai::DataOutputQueue> imgQueue = device.getOutputQueue("video", 30, false);
    
    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                  pnh, 
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter, // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");

    rgbPublish.addPublisherCallback();
    ros::spin();

    return 0;
}

