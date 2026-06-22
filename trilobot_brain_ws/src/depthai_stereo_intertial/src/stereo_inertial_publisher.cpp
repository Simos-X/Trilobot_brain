#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <chrono>
#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_driver/utils.hpp"

std::vector<std::string> usbStrings = { "UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS" };

namespace dai {
    namespace ros {

    using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>;

        class CompressedImageConverter {
            private:
                const std::string _frameName;
                bool _updateRosBaseTime;
                rclcpp::Time _rosBaseTime;
                SteadyTimePoint _steadyBaseTime;

            public:
                CompressedImageConverter(const std::string frameName, bool updateRosBaseTime = false)
                    : _frameName(frameName),
                    _updateRosBaseTime(updateRosBaseTime),
                    _rosBaseTime(0, 0, RCL_ROS_TIME),
                    _steadyBaseTime(std::chrono::steady_clock::now()) {}

                void updateRosBaseTime() {
                    _rosBaseTime = rclcpp::Clock(RCL_ROS_TIME).now();
                    _steadyBaseTime = std::chrono::steady_clock::now();
                }

                void setUpdateRosBaseTimeOnToRosMsg(bool val = false) {
                    _updateRosBaseTime = val;
                }

                inline rclcpp::Time getFrameTime(SteadyTimePoint daiTs) {
                    return dai::ros::getFrameTime(_rosBaseTime, _steadyBaseTime, daiTs);
                }

                void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<sensor_msgs::msg::CompressedImage> &opCompImgMsgs) {
                    sensor_msgs::msg::CompressedImage opCompImgMsg;
                    builtin_interfaces::msg::Time stamp = inData->getTimestampDevice().time_since_epoch().count() == 0
                                                            ? rclcpp::Clock(RCL_ROS_TIME).now()
                                                            : getFrameTime(inData->getTimestampDevice());
                    opCompImgMsg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
                    opCompImgMsg.header.frame_id = _frameName;
                    opCompImgMsg.format = "jpeg";
                    opCompImgMsg.data = inData->getData();
                    opCompImgMsgs.push_back(opCompImgMsg);
                    if (_updateRosBaseTime) updateRosBaseTime();
                }
        };

    } 
} 

std::tuple<dai::Pipeline, int, int, int, int> createPipeline(
    bool enableDepth,
    bool enableSpatialDetection,
    bool lrcheck,
    bool extended,
    bool subpixel,
    bool rectify,
    bool depth_aligned,
    int stereo_fps,
    int confidence,
    int LRchecktresh,
    int detectionClassesCount,
    std::string stereoResolution,
    std::string rgbResolutionStr,
    int rgbScaleNumerator,
    int rgbScaleDinominator,
    int previewWidth,
    int previewHeight) {

    dai::Pipeline pipeline;
    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    // auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    // Add encoders for compression
    auto leftEnc = pipeline.create<dai::node::VideoEncoder>();
    auto rightEnc = pipeline.create<dai::node::VideoEncoder>();
    auto rgbEnc = pipeline.create<dai::node::VideoEncoder>();

    controlIn->setStreamName("control");
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    // if(enableDepth) {
    // xoutDepth->setStreamName("depth");
    // } else {
    // xoutDepth->setStreamName("disparity");
    // }
    xoutImu->setStreamName("imu");

    // Set names for left/right/rgb XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutRgb->setStreamName("rgb");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int stereoWidth = 0, stereoHeight = 0, rgbWidth = 0, rgbHeight = 0;

    if (stereoResolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        stereoWidth = 1280;
        stereoHeight = 720;
    } else if (stereoResolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        stereoWidth = 640;
        stereoHeight = 400;
    } else if (stereoResolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        stereoWidth = 1280;
        stereoHeight = 800;
    } else if (stereoResolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        stereoWidth = 640;
        stereoHeight = 480;
    } else {
        DEPTHAI_ROS_ERROR_STREAM("DEPTHAI", "Invalid parameter. -> monoResolution: " << stereoResolution);
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(stereo_fps);

    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(stereo_fps);

    stereo->initialConfig.setConfidenceThreshold(confidence); 
    stereo->setRectifyEdgeFillColor(0);                        
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
   
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 100);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20); 
    
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    dai::node::ColorCamera::Properties::SensorResolution rgbResolution;

    if (rgbResolutionStr == "1080p") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
        rgbWidth = 1920;
        rgbHeight = 1080;
    } else if (rgbResolutionStr == "4K") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_4_K;
        rgbWidth = 3840;
        rgbHeight = 2160;
    } else if (rgbResolutionStr == "12MP") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP;
        rgbWidth = 4056;
        rgbHeight = 3040;
    } else if (rgbResolutionStr == "13MP") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_13_MP;
        rgbWidth = 4208;
        rgbHeight = 3120;
    } else {
        DEPTHAI_ROS_ERROR_STREAM("DEPTHAI", "Invalid parameter. -> rgbResolution: " << rgbResolutionStr);
        throw std::runtime_error("Invalid color camera resolution.");
    }

    camRgb->setResolution(rgbResolution);
    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;
    camRgb->setIspScale(rgbScaleNumerator, rgbScaleDinominator);

    // RGB settings
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setInterleaved(true);
    camRgb->setPreviewSize(previewWidth, previewHeight);
    camRgb->setFps(stereo_fps);

    // Configure encoders
    leftEnc->setDefaultProfilePreset(stereo_fps, dai::VideoEncoderProperties::Profile::MJPEG);
    leftEnc->setQuality(50);
    rightEnc->setDefaultProfilePreset(stereo_fps, dai::VideoEncoderProperties::Profile::MJPEG);
    rightEnc->setQuality(50);
    rgbEnc->setDefaultProfilePreset(stereo_fps, dai::VideoEncoderProperties::Profile::MJPEG);
    rgbEnc->setQuality(50);

    // link rgb video to encoder to xoutRgb
    camRgb->video.link(rgbEnc->input);
    rgbEnc->bitstream.link(xoutRgb->input);

    // Link stereo left/right outputs to encoders to left/right XLinkOut 
    if (rectify) {
        stereo->rectifiedLeft.link(leftEnc->input);
        leftEnc->bitstream.link(xoutLeft->input);
        stereo->rectifiedRight.link(rightEnc->input);
        rightEnc->bitstream.link(xoutRight->input);
    } else {
        stereo->syncedLeft.link(leftEnc->input);
        leftEnc->bitstream.link(xoutLeft->input);
        stereo->syncedRight.link(rightEnc->input);
        rightEnc->bitstream.link(xoutRight->input);
    }

    // Link plugins CAM -> STEREO -> XLINK
    stereo->setRectifyEdgeFillColor(0);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    imu->out.link(xoutImu->input);

    std::cout << "Stereo: " << stereoWidth << "x" << stereoHeight << " RGB: " << rgbWidth << "x" << rgbHeight << std::endl;

    return std::make_tuple(pipeline, stereoWidth, stereoHeight, rgbWidth, rgbHeight);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    std::string tfPrefix, mode, mxId, resourceBaseFolder;
    std::string monoResolution = "720p", rgbResolution = "1080p";
    int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, detectionClassesCount, expTime, sensIso;
    int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned, manualExposure;
    bool enableSpatialDetection, enableDotProjector, enableFloodLight;
    bool usb2Mode, poeMode;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectorIntensity, floodLightIntensity;
    bool enableRosBaseTimeUpdate;

    node->declare_parameter("mxId", "");
    node->declare_parameter("usb2Mode", false);
    node->declare_parameter("poeMode", false);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("imuMode", 1);
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("rectify", false);
    node->declare_parameter("depth_aligned", true);
    node->declare_parameter("stereo_fps", 15);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");
    node->declare_parameter("rgbResolution", "1080p");
    node->declare_parameter("manualExposure", false);
    node->declare_parameter("expTime", 20000);
    node->declare_parameter("sensIso", 800);
    node->declare_parameter("rgbScaleNumerator", 2);
    node->declare_parameter("rgbScaleDinominator", 3);
    node->declare_parameter("previewWidth", 416);
    node->declare_parameter("previewHeight", 416);
    node->declare_parameter("angularVelCovariance", 0.02);
    node->declare_parameter("linearAccelCovariance", 0.0);
    node->declare_parameter("enableSpatialDetection", true);
    node->declare_parameter("detectionClassesCount", 80);
    node->declare_parameter("enableDotProjector", false);
    node->declare_parameter("enableFloodLight", false);
    node->declare_parameter("dotProjectorIntensity", 0.5);
    node->declare_parameter("floodLightIntensity", 0.5);
    node->declare_parameter("enableRosBaseTimeUpdate", true);

    node->get_parameter("mxId", mxId);
    node->get_parameter("usb2Mode", usb2Mode);
    node->get_parameter("poeMode", poeMode);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mode", mode);
    node->get_parameter("imuMode", imuModeParam);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("rectify", rectify);
    node->get_parameter("depth_aligned", depth_aligned);
    node->get_parameter("stereo_fps", stereo_fps);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("rgbResolution", rgbResolution);
    node->get_parameter("manualExposure", manualExposure);
    node->get_parameter("expTime", expTime);
    node->get_parameter("sensIso", sensIso);
    node->get_parameter("rgbScaleNumerator", rgbScaleNumerator);
    node->get_parameter("rgbScaleDinominator", rgbScaleDinominator);
    node->get_parameter("previewWidth", previewWidth);
    node->get_parameter("previewHeight", previewHeight);
    node->get_parameter("angularVelCovariance", angularVelCovariance);
    node->get_parameter("linearAccelCovariance", linearAccelCovariance);
    node->get_parameter("enableSpatialDetection", enableSpatialDetection);
    node->get_parameter("detectionClassesCount", detectionClassesCount);
    node->get_parameter("enableDotProjector", enableDotProjector);
    node->get_parameter("enableFloodLight", enableFloodLight);
    node->get_parameter("dotProjectorIntensity", dotProjectorIntensity);
    node->get_parameter("floodLightIntensity", floodLightIntensity);
    node->get_parameter("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

    if (mode == "depth") {
        enableDepth = true;
    } else {
        enableDepth = false;
    }

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::Pipeline pipeline;
    int stereoWidth = 0, stereoHeight = 0, rgbWidth = 0, rgbHeight = 0;
    bool isDeviceFound = false;

    std::tie(pipeline, stereoWidth, stereoHeight, rgbWidth, rgbHeight) = createPipeline(
        enableDepth,
        enableSpatialDetection,
        lrcheck,
        extended,
        subpixel,
        rectify,
        depth_aligned,
        stereo_fps,
        confidence,
        LRchecktresh,
        detectionClassesCount,
        monoResolution,
        rgbResolution,
        rgbScaleNumerator,
        rgbScaleDinominator,
        previewWidth,
        previewHeight);

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for (auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if (deviceInfo.getMxId() == mxId) {
            if (deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                if (poeMode) {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                } else {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            } else if (deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("\" DepthAI Device with MxId \"" + mxId + "\" is already booted on different process. \"");
            }
        } else if (mxId == "x") {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if (!isDeviceFound) {
        throw std::runtime_error("\" DepthAI Device with MxId \"" + mxId + "\" not found. \"");
    }

    if (!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    // Apply camera controls
    auto controlQueue = device->getInputQueue("control");

    // Set manual exposure
    if (manualExposure) {
        dai::CameraControl ctrl;
        ctrl.setManualExposure(expTime, sensIso);
        controlQueue->send(ctrl);
    }
	
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto leftQueue = device->getOutputQueue("left", 30, false);
    auto rightQueue = device->getOutputQueue("right", 30, false);
    auto rgbQueue = device->getOutputQueue("rgb", 30, false);
    auto calibrationHandler = device->readCalibration();
    auto boardName = calibrationHandler.getEepromData().boardName;

    if (stereoHeight > 480 && boardName == "OAK-D-LITE" && depth_aligned == false) {
        stereoWidth = 640;
        stereoHeight = 480;
    }

    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if (!irDrivers.empty()) {
        if (enableDotProjector) {
            device->setIrLaserDotProjectorIntensity(dotProjectorIntensity);
        }
        if (enableFloodLight) {
            device->setIrFloodLightIntensity(floodLightIntensity);
        }
    }

    // Prepare converters
    dai::ros::CompressedImageConverter leftConverter(tfPrefix + "_left_camera_optical_frame");
    if (enableRosBaseTimeUpdate) {
        leftConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    dai::ros::CompressedImageConverter rightConverter(tfPrefix + "_right_camera_optical_frame");
    if (enableRosBaseTimeUpdate) {
        rightConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if (enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    // IMU publisher
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");
    imuPublish.addPublisherCallback();

    // CameraInfo for left/right/rgb
    dai::rosBridge::ImageConverter tempLeftConverter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = tempLeftConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, stereoWidth, stereoHeight);

    dai::rosBridge::ImageConverter tempRightConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = tempRightConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, stereoWidth, stereoHeight);

    dai::ros::CompressedImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame");
    if (enableRosBaseTimeUpdate) {
        rgbConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    dai::rosBridge::ImageConverter tempRgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = tempRgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);

    auto left_info_pub  = node->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
    auto right_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);
    auto color_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/color/camera_info", 10);  

    // Bridge publishers for left/right/rgb
    const std::string leftPubName = rectify ? std::string("left/image_rect/compressed") : std::string("left/image_raw/compressed");
    const std::string rightPubName = rectify ? std::string("right/image_rect/compressed") : std::string("right/image_raw/compressed");
    const std::string color_topic = "/color/image/compressed";
    
    auto left_func = [&leftConverter, &left_info_pub, &leftCameraInfo](std::shared_ptr<dai::ImgFrame> inData, std::deque<sensor_msgs::msg::CompressedImage>& opMsgs) {
    leftConverter.toRosMsg(inData, opMsgs);
    if (!opMsgs.empty()) {
            leftCameraInfo.header = opMsgs.back().header;
            left_info_pub->publish(leftCameraInfo);
        }
    };

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::CompressedImage, dai::ImgFrame> leftPublish(
        leftQueue, node, leftPubName, left_func, 30, leftCameraInfo, "left");

        auto right_func = [&rightConverter, &right_info_pub, &rightCameraInfo](std::shared_ptr<dai::ImgFrame> inData, std::deque<sensor_msgs::msg::CompressedImage>& opMsgs) {
            rightConverter.toRosMsg(inData, opMsgs);
            if (!opMsgs.empty()) {
                rightCameraInfo.header = opMsgs.back().header;
                right_info_pub->publish(rightCameraInfo);
            }
    };

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::CompressedImage, dai::ImgFrame> rightPublish(
        rightQueue, node, rightPubName, right_func, 30, rightCameraInfo, "right");

        auto rgb_func = [&rgbConverter, &color_info_pub, &rgbCameraInfo](std::shared_ptr<dai::ImgFrame> inData, std::deque<sensor_msgs::msg::CompressedImage>& opMsgs) {
            rgbConverter.toRosMsg(inData, opMsgs);
            if (!opMsgs.empty()) {
                rgbCameraInfo.header = opMsgs.back().header;
                color_info_pub->publish(rgbCameraInfo);
            }
    };

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::CompressedImage, dai::ImgFrame> rgbPublish(rgbQueue, node, color_topic, rgb_func, 30, rgbCameraInfo, "color");

    // Publish the images
    leftPublish.addPublisherCallback();
    rightPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();

    rclcpp::spin(node);
    return 0;
}
