//
// Created by burban on 30.01.22.
//

#include "DepthAiCamera.h"
#include "constants.h"



static void updateBlendWeights(int percentRgb, void* ctx) {
   // rgbWeight = float(percentRgb) / 100.f;
  //  depthWeight = 1.f - rgbWeight;
}
DepthAiCamera::DepthAiCamera() {
    transformation_matrix_1 = CONSTANTS::CAMERA::transformation_matrix_1;
    transformation_matrix_2 = CONSTANTS::CAMERA::transformation_matrix_2;

}


void DepthAiCamera::startCamera() {

    static std::atomic<bool> downscaleColor{true};
    static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
    static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

    static float rgbWeight = 0.6f;
    static float depthWeight = 0.4f;


    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();
    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    if(downscaleColor) camRgb->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    camRgb->initialControl.setManualFocus(135);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    //stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);


    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);

    // Connect to device and start pipeline
    this->device.startPipeline(pipeline);


    // Sets queues size and behavior
    for(const auto& name : queueNames) {
        device.getOutputQueue(name, 4, false);
    }

}

bool DepthAiCamera::getNextFrame(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {

    std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

    //auto queueEvents = device.getQueueEvents(queueNames);
 /*   for(const auto& name : queueEvents) {
        auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
        auto count = packets.size();
        if(count > 0) {
            latestPacket[name] = packets[count - 1];
        }
    }*/


    auto queueEvents = device.getQueueEvents(queueNames);
    for(const auto& name : queueEvents) {
        auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
        auto count = packets.size();
        if(count > 0) {
            latestPacket[name] = packets[count - 1];
            if(name == "rgb")
                currentColorImage = latestPacket["rgb"]->getData();
            if(name == "depth")
                currentDepthImage = latestPacket["depth"]->getData();

        }
        else {
            return false;
        }
    }

    if (currentColorImage.size() == 0 || currentDepthImage.size() == 0)
        return false;
    size_t height = 1280; //latestPacket["rgb"]->getHeight();
    size_t width = 720; //latestPacket["rgb"]->getWidth();
    colors.resize(height*width);
    points.resize(height*width);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint16_t depth = currentDepthImage.at(y+x);
            float r = (float) currentDepthImage.at(y+x) / 255.0F;
            float g = (float) currentDepthImage.at(y+x+1) / 255.0F;
            float b = (float) currentDepthImage.at(y+x+2) / 255.0F;
            float point[3];
            float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};

            colors.at(x + y * width) = Eigen::Vector3d{r, g, b};
            points.at(x + y * width) = transformation_matrix_2 * (transformation_matrix_1 *
                                                                  Eigen::Vector3d{point[0], point[1],
                                                                                  point[2]} *
                                                                  depth_scale);

        }
    }

    return true;
}