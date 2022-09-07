//
// Created by burban on 30.01.22.
//

#ifndef CPP3D_DEPTHAICAMERA_H
#define CPP3D_DEPTHAICAMERA_H
#include "iostream"
#include "depthai/depthai.hpp"
#include "Eigen/Dense"
class DepthAiCamera  {
public:
    DepthAiCamera();
    void warmup(int n_frames);
    void startCameraWithRecord(std::string filename);
    void startCameraFromFile(std::string filename);
    void startCamera();
    bool getNextFrame(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);
    bool shutdown();


private:
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;
    dai::Device device;
    float depth_scale = 1000.0;


    std::vector<uint8_t> currentColorImage;
    std::vector<uint8_t> currentDepthImage;
    Eigen::Matrix3d transformation_matrix_1;
    Eigen::Matrix3d transformation_matrix_2;
};


#endif //CPP3D_DEPTHAICAMERA_H
