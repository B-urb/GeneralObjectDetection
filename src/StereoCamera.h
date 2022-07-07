//
// Created by burban on 30.01.22.
//

#ifndef CPP3D_STEREOCAMERA_H
#define CPP3D_STEREOCAMERA_H


class StereoCamera {

    virtual void start() = 0;

    virtual void setUpIntrinsicsAndSensor() = 0;

public:
    virtual void setCurrentColorframe() = 0;

    virtual void setCurrentDepthframe() = 0;

    virtual void startCameraWithRecord() = 0;

    virtual void startCameraFromFile() = 0;

    virtual void startCamera() = 0;

    virtual void warmup() = 0;

    virtual bool getNextFrame() = 0;

    virtual ~StereoCamera() = 0;

    virtual bool shutdown() = 0;

    static const rs2_intrinsics &getIntrinsics();

    const rs2::frame &getCurrentColorframe() const;

    const rs2::frame &getCurrentDepthframe() const;

    static std::pair<int,int> get2DPixelFromPoint(Eigen::Vector3d& point);
};


#endif //CPP3D_STEREOCAMERA_H
