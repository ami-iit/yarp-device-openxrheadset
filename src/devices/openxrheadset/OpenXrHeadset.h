/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRHEADSET_H
#define YARP_DEV_OPENXRHEADSET_H

#include <vector>
#include <atomic>
#include <mutex>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ServiceInterfaces.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <OpenXrInterface.h>
#include <PortToQuadLayer.h>

#include <Eigen/Core>

namespace yarp {
namespace dev {

class OpenXrHeadset;

}
}

class yarp::dev::OpenXrHeadset : public yarp::dev::DeviceDriver,
                                 public yarp::os::PeriodicThread,
                                 public yarp::dev::IService,
                                 public yarp::dev::IJoypadController
{
public:
    OpenXrHeadset();

    virtual ~OpenXrHeadset();

    // yarp::dev::DeviceDriver methods
    virtual bool open(yarp::os::Searchable& cfg) override;
    virtual bool close() override;

    // yarp::os::RateThread methods
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

    //  yarp::dev::IService methods
    virtual bool startService() override;
    virtual bool updateService() override;
    virtual bool stopService() override;

    // yarp::dev::IJoypadController methods
    virtual bool getAxisCount(unsigned int& axis_count) override;
    virtual bool getButtonCount(unsigned int& button_count) override;
    virtual bool getTrackballCount(unsigned int& trackball_count) override;
    virtual bool getHatCount(unsigned int& hat_count) override;
    virtual bool getTouchSurfaceCount(unsigned int& touch_count) override;
    virtual bool getStickCount(unsigned int& stick_count) override;
    virtual bool getStickDoF(unsigned int stick_id, unsigned int& dof) override;
    virtual bool getButton(unsigned int button_id, float& value) override;
    virtual bool getTrackball(unsigned int trackball_id, yarp::sig::Vector& value) override;
    virtual bool getHat(unsigned int hat_id, unsigned char& value) override;
    virtual bool getAxis(unsigned int axis_id, double& value) override;
    virtual bool getStick(unsigned int stick_id, yarp::sig::Vector& value, JoypadCtrl_coordinateMode coordinate_mode) override;
    virtual bool getTouch(unsigned int touch_id, yarp::sig::Vector& value) override;

private:

    struct guiParam
    {
        float         width;
        float         height;
        float         x;
        float         y;
        float         z;
        std::string    portName;
        PortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgba>> layer;
    };

    yarp::os::BufferedPort<yarp::os::Bottle>* orientationPort;
    yarp::os::BufferedPort<yarp::os::Bottle>* positionPort;
    yarp::os::BufferedPort<yarp::os::Bottle>* angularVelocityPort;
    yarp::os::BufferedPort<yarp::os::Bottle>* linearVelocityPort;
    yarp::os::Stamp stamp;

    std::string m_prefix;

    std::array<PortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>>, 2> displayPorts;

    std::vector<guiParam> huds;

    bool getStickAsAxis;

    IFrameTransform* tfPublisher;
    std::string      left_frame;
    std::string      right_frame;
    std::string      head_frame;
    std::string      root_frame;
    PolyDriver       driver;

    std::atomic_bool closed{ false };

    OpenXrInterface openXrInterface;

    yarp::sig::Matrix headPose;
    yarp::sig::Matrix leftHandPose;
    yarp::sig::Matrix rightHandPose;


    std::vector<bool> buttons;
    std::vector<float> axes;
    std::vector<Eigen::Vector2f> thumbsticks;

    std::mutex m_mutex;

};

#endif // YARP_DEV_OPENXRHEADSET_H
