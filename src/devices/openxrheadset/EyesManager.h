/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_EYESMANAGER_H
#define YARP_DEV_EYESMANAGER_H

#include <SingleEyePort.h>
#include <OpenXrInterface.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class EyesManager
{
public:

    struct Options
    {
        std::shared_ptr<IOpenXrQuadLayer> leftEyeQuadLayer;
        std::shared_ptr<IOpenXrQuadLayer> rightEyeQuadLayer;
        std::string portPrefix;
        double leftAzimuthOffset{0.0};
        double leftElevationOffset{0.0};
        double rightAzimuthOffset{0.0};
        double rightElevationOffset{0.0};
        double eyeZPosition{-1.0};
        double interCameraDistance{0.07};
        bool splitEyes{true};
    };

    const Options& options() const;

    Options& options();

    bool initialize();

    void close();

    bool update();

    std::vector<double> getLeftImageDimensions();

    std::vector<double> getRightImageDimensions();

    std::vector<double> getLeftImageAnglesOffsets();

    std::vector<double> getRightImageAnglesOffsets();

    bool setLeftImageAnglesOffsets(const double azimuth, const double elevation);

    bool setRightImageAnglesOffsets(const double azimuth, const double elevation);

    bool isLeftEyeActive();

    bool isRightEyeActive();

    double getEyesZPosition();

    bool setEyesZPosition(const double eyesZPosition);

    double getInterCameraDistance();

    bool setInterCameraDistance(const double distance);

    std::string getLeftImageControlPortName();

    std::string getRightImageControlPortName();

private:

    SingleEyePort m_leftEye, m_rightEye;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> m_commonImagePort;
    Options m_options;
};

#endif // YARP_DEV_EYESMANAGER_H
