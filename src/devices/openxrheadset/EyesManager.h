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

class EyesManager
{
public:

    struct Options
    {
        std::shared_ptr<IOpenXrQuadLayer> leftEyeQuadLayer;
        std::shared_ptr<IOpenXrQuadLayer> rightEyeQuadLayer;
        std::string portPrefix;
        std::string leftEyeFrame;
        std::string rightEyeFrame;
        double leftAzimuthOffset;
        double leftElevationOffset;
        double eyeZPosition;
        double interCameraDistance;
        double rightAzimuthOffset;
        double rightElevationOffset;
    };

    const Options& options() const;

    Options& options();

    bool initialize(yarp::dev::IFrameTransform *tfPublisher, const std::string& headFrame);

    void close();

    bool update();

    void publishEyesTransforms();

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
    Options m_options;
};

#endif // YARP_DEV_EYESMANAGER_H
