/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_EYEPORT_H
#define YARP_DEV_EYEPORT_H

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <OpenXrInterface.h>
#include <ImagePortToQuadLayer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class SingleEyePort
{
    float m_azimuthOffset {0.0};
    float m_elevationOffset {0.0};
    Eigen::Quaternionf m_desiredRotation;
    Eigen::Quaternionf m_rotationOffset;
    Eigen::Quaternionf m_imageRotation;
    Eigen::Vector3f m_eyePosition;
    Eigen::Vector3f m_eyeRelativeImagePosition;
    ImagePortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>> m_layer;
    yarp::os::BufferedPort<yarp::sig::Vector> m_eyeAnglesPort;
    bool m_initialized{false};

    bool updateControlAngles();

public:

    bool open(std::shared_ptr<IOpenXrQuadLayer> quadLayer, const std::string& anglesPortName);

    bool openImagePort(const std::string& imagePortName);

    void close();

    void setEyePosition(const Eigen::Vector3f& position);

    void setEyeRotationOffset(double azimuth, double elevation);

    void setEyeRotation(double azimuth, double elevation);

    void setImageRotation(double ccwRotationInRad);

    void setImageDimensions(float widthInMeters, float heightInMeters);

    double azimuthOffset() const;

    double elevationOffset() const;

    void setEyeRelativeImagePosition(const Eigen::Vector3f& position);

    void setVisibility(const IOpenXrQuadLayer::Visibility& visibility);

    float layerWidth() const;

    float layerHeight() const;

    bool update();

    bool update(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& img, GLint startX, GLint startY, GLint endX, GLint endY);

    bool active() const;

    std::string controlPortName() const;
};

#endif // YARP_DEV_EYEPORT_H
