/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <EyePort.h>
#include <OpenXrYarpUtilities.h>

bool EyePort::open(std::shared_ptr<IOpenXrQuadLayer> quadLayer, const std::string &inputPortName,
                                               yarp::dev::IFrameTransform *tfPublisher, const std::string &tfFrame, const std::string &rootFrame)
{
    yCTrace(OPENXRHEADSET);
    m_initialized = false;
    if (!m_layer.initialize(quadLayer, inputPortName)) {
        return false;
    }

    if (!tfPublisher)
    {
        yCError(OPENXRHEADSET) << "The transform server interface is not valid.";
        return false;
    }

    m_desiredRotation.setIdentity();
    m_rotationOffset.setIdentity();
    m_eyeRelativeImagePosition.setZero();
    m_eyeRelativeImagePosition[2] = -1.0;
    m_eyePosition.setZero();

    m_tfPublisher = tfPublisher;
    m_tfFrame = tfFrame;
    m_rootFrame = rootFrame;

    m_initialized = true;

    return true;
}

void EyePort::setEyePosition(const Eigen::Vector3f &position)
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_eyePosition = position;

    m_layer.setPosition(m_rotationOffset * m_desiredRotation * m_eyeRelativeImagePosition + m_eyePosition);
}

void EyePort::publishEyeTransform()
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    poseToYarpMatrix(m_layer.layerPosition(), m_layer.layerQuaternion(), m_localPose);
    if (!m_tfPublisher->setTransform(m_tfFrame, m_rootFrame, m_localPose))
    {
        yCWarning(OPENXRHEADSET) << "Failed to publish" << m_tfFrame << "frame.";
    }
}

void EyePort::setEyeRotationOffset(double azimuth, double elevation)
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_azimuthOffset = azimuth;
    m_elevationOffset = elevation;

    m_rotationOffset = Eigen::AngleAxisf(elevation, Eigen::Vector3f::UnitX()) * //The X axis is pointing to the right in the VIEW space
            Eigen::AngleAxisf(azimuth, Eigen::Vector3f::UnitY()); //The Y axis is pointing upwards in the VIEW space

    Eigen::Quaternionf desiredRotation = m_rotationOffset * m_desiredRotation;
    m_layer.setPose(desiredRotation * m_eyeRelativeImagePosition + m_eyePosition, desiredRotation);
}

void EyePort::setEyeRotation(double azimuth, double elevation)
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_desiredRotation = Eigen::AngleAxisf(elevation, Eigen::Vector3f::UnitX()) * //The X axis is pointing to the right in the VIEW space
            Eigen::AngleAxisf(azimuth, Eigen::Vector3f::UnitY()); //The Y axis is pointing upwards in the VIEW space

    Eigen::Quaternionf desiredRotation = m_rotationOffset * m_desiredRotation;
    m_layer.setNewImageDesiredPose(desiredRotation * m_eyeRelativeImagePosition + m_eyePosition, desiredRotation);
}

double EyePort::azimuthOffset() const
{
    yCTrace(OPENXRHEADSET);
    return m_azimuthOffset;
}

double EyePort::elevationOffset() const
{
    yCTrace(OPENXRHEADSET);
    return m_elevationOffset;
}

void EyePort::setEyeRelativeImagePosition(const Eigen::Vector3f &position)
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_eyeRelativeImagePosition = position;

    m_layer.setPosition(m_rotationOffset * m_desiredRotation * m_eyeRelativeImagePosition + m_eyePosition);
}

void EyePort::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_layer.setVisibility(visibility);
}

float EyePort::layerWidth() const
{
    yCTrace(OPENXRHEADSET);

    return m_layer.layerWidth();
}

float EyePort::layerHeight() const
{
    yCTrace(OPENXRHEADSET);

    return m_layer.layerHeight();
}

bool EyePort::updateTexture()
{
    yCTrace(OPENXRHEADSET);

    return m_layer.updateTexture();
}
