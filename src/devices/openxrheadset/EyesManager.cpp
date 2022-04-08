/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <EyesManager.h>
#include <OpenXrHeadsetLogComponent.h>


const EyesManager::Options &EyesManager::options() const
{
    return m_options;
}

EyesManager::Options &EyesManager::options()
{
    return m_options;
}

bool EyesManager::initialize(yarp::dev::IFrameTransform *tfPublisher, const std::string &headFrame)
{
    if (!m_leftEye.open(m_options.leftEyeQuadLayer,
                        m_options.portPrefix + "/display/left:i",
                        m_options.portPrefix + "/eyeAngles/left:i",
                        tfPublisher, m_options.leftEyeFrame, headFrame)) {
        yCError(OPENXRHEADSET) << "Cannot initialize left display texture.";
        return false;
    }
    m_leftEye.setVisibility(IOpenXrQuadLayer::Visibility::LEFT_EYE);
    m_leftEye.setEyePosition(Eigen::Vector3f(-m_options.interCameraDistance / 2.0, 0.0, 0.0));
    m_leftEye.setEyeRotationOffset(m_options.leftAzimuthOffset, m_options.leftElevationOffset);
    m_leftEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));

    if (!m_rightEye.open(m_options.rightEyeQuadLayer,
                         m_options.portPrefix + "/display/right:i",
                         m_options.portPrefix + "/eyeAngles/right:i",
                         tfPublisher, m_options.rightEyeFrame, headFrame)) {
        yCError(OPENXRHEADSET) << "Cannot initialize right display texture.";
        return false;
    }
    m_rightEye.setVisibility(IOpenXrQuadLayer::Visibility::RIGHT_EYE);
    m_rightEye.setEyePosition(Eigen::Vector3f(m_options.interCameraDistance / 2.0, 0.0, 0.0));
    m_rightEye.setEyeRotationOffset(m_options.rightAzimuthOffset, m_options.rightElevationOffset);
    m_rightEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));

    m_options = m_options;

    return true;
}

void EyesManager::close()
{
    m_leftEye.close();
    m_rightEye.close();
}

bool EyesManager::update()
{
    if (!m_leftEye.update()) {
        yCError(OPENXRHEADSET) << "Failed to update left eye.";
        return false;
    }

    if (!m_rightEye.update()) {
        yCError(OPENXRHEADSET) << "Failed to update right eye.";
        return false;
    }

    return true;
}

void EyesManager::publishEyesTransforms()
{
    m_leftEye.publishEyeTransform();
    m_rightEye.publishEyeTransform();
}

std::vector<double> EyesManager::getLeftImageDimensions()
{
    std::vector<double> output(2);
    output[0] = m_leftEye.layerWidth();
    output[1] = m_leftEye.layerHeight();

    return output;
}

std::vector<double> EyesManager::getRightImageDimensions()
{
    std::vector<double> output(2);
    output[0] = m_rightEye.layerWidth();
    output[1] = m_rightEye.layerHeight();

    return output;
}

std::vector<double> EyesManager::getLeftImageAnglesOffsets()
{
    std::vector<double> output(2);
    output[0] = m_leftEye.azimuthOffset();
    output[1] = m_leftEye.elevationOffset();

    return output;
}

std::vector<double> EyesManager::getRightImageAnglesOffsets()
{
    std::vector<double> output(2);
    output[0] = m_rightEye.azimuthOffset();
    output[1] = m_rightEye.elevationOffset();

    return output;
}

bool EyesManager::setLeftImageAnglesOffsets(const double azimuth, const double elevation)
{
    m_leftEye.setEyeRotationOffset(azimuth, elevation);

    return true;
}

bool EyesManager::setRightImageAnglesOffsets(const double azimuth, const double elevation)
{
    m_rightEye.setEyeRotationOffset(azimuth, elevation);

    return true;
}

bool EyesManager::isLeftEyeActive()
{
    return m_leftEye.active();
}

bool EyesManager::isRightEyeActive()
{
    return m_rightEye.active();
}

double EyesManager::getEyesZPosition()
{
    return m_options.eyeZPosition;
}

bool EyesManager::setEyesZPosition(const double eyesZPosition)
{
    m_options.eyeZPosition = -std::max(0.01, std::abs(eyesZPosition));

    m_leftEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));
    m_rightEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));

    return true;
}

double EyesManager::getInterCameraDistance()
{
    return m_options.interCameraDistance;
}

bool EyesManager::setInterCameraDistance(const double distance)
{
    m_options.interCameraDistance = std::abs(distance);
    m_leftEye.setEyePosition(Eigen::Vector3f(-m_options.interCameraDistance / 2.0, 0.0, 0.0));
    m_rightEye.setEyePosition(Eigen::Vector3f(m_options.interCameraDistance / 2.0, 0.0, 0.0));

    return true;
}

std::string EyesManager::getLeftImageControlPortName()
{
    return m_leftEye.controlPortName();
}

std::string EyesManager::getRightImageControlPortName()
{
    return m_rightEye.controlPortName();
}

