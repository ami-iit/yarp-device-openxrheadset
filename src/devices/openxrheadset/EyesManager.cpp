/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
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

bool EyesManager::initialize()
{
    if (!m_leftEye.open(m_options.leftEyeQuadLayer,
                        m_options.portPrefix + "/eyeAngles/left:i")) {
        yCError(OPENXRHEADSET) << "Cannot initialize left display texture.";
        return false;
    }
    m_leftEye.setVisibility(IOpenXrQuadLayer::Visibility::LEFT_EYE);
    m_leftEye.setEyePosition(Eigen::Vector3f(-m_options.interCameraDistance / 2.0, 0.0, 0.0));
    m_leftEye.setEyeRotationOffset(m_options.leftAzimuthOffset, m_options.leftElevationOffset);
    m_leftEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));
    m_leftEye.setImageRotation(m_options.leftImageRotation);
    m_leftEye.setImageDimensions(1.5, 1.5);

    if (!m_rightEye.open(m_options.rightEyeQuadLayer,
                         m_options.portPrefix + "/eyeAngles/right:i")) {
        yCError(OPENXRHEADSET) << "Cannot initialize right display texture.";
        return false;
    }
    m_rightEye.setVisibility(IOpenXrQuadLayer::Visibility::RIGHT_EYE);
    m_rightEye.setEyePosition(Eigen::Vector3f(m_options.interCameraDistance / 2.0, 0.0, 0.0));
    m_rightEye.setEyeRotationOffset(m_options.rightAzimuthOffset, m_options.rightElevationOffset);
    m_rightEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_options.eyeZPosition));
    m_rightEye.setImageRotation(m_options.rightImageRotation);
    m_rightEye.setImageDimensions(1.5, 1.5);

    if (m_options.mode == Mode::STEREO_DUAL_PORT)
    {
        if (!m_leftEye.openImagePort(m_options.portPrefix + "/display/left:i"))
        {
            yCError(OPENXRHEADSET) << "Failed to open left display port.";
            return false;
        }

        if (!m_rightEye.openImagePort(m_options.portPrefix + "/display/right:i"))
        {
            yCError(OPENXRHEADSET) << "Failed to open right display port.";
            return false;
        }
    }
    else
    {
        if (!m_commonImagePort.open(m_options.portPrefix + "/display:i"))
        {
            yCError(OPENXRHEADSET) << "Failed to open " + m_options.portPrefix + "/display:i port.";
            return false;
        }

        m_commonImagePort.setReadOnly();
    }

    return true;
}

void EyesManager::close()
{
    m_leftEye.close();
    m_rightEye.close();
    m_commonImagePort.close();
    m_options.leftEyeQuadLayer = nullptr;
    m_options.rightEyeQuadLayer = nullptr;
}

bool EyesManager::update()
{
    if (m_options.mode == Mode::STEREO_DUAL_PORT)
    {
        if (!m_leftEye.update()) {
            yCError(OPENXRHEADSET) << "Failed to update left eye.";
            return false;
        }

        if (!m_rightEye.update()) {
            yCError(OPENXRHEADSET) << "Failed to update right eye.";
            return false;
        }
    }
    else
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* image = m_commonImagePort.read(false);
        if (!image)
        {
            return true;
        }

        GLint leftEndX = image->width();
        GLint rightBeginX = 0;

        if (m_options.mode == Mode::STEREO_SINGLE_PORT)
        {
            leftEndX= static_cast<GLint>(std::round(leftEndX/2.0));
            rightBeginX = leftEndX + 1;
        }

        if (!m_leftEye.update(*image, 0, 0, leftEndX, image->height()))
        {
            yCError(OPENXRHEADSET) << "Failed to update left eye.";
            return false;
        }
        if (!m_rightEye.update(*image, rightBeginX, 0, image->width(), image->height()))
        {
            yCError(OPENXRHEADSET) << "Failed to update right eye.";
            return false;
        }
    }

    return true;
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

double EyesManager::getLeftImageRotation()
{
    return m_options.leftImageRotation;
}

double EyesManager::getRightImageRotation()
{
    return m_options.rightImageRotation;
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

bool EyesManager::setLeftImageRotation(double ccwRotationInRad)
{
    m_options.leftImageRotation = ccwRotationInRad;
    m_leftEye.setImageRotation(ccwRotationInRad);

    return true;
}

bool EyesManager::setRightImageRotation(double ccwRotationInRad)
{
    m_options.rightImageRotation = ccwRotationInRad;
    m_rightEye.setImageRotation(ccwRotationInRad);

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
    m_options.interCameraDistance = distance;
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

Eigen::Quaternionf EyesManager::getLeftEyeDesiredRotation() const
{
    return m_leftEye.getDesiredEyeRotation();
}

Eigen::Quaternionf EyesManager::getRightEyeDesiredRotation() const
{
    return m_rightEye.getDesiredEyeRotation();
}

