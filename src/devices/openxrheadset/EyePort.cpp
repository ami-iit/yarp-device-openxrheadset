/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <EyePort.h>
#include <OpenXrYarpUtilities.h>

bool EyePort::open(std::shared_ptr<IOpenXrQuadLayer> quadLayer, const std::string &imagePortName, const std::string &anglesPortName,
                   yarp::dev::IFrameTransform *tfPublisher, const std::string &tfFrame, const std::string &rootFrame)
{
    yCTrace(OPENXRHEADSET);
    m_initialized = false;
    if (!m_layer.initialize(quadLayer, imagePortName)) {
        return false;
    }


    if (!m_eyeAnglesPort.open(anglesPortName))
    {
        yCError(OPENXRHEADSET) << " Failed to open port" << anglesPortName;
        return false;
    }

    m_eyeAnglesPort.setReadOnly();

    if (!tfPublisher)
    {
        yCError(OPENXRHEADSET) << "The transform server interface is not valid.";
        return false;
    }

    m_desiredRotation.setIdentity();
    m_rotationOffset.setIdentity();
    m_eyeRelativeImagePosition.setZero();
    m_eyeRelativeImagePosition[2] = -1.0; //Initialize the z position of the camera image visualization at 1m distance
    m_eyePosition.setZero();

    m_localPose.resize(4,4);
    m_localPose.eye();

    m_tfPublisher = tfPublisher;
    m_tfFrame = tfFrame;
    m_rootFrame = rootFrame;

    m_initialized = true;

    return true;
}

void EyePort::close()
{
    m_initialized = false;
    m_eyeAnglesPort.close();
    m_tfPublisher = nullptr;
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

bool EyePort::active() const
{
    yCTrace(OPENXRHEADSET);

    return m_initialized && m_layer.active();
}

std::string EyePort::controlPortName() const
{
    yCTrace(OPENXRHEADSET);

    if (!m_initialized)
    {
        return "";
    }
    return m_eyeAnglesPort.getName();
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

    //The pose of the image is computed as a series of transforms
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length


    Eigen::Quaternionf eyeRotation = m_rotationOffset * m_desiredRotation;
    m_layer.setPose(eyeRotation * m_eyeRelativeImagePosition + m_eyePosition, eyeRotation);
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

    Eigen::Quaternionf eyeRotation = m_rotationOffset * m_desiredRotation;

    //The pose of the image is computed as a series of transforms
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length

    m_layer.setNewImageDesiredPose(eyeRotation * m_eyeRelativeImagePosition + m_eyePosition, eyeRotation);
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

    //The pose of the image is computed as a series of transforms
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length

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

bool EyePort::update()
{
    yCTrace(OPENXRHEADSET);

    yarp::sig::Vector* inputAngles = m_eyeAnglesPort.read(false);

    if (inputAngles)
    {
        if (inputAngles->size() != 2)
        {
            yCError(OPENXRHEADSET) << "The input to " + m_eyeAnglesPort.getName() + " is supposed to be 2-dimensional."
                                      " The vector is supposed to contain the azimuth (positive anticlockwise) and elevation (positive upwards) angles in radians, in this order.";
            return false;
        }
        setEyeRotation(inputAngles->operator()(0), inputAngles->operator()(1));
    }

    return m_layer.updateTexture();
}
