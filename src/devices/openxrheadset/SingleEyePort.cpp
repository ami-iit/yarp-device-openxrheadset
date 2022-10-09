/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <SingleEyePort.h>
#include <OpenXrYarpUtilities.h>

bool SingleEyePort::updateControlAngles()
{
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

    return true;
}

bool SingleEyePort::open(std::shared_ptr<IOpenXrQuadLayer> quadLayer, const std::string &anglesPortName)
{
    m_initialized = false;
    if (!m_layer.initialize(quadLayer)) {
        return false;
    }


    if (!m_eyeAnglesPort.open(anglesPortName))
    {
        yCError(OPENXRHEADSET) << " Failed to open port" << anglesPortName;
        return false;
    }

    m_eyeAnglesPort.setReadOnly();


    m_desiredRotation.setIdentity();
    m_rotationOffset.setIdentity();
    m_imageRotation.setIdentity();
    m_eyeRelativeImagePosition.setZero();
    m_eyeRelativeImagePosition[2] = -1.0; //Initialize the z position of the camera image visualization at 1m distance
    m_eyePosition.setZero();

    m_initialized = true;

    return true;
}

bool SingleEyePort::openImagePort(const std::string &imagePortName)
{
    return m_layer.openImagePort(imagePortName);
}

void SingleEyePort::close()
{
    m_initialized = false;
    m_eyeAnglesPort.close();
    m_layer.close();
}

void SingleEyePort::setEyePosition(const Eigen::Vector3f &position)
{
    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_eyePosition = position;

    m_layer.setPosition(m_rotationOffset * m_desiredRotation * m_eyeRelativeImagePosition + m_eyePosition);
}

bool SingleEyePort::active() const
{
    return m_initialized && m_layer.active();
}

std::string SingleEyePort::controlPortName() const
{
    if (!m_initialized)
    {
        return "";
    }
    return m_eyeAnglesPort.getName();
}

void SingleEyePort::setEyeRotationOffset(double azimuth, double elevation)
{
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
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i * i_H_(i*)
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length
    // - i_H_(i*) is a pure rotation to account the image rotation around an axis perpendicular to it


    Eigen::Quaternionf eyeRotation = m_rotationOffset * m_desiredRotation;
    m_layer.setPose(eyeRotation * m_eyeRelativeImagePosition + m_eyePosition, eyeRotation * m_imageRotation);
}

void SingleEyePort::setEyeRotation(double azimuth, double elevation)
{
    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_desiredRotation = Eigen::AngleAxisf(elevation, Eigen::Vector3f::UnitX()) * //The X axis is pointing to the right in the VIEW space
            Eigen::AngleAxisf(azimuth, Eigen::Vector3f::UnitY()); //The Y axis is pointing upwards in the VIEW space

    Eigen::Quaternionf eyeRotation = m_rotationOffset * m_desiredRotation;

    //The pose of the image is computed as a series of transforms
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i * i_H_(i*)
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length
    // - i_H_(i*) is a pure rotation to account the image rotation around an axis perpendicular to it

    m_layer.setNewImageDesiredPose(eyeRotation * m_eyeRelativeImagePosition + m_eyePosition, eyeRotation * m_imageRotation);
}

void SingleEyePort::setImageRotation(double ccwRotationInRad)
{
    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_imageRotation = Eigen::AngleAxisf(ccwRotationInRad, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf eyeRotation = m_rotationOffset * m_desiredRotation;

    //The pose of the image is computed as a series of transforms
    //w_H_i  = w_H_we * we_H_(e*) * (e*)_H_e * e_H_i * i_H_(i*)
    //where:
    // - w_H_we is the transformation from the world of the headset, to the world of the eye.
    //          This is generally only a translation along the x axis equal to half of the IPD with
    //           the sign depending on whether it is the left or right eye.
    // - we_H_(e*) is a correction rotation that considers eventual mechanical miscalibrations on the robot eyes (rotation offset).
    // - (e*)_H_e is a rotation depending on the eye's elevation and azimuth angle.
    // - e_H_i indicates the pose of the image frame with respect to the eye. This is usually just a simple translation on the z axis
    //         with a modulus dependent on the camera focal length
    // - i_H_(i*) is a pure rotation to account the image rotation around an axis perpendicular to it

    m_layer.setQuaternion(eyeRotation * m_imageRotation);
}

void SingleEyePort::setImageDimensions(float widthInMeters, float heightInMeters)
{
    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_layer.setDimensions(widthInMeters, heightInMeters);
}

double SingleEyePort::azimuthOffset() const
{
    return m_azimuthOffset;
}

double SingleEyePort::elevationOffset() const
{
    return m_elevationOffset;
}

void SingleEyePort::setEyeRelativeImagePosition(const Eigen::Vector3f &position)
{
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

void SingleEyePort::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    if (!m_initialized)
    {
        yCError(OPENXRHEADSET) << "Eye port not initialized.";
        return;
    }

    m_layer.setVisibility(visibility);
}

float SingleEyePort::layerWidth() const
{
    return m_layer.layerWidth();
}

float SingleEyePort::layerHeight() const
{
    return m_layer.layerHeight();
}

bool SingleEyePort::update()
{
    if (!updateControlAngles())
    {
        return false;
    }

    return m_layer.updateTexture();
}

bool SingleEyePort::update(const yarp::sig::ImageOf<yarp::sig::PixelRgb> &img, GLint startX, GLint startY, GLint endX, GLint endY)
{
    if (!updateControlAngles())
    {
        return false;
    }

    return m_layer.updateTexture(img, startX, startY, endX, endY);
}
