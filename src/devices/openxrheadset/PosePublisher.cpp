/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <PosePublisher.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrYarpUtilities.h>
#include <yarp/os/LogStream.h>

bool PosePublisher::positionJumped()
{
    if (!m_lastValidData.pose.positionValid || !m_data.pose.positionValid)
    {
        return false; //Also the last "valid" pose is not valid, or the input is already not valid
    }

    if (!m_lastValidData.velocity.linearValid)
    {
        yCWarning(OPENXRHEADSET) << "The last valid pose of" << m_label << "has non-valid linear velocity. Assuming there are jumps.";
        return true;
    }

    Eigen::Vector3f expectedPosition = m_lastValidData.pose.position + m_settings->period * m_lastValidData.velocity.linear;

    return (expectedPosition - m_data.pose.position).norm() > m_settings->checks.maxDistance;
}

bool PosePublisher::rotationJumped()
{
    if (!m_lastValidData.pose.rotationValid || !m_data.pose.rotationValid)
    {
        return false; //Also the last "valid" pose is not valid, or the input is already not valid
    }

    if (!m_lastValidData.velocity.angularValid)
    {
        yCWarning(OPENXRHEADSET) << "The last valid pose of" << m_label << "has non-valid angular velocity. Assuming there are jumps.";
        return true;
    }

    double angularVelocity = m_lastValidData.velocity.angular.norm();
    Eigen::Vector3f angularAxis = Eigen::Vector3f::UnitX();

    if (angularVelocity > 1e-15)
    {
        angularAxis = m_lastValidData.velocity.angular.normalized();
    }

    Eigen::Quaternionf expectedRotationDifference;
    expectedRotationDifference = Eigen::AngleAxisf(m_settings->period * angularVelocity, angularAxis);

    Eigen::Quaternionf actualRotationDifference = m_data.pose.rotation * m_lastValidData.pose.rotation.inverse();

    return std::abs(actualRotationDifference.angularDistance(expectedRotationDifference)) > m_settings->checks.maxAngularDistanceInRad;
}

void PosePublisher::filterJumps()
{
    bool positionHasJumped = positionJumped();
    bool rotationHasJumped = rotationJumped();

    if (positionHasJumped)
    {
        yCWarning(OPENXRHEADSET) << m_label << "position had a jump. Keeping the old position.";
        m_data.pose.position = m_lastValidData.pose.position;
    }

    if (rotationHasJumped)
    {
        yCWarning(OPENXRHEADSET) << m_label << "rotation had a jump. Keeping the old rotation.";
        m_data.pose.rotation = m_lastValidData.pose.rotation;
    }

    m_lastValidData = m_data;

    if (!positionHasJumped && !rotationHasJumped)
    {
        m_lastValidDataTime = yarp::os::Time::now();
    }

    if ((m_lastValidDataTime - yarp::os::Time::now()) > m_settings->checks.lastDataExpirationTime)
    {
        yCWarning(OPENXRHEADSET) << m_label << "last valid data has expired. The pose will be aligned to the measured one.";
        m_lastValidData.pose.positionValid = false;
        m_lastValidData.pose.rotationValid = false;
    }
}

void PosePublisher::resetWarnings()
{
    m_lastWarningTime = 0.0;
    m_warningCount = 0;
}

void PosePublisher::publishOldTransform()
{
    if (!m_publishedOnce)
    {
        return;
    }

    //Publish old pose
    if (!m_settings->tfPublisher->setTransform(m_label, m_settings->rootFrame, m_localPose))
    {
        yCWarning(OPENXRHEADSET) << "Failed to publish" << m_label << "frame.";
    }

    if (m_warningCount == 0 || yarp::os::Time::now() - m_lastWarningTime > 5.0)
    {
        yCWarning(OPENXRHEADSET) << m_label << " is not valid. Publishing its last known pose.";
        m_lastWarningTime = yarp::os::Time::now();
        m_warningCount++;
    }

    if (m_warningCount > 6)
    {
        yCWarning(OPENXRHEADSET) << m_label << " was not valid for 30s. Deactivated.";
        m_lastWarningTime = 0.0;
        m_warningCount = 0;
        m_active = false;
    }
}

void PosePublisher::publishNewTransform()
{
    if (!m_publishedOnce)
    {
        if (m_label.empty())
        {
            m_label = m_data.name;
        }
        m_publishedOnce = true;
    }

    poseToYarpMatrix(m_data.pose, m_localPose);

    if (!m_settings->tfPublisher->setTransform(m_label, m_settings->rootFrame, m_localPose))
    {
        yCWarning(OPENXRHEADSET) << "Failed to publish" << m_label << "frame.";
    }

    m_data.pose.positionValid = false; //We invalidate the data after use. This is to detect if some pose do not get updated.
    m_data.pose.rotationValid = false;
}

PosePublisher::PosePublisher()
{
    m_localPose.resize(4,4);
    m_localPose.eye();
}

void PosePublisher::setLabel(const std::string &label)
{
    m_label = label;
}

void PosePublisher::configure(std::shared_ptr<PosePublisherSettings> settings)
{
    m_settings = settings;
}

bool PosePublisher::configured() const
{
    return m_settings == nullptr;
}

void PosePublisher::update(const OpenXrInterface::NamedPoseVelocity &input)
{
    if (!configured())
    {
        return;
    }

    m_data = input;
    bool inputValid = m_data.pose.positionValid && m_data.pose.rotationValid; //we consider the input valid if both the position and the rotation are valid
    bool wasActive = m_publishedOnce && !m_active; //if the pose was published once, but now it is not active, it means that there has been problems. So we should be active only if the input is valid
    m_active = !wasActive || inputValid; //We are active if: - it is the first time we update, - we were active the step before, - we received a valid input
}

void PosePublisher::publish()
{
    if (!configured() || !m_active)
    {
        return;
    }

    bool inputValid = m_data.pose.positionValid && m_data.pose.rotationValid;

    if (!inputValid)
    {
        publishOldTransform();
        return;
    }

    filterJumps();

    resetWarnings();

    publishNewTransform();
}
