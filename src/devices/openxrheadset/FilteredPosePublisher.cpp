/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <FilteredPosePublisher.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrYarpUtilities.h>
#include <yarp/os/LogStream.h>

bool FilteredPosePublisher::positionJumped(const OpenXrInterface::NamedPoseVelocity& input)
{
    if (!m_lastValidData.pose.positionValid || !input.pose.positionValid)
    {
        return false; //Also the last "valid" pose is not valid, or the input is already not valid
    }

    if (!m_lastValidData.velocity.linearValid)
    {
        yCWarning(OPENXRHEADSET) << "The last valid pose of" << label() << "has non-valid linear velocity. Assuming there are jumps.";
        return true;
    }

    Eigen::Vector3f expectedPosition = m_lastValidData.pose.position + m_settings->period * m_lastValidData.velocity.linear;

    return (expectedPosition - input.pose.position).norm() > m_settings->checks.maxDistance;
}

bool FilteredPosePublisher::rotationJumped(const OpenXrInterface::NamedPoseVelocity &input)
{
    if (!m_lastValidData.pose.rotationValid || !input.pose.rotationValid)
    {
        return false; //Also the last "valid" pose is not valid, or the input is already not valid
    }

    if (!m_lastValidData.velocity.angularValid)
    {
        yCWarning(OPENXRHEADSET) << "The last valid pose of" << label() << "has non-valid angular velocity. Assuming there are jumps.";
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

    Eigen::Quaternionf actualRotationDifference = input.pose.rotation * m_lastValidData.pose.rotation.inverse();

    return std::abs(actualRotationDifference.angularDistance(expectedRotationDifference)) > m_settings->checks.maxAngularDistanceInRad;
}

OpenXrInterface::NamedPoseVelocity FilteredPosePublisher::filterJumps(const OpenXrInterface::NamedPoseVelocity &input)
{
    bool positionHasJumped = positionJumped(input);
    bool rotationHasJumped = rotationJumped(input);
    OpenXrInterface::NamedPoseVelocity output{input};

    double interpolationFactor = 0.0;

    if (!positionHasJumped && !rotationHasJumped)
    {
        m_lastValidDataTime = yarp::os::Time::now();
        m_convergingToJump = false;
    }
    else
    {
        if ((yarp::os::Time::now() - m_lastValidDataTime) > (m_settings->checks.lastDataExpirationTime))
        {
            if (!m_convergingToJump) //Print the warning only once
            {
                yCWarning(OPENXRHEADSET) << label() << "last valid data has expired. The pose will converge to the measured one.";
            }
            m_convergingToJump = true;
            interpolationFactor = m_settings->checks.convergenceRatio;
        }
    }

    if (positionHasJumped)
    {
        yCWarning(OPENXRHEADSET) << label() << "position had a jump.";
        output.pose.position = (1 - interpolationFactor) * m_lastValidData.pose.position + interpolationFactor * output.pose.position;
    }

    if (rotationHasJumped)
    {
        yCWarning(OPENXRHEADSET) << label() << "rotation had a jump.";
        output.pose.rotation = m_lastValidData.pose.rotation.slerp(interpolationFactor, output.pose.rotation);
    }

    m_lastValidData = output;

    if ((yarp::os::Time::now() - m_lastValidDataTime) > (m_settings->checks.lastDataExpirationTime + m_settings->checks.maxConvergenceTime))
    {
        yCWarning(OPENXRHEADSET) << label() << "last valid convergence has expired. The pose will be aligned to the measured one.";
        resetLastValidData();
    }

    return output;
}

void FilteredPosePublisher::resetLastValidData()
{
    //Invalidate previous pose
    m_lastValidData.pose.positionValid = false;
    m_lastValidData.pose.rotationValid = false;

    m_convergingToJump = false;
}

void FilteredPosePublisher::deactivate()
{
    PosePublisher::deactivate();
    resetLastValidData();
}

void FilteredPosePublisher::configure(std::shared_ptr<FilteredPosePublisherSettings> settings)
{
    m_settings = settings;
    configurePublisher(m_settings);
}

bool FilteredPosePublisher::configured() const
{
    return m_settings != nullptr && PosePublisher::configured();
}

void FilteredPosePublisher::updateInputPose(const OpenXrInterface::NamedPoseVelocity &input)
{
    if (!configured())
    {
        return;
    }

    bool inputValid = input.pose.positionValid && input.pose.rotationValid;

    if (!inputValid)
    {
        return;
    }
<<<<<<< HEAD

    //PosePublisher::updateInputPose(filterJumps(input));
    PosePublisher::updateInputPose(input);
=======
    switch (input.filterType)
    {
        case PoseFilterType::JUMP_FILTER:
            PosePublisher::updateInputPose(filterJumps(input));
            break;
        default:
            PosePublisher::updateInputPose(input);
            break;
    }
>>>>>>> b8808df9357e8e653cdaf14760dedc9cd0e9d46a
}
