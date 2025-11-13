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

void PosePublisher::resetWarnings()
{
    m_lastWarningTime = 0.0;
    m_warningCount = 0;
}

bool PosePublisher::usePreviousPose() const
{
    return !m_data.pose.positionValid || !m_data.pose.rotationValid;
}

bool PosePublisher::canPublish() const
{
    return configured() && m_active && (m_publishedOnce || !usePreviousPose());
}

void PosePublisher::deactivate()
{
    resetWarnings();
    m_active = false;
}

PosePublisher::PosePublisher()
{
    m_localPose.resize(4,4);
    m_localPose.eye();
}

void PosePublisher::configurePublisher(std::shared_ptr<PosePublisherSettings> settings)
{
    if (!settings)
        return;

    m_tfBaseFrame = settings->tfBaseFrame;
    m_tfPublisher = settings->tfPublisher;
    m_staticPose = settings->staticPose;
}

void PosePublisher::setLabel(const std::string &label)
{
    m_label = label;
}

const std::string &PosePublisher::label() const
{
    return m_label;
}

const bool PosePublisher::staticPose() const
{
    return m_staticPose;
}

bool PosePublisher::configured() const
{
    return m_tfPublisher != nullptr;
}

void PosePublisher::updateInputPose(const OpenXrInterface::NamedPoseVelocity &input)
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

OpenXrInterface::NamedPoseVelocity PosePublisher::data() const
{
    if (!canPublish())
    {
        return OpenXrInterface::NamedPoseVelocity();
    }

    if (usePreviousPose())
    {
        return m_previouslyPublishedData;
    }

    return m_data;
}

void PosePublisher::publish()
{
    if (!canPublish())
    {
        return;
    }

    if (m_staticPose && m_publishedOnce)
    {
        return;
    }

    if (!m_publishedOnce)
    {
        if (m_label.empty())
        {
            m_label = m_data.name;
        }
    }

    if (!usePreviousPose())
    {
        poseToYarpMatrix(m_data.pose, m_localPose);
        m_publishedOnce = true;
        m_previouslyPublishedData = m_data;
        m_data.pose.positionValid = false; //We invalidate the data after use. This is to detect if some pose do not get updated.
        m_data.pose.rotationValid = false;
    }
    else
    {
        if (m_warningCount == 0 || yarp::os::Time::now() - m_lastWarningTime > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_label << " is not valid. Publishing its last known pose.";
            m_lastWarningTime = yarp::os::Time::now();
            m_warningCount++;
        }

        if (m_warningCount > 6)
        {
            yCWarning(OPENXRHEADSET) << m_label << " was not valid for 30s. Deactivated.";
            deactivate();
            return;
        }
    }

    std::string parentFrame = m_tfBaseFrame;
    if (!m_data.parentFrame.empty())
    {
        parentFrame = m_data.parentFrame;
    }

    if (m_staticPose)
    {
        if (!m_tfPublisher->setTransformStatic(m_label, parentFrame, m_localPose))
        {
            yCWarning(OPENXRHEADSET) << "Failed to publish" << m_label << "frame (static). It will not be published again.";
        }
        else
        {
            yCInfo(OPENXRHEADSET) << "Published transformation from" << parentFrame << "to" << m_label << " (static).";
        }
        return;
    }

    if (!m_tfPublisher->setTransform(m_label, parentFrame, m_localPose))
    {
        yCWarning(OPENXRHEADSET) << "Failed to publish" << m_label << "frame.";
    }
}

void PosePublisher::reset()
{
    m_publishedOnce = false;
    m_data = OpenXrInterface::NamedPoseVelocity();
    m_previouslyPublishedData = OpenXrInterface::NamedPoseVelocity();
    m_localPose.eye();
    deactivate();
}
