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
    m_data = input;
    m_active = true;
}

void PosePublisher::publish()
{
    if (!configured())
    {
        return;
    }

    if (m_active)
    {
        if (m_data.pose.positionValid && m_data.pose.rotationValid)
        {
            m_lastWarningTime = 0.0;
            m_warningCount = 0;

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
        else if (m_publishedOnce)
        {
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
    }
}
