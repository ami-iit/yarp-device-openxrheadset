/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <AdditionalPosesPublisher.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrYarpUtilities.h>
#include <yarp/os/LogStream.h>

void AdditionalPosesPublisher::initialize(yarp::dev::IFrameTransform *tfPublisher, const std::vector<Label>& labels, const std::string &rootFrame)
{
    m_tfPublisher = tfPublisher;
    m_rootFrame = rootFrame;

    for (auto& label : labels)
    {
        m_additionalPoses[label.original].label = label.modified;
    }
}

std::vector<OpenXrInterface::NamedPoseVelocity> &AdditionalPosesPublisher::inputs()
{
    return m_additionalPosesInputList;
}

void AdditionalPosesPublisher::publishFrames()
{
    for (auto& inputPose : m_additionalPosesInputList)
    {
        AdditionalPoseInfo& poseInfo = m_additionalPoses[inputPose.name];
        poseInfo.active = true;
        poseInfo.data = inputPose;
    }

    for (auto& poseIt : m_additionalPoses)
    {
        AdditionalPoseInfo& poseInfo = poseIt.second;
        if (poseInfo.active)
        {
            if (poseInfo.data.pose.positionValid && poseInfo.data.pose.rotationValid)
            {
                poseInfo.lastWarningTime = 0.0;
                poseInfo.warningCount = 0;

                if (!poseInfo.publishedOnce)
                {
                    poseInfo.localPose.resize(4,4);
                    poseInfo.localPose.eye();
                    if (poseInfo.label.empty())
                    {
                        poseInfo.label = poseInfo.data.name;
                    }
                    poseInfo.publishedOnce = true;
                }
                poseToYarpMatrix(poseInfo.data.pose, poseInfo.localPose);

                if (!m_tfPublisher->setTransform(poseInfo.label, m_rootFrame, poseInfo.localPose))
                {
                    yCWarning(OPENXRHEADSET) << "Failed to publish" << poseInfo.label << "frame.";
                }

                poseInfo.data.pose.positionValid = false; //We invalidate the data after use. This is to detect if some pose do not get updated.
                poseInfo.data.pose.rotationValid = false;
            }
            else if (poseInfo.publishedOnce)
            {
                //Publish old pose
                if (!m_tfPublisher->setTransform(poseInfo.label, m_rootFrame, poseInfo.localPose))
                {
                    yCWarning(OPENXRHEADSET) << "Failed to publish" << poseInfo.label << "frame.";
                }

                if (poseInfo.warningCount == 0 || yarp::os::Time::now() - poseInfo.lastWarningTime > 5.0)
                {
                    yCWarning(OPENXRHEADSET) << poseInfo.label << " is not valid. Publishing its last known pose.";
                    poseInfo.lastWarningTime = yarp::os::Time::now();
                    poseInfo.warningCount++;
                }

                if (poseInfo.warningCount > 6)
                {
                    yCWarning(OPENXRHEADSET) << poseInfo.label << " was not valid for 30s. Deactivated.";
                    poseInfo.lastWarningTime = 0.0;
                    poseInfo.warningCount = 0;
                    poseInfo.active = false;
                }
            }
        }
    }
}
