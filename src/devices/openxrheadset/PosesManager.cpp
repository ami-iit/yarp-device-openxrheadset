/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <PosesManager.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrYarpUtilities.h>
#include <yarp/os/LogStream.h>

void PosesManager::initialize(const std::vector<Label>& labels, const PosePublisherSettings &settings)
{
    m_settings = std::make_shared<PosePublisherSettings>(settings);

    for (auto& label : labels)
    {
        m_poses[label.original].setLabel(label.modified);
    }
}

std::vector<OpenXrInterface::NamedPoseVelocity> &PosesManager::inputs()
{
    return m_posesInputList;
}

void PosesManager::publishFrames()
{
    for (auto& inputPose : m_posesInputList)
    {
        PosePublisher& publisher = m_poses[inputPose.name];

        if (!publisher.configured())
        {
            publisher.configure(m_settings);
        }

        publisher.update(inputPose);
    }

    for (auto& poseIt : m_poses)
    {
        poseIt.second.publish();
    }
}
