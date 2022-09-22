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

void PosesManager::initialize(const std::vector<Label>& labels, const std::vector<CustomPosePublisherSettings> &customPoses, const FilteredPosePublisherSettings &settings)
{
    m_settings = std::make_shared<FilteredPosePublisherSettings>(settings);

    for (auto& label : labels)
    {
        m_poses[label.original].setLabel(label.modified);
        m_labelsReverseMap[label.modified] = label.original;
    }

    for (auto& customPose : customPoses)
    {
        m_customPoses.emplace_back();
        m_customPoses.back().configure(std::make_shared<CustomPosePublisherSettings>(customPose));
    }

    m_rootPose = OpenXrInterface::NamedPoseVelocity::Identity(m_settings->rootFrame);;
}

std::vector<OpenXrInterface::NamedPoseVelocity> &PosesManager::inputs()
{
    return m_posesInputList;
}

void PosesManager::publishFrames()
{
    for (auto& inputPose : m_posesInputList)
    {
        FilteredPosePublisher& publisher = m_poses[inputPose.name];

        if (!publisher.configured())
        {
            publisher.configure(m_settings);
        }

        publisher.updateInputPose(inputPose);
    }

    for (auto& customPose : m_customPoses)
    {
        const std::string& relativeFrame = customPose.relativeFrame();
        if (relativeFrame == m_settings->rootFrame)
        {
            customPose.updateInputPose(m_rootPose);
        }

        const auto& labelIt = m_labelsReverseMap.find(relativeFrame);
        if (labelIt != m_labelsReverseMap.end()) //The parent frame name is a label
        {
            customPose.updateInputPose(m_poses[labelIt->second].data());
        }
        else
        {
            customPose.updateInputPose(m_poses[relativeFrame].data());
        }
    }

    for (auto& poseIt : m_poses)
    {
        poseIt.second.publish();
    }

    for (auto& customPose : m_customPoses)
    {
        customPose.publish();
    }
}
