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

void PosesManager::initialize(const std::string &editedRootFrame, const std::vector<Label>& labels, const std::vector<CustomPosePublisherSettings> &customPoses, const FilteredPosePublisherSettings &settings)
{
    m_settings = std::make_shared<FilteredPosePublisherSettings>(settings);

    auto rootFramePublisherOptions = std::make_shared<CustomPosePublisherSettings>();
    rootFramePublisherOptions->tfPublisher = m_settings->tfPublisher;
    rootFramePublisherOptions->rawRootFrame = m_settings->rawRootFrame;
    rootFramePublisherOptions->name = editedRootFrame;
    rootFramePublisherOptions->parentFrame = m_settings->rawRootFrame;
    rootFramePublisherOptions->relativePosition.setZero();
    rootFramePublisherOptions->relativeRotation.setZero();
    m_rootFramePublisher.configure(rootFramePublisherOptions);

    for (auto& label : labels)
    {
        m_poses[label.original].setLabel(label.modified);
        m_labelsReverseMap[label.modified] = label.original;
    }

    for (auto& customPose : customPoses)
    {
        m_customPoses.emplace_back();
        auto customOptions = std::make_shared<CustomPosePublisherSettings>(customPose);
        customOptions->tfPublisher = m_settings->tfPublisher;
        customOptions->rawRootFrame = m_settings->rawRootFrame;
        m_customPoses.back().configure(customOptions);
    }

    m_rootPose = OpenXrInterface::NamedPoseVelocity::Identity(editedRootFrame);
}

std::vector<OpenXrInterface::NamedPoseVelocity> &PosesManager::inputs()
{
    return m_posesInputList;
}

void PosesManager::publishFrames()
{

    m_rootFramePublisher.updateInputPose(m_rootPose);

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
        OpenXrInterface::NamedPoseVelocity parentFramePose;

        if (relativeFrame == m_rootFramePublisher.name())
        {
            parentFramePose = m_rootPose;
        }
        else
        {
            const auto& labelIt = m_labelsReverseMap.find(relativeFrame);

            OpenXrInterface::NamedPoseVelocity poseRawRootToParent;

            if (labelIt != m_labelsReverseMap.end()) //The parent frame name is a label
            {
                poseRawRootToParent = m_poses[labelIt->second].data();
            }
            else
            {
                poseRawRootToParent = m_poses[relativeFrame].data();
            }

            //The pose of the parent frame is expressed with respect to the raw root, i.e. the frame wrt we recieve the poses from OpenXR.
            //The user is interested on the root_frame that can be aligned to headset via RPC call.

            parentFramePose.name = poseRawRootToParent.name;
            parentFramePose.pose.positionValid = poseRawRootToParent.pose.positionValid;
            parentFramePose.pose.rotationValid = poseRawRootToParent.pose.rotationValid;

            parentFramePose.pose.position = m_rootFrameRawRelativePoseInverse.position + m_rootFrameRawRelativePoseInverse.rotation * poseRawRootToParent.pose.position;
            parentFramePose.pose.rotation = m_rootFrameRawRelativePoseInverse.rotation * poseRawRootToParent.pose.rotation;

        }

        customPose.updateInputPose(parentFramePose);
    }

    m_rootFramePublisher.publish();

    for (auto& poseIt : m_poses)
    {
        poseIt.second.publish();
    }

    for (auto& customPose : m_customPoses)
    {
        customPose.publish();
    }
}

void PosesManager::setTransformFromRawToRootFrame(const OpenXrInterface::Pose &relativePose)
{
    m_rootFrameRawRelativePose = relativePose;
    m_rootFramePublisher.setRelativePosition(relativePose.position);
    m_rootFramePublisher.setRelativeOrientation(relativePose.rotation);

    m_rootFrameRawRelativePoseInverse.positionValid = relativePose.positionValid;
    m_rootFrameRawRelativePoseInverse.rotationValid = relativePose.rotationValid;

    m_rootFrameRawRelativePoseInverse.rotation = relativePose.rotation.inverse();
    m_rootFrameRawRelativePoseInverse.position = -(m_rootFrameRawRelativePoseInverse.rotation * relativePose.position);
}
