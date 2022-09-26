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
    rootFramePublisherOptions->tfBaseFrame = m_settings->tfBaseFrame;
    rootFramePublisherOptions->name = editedRootFrame;
    rootFramePublisherOptions->parentFrame = m_settings->tfBaseFrame;
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
        customOptions->tfBaseFrame = editedRootFrame;
        m_customPoses.back().configure(customOptions);
        m_customPosesMap[customPose.name] = m_customPoses.size() - 1;
    }

    m_rootPose = OpenXrInterface::NamedPoseVelocity::Identity(editedRootFrame);
    m_rootFrameRawRelativePoseInverse = OpenXrInterface::NamedPoseVelocity::Identity(editedRootFrame).pose;
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

    for (size_t i = 0; i < m_customPoses.size(); ++i)
    {
        auto& customPose = m_customPoses[i];
        const std::string& relativeFrame = customPose.relativeFrame();
        OpenXrInterface::NamedPoseVelocity parentFramePose;

        if (relativeFrame == m_rootFramePublisher.name())
        {
            parentFramePose = m_rootPose;
        }
        else
        {
            auto labelIt = m_labelsReverseMap.find(relativeFrame);
            auto posesIt = m_poses.find(relativeFrame);
            auto customPoseIt = m_customPosesMap.find(relativeFrame);

            if (labelIt != m_labelsReverseMap.end() || posesIt != m_poses.end())
            {

                OpenXrInterface::NamedPoseVelocity rawRootToParentPose;

                if (labelIt != m_labelsReverseMap.end()) //The parent frame name is a label
                {
                    rawRootToParentPose = m_poses[labelIt->second].data();
                }
                else
                {
                    rawRootToParentPose = posesIt->second.data();
                }

                //The pose of the parent frame is expressed with respect to the raw root, i.e. the frame wrt we receive the poses from OpenXR.
                //We publish the custom poses wrt the modified root, i.e. openxr_origin, thus we need to provide the parent frame wrt this other frame

                parentFramePose.name = rawRootToParentPose.name;
                parentFramePose.pose.positionValid = rawRootToParentPose.pose.positionValid;
                parentFramePose.pose.rotationValid = rawRootToParentPose.pose.rotationValid;

                parentFramePose.pose.position = m_rootFrameRawRelativePoseInverse.position + m_rootFrameRawRelativePoseInverse.rotation * rawRootToParentPose.pose.position;
                parentFramePose.pose.rotation = m_rootFrameRawRelativePoseInverse.rotation * rawRootToParentPose.pose.rotation;
            }
            else if (customPoseIt != m_customPosesMap.end())
            {
                if (customPoseIt->second < i)
                {
                    parentFramePose = m_customPoses[customPoseIt->second].data();
                }
                else
                {
                    yCWarningThrottle(OPENXRHEADSET, 10) << "The custom pose" << customPose.name()
                                                         << "requires" << relativeFrame << "that is not updated yet (custom poses are updated in order depending on the configuration file).";
                }
            }
            else
            {
                yCWarningThrottle(OPENXRHEADSET, 10) << "The custom pose" << customPose.name()
                                                     << "requires" << relativeFrame << "but it does not seem to be available.";
            }
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
