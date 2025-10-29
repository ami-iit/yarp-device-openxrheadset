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
        if (customOptions->staticPose)
        {
            customOptions->tfBaseFrame = customOptions->parentFrame; //for static poses we publish wrt the parent frame
        }
        else
        {
            customOptions->tfBaseFrame = editedRootFrame; //for dynamic poses we publish wrt the root frame
        }
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

    for (const auto& inputPose : m_posesInputList)
    {
        FilteredPosePublisher& publisher = m_poses[inputPose.name];

        if (!publisher.configured())
        {
            publisher.configure(m_settings);
        }
        OpenXrInterface::NamedPoseVelocity inputPoseModified = inputPose;
        if (!inputPose.parentFrame.empty())
        {
            // The parent frame might have a label, so when publishing the frame we use the correct ID
            // Note that the parent frame in the input pose might have been set only by the OpenXR interface
            // hence, we don't need to check in the custom poses, and we can expect the parent frame to be
            // in the m_poses map
            auto parentIt = m_poses.find(inputPose.parentFrame);
            if (parentIt != m_poses.end())
            {
                inputPoseModified.parentFrame = parentIt->second.label();
            }
        }
        publisher.updateInputPose(inputPoseModified);
    }

    for (size_t i = 0; i < m_customPoses.size(); ++i)
    {
        auto& customPose = m_customPoses[i];
        const std::string& relativeFrame = customPose.relativeFrame();
        OpenXrInterface::NamedPoseVelocity parentFramePose;
        if (customPose.staticPose())
        {
            parentFramePose = OpenXrInterface::NamedPoseVelocity::Identity(relativeFrame);
        }
        else if (relativeFrame == m_rootFramePublisher.name())
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
                // The parent frame might be expressed wrt another frame, so we reconstruct its pose wrt the raw root
                while (!rawRootToParentPose.parentFrame.empty())
                {
                    OpenXrInterface::NamedPoseVelocity parentPose;
                    auto posesIt2 = m_poses.find(rawRootToParentPose.parentFrame);
                    //The parent frame is filled only by the OpenXR interface, so it is not using labels nor custom poses
                    if (posesIt2 != m_poses.end())
                    {
                        parentPose = posesIt2->second.data();
                    }
                    else
                    {
                        break;
                    }
                    //Compose the poses
                    rawRootToParentPose.parentFrame = parentPose.parentFrame;
                    rawRootToParentPose.pose = parentPose.pose * rawRootToParentPose.pose;
                }

                //The pose of the parent frame is expressed with respect to the raw root, i.e. the frame wrt we receive the poses from OpenXR.
                //We publish the custom poses wrt the modified root, i.e. openxr_origin, thus we need to provide the parent frame wrt this other frame

                parentFramePose.name = rawRootToParentPose.name;
                parentFramePose.parentFrame = ""; //Non-static custom poses are published with respect to the origin
                parentFramePose.pose = m_rootFrameRawRelativePoseInverse * rawRootToParentPose.pose;
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

bool PosesManager::setCustomPoseRelativePosition(const std::string &customFrameName, const Eigen::Vector3f &relativePosition)
{
    auto customFrameIt = m_customPosesMap.find(customFrameName);

    if (customFrameIt == m_customPosesMap.end())
    {
        yCError(OPENXRHEADSET) << "The frame" << customFrameName << "does not exist.";
        return false;
    }

    m_customPoses[customFrameIt->second].setRelativePosition(relativePosition);
    return true;
}

bool PosesManager::setCustomPoseRelativeOrientation(const std::string &customFrameName, const Eigen::Quaternionf &relativeOrientation)
{
    auto customFrameIt = m_customPosesMap.find(customFrameName);

    if (customFrameIt == m_customPosesMap.end())
    {
        yCError(OPENXRHEADSET) << "The frame" << customFrameName << "does not exist.";
        return false;
    }

    m_customPoses[customFrameIt->second].setRelativeOrientation(relativeOrientation);
    return true;
}

bool PosesManager::setCustomPoseRelativeOrientation(const std::string &customFrameName, const Eigen::Vector3f &relativeOrientationEulerAngles)
{
    auto customFrameIt = m_customPosesMap.find(customFrameName);

    if (customFrameIt == m_customPosesMap.end())
    {
        yCError(OPENXRHEADSET) << "The frame" << customFrameName << "does not exist.";
        return false;
    }

    m_customPoses[customFrameIt->second].setRelativeOrientation(relativeOrientationEulerAngles);
    return true;
}

void PosesManager::reset()
{
    m_rootFramePublisher.reset();

    for (auto& poseIt : m_poses)
    {
        poseIt.second.reset();
    }

    for (auto& customPose : m_customPoses)
    {
        customPose.reset();
    }
}
