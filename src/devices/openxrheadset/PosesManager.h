/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_POSESMANAGER_H
#define YARP_DEV_POSESMANAGER_H

#include <OpenXrInterface.h>
#include <FilteredPosePublisher.h>
#include <CustomPosePublisher.h>
#include <yarp/dev/IFrameTransform.h>
#include <string>
#include <unordered_map>
#include <openxr/openxr.h>


class PosesManager
{
    std::shared_ptr<FilteredPosePublisherSettings> m_settings{nullptr};

    std::vector<OpenXrInterface::NamedPoseVelocity> m_posesInputList;

    std::unordered_map<std::string, FilteredPosePublisher> m_poses;

    std::vector<CustomPosePublisher> m_customPoses;

    std::unordered_map<std::string, size_t> m_customPosesMap;

    std::unordered_map<std::string, std::string> m_labelsReverseMap;

    OpenXrInterface::NamedPoseVelocity m_rootPose;

    CustomPosePublisher m_rootFramePublisher;

    OpenXrInterface::Pose m_rootFrameRawRelativePose, m_rootFrameRawRelativePoseInverse;

public:

    struct Label
    {
        std::string original;
        std::string modified;
    };

    void initialize(const std::string& editedRootFrame,
                    const std::vector<Label> &labels,
                    const std::vector<CustomPosePublisherSettings> &customPoses,
                    const FilteredPosePublisherSettings &settings);

    std::vector<OpenXrInterface::NamedPoseVelocity>& inputs();

    void publishFrames();

    void setTransformFromRawToRootFrame(const OpenXrInterface::Pose& relativePose);

    bool setCustomPoseRelativePosition(const std::string& customFrameName, const Eigen::Vector3f& relativePosition);

    bool setCustomPoseRelativeOrientation(const std::string& customFrameName, const Eigen::Quaternionf& relativeOrientation);

    bool setCustomPoseRelativeOrientation(const std::string& customFrameName, const Eigen::Vector3f& relativeOrientationEulerAngles);

    void setHandJoints(const std::vector<XrHandJointLocationEXT>&leftHandJoints, const std::vector<XrHandJointLocationEXT>&rightHandJoints);
    
    std::vector<XrHandJointLocationEXT> m_leftHandJoints;
    std::vector<XrHandJointLocationEXT> m_rightHandJoints;

    void reset();

};


#endif // YARP_DEV_POSESMANAGER_H
