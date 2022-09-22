/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_CUSTOMPOSEPUBLISHER_H
#define YARP_DEV_CUSTOMPOSEPUBLISHER_H

#include <PosePublisher.h>
#include <OpenXrInterface.h>
#include <array>
#include <string>
#include <memory>
#include <yarp/os/Bottle.h>

struct CustomPosePublisherSettings : PosePublisherSettings
{
    enum class RotationAxis : Eigen::Index
    {
        x = 0,
        y = 1,
        z = 2
    };

    std::string parentFrame;
    std::string name;

    std::array<bool, 3> positionMask = {false, false, false};
    Eigen::Vector3f relativePosition;

    std::array<bool, 3> rotationMask = {false, false, false};
    std::array<RotationAxis, 3> anglesOrder = {RotationAxis::y, RotationAxis::x, RotationAxis::z};
    Eigen::Vector3f relativeRotation;

    bool parseFromConfigurationFile(yarp::dev::IFrameTransform* publisher,
                                    const std::string& rootFrameName,
                                    const yarp::os::Bottle& inputGroup);

};

class CustomPosePublisher : public PosePublisher
{
    std::shared_ptr<CustomPosePublisherSettings> m_settings;
    std::string m_parentFrame;

public:

    void configure(std::shared_ptr<CustomPosePublisherSettings> settings);

    const std::string& relativeFrame() const;

    virtual bool configured() const override;

    virtual void updateInputPose(const OpenXrInterface::NamedPoseVelocity& input) override;

};

//Notes
// if it is the openxr_origin, or its alias, then the input pose is the identity
//the pose of the parent frame should be set from outside
//the name of the parent frame should be chosen amongs the set of labels first
//and then among the set of pose publisher

#endif // YARP_DEV_CUSTOMPOSEPUBLISHER_H
