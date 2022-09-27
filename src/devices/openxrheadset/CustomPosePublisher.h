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
#include <EulerAngles.h>
#include <array>
#include <string>
#include <memory>
#include <yarp/os/Bottle.h>

struct CustomPosePublisherSettings : PosePublisherSettings
{
    std::string parentFrame;
    std::string name;

    std::array<bool, 3> positionMask = {true, true, true};
    Eigen::Vector3f relativePosition;

    std::array<bool, 3> rotationMask = {true, true, true};
    std::array<RotationAxis, 3> anglesOrder = {RotationAxis::Y, RotationAxis::X, RotationAxis::Z};
    Eigen::Vector3f relativeRotation;

    bool parseFromConfigurationFile(const yarp::os::Bottle& inputGroup);

};

class CustomPosePublisher : public PosePublisher
{
    std::shared_ptr<CustomPosePublisherSettings> m_settings;
    std::string m_parentFrame;
    std::string m_name;

public:

    void configure(std::shared_ptr<CustomPosePublisherSettings> settings);

    const std::string& name() const;

    const std::string& relativeFrame() const;

    void setRelativePosition(const Eigen::Vector3f relativePosition);

    void setRelativeOrientation(const Eigen::Quaternionf& relativeOrientation);

    void setRelativeOrientation(const Eigen::Vector3f& relativeOrientationEulerAngles);

    virtual bool configured() const override;

    virtual void updateInputPose(const OpenXrInterface::NamedPoseVelocity& input) override;

};

#endif // YARP_DEV_CUSTOMPOSEPUBLISHER_H
