/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <CustomPosePublisher.h>
#include <OpenXrHeadsetLogComponent.h>
#include <yarp/os/LogStream.h>
#include <cctype>

void CustomPosePublisher::configure(std::shared_ptr<CustomPosePublisherSettings> settings)
{
    if (settings)
    {
        m_settings = settings;
        m_parentFrame = settings->parentFrame;
        //We publish the transform relative to the parent frame
        m_settings->rootFrame = m_parentFrame;
        PosePublisher::configurePublisher(settings);
    }
}

const std::string &CustomPosePublisher::relativeFrame() const
{
    return m_parentFrame;
}

bool CustomPosePublisher::configured() const
{
    return m_settings != nullptr && !m_settings->name.empty() && PosePublisher::configured();
}

void CustomPosePublisher::updateInputPose(const OpenXrInterface::NamedPoseVelocity &input)
{
    if (!configured())
    {
        return;
    }

    bool inputValid = input.pose.positionValid && input.pose.rotationValid;

    if (!inputValid)
    {
        return;
    }

    OpenXrInterface::NamedPoseVelocity output;
    output.name = m_settings->name;
    output.pose.positionValid = true;
    output.pose.rotationValid = true;
    output.velocity.linearValid = false;
    output.velocity.angularValid = false;

    Eigen::Quaternionf inverseRotation = input.pose.rotation.inverse();
    Eigen::Vector3f inversePosition = -inverseRotation.toRotationMatrix() * input.pose.position;

    for (size_t i = 0; i < 3; ++i)
    {
        if (m_settings->positionMask[i])
        {
            output.pose.position[i] = m_settings->relativePosition[i];
        }
        else
        {
            output.pose.position[i] = inversePosition[i];
        }
    }

    Eigen::Vector3f parentFrameEulerAngles =
            input.pose.rotation.toRotationMatrix().eulerAngles(
                static_cast<Eigen::Index>(m_settings->anglesOrder[0]),
                static_cast<Eigen::Index>(m_settings->anglesOrder[1]),
                static_cast<Eigen::Index>(m_settings->anglesOrder[2]));

    Eigen::Matrix3f absoluteOrientation;
    absoluteOrientation.setIdentity();

    for (size_t i = 0; i < 3; ++i)
    {
        float angle = 0;
        if (m_settings->rotationMask[i])
        {
            angle = parentFrameEulerAngles[i] + m_settings->relativeRotation[i];

        }
        absoluteOrientation = absoluteOrientation * Eigen::AngleAxisf(angle,
                                                                      Eigen::Vector3f::Unit(static_cast<Eigen::Index>(m_settings->anglesOrder[i])));
    }

    output.pose.rotation = inverseRotation * Eigen::Quaternionf(absoluteOrientation);

    PosePublisher::updateInputPose(output);
}


bool CustomPosePublisherSettings::parseFromConfigurationFile(yarp::dev::IFrameTransform *publisher, const std::string &rootFrameName, const yarp::os::Bottle &inputGroup)
{
    if (!publisher)
    {
        yCError(OPENXRHEADSET) << "The tfPublisher is not valid.";
        return false;
    }

    if (inputGroup.isNull())
    {
        yCError(OPENXRHEADSET) << "The input group is not found in the configuration file.";
        return false;
    }

    tfPublisher = publisher;
    rootFrame = rootFrameName;

    name = inputGroup.check("name", yarp::os::Value("")).toString();
    if (name == "")
    {
        yCError(OPENXRHEADSET) << "Failed to find name parameter";
        return false;
    }

    parentFrame = inputGroup.check("parent_frame", yarp::os::Value("")).toString();
    if (parentFrame == "")
    {
        yCError(OPENXRHEADSET) << "Failed to find parent_frame parameter";
        return false;
    }

    std::string parametrization = inputGroup.check("euler_angles", yarp::os::Value("")).toString();

    if (parametrization.size() != 3)
    {
        yCError(OPENXRHEADSET) << "Failed to parse euler_angles parameter. It is either not present, or it is not a string of three values.";
        return false;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        switch (std::tolower(parametrization[i]))
        {
        case 'x':
            anglesOrder[i] = RotationAxis::x;
            break;

        case 'y':
            anglesOrder[i] = RotationAxis::y;
            break;

        case 'z':
            anglesOrder[i] = RotationAxis::z;
            break;

        default:
            yCError(OPENXRHEADSET) << "Failed to parse euler_angles parameter."
                                   << "The element at position" << i << "is not in the set (x, y, z)";
            return false;
        }
    }

    auto parseMask = [](const yarp::os::Bottle& inputGroup, const std::string& maskName,
            std::array<bool, 3> &outputMask, Eigen::Vector3f &outputOffset) -> bool
    {
        yarp::os::Value maskValue = inputGroup.find(maskName);
        if (!maskValue.isList())
        {
            yCError(OPENXRHEADSET) << "Failed to parse" << maskName <<", it is not present, or not a list.";
            return false;
        }

        yarp::os::Bottle* positionMaskList = maskValue.asList();
        if (positionMaskList->size() != 3)
        {
            yCError(OPENXRHEADSET) << "Failed to parse" << maskName <<", the list has dimension different from 3.";
            return false;
        }

        for (size_t i = 0; i < 3; ++i)
        {
            yarp::os::Value& value = positionMaskList->get(i);
            if (value.isFloat32())
            {
                outputOffset[i] = value.asFloat32();
                outputMask[i] = true;
            }
            else if (value.isInt8() && value.asInt8() == '*')
            {
                outputOffset[i] = 0;
                outputMask[i] = false;
            }
            else
            {
                yCError(OPENXRHEADSET) << "Failed to parse" << maskName
                                       << "at element" << i
                                       << ". Only float numbers and the special carachter '*' are allowed";
                return false;
            }
        }

        return true;
    };

    if (!parseMask(inputGroup, "relative_position", positionMask, relativePosition))
    {
        return false;
    }

    if (!parseMask(inputGroup, "relative_rotation", rotationMask, relativeRotation))
    {
        return false;
    }

    return true;
}
