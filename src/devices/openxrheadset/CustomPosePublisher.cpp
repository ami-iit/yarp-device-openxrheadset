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
        m_name = settings->name;
        PosePublisher::configurePublisher(settings);
    }
}

const std::string &CustomPosePublisher::name() const
{
    return m_name;
}

const std::string &CustomPosePublisher::relativeFrame() const
{
    return m_parentFrame;
}

void CustomPosePublisher::setRelativePosition(const Eigen::Vector3f relativePosition)
{
    if (!configured())
    {
        return;
    }

    m_settings->relativePosition = relativePosition;
}

void CustomPosePublisher::setRelativeOrientation(const Eigen::Quaternionf &relativeOrientation)
{
    if (!configured())
    {
        return;
    }

    m_settings->relativeRotation = eulerAngles(m_settings->anglesOrder, relativeOrientation);
}

void CustomPosePublisher::setRelativeOrientation(const Eigen::Vector3f &relativeOrientationEulerAngles)
{
    if (!configured())
    {
        return;
    }

    m_settings->relativeRotation = relativeOrientationEulerAngles;
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

    Eigen::Vector3f relativePosition = input.pose.position + input.pose.rotation * m_settings->relativePosition;

    for (size_t i = 0; i < 3; ++i)
    {
        if (m_settings->positionMask[i])
        {
            output.pose.position[i] = relativePosition[i];
        }
        else
        {
            output.pose.position[i] = 0.0;
        }
    }


    Eigen::Vector3f inverseParentFrameEulerAngles = eulerAngles(m_settings->anglesOrder, input.pose.rotation.inverse());

    Eigen::Quaternionf relativeRotation;
    relativeRotation.setIdentity();
    for (size_t i = 0; i < 3; ++i)
    {
        float angle = inverseParentFrameEulerAngles[i];
        if (m_settings->rotationMask[i])
        {
            angle = m_settings->relativeRotation[i];

        }
        relativeRotation = relativeRotation * Eigen::AngleAxisf(angle,
                                                                Eigen::Vector3f::Unit(static_cast<Eigen::Index>(m_settings->anglesOrder[i])));
    }
    output.pose.rotation = input.pose.rotation * relativeRotation;

    PosePublisher::updateInputPose(output);
}


bool CustomPosePublisherSettings::parseFromConfigurationFile(const yarp::os::Bottle &inputGroup)
{
    if (inputGroup.isNull())
    {
        yCError(OPENXRHEADSET) << "The input group is not found in the configuration file.";
        return false;
    }

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

    staticPose = inputGroup.check("static_pose") && (inputGroup.find("static_pose").isNull() || inputGroup.find("static_pose").asBool());

    std::string parametrization = inputGroup.check("euler_angles", yarp::os::Value("")).toString();

    if (parametrization.size() != 3)
    {
        yCError(OPENXRHEADSET) << "Failed to parse euler_angles parameter. It is either not present, or it is not a string of three values.";
        return false;
    }

    if (!EulerAngles::isParametrizationAvailable(parametrization))
    {
        yCError(OPENXRHEADSET) << "Failed to parse euler_angles parameter. The parametrization" << parametrization << "is not available.";
        return false;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        switch (std::tolower(parametrization[i]))
        {
        case 'x':
            anglesOrder[i] = RotationAxis::X;
            break;

        case 'y':
            anglesOrder[i] = RotationAxis::Y;
            break;

        case 'z':
            anglesOrder[i] = RotationAxis::Z;
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
            if (value.isFloat32() || value.isFloat64())
            {
                outputOffset[i] = value.asFloat32();
                outputMask[i] = true;
            }
            else if (value.isString() && value.asString() == "*")
            {
                outputOffset[i] = 0;
                outputMask[i] = false;
            }
            else
            {
                yCError(OPENXRHEADSET) << "Failed to parse" << maskName
                                       << "at element" << i
                                       << ". Only float numbers and the special character \"*\" are allowed";
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

    if (staticPose)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (!positionMask[i])
            {
                yCError(OPENXRHEADSET) << "The relative_position mask is not valid for a static pose.";
                return false;
            }

            if (!rotationMask[i])
            {
                yCError(OPENXRHEADSET) << "The relative_rotation mask is not valid for a static pose.";
                return false;
            }
        }
    }

    return true;
}
