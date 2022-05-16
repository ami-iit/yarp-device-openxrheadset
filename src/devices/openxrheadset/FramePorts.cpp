/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <FramePorts.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrYarpUtilities.h>
#include <yarp/os/LogStream.h>

bool FramePorts::open(const std::string &name, const std::string &portPrefix, yarp::dev::IFrameTransform *tfPublisher, const std::string &tfFrame, const std::string &rootFrame)
{
    //opening ports
    std::initializer_list<std::pair<yarp::os::BufferedPort<yarp::os::Bottle>**,
                                    std::string>> ports =
    {
        { &m_orientationPort,                  "quaternion"                   },
        { &m_positionPort,                     "position"                     },
        { &m_angularVelocityPort,              "angularVelocity"              },
        { &m_linearVelocityPort,               "linearVelocity"               }
    };

    for (auto port : ports)
    {
        if (*port.first)
        {
            yCError(OPENXRHEADSET) << port.second <<  "is already open.";
            continue;
        }

        std::string portName;

        *port.first = new yarp::os::BufferedPort<yarp::os::Bottle>;
        portName        = portPrefix + "/" + port.second + ":o";

        if (!(*port.first)->open(portName))
        {
            yCError(OPENXRHEADSET) << "Cannot open" << portName << "port";
            close();
            return false;
        }

        (*port.first)->setWriteOnly();
    }

    double timeNow = yarp::os::Time::now();
    m_lastWarning["quaternion"] = timeNow - 10.0;
    m_lastWarning["position"] = timeNow - 10.0;
    m_lastWarning["angularVelocity"] = timeNow - 10.0;
    m_lastWarning["linearVelocity"] = timeNow - 10.0;
    m_lastWarning["republish"] = timeNow - 10.0;

    m_name = name;
    m_tfPublisher = tfPublisher;
    m_tfFrame = tfFrame;
    m_rootFrame = rootFrame;

    m_localPose.resize(4,4);
    m_localPose.eye();

    return true;
}

void FramePorts::close()
{
    m_localPoseValid = false;

    //Closing and deleting ports
    std::initializer_list<yarp::os::BufferedPort<yarp::os::Bottle>**> ports =
    {
        &m_orientationPort,
        &m_positionPort,
        &m_angularVelocityPort,
        &m_linearVelocityPort,
    };

    for (auto port : ports)
    {
        if (*port)
        {
            (*port)->close();

            delete *port;

            *port = nullptr;
        }
    }
}

void FramePorts::publishFrame(const OpenXrInterface::Pose &pose, const OpenXrInterface::Velocity &velocity, yarp::os::Stamp &stamp)
{
    if (pose.positionValid && pose.rotationValid)
    {
        poseToYarpMatrix(pose, m_localPose);
        m_localPoseValid = true;
        if (!m_tfPublisher->setTransform(m_tfFrame, m_rootFrame, m_localPose))
        {
            yCWarning(OPENXRHEADSET) << "Failed to publish" << m_tfFrame << "frame.";
        }
    }
    else
    {
        if (m_localPoseValid)
        {
            if (!m_tfPublisher->setTransform(m_tfFrame, m_rootFrame, m_localPose))
            {
                yCWarning(OPENXRHEADSET) << "Failed to publish" << m_tfFrame << "frame.";
            }

            if (yarp::os::Time::now() - m_lastWarning["republish"] > 1.0)
            {
                yCWarning(OPENXRHEADSET) << "Publishing last" << m_name << "known pose.";
                m_lastWarning["republish"] = yarp::os::Time::now();
            }
        }
    }

    if (pose.positionValid)
    {
        writeVec3OnPort(m_positionPort, pose.position, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["position"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "position not valid.";
            m_lastWarning["position"] = yarp::os::Time::now();
        }
    }

    if (pose.rotationValid)
    {
        writeQuaternionOnPort(m_orientationPort, pose.rotation, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["quaternion"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "rotation not valid.";
            m_lastWarning["quaternion"] = yarp::os::Time::now();
        }
    }

    if (velocity.linearValid)
    {
        writeVec3OnPort(m_linearVelocityPort, velocity.linear, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["linearVelocity"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "linear velocity not valid.";
            m_lastWarning["linearVelocity"] = yarp::os::Time::now();
        }
    }

    if (velocity.angularValid)
    {
        writeVec3OnPort(m_angularVelocityPort, velocity.angular, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["angularVelocity"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "angular velocity not valid.";
            m_lastWarning["angularVelocity"] = yarp::os::Time::now();
        }
    }
}
