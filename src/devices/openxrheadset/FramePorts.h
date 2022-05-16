/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */
#ifndef YARP_DEV_FRAMEPORTS_H
#define YARP_DEV_FRAMEPORTS_H

#include <OpenXrInterface.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IFrameTransform.h>
#include <string>
#include <unordered_map>

class FramePorts
{
    yarp::os::BufferedPort<yarp::os::Bottle>* m_orientationPort{nullptr};
    yarp::os::BufferedPort<yarp::os::Bottle>* m_positionPort{nullptr};
    yarp::os::BufferedPort<yarp::os::Bottle>* m_angularVelocityPort{nullptr};
    yarp::os::BufferedPort<yarp::os::Bottle>* m_linearVelocityPort{nullptr};

    yarp::dev::IFrameTransform* m_tfPublisher;

    std::string m_tfFrame;
    std::string m_rootFrame;
    std::string m_name;

    std::unordered_map<std::string, double> m_lastWarning;

    yarp::sig::Matrix m_localPose;
    bool m_localPoseValid{false};


public:

    bool open(const std::string& name, const std::string& portPrefix,
              yarp::dev::IFrameTransform* tfPublisher, const std::string& tfFrame, const std::string& rootFrame);

    void close();

    void publishFrame(const OpenXrInterface::Pose& pose,
                      const OpenXrInterface::Velocity& velocity,
                      yarp::os::Stamp &stamp);
};

#endif // YARP_DEV_FRAMEPORTS_H
