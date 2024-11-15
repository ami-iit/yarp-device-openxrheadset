/*
 * Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_EXPRESSIONSMANAGER_H
#define YARP_DEV_EXPRESSIONSMANAGER_H

#include <string>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <OpenXrInterface.h>

class ExpressionsManager
{
    yarp::os::BufferedPort<yarp::sig::Vector> m_eyeExpressionsPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_lipExpressionsPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_gazePort;
    bool m_eyeSupported{ false };
    bool m_lipSupported{ false };
    bool m_gazeSupported{ false };

public:

    bool configure(const std::string& prefix, bool eyeSupported, bool lipSupported, bool gazeSupported);

    void setExpressions(const std::vector<float>& eyeExpressions, const std::vector<float>& lipExpressions);

    void setGaze(const OpenXrInterface::Pose& headPose, const OpenXrInterface::Pose& gaze);

    void close();
};



#endif // YARP_DEV_EXPRESSIONSMANAGER_H