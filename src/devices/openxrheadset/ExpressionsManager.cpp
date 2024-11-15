/*
 * Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <ExpressionsManager.h>

bool ExpressionsManager::configure(const std::string& prefix, bool eyeSupported, bool lipSupported, bool gazeSupported)
{
    m_eyeSupported = eyeSupported;
    m_lipSupported = lipSupported;
    m_gazeSupported = gazeSupported;

    if (m_eyeSupported)
    {
        if (!m_eyeExpressionsPort.open(prefix + "/expressions/eye"))
        {
            return false;
        }
    }

    if (m_lipSupported)
    {
        if (!m_lipExpressionsPort.open(prefix + "/expressions/lip"))
        {
            return false;
        }
    }

    if (m_gazeSupported)
    {
        if (!m_gazePort.open(prefix + "/expressions/gaze"))
        {
            return false;
        }
    }

    return true;
}

void ExpressionsManager::setExpressions(const std::vector<float>& eyeExpressions, const std::vector<float>& lipExpressions)
{
    if (m_eyeSupported)
    {
        yarp::sig::Vector& eyeExpressionsVector = m_eyeExpressionsPort.prepare();
        eyeExpressionsVector.resize(eyeExpressions.size());
        for (size_t i = 0; i < eyeExpressions.size(); ++i)
        {
            eyeExpressionsVector[i] = eyeExpressions[i];
        }
        m_eyeExpressionsPort.write();
    }

    if (m_lipSupported)
    {
        yarp::sig::Vector& lipExpressionsVector = m_lipExpressionsPort.prepare();
        lipExpressionsVector.resize(lipExpressions.size());
        for (size_t i = 0; i < lipExpressions.size(); ++i)
        {
            lipExpressionsVector[i] = lipExpressions[i];
        }
        m_lipExpressionsPort.write();
    }
}

void ExpressionsManager::setGaze(const OpenXrInterface::Pose& headPose, const OpenXrInterface::Pose& gaze)
{
    if (!m_gazeSupported || !gaze.positionValid || !headPose.positionValid || !headPose.rotationValid)
    {
        return;
    }

    Eigen::Vector3f gazeInHead = headPose.rotation.inverse() * (gaze.position - headPose.position);

    yarp::sig::Vector& gazeVector = m_gazePort.prepare();
    gazeVector.resize(3);
    gazeVector[0] = gazeInHead.x();
    gazeVector[1] = gazeInHead.y();
    gazeVector[2] = gazeInHead.z();
    m_gazePort.write();
}

void ExpressionsManager::close()
{
    m_eyeExpressionsPort.close();
    m_lipExpressionsPort.close();
    m_gazePort.close();
}
