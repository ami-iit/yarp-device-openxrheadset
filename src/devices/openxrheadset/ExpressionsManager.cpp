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
    m_eyeExpressionsPortName = prefix + "/expressions/eye";
    m_lipExpressionsPortName = prefix + "/expressions/lip";
    m_gazePortName = prefix + "/expressions/gaze";

    if (m_eyeSupported)
    {
        if (!m_eyeExpressionsPort.open(m_eyeExpressionsPortName))
        {
            return false;
        }
    }

    if (m_lipSupported)
    {
        if (!m_lipExpressionsPort.open(m_lipExpressionsPortName))
        {
            return false;
        }
    }

    if (m_gazeSupported)
    {
        if (!m_gazePort.open(m_gazePortName))
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

    Eigen::Vector3f gazeDirectionInHead = headPose.rotation.inverse() * gaze.rotation * Eigen::Vector3f::UnitZ();

    yarp::sig::Vector& gazeVector = m_gazePort.prepare();
    gazeVector.resize(3);
    gazeVector[0] = gazeDirectionInHead.x();
    gazeVector[1] = gazeDirectionInHead.y();
    gazeVector[2] = gazeDirectionInHead.z();
    m_gazePort.write();
}

void ExpressionsManager::close()
{
    m_eyeExpressionsPort.close();
    m_lipExpressionsPort.close();
    m_gazePort.close();
}

std::string ExpressionsManager::getEyeExpressionsPortName() const
{
    return m_eyeExpressionsPortName;
}

std::string ExpressionsManager::getLipExpressionsPortName() const
{
    return m_lipExpressionsPortName;
}

std::string ExpressionsManager::getGazePortName() const
{
    return m_gazePortName;
}
