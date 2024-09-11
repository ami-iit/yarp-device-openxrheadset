/*
 * Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <ExpressionsManager.h>

bool ExpressionsManager::configure(const std::string& prefix, bool eyeSupported, bool lipSupported)
{
    m_eyeSupported = eyeSupported;
    m_lipSupported = lipSupported;

    if (m_eyeSupported)
    {
        if (!m_eyeExpressionsPort.open(prefix + "/eyeExpressions"))
        {
            return false;
        }
    }

    if (m_lipSupported)
    {
        if (!m_lipExpressionsPort.open(prefix + "/lipExpressions"))
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

void ExpressionsManager::close()
{
    m_eyeExpressionsPort.close();
    m_lipExpressionsPort.close();
}
