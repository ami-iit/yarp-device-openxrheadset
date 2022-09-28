/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

//This code has been taken from https://www.geometrictools.com/Documentation/EulerAngles.pdf

#include <EulerAngles.h>
#include <cmath>
#include <unordered_set>
#include <algorithm>
#include <yarp/os/LogStream.h>
#include <OpenXrHeadsetLogComponent.h>

Eigen::Vector3f EulerAngles::XYZ(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaX = output[0];
    float& thetaY = output[1];
    float& thetaZ = output[2];

    if ( r(0, 2) < +1)
    {
        if ( r(0, 2) > -1)
        {
            thetaY = asin(r(0, 2));
            thetaX = atan2(-r(1, 2) , r(2, 2));
            thetaZ = atan2(-r(0, 1) , r(0, 0));
        }
        else // r 0 2 = -1
        {
            // Not a unique solution: thetaZ - thetaX = atan2(r10 , r11)
            thetaY = -M_PI / 2;
            thetaX = -atan2(r(1,0) , r(1, 1));
            thetaZ = 0;
        }
    }
    else // r02 = +1
    {
        // Not a unique solution: thetaZ + thetaX = atan2( r10, r11)
        thetaY = +M_PI / 2;
        thetaX = atan2(r(1,0) , r(1,1));
        thetaZ = 0;
    }

    return output;
}

Eigen::Vector3f EulerAngles::XZY(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaX = output[0];
    float& thetaZ = output[1];
    float& thetaY = output[2];

    if (r(0, 1) < +1)
    {
        if ( r(0, 1) > -1)
        {
            thetaZ = asin(-r(0, 1));
            thetaX = atan2(r(2, 1) , r(1,1));
            thetaY = atan2(r(0,2) , r(0,0));
        }
        else // r01 = -1
        {
            // Not a unique solution: thetaY - thetaX = atan2(-r20 , r22)
            thetaZ = +M_PI / 2;
            thetaX = -atan2(-r(2, 0) , r(2, 2));
            thetaY = 0;
        }
    }
    else // r01 = +1
    {
        // Not a unique solution: thetaY + thetaX = atan2(-r20, r22)
        thetaZ = -M_PI / 2;
        thetaX = atan2(-r(2,0) , r(2, 2));
        thetaY = 0;
    }

    return output;
}

Eigen::Vector3f EulerAngles::YXZ(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaY = output[0];
    float& thetaX = output[1];
    float& thetaZ = output[2];

    if (r(1, 2) < +1)
    {
        if ( r(1, 2) > -1)
        {
            thetaX = asin(-r(1, 2));
            thetaY = atan2(r(0, 2), r(2, 2));
            thetaZ = atan2(r(1, 0), r(1,1));
        }
        else // r12 = -1
        {
            // Not a unique solution : thetaZ - thetaY = atan2(-r01, r00)
            thetaX = +M_PI / 2;
            thetaY = -atan2(-r(0,1) , r(0,0));
            thetaZ = 0;
        }
    }
    else // r12 = +1
    {
        // Not a unique solution: thetaZ + thetaY = atan2(-r01 , r00)
        thetaX = -M_PI / 2 ;
        thetaY = atan2(-r(0,1) , r(0,0));
        thetaZ = 0;
    }

    return output;
}

Eigen::Vector3f EulerAngles::YZX(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaY = output[0];
    float& thetaZ = output[1];
    float& thetaX = output[2];

    if (r(1, 0) < +1)
    {
        if (r(1, 0) > -1)
        {
            thetaZ = asin(r(1, 0));
            thetaY = atan2(-r(2, 0), r(0, 0));
            thetaX = atan2(-r(1, 2) , r(1,1));
        }
        else // r10 = -1
        {
            // Not a unique solution: thetaX - thetaY = atan2(r21, r22)
            thetaZ = -M_PI / 2 ;
            thetaY = -atan2(r(2, 1) , r(2, 2));
            thetaX = 0;
        }
    }
    else
    {
        // Not a unique solution: thetaX + thetaY = atan2(r21, r22)
        thetaZ = +M_PI / 2 ;
        thetaY = atan2(r(2, 1), r(2, 2));
        thetaX = 0;
    }

    return output;
}

Eigen::Vector3f EulerAngles::ZXY(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaZ = output[0];
    float& thetaX = output[1];
    float& thetaY = output[2];

    if (r(2, 1) < +1)
    {
        if (r(2, 1) > -1)
        {
            thetaX = asin(r(2,1));
            thetaZ = atan2(-r(0, 1) , r(1, 1));
            thetaY = atan2(-r(2, 0) , r(2, 2));
        }
        else // r21 = -1
        {
            // Not a unique solution: thetaY - thetaZ = atan2(r02, r00)
            thetaX = -M_PI / 2;
            thetaZ = -atan2(r(0, 2), r(0, 0));
            thetaY = 0;
        }
    }
    else // r21 = +1
    {
        // Not a unique solution: thetaY + thetaZ = atan2(r02, r00)
        thetaX = +M_PI / 2;
        thetaZ = atan2(r(0, 2), r(0, 0));
        thetaY = 0;
    }

    return output;
}

Eigen::Vector3f EulerAngles::ZYX(const Eigen::Matrix3f &r)
{
    Eigen::Vector3f output;
    float& thetaZ = output[0];
    float& thetaY = output[1];
    float& thetaX = output[2];

    if (r(2, 0) < +1)
    {
        if (r(2, 0) > -1)
        {
            thetaY = asin(-r(2, 0));
            thetaZ = atan2(r(1, 0), r(0, 0));
            thetaX = atan2(r(2, 1), r(2, 2));
        }
        else // r20 = -1
        {
            // Not a unique solution: thetaX - thetaZ = atan2(-r12 , r11)
            thetaY = +M_PI/ 2;
            thetaZ = -atan2(-r(1, 2), r(1, 1));
            thetaX = 0;
        }
    }
    else // r20 = +1
    {
        // Not a unique solution: thetaX + thetaZ = atan2(-r12, r11)
        thetaY = -M_PI / 2 ;
        thetaZ = atan2(-r(1, 2), r(1, 1));
        thetaX = 0;
    }

    return output;
}

bool EulerAngles::isParametrizationAvailable(const std::string &parametrization)
{
    std::unordered_set<std::string> availableParametrizations{"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
    std::string upperCased = parametrization;
    std::transform(upperCased.begin(), upperCased.end(),upperCased.begin(), ::toupper);

    return availableParametrizations.find(upperCased) != availableParametrizations.end();
}

Eigen::Vector3f eulerAngles(const std::array<RotationAxis, 3> &parametrization, const Eigen::Quaternionf &rotation)
{

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::X, RotationAxis::Y, RotationAxis::Z})
    {
        return EulerAngles::XYZ(rotation.toRotationMatrix());
    }

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::X, RotationAxis::Z, RotationAxis::Y})
    {
        return EulerAngles::XZY(rotation.toRotationMatrix());
    }

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::Y, RotationAxis::X, RotationAxis::Z})
    {
        return EulerAngles::YXZ(rotation.toRotationMatrix());
    }

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::Y, RotationAxis::Z, RotationAxis::X})
    {
        return EulerAngles::YZX(rotation.toRotationMatrix());
    }

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::Z, RotationAxis::X, RotationAxis::Y})
    {
        return EulerAngles::ZXY(rotation.toRotationMatrix());
    }

    if (parametrization == std::array<RotationAxis, 3>{RotationAxis::Z, RotationAxis::Y, RotationAxis::X})
    {
        return EulerAngles::ZYX(rotation.toRotationMatrix());
    }

    yCError(OPENXRHEADSET) << "Parametrization not supported.";

    return Eigen::Vector3f::Zero();
}

